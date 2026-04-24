#include "CanBridge.hpp"
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <cstring>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <cerrno>

CanBridge::CanBridge() {
    serial_fd = -1;

};

bool CanBridge::open(const std::string& interface) {
        
    serial_fd = ::open(interface.c_str(), O_RDWR | O_NOCTTY);
    if(serial_fd == -1) { std::cerr<< "Blad otwarcia seriala: " << strerror(errno) << "\n"; return false;}
    std::string cmd = "stty -F " + interface + " 2000000 raw -echo";
    system(cmd.c_str());

    struct termios tty;
    tcgetattr(serial_fd, &tty);
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN]  = 0;
    tcsetattr(serial_fd, TCSANOW, &tty);
    return true;
}

void CanBridge::send(uint32_t id, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> packet;
    packet.push_back(0xAA);
    packet.push_back(0xE0 | data.size());
    packet.push_back(id >> 0 & 0xFF);
    packet.push_back(id >> 8 & 0xFF);
    packet.push_back(id >> 16 & 0xFF);
    packet.push_back(id >> 24 & 0xFF);
    packet.insert(packet.end(), data.begin(),data.end());
    packet.push_back(0x55);
    int result = write(serial_fd, packet.data(), packet.size());
    if(result == -1){ std::cerr << "Blad wysylania CAN\n";}

}

bool CanBridge::readByte(uint8_t& b) {
    return ::read(serial_fd, &b, 1) == 1;
}

bool CanBridge::receive(uint32_t& id, std::vector<uint8_t>& data) {
    uint8_t b;
    // szukaj 0xAA - przeskakuj smieci w buforze
    do {
        if (!readByte(b)) return false;
    } while (b != 0xAA);
    // czytaj drugi bajt
    if (!readByte(b)) return false;
    uint8_t data_len = b & 0x0F;

    // czytaj 4 bajty CAN ID
    uint8_t id_bytes[4];
    for (int i = 0; i < 4; i++) {
        if (!readByte(id_bytes[i])) return false;
    }

    // czytaj dane
    data.resize(data_len);
    for (int i = 0; i < data_len; i++) {
        if (!readByte(data[i])) return false;
    }

    // czytaj stop byte
    if (!readByte(b) || b != 0x55) return false;

    id = id_bytes[0] | (id_bytes[1] << 8) | (id_bytes[2] << 16) | (id_bytes[3] << 24);
    return true;
}


void CanBridge::close() {
    ::close(serial_fd);
    serial_fd = -1;
}

CanBridge::~CanBridge() {
    CanBridge::close();
};
