#include "CanBridge.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

CanBridge::CanBridge() : serial_fd(-1) {}

CanBridge::~CanBridge() {
    CanBridge::close();
}

bool CanBridge::open(const std::string& port) {
    serial_fd = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd == -1) {
        std::cerr << "Blad otwarcia portu " << port << ": " << strerror(errno) << "\n";
        return false;
    }

    // stty ustawia baudrate 2 Mbd — termios nie obsluguje niestandardowych baudow.
    std::string cmd = "stty -F " + port + " 2000000 raw -echo";
    system(cmd.c_str());

    // Odczyt nieblokujacy: timeout 100 ms, minimalna ilosc bajtow = 0.
    struct termios tty;
    tcgetattr(serial_fd, &tty);
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN]  = 0;
    tcsetattr(serial_fd, TCSANOW, &tty);

    return true;
}

void CanBridge::close() {
    if (serial_fd != -1) {
        ::close(serial_fd);
        serial_fd = -1;
    }
}

// Format wysylanej ramki: AA | (E0 | len) | id[0..3] LE | data[0..n] | 55
void CanBridge::send(uint32_t id, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> packet;
    packet.reserve(6 + data.size() + 1);

    packet.push_back(0xAA);
    packet.push_back(0xE0 | static_cast<uint8_t>(data.size()));
    packet.push_back((id >>  0) & 0xFF);
    packet.push_back((id >>  8) & 0xFF);
    packet.push_back((id >> 16) & 0xFF);
    packet.push_back((id >> 24) & 0xFF);
    packet.insert(packet.end(), data.begin(), data.end());
    packet.push_back(0x55);

    if (::write(serial_fd, packet.data(), packet.size()) == -1)
        std::cerr << "Blad wysylania CAN: " << strerror(errno) << "\n";
}

bool CanBridge::readByte(uint8_t& b) {
    return ::read(serial_fd, &b, 1) == 1;
}

// Format odbieranej ramki: AA | (??|len) | id[0..3] LE | data[0..len-1] | 55
bool CanBridge::receive(uint32_t& id, std::vector<uint8_t>& data) {
    uint8_t b;

    // Synchronizacja: szukaj bajtu startowego 0xAA.
    do {
        if (!readByte(b)) return false;
    } while (b != 0xAA);

    if (!readByte(b)) return false;
    uint8_t data_len = b & 0x0F;

    uint8_t id_bytes[4];
    for (int i = 0; i < 4; i++) {
        if (!readByte(id_bytes[i])) return false;
    }

    data.resize(data_len);
    for (int i = 0; i < data_len; i++) {
        if (!readByte(data[i])) return false;
    }

    // Weryfikacja bajtu stopu.
    if (!readByte(b) || b != 0x55) return false;

    id = id_bytes[0]
       | (static_cast<uint32_t>(id_bytes[1]) << 8)
       | (static_cast<uint32_t>(id_bytes[2]) << 16)
       | (static_cast<uint32_t>(id_bytes[3]) << 24);
    return true;
}

bool CanBridge::waitForData(int timeout_ms) {
    struct pollfd pfd = { serial_fd, POLLIN, 0 };
    return poll(&pfd, 1, timeout_ms) > 0;
}
