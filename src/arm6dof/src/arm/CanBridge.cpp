#include "CanBridge.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
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
        std::cerr << "Failed to open port " << port << ": " << strerror(errno) << "\n";
        return false;
    }

    // termios does not support 2 Mbps baudrate — configure via stty instead.
    std::string cmd = "stty -F " + port + " 2000000 raw -echo";
    system(cmd.c_str());

    // Non-blocking mode: VTIME=1 (100ms timeout), VMIN=0.
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

// Format: AA | (C0 | len) | id_low | id_high | data[0..n] | 55
void CanBridge::sendStd(uint16_t id, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> packet;
    packet.reserve(4 + data.size() + 1);

    packet.push_back(0xAA);
    packet.push_back(0xC0 | static_cast<uint8_t>(data.size()));
    packet.push_back(id & 0xFF);
    packet.push_back((id >> 8) & 0x07);
    packet.insert(packet.end(), data.begin(), data.end());
    packet.push_back(0x55);

    if (::write(serial_fd, packet.data(), packet.size()) == -1)
        std::cerr << "CAN send error: " << strerror(errno) << "\n";
}

void CanBridge::sendExt(uint32_t id, const std::vector<uint8_t>& data) {
    if (data.size() > 8) {
        std::cerr << "CAN extended send error: DLC > 8\n";
        return;
    }

    if (id > 0x1FFFFFFF) {
        std::cerr << "CAN extended send error: ID > 29 bits\n";
        return;
    }

    std::vector<uint8_t> packet;
    packet.reserve(6 + data.size() + 1);

    packet.push_back(0xAA);

    // Extended CAN data frame + DLC
    packet.push_back(0xE0 | static_cast<uint8_t>(data.size()));

    // 29-bit extended CAN ID, little-endian
    packet.push_back(id & 0xFF);
    packet.push_back((id >> 8) & 0xFF);
    packet.push_back((id >> 16) & 0xFF);
    packet.push_back((id >> 24) & 0x1F);

    packet.insert(packet.end(), data.begin(), data.end());

    packet.push_back(0x55);

    if (::write(serial_fd, packet.data(), packet.size()) == -1) {
        std::cerr << "CAN extended send error: " << strerror(errno) << "\n";
    }
}

bool CanBridge::readByte(uint8_t& b) {
    return ::read(serial_fd, &b, 1) == 1;
}

bool CanBridge::waitForData(int timeout_ms) {
    struct pollfd pfd = { serial_fd, POLLIN, 0 };
    return poll(&pfd, 1, timeout_ms) > 0;
}

// Reads one frame. Detects type from bit 5 of type_byte:
//   bit5 = 1 -> extended (4B ID)
//   bit5 = 0 -> standard (2B ID)
bool CanBridge::receive(uint32_t& id, std::vector<uint8_t>& data) {
    uint8_t b;

    do {
        if (!readByte(b)) return false;
    } while (b != 0xAA);

    if (!readByte(b)) return false;

    uint8_t frame_info = b;
    uint8_t data_len = frame_info & 0x0F;
    bool is_extended = (frame_info & 0x20) != 0;
    int id_len = is_extended ? 4 : 2;

    if (data_len > 8) {
        return false;
    }

    uint8_t id_bytes[4] = {};
    for (int i = 0; i < id_len; i++) {
        if (!readByte(id_bytes[i])) return false;
    }

    if (is_extended) {
        id =
            static_cast<uint32_t>(id_bytes[0]) |
            (static_cast<uint32_t>(id_bytes[1]) << 8) |
            (static_cast<uint32_t>(id_bytes[2]) << 16) |
            ((static_cast<uint32_t>(id_bytes[3]) & 0x1F) << 24);
    } else {
        id =
            static_cast<uint32_t>(id_bytes[0]) |
            ((static_cast<uint32_t>(id_bytes[1]) & 0x07) << 8);
    }

    data.resize(data_len);
    for (uint8_t i = 0; i < data_len; i++) {
        if (!readByte(data[i])) return false;
    }

    uint8_t tail;
    if (!readByte(tail)) return false;

    if (tail != 0x55) {
        return false;
    }

    return true;
}
