#pragma once

#include <string>
#include <vector>
#include <cstdint>

class CanBridge {
private:
    int serial_fd;

public:
    CanBridge();
    ~CanBridge();
    bool open(const std::string& interface);
    void send(uint32_t id ,const std::vector<uint8_t>& data);
    bool receive(uint32_t& id, std::vector<uint8_t>& data);
private:
    bool readByte(uint8_t& b);
    void close();
};




