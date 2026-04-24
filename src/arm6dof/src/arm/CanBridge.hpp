#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Komunikacja z adapterem USB-CAN (format ramki: AA | E0|len | id[4] LE | data | 55).
class CanBridge {
public:
    CanBridge();
    ~CanBridge();

    // Otwiera port szeregowy i ustawia baudrate 2 Mbd.
    bool open(const std::string& port);

    // Wysyla ramke CAN: 4-bajtowe ID (little-endian) + payload.
    void send(uint32_t id, const std::vector<uint8_t>& data);

    // Czeka na jedna kompletna ramke CAN. Zwraca false przy timeoucie lub bledzie.
    bool receive(uint32_t& id, std::vector<uint8_t>& data);

private:
    int serial_fd;

    bool readByte(uint8_t& b);
    void close();
};
