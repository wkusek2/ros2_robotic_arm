#pragma once

// ============================================================
// CanBridge — serial communication layer for Waveshare USB-CAN-A
// ============================================================
// The adapter connects via USB-CDC serial at 2 Mbps baudrate.
// Frame format (standard 11-bit, MIT mode):
//   Sending:   AA | (C0 | len) | id_low | id_high | data[len] | 55
//   Receiving: AA | type_byte  | id[2B LE] | data[len] | 55
// ============================================================

#include <cstdint>
#include <string>
#include <vector>

class CanBridge {
public:
    CanBridge();
    ~CanBridge();

    bool open(const std::string& port);

    // Standard 11-bit frame — MIT mode (control commands, enable/disable).
    void sendStd(uint16_t id, const std::vector<uint8_t>& data);

    void sendExt(uint32_t id, const std::vector<uint8_t>& data);

    // Blocks until data arrives (or timeout_ms elapses).
    bool waitForData(int timeout_ms);

    // Reads one complete CAN frame. Returns false on timeout or sync error.
    bool receive(uint32_t& id, std::vector<uint8_t>& data);

private:
    int serial_fd;

    bool readByte(uint8_t& b);
    void close();
};
