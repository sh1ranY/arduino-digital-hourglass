#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <deque>
#include <array>
#include <stdexcept>
#include <algorithm>
#include <cmath>

// Register-backed MPU6050 I2C test double. No orientation or timer decisions
// are implemented here: the actual sketch must obtain all sensor information
// through transactions and parse the big-endian signed register values.
struct WireTransaction {
  uint8_t address;
  std::vector<uint8_t> bytes;
  bool stop;
  uint8_t result;
};
class TwoWire {
 public:
  std::array<uint8_t, 128> registers{};
  std::vector<WireTransaction> transactions;
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  bool present = true;
  bool begun = false;
  uint8_t whoAmI = 0x68;
  uint8_t deviceAddress = 0x68;
  uint8_t address = 0;
  uint8_t pointer = 0;
  uint32_t clock = 0;
  unsigned readRequests = 0;
  unsigned protocolErrors = 0;
  int shortReadCount = -1;
  unsigned nackCount = 0;
  unsigned beginCalls = 0, endCalls = 0, resetCommands = 0;
  bool controllerStuck = false, requiresDeviceReset = false;
  bool wasEnded = false;
  uint32_t transactionLatencyMs = 0;
  TwoWire() { resetRegisters(); }
  void resetRegisters() {
    registers.fill(0);
    registers[0x6b] = 0x40;  // power-on sleep bit
    registers[0x75] = whoAmI;
  }
  void end() { begun = false; wasEnded = true; ++endCalls; }
  void begin() { begun = true; ++beginCalls; if (wasEnded) controllerStuck = false; wasEnded = false; }
  void setClock(uint32_t hz) { clock = hz; }
  void setWireTimeout(unsigned long, bool = false) {}
  void setTimeout(unsigned long) {}
  void beginTransmission(uint8_t a) { address = a; tx.clear(); }
  size_t write(uint8_t b) { tx.push_back(b); return 1; }
  size_t write(const uint8_t* p, size_t n) {
    tx.insert(tx.end(), p, p + n); return n;
  }
  uint8_t endTransmission(bool stop = true) {
    if (transactionLatencyMs) delay(transactionLatencyMs);
    if (!begun) throw std::runtime_error("I2C transaction before Wire.begin");
    uint8_t result = 0;
    if (!present || address != deviceAddress) result = 2;
    if (controllerStuck) result = 4;
    // A wedged MPU still identifies but will not serve data until DEVICE_RESET.
    if (requiresDeviceReset && tx.size() == 1 && tx[0] == 0x3b) result = 4;
    if (nackCount) { --nackCount; result = 2; }
    transactions.push_back({address, tx, stop, result});
    if (result || tx.empty()) return result;
    pointer = tx[0];
    if (pointer >= registers.size()) { ++protocolErrors; return 3; }
    for (size_t i = 1; i < tx.size(); ++i) {
      if (pointer == 0x6b && (tx[i] & 0x80)) {
        // Real reset does not remove physical gravity; keep latest sensor outputs.
        const auto samples = registers;
        resetRegisters();
        for (unsigned j = 0x3b; j <= 0x48; ++j) registers[j] = samples[j];
        ++resetCommands; requiresDeviceReset = false;
      }
      else registers[pointer] = tx[i];
      ++pointer;
      if (pointer >= registers.size()) { ++protocolErrors; break; }
    }
    // A register-select transaction must leave its pointer at the selected byte.
    if (tx.size() == 1) pointer = tx[0];
    return 0;
  }
  uint8_t receiveRegisters(uint8_t a, size_t n) {
    ++readRequests; rx.clear();
    if (!present || a != deviceAddress) return 0;
    unsigned actual = n;
    if (shortReadCount >= 0) { actual = std::min<unsigned>(actual, shortReadCount); shortReadCount = -1; }
    for (unsigned i = 0; i < actual; ++i) {
      if (pointer >= registers.size()) { ++protocolErrors; break; }
      rx.push_back(pointer == 0x75 ? whoAmI : registers[pointer]);
      ++pointer;
    }
    return static_cast<uint8_t>(rx.size());
  }
#if defined(ARDUINO_ARCH_AVR)
  // Mirror the AVR Wire overload set that exposed the browser build ambiguity.
  // This compiles the real sketch's AVR branch without silently accepting the
  // R4 size_t/bool call through an unrealistically permissive test interface.
  uint8_t requestFrom(uint8_t a, uint8_t n, uint8_t) { return receiveRegisters(a,n); }
  uint8_t requestFrom(uint8_t a, uint8_t n) { return receiveRegisters(a,n); }
  uint8_t requestFrom(int a, int n, int) { return receiveRegisters(uint8_t(a),uint8_t(n)); }
  uint8_t requestFrom(int a, int n) { return receiveRegisters(uint8_t(a),uint8_t(n)); }
#else
  uint8_t requestFrom(uint8_t a, size_t n, bool = true) { return receiveRegisters(a,n); }
#endif
  int available() { return static_cast<int>(rx.size()); }
  int read() {
    if (rx.empty()) { ++protocolErrors; return -1; }
    const uint8_t b = rx.front(); rx.pop_front(); return b;
  }
  void setAccelerationRaw(int16_t x, int16_t y, int16_t z) {
    const int16_t v[3] = {x, y, z};
    for (unsigned i = 0; i < 3; ++i) {
      registers[0x3b + i * 2] = uint8_t(uint16_t(v[i]) >> 8);
      registers[0x3c + i * 2] = uint8_t(v[i]);
    }
  }
  void setAcceleration(float x, float y, float z) {
    const float scale = 16384.0f / float(1U << ((registers[0x1c] >> 3) & 3));
    setAccelerationRaw(int16_t(std::lround(x * scale)), int16_t(std::lround(y * scale)), int16_t(std::lround(z * scale)));
  }
};
extern TwoWire Wire;
