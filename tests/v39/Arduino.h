#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <stdexcept>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define INPUT_PULLUP 2
#define OUTPUT 1
#define HEX 16
#define DEC 10
static const uint8_t SDA = 18, SCL = 19;
#define F(value) value
using byte = uint8_t;
using boolean = bool;
unsigned long millis();
unsigned long micros();
void pinMode(uint8_t, uint8_t);
int digitalRead(uint8_t);
void digitalWrite(uint8_t, uint8_t);
void delay(unsigned long);
void delayMicroseconds(unsigned int);
void tone(uint8_t, unsigned int, unsigned long = 0);
void noTone(uint8_t);
void fakeSerialBlock(const std::string&);
class FakeSerial {
 public:
  std::ostringstream out;
  int capacity = 4096;
  bool connected = true;
  size_t bytesWritten = 0;
  void begin(unsigned long) {}
  int availableForWrite() const { return capacity; }
  size_t write(uint8_t value) { return write(&value, 1); }
  size_t write(const uint8_t* p, size_t count) {
#if defined(ARDUINO_UNOR4_WIFI) && defined(NO_USB)
    if (count > 8) throw std::runtime_error("bridge UART chunk exceeded eight bytes");
#else
    if (count > size_t(std::max(capacity, 0))) throw std::runtime_error("serial write exceeds free FIFO");
#endif
    out.write(reinterpret_cast<const char*>(p), count);
    bytesWritten += count;
    return count;
  }
  void print(uint8_t v) { out << unsigned(v); }
  void print(int8_t v) { out << int(v); }
  void print(const char* v) { fakeSerialBlock(v); out << v; }
  void print(unsigned int v, int base) {
    if (base == HEX) out << std::hex << v << std::dec;
    else out << v;
  }
  void print(int v, int base) { print(static_cast<unsigned int>(v), base); }
  void print(float v, int precision) { out << std::fixed << std::setprecision(precision) << v; }
  void print(double v, int precision) { out << std::fixed << std::setprecision(precision) << v; }
  template<class T> void print(T v) { out << v; }
  template<class T> void println(T v) { print(v); out << '\n'; }
  template<class T> void println(T v, int format) { print(v, format); out << '\n'; }
  void println() { out << '\n'; }
  explicit operator bool() const { return connected; }
  std::string str() const { return out.str(); }
};
extern FakeSerial Serial;
