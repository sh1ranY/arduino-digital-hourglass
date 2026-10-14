#pragma once
#include <cstdint>
#include <sstream>
#include <string>
#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define OUTPUT 1
extern uint32_t fakeNow;
extern uint8_t fakePins[32];
extern uint8_t fakeModes[32];
unsigned long millis();
void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t value);
void delayMicroseconds(unsigned int value);
class FakeSerial {
 public:
  std::ostringstream out;
  void begin(unsigned long) {}
  void print(uint8_t v) { out << unsigned(v); }
  template<class T> void print(T v) { out << v; }
  template<class T> void println(T v) { print(v); out << '\n'; }
  std::string str() const { return out.str(); }
  void clear() { out.str(""); out.clear(); }
};
extern FakeSerial Serial;

void tone(uint8_t pin, unsigned int frequency, unsigned long duration = 0);
void noTone(uint8_t pin);
