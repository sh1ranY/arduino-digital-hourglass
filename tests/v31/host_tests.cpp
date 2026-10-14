#include "Arduino.h"
#include <cstring>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>

uint32_t fakeNow = 0;
uint8_t fakePins[32] = {};
uint8_t fakeModes[32] = {};
FakeSerial Serial;
static std::vector<uint8_t> spiBits;
static uint8_t maxRegs[2][16] = {};
static unsigned int malformedTransactions = 0;
unsigned long millis() { return fakeNow; }
void pinMode(uint8_t p, uint8_t mode) { fakeModes[p] = mode; }
int digitalRead(uint8_t p) { return fakePins[p]; }
void delayMicroseconds(unsigned int) {}
void digitalWrite(uint8_t p, uint8_t value) {
 const uint8_t old = fakePins[p];
 fakePins[p] = value;
 if (p == 10 && value == LOW && old != LOW) spiBits.clear();
 if (p == 13 && value == HIGH && old != HIGH && fakePins[10] == LOW)
  spiBits.push_back(fakePins[11]);
 if (p == 10 && value == HIGH && old == LOW) {
  if (spiBits.size() != 32) { ++malformedTransactions; return; }
  uint8_t b[4] = {};
  for (unsigned int i = 0; i < 32; ++i) b[i / 8] = uint8_t((b[i / 8] << 1) | spiBits[i]);
  maxRegs[1][b[0] & 15] = b[1];
  maxRegs[0][b[2] & 15] = b[3];
 }
}

// Execute the actual candidate, including its input debounce, display encoder,
// state transitions and grain storage. No duplicate timer/grain algorithm.
#include "../../firmware/Hourglass_v3/Hourglass_v3.ino"

static void check(bool ok, const std::string& what) {
 if (!ok) throw std::runtime_error(what);
}
static unsigned int litCount() {
 unsigned int n = 0;
 for (unsigned int m = 0; m < 2; ++m)
  for (unsigned int r = 0; r < 8; ++r)
   for (unsigned int bit = 0; bit < 8; ++bit) n += ((frame[m][r] >> bit) & 1);
 return n;
}
static void assertConserved() {
 check(grainCount(0) + grainCount(1) == 36, "grain storage must total 36");
 check(Serial.str().find("ERROR:") == std::string::npos, "candidate emitted transfer ERROR");
}
static void assertDisplayLatched() {
 check(malformedTransactions == 0, "MAX7219 transaction must contain 32 bits");
 for (unsigned int m = 0; m < 2; ++m)
  for (unsigned int r = 0; r < 8; ++r)
   check(maxRegs[m][r + 1] == frame[m][r], "latched MAX7219 row must match candidate frame");
}
static void tick(uint32_t ms) {
 fakeNow += ms; loop(); assertConserved(); assertDisplayLatched();
}
static void runFor(uint32_t ms, uint32_t step = 10) {
 while (ms) { const uint32_t d = ms < step ? ms : step; tick(d); ms -= d; }
}
static void raw(uint8_t pin, uint8_t value) { fakePins[pin] = value; tick(0); }
static void press(uint8_t pin) { raw(pin, LOW); runFor(31); }
static void release(uint8_t pin) { raw(pin, HIGH); runFor(31); }
static void boot(bool high = true, uint32_t at = 0) {
 fakeNow = at; std::memset(fakePins, HIGH, sizeof(fakePins));
 std::memset(fakeModes, 0, sizeof(fakeModes));
 std::memset(maxRegs, 0, sizeof(maxRegs)); spiBits.clear(); malformedTransactions = 0; Serial.clear();
 fakePins[PIN_TILT] = high ? HIGH : LOW;
 tilt = DebouncedInput(PIN_TILT, TILT_DEBOUNCE_MS);
 startPause = DebouncedInput(PIN_START_PAUSE, BUTTON_DEBOUNCE_MS);
 resetButton = DebouncedInput(PIN_RESET, BUTTON_DEBOUNCE_MS);
 lastStatusMs = 0; rngState = 0x281036UL;
 setup(); runFor(301);
 check(state == RunState::READY && tilt.ready(), "boot must settle READY");
 for (uint8_t p : {PIN_TILT, PIN_START_PAUSE, PIN_RESET})
  check(fakeModes[p] == INPUT_PULLUP, "all contacts must use INPUT_PULLUP");
}
static void start() { press(PIN_START_PAUSE); check(state == RunState::RUNNING, "B1 must start"); release(PIN_START_PAUSE); }
static void pause() { press(PIN_START_PAUSE); check(state == RunState::PAUSED, "B1 must pause"); release(PIN_START_PAUSE); }

static void testFramesAndCompletion() {
 for (bool high : {true, false}) {
  boot(high);
  check(litCount() == 40, "READY must show 36 grains plus four status pixels");
  writeBoth(1, 0x12, 0x34);
  check(maxRegs[0][1] == 0x12 && maxRegs[1][1] == 0x34, "serial chain near/far values swapped");
  renderGrains(); assertDisplayLatched();
  start();
  const uint32_t elapsedBefore = elapsedMs;
  runFor(5000);
  check(elapsedMs == elapsedBefore + 5000, "RUNNING must count elapsed time");
  check(transferred == (uint64_t(elapsedMs) * 36) / 60000, "transfer progress must track independent timer");
  // Check both running heartbeat phases without advancing the timer/state.
  uint32_t savedNow = fakeNow;
  fakeNow = 100; renderGrains(); check(litCount() == 37, "heartbeat ON must add one LED");
  fakeNow = 600; renderGrains(); check(litCount() == 36, "heartbeat OFF must display only grains");
  fakeNow = savedNow; renderGrains();
  pause();
  const uint32_t frozenElapsed = elapsedMs;
  bool frozen[2][8][8]; std::memcpy(frozen, grains, sizeof(grains));
  runFor(1400);
  check(elapsedMs == frozenElapsed && std::memcmp(frozen, grains, sizeof(grains)) == 0, "PAUSED must freeze timer and grains");
  check(litCount() == 42, "ordinary settled PAUSED must show six pause pixels plus 36 grains");
  press(PIN_START_PAUSE); check(state == RunState::RUNNING, "B1 must resume"); release(PIN_START_PAUSE);
  runFor(60000 - elapsedMs, 60);
  check(state == RunState::DONE && elapsedMs == 60000, "cycle must complete at 60000 effective milliseconds");
  check(grainCount(upperModule()) == 0 && grainCount(1-upperModule()) == 36, "DONE must put all grains in bottom");
  check(litCount() == 40, "DONE must show four check pixels plus 36 bottom grains");
  runFor(1200);
  check(state == RunState::DONE && elapsedMs == 60000, "DONE must stay completed");
 }
}
static void testPendingFlipReset() {
 boot(); start(); runFor(2300);
 raw(PIN_TILT, LOW); runFor(100);
 press(PIN_RESET);
 check(state == RunState::READY && !tilt.ready() && !orientationHigh, "B2 must take fresh LOW baseline and stay READY");
 const size_t afterReset = Serial.str().size();
 release(PIN_RESET); runFor(500);
 check(state == RunState::READY && tilt.ready() && elapsedMs == 0 && upperModule() == 1, "old delayed flip must not undo reset");
 check(Serial.str().substr(afterReset).find("Stable flip") == std::string::npos, "pending orientation must become baseline, not flip event");
 check(grainCount(1) == 36 && grainCount(0) == 0, "reset baseline must select correct top");
 start(); raw(PIN_TILT, HIGH); runFor(301);
 check(state == RunState::RUNNING && orientationHigh && elapsedMs <= 1, "later deliberate flip must still work");
}
static void testSameLoopPriorities() {
 boot(); start(); runFor(1000);
 raw(PIN_TILT, LOW); runFor(269);
 raw(PIN_RESET, LOW); runFor(31);
 check(state == RunState::READY && !tilt.ready() && elapsedMs == 0, "B2 must win over flip accepted in same loop");
 release(PIN_RESET); runFor(400);
 check(state == RunState::READY, "fresh baseline must retain READY");
 release(PIN_START_PAUSE);
 fakePins[PIN_START_PAUSE] = LOW; fakePins[PIN_RESET] = LOW; tick(0); runFor(31);
 check(state == RunState::READY && elapsedMs == 0, "simultaneous B1+B2 must reset, not start");
}
static void testPausedFlip() {
 boot(); start(); runFor(5100); pause();
 raw(PIN_TILT, LOW); runFor(300);
 check(state == RunState::PAUSED && elapsedMs == 0 && upperModule() == 1, "paused flip must replenish opposite chamber and remain PAUSED");
 check(grainCount(1) == 36 && grainCount(0) == 0, "paused flip replenishment counts");
 check(litCount() == 6, "first paused-flip flash must hide sand but keep pause icon");
 runFor(120); check(litCount() == 42, "sand must return after first flash");
 runFor(180); check(litCount() == 6, "second paused-flip flash must hide sand");
 runFor(120); check(litCount() == 42, "sand must return after second flash");
 runFor(240); check(state == RunState::PAUSED && elapsedMs == 0 && litCount() == 42, "paused flip flash must end without changing grain storage");
 press(PIN_START_PAUSE); check(state == RunState::RUNNING && elapsedMs <= 1, "B1 must resume flipped paused cycle");
 const uint32_t resumedElapsed = elapsedMs;
 runFor(100); check(elapsedMs == resumedElapsed + 100, "resume must begin new clock from zero");
}
static void testEarlyPauseHasNoFlipFlash() {
 boot(); start(); runFor(100); pause();
 const uint32_t untilSecondWindow = uint32_t(lastRestartMs + 300UL - fakeNow);
 runFor(untilSecondWindow);
 renderGrains();
 check(litCount() == 42, "B1-only pause within 600ms incorrectly flashes/hides sand although no flip occurred");
}
int main() {
 unsigned int failures = 0;
 const std::vector<std::pair<const char*, std::function<void()>>> cases = {
  {"four states, heartbeat, MAX chain and 60s completion in both orientations", testFramesAndCompletion},
  {"reset cancels pending tilt without disabling future flips", testPendingFlipReset},
  {"same-loop reset/flip and B1/B2 priorities", testSameLoopPriorities},
  {"paused flip, two flashes and resume", testPausedFlip},
  {"ordinary early B1 pause does not impersonate a flip", testEarlyPauseHasNoFlipFlash}
 };
 for (const auto& c : cases) {
  try { c.second(); std::cout << "PASS: " << c.first << '\n'; }
  catch (const std::exception& e) { ++failures; std::cout << "FAIL: " << c.first << ": " << e.what() << '\n'; }
 }
 std::cout << "RESULT " << cases.size()-failures << '/' << cases.size() << " scenario groups passed\n";
 return failures ? 1 : 0;
}
