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
struct AudioEvent { uint32_t at; bool on; uint8_t pin; unsigned int frequency; };
static std::vector<AudioEvent> audioEvents;
static bool fakeSoundActive = false;
static unsigned int toneCalls = 0;
static unsigned int noToneCalls = 0;
static void recordSound(bool on, uint8_t pin, unsigned int frequency) {
 if (on == fakeSoundActive) return;
 fakeSoundActive = on;
 audioEvents.push_back({fakeNow, on, pin, frequency});
}

// Execute the actual candidate, including its input debounce, display encoder,
// state transitions and grain storage. No duplicate timer/grain algorithm.
#include "../../firmware/Hourglass_v32/Hourglass_v32.ino"

void tone(uint8_t p, unsigned int frequency, unsigned long duration) {
 if (duration != 0) throw std::runtime_error("firmware must schedule tone duration itself");
 ++toneCalls;
 recordSound(true, p, frequency);
}
void noTone(uint8_t p) { ++noToneCalls; recordSound(false, p, 0); }

unsigned long millis() { return fakeNow; }
void pinMode(uint8_t p, uint8_t mode) { fakeModes[p] = mode; }
int digitalRead(uint8_t p) { return fakePins[p]; }
void delayMicroseconds(unsigned int) {}
void digitalWrite(uint8_t p, uint8_t value) {
 const uint8_t old = fakePins[p];
 fakePins[p] = value;
#if HOURGLASS_ACTIVE_BUZZER
 if (p == 5) recordSound(value == HIGH, p, 0);
#endif

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
 audioEvents.clear(); fakeSoundActive = false; toneCalls = 0; noToneCalls = 0;
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

static unsigned int soundOnCount() {
 unsigned int n = 0;
 for (const auto& e : audioEvents) if (e.on) ++n;
 return n;
}
static std::vector<AudioEvent> soundOns() {
 std::vector<AudioEvent> result;
 for (const auto& e : audioEvents) if (e.on) result.push_back(e);
 return result;
}
static void checkAudioBackend() {
 for (const auto& e : audioEvents) {
  check(e.pin == PIN_BUZZER && PIN_BUZZER == 5, "audio must only control D5");
#if HOURGLASS_ACTIVE_BUZZER
  check(e.frequency == 0, "active-buzzer mode must use levels, not a tone frequency");
#else
  if (e.on) check(e.frequency == 2000, "passive start/end frequency must be 2000 Hz");
#endif
 }
#if HOURGLASS_ACTIVE_BUZZER
 check(toneCalls == 0 && noToneCalls == 0, "active mode must avoid tone/noTone");
#else
 check(toneCalls == soundOnCount(), "passive tone must not be retriggered every loop");
#endif
 check(fakeModes[PIN_BUZZER] == OUTPUT, "D5 must be OUTPUT for the driver signal");
}
static void completeCycle() {
 runFor(60000 - elapsedMs, 10);
 check(state == RunState::DONE && elapsedMs == 60000, "completion must retain 60 s timing");
}
static void testStartCueBoundary() {
 boot();
 check(soundOnCount() == 0 && !fakeSoundActive, "boot and settled baseline must be silent");
 press(PIN_START_PAUSE);
 check(state == RunState::RUNNING && soundOnCount() == 1 && fakeSoundActive, "new B1 cycle must sound once");
 const uint32_t t = soundOns()[0].at;
 const uint32_t age = uint32_t(fakeNow - t);
 check(age < 120, "start cue first observation must be within its short pulse");
 runFor(119 - age);
 check(fakeSoundActive, "start cue must remain on through 119 ms");
 tick(1);
 check(!fakeSoundActive && playingBuzzerCue == BuzzerCue::NONE, "start cue must end at 120 ms");
 check(uint32_t(audioEvents.back().at - t) == 120, "start cue duration must be exactly 120 host ms");
 runFor(2000);
 check(soundOnCount() == 1, "held B1 / later loops must not repeat start cue");
 checkAudioBackend();
}
static void testPauseResumeNoNewCue() {
 boot(); start(); pause();
 const uint32_t frozen = elapsedMs;
 runFor(500);
 check(elapsedMs == frozen && state == RunState::PAUSED, "sound scheduling must not unpause timing");
 press(PIN_START_PAUSE); release(PIN_START_PAUSE); runFor(500);
 check(state == RunState::RUNNING && soundOnCount() == 1, "pause/resume must not create a new-cycle cue");
 checkAudioBackend();
}
static void testThreeFinishPulses() {
 boot(); start(); completeCycle();
 const uint32_t t = soundOns().back().at;
 check(playingBuzzerCue == BuzzerCue::FINISH && fakeSoundActive, "DONE entry must begin completion cue");
 runFor(560);
 const auto ons = soundOns();
 check(ons.size() == 4, "expected one start and exactly three completion pulses");
 for (unsigned int i = 0; i < 3; ++i)
  check(uint32_t(ons[i+1].at - t) == i * 220, "completion pulse onset spacing must be 220 ms");
 std::vector<AudioEvent> tail;
 for (const auto& e : audioEvents) if (uint32_t(e.at - t) <= 560) tail.push_back(e);
 check(tail.size() == 6, "completion sequence must have three on/off pairs");
 const uint32_t offsets[] = {0,120,220,340,440,560};
 for (unsigned int i = 0; i < 6; ++i) {
  check(uint32_t(tail[i].at - t) == offsets[i], "completion pulse duration/gap boundary incorrect");
  check(tail[i].on == ((i % 2) == 0), "completion cue must alternate sound and silence");
 }
 check(!fakeSoundActive && playingBuzzerCue == BuzzerCue::NONE, "finish cue must stop after its last pulse");
 runFor(2500);
 check(soundOnCount() == 4 && state == RunState::DONE, "DONE must not replay completion sound");
 check(grainCount(upperModule()) == 0 && grainCount(1-upperModule()) == 36, "audio must preserve DONE grain placement");
 checkAudioBackend();
}
static void testResetCancelsCue() {
 for (bool finish : {false,true}) {
  boot(); start();
  if (finish) completeCycle();
  check(fakeSoundActive, "cancellation scenario must start during a pulse");
  press(PIN_RESET);
  check(state == RunState::READY && !fakeSoundActive &&
        playingBuzzerCue == BuzzerCue::NONE && pendingBuzzerCue == BuzzerCue::NONE,
        "B2 must cancel pending/current cue and drive quiet");
  const unsigned int count = soundOnCount();
  release(PIN_RESET); runFor(1000);
  check(soundOnCount() == count && state == RunState::READY, "cancelled cue must not resume after reset");
  checkAudioBackend();
 }
}
static void testFlipReplacesFinishCue() {
 boot(); start(); completeCycle();
 const uint32_t completedAt = fakeNow;
 runFor(150); raw(PIN_TILT, LOW); runFor(300);
 check(state == RunState::RUNNING && elapsedMs == 0 && upperModule() == 1,
       "stable flip must keep original reset/auto-start operation");
 check(playingBuzzerCue == BuzzerCue::START && fakeSoundActive,
       "new flip must replace the completion sequence with a short start cue");
 const uint32_t restartedAt = soundOns().back().at;
 check(uint32_t(restartedAt - completedAt) == 450, "flip acceptance must preserve 300 ms debounce");
 runFor(110);
 check(fakeSoundActive, "old finish deadline must not stop the replacement start cue");
 runFor(10);
 check(!fakeSoundActive && playingBuzzerCue == BuzzerCue::NONE, "replacement start must stop at its own 120 ms deadline");
 const unsigned int n = soundOnCount();
 runFor(1000);
 check(soundOnCount() == n, "replaced finish must not resume later");
 checkAudioBackend();
}
static void testPausedFlipNoStartCue() {
 boot(); start(); runFor(250); pause();
 const unsigned int n = soundOnCount();
 raw(PIN_TILT, LOW); runFor(300);
 check(state == RunState::PAUSED && elapsedMs == 0 && upperModule() == 1,
       "paused flip must remain paused with new top chamber");
 check(soundOnCount() == n && !fakeSoundActive, "paused flip must not sound a new running start");
 press(PIN_START_PAUSE); release(PIN_START_PAUSE);
 check(state == RunState::RUNNING && soundOnCount() == n, "resume after paused flip must remain a resume, not a start cue");
 checkAudioBackend();
}
static void testSameLoopAudioPriorities() {
 // B1+B2 accepted together: no transient start sound.
 boot(); raw(PIN_START_PAUSE, LOW); raw(PIN_RESET, LOW); runFor(30);
 check(state == RunState::READY && soundOnCount() == 0, "B2 must suppress same-loop B1 cue");
 // Tilt and B2 accepted together: no transient flip-start sound.
 boot(); start(); runFor(1000);
 const unsigned int n = soundOnCount();
 raw(PIN_TILT, LOW); runFor(270); raw(PIN_RESET, LOW); runFor(30);
 check(state == RunState::READY && soundOnCount() == n && !fakeSoundActive,
       "B2 must suppress same-loop stable-flip start cue");
 // DONE and B2 accepted together: no transient finish sound.
 boot(); start(); runFor(60000 - elapsedMs - 30);
 const unsigned int beforeDoneReset = soundOnCount();
 raw(PIN_RESET, LOW); runFor(30);
 check(state == RunState::READY && soundOnCount() == beforeDoneReset && !fakeSoundActive,
       "B2 must suppress same-loop DONE cue");
 checkAudioBackend();
}
static void testDoneRestartFromB1() {
 boot(); start(); completeCycle();
 press(PIN_START_PAUSE);
 check(state == RunState::RUNNING && playingBuzzerCue == BuzzerCue::START &&
       elapsedMs <= 1, "B1 at DONE must start a new cycle and replace remaining finish cue");
 checkAudioBackend();
}
static void testSoundAndTimerWrap() {
 boot(true, uint32_t(0xFFFFFFFFUL - 380UL));
 press(PIN_START_PAUSE);
 const uint32_t t = soundOns()[0].at;
 runFor(120 - uint32_t(fakeNow - t));
 check(!fakeSoundActive && uint32_t(audioEvents.back().at - t) == 120,
       "start sound spanning millis wrap must end after 120 ms");
 release(PIN_START_PAUSE); completeCycle(); runFor(560);
 check(state == RunState::DONE && elapsedMs == 60000 && soundOnCount() == 4,
       "active 60 s timer and grain transfer must survive millis wrap");
 // Choose boot time so DONE begins 50 ms before wrap, exercising all finish edges.
 boot(true, uint32_t(0xFFFFFFFFUL - 50UL - 60000UL - 331UL));
 start(); completeCycle();
 const uint32_t completedAt = soundOns().back().at;
 runFor(560);
 const auto ons = soundOns();
 check(ons.size() == 4 && uint32_t(ons[2].at - completedAt) == 220 &&
       uint32_t(ons[3].at - completedAt) == 440 &&
       uint32_t(audioEvents.back().at - completedAt) == 560 && !fakeSoundActive,
       "three completion pulses spanning wrap must retain their sequence and stop");
 checkAudioBackend();
}
static void testCalibrationSilent() {
 fakeNow = 0; std::memset(fakePins, HIGH, sizeof(fakePins));
 std::memset(fakeModes, 0, sizeof(fakeModes)); std::memset(maxRegs,0,sizeof(maxRegs));
 spiBits.clear(); malformedTransactions = 0; Serial.clear();
 audioEvents.clear(); fakeSoundActive = false; toneCalls = 0; noToneCalls = 0;
 tilt = DebouncedInput(PIN_TILT,TILT_DEBOUNCE_MS);
 startPause = DebouncedInput(PIN_START_PAUSE,BUTTON_DEBOUNCE_MS);
 resetButton = DebouncedInput(PIN_RESET,BUTTON_DEBOUNCE_MS);
 setup(); runFor(1000);
 raw(PIN_START_PAUSE,LOW); runFor(100); raw(PIN_TILT,LOW); runFor(400);
 raw(PIN_RESET,LOW); runFor(70000);
 check(state == RunState::READY && elapsedMs == 0, "calibration must still disable operating state machine");
 check(soundOnCount() == 0 && toneCalls == 0 && !fakeSoundActive && fakePins[5] == LOW,
       "calibration must remain silent despite buttons, tilt and elapsed wall time");
 check(litCount() == 6, "calibration must retain three corner pixels per display");
 checkAudioBackend();
}
int main() {
 unsigned int failures = 0;
 std::vector<std::pair<const char*, std::function<void()>>> cases;
 if (HOURGLASS_CORNER_CALIBRATION) {
  cases = {{"calibration stays silent with all contacts and long wall time", testCalibrationSilent}};
 } else {
  cases = {
   {"existing: states, MAX chain and 60s completion in both orientations", testFramesAndCompletion},
   {"existing: reset cancels pending tilt, permits later flips", testPendingFlipReset},
   {"existing: reset/flip and B1/B2 priorities", testSameLoopPriorities},
   {"existing: paused flip, flashes and resume", testPausedFlip},
   {"existing: ordinary early pause is not flip feedback", testEarlyPauseHasNoFlipFlash},
   {"sound: silent boot, 120ms start, no held-repeat", testStartCueBoundary},
   {"sound: pause/resume does not start a new cue", testPauseResumeNoNewCue},
   {"sound: 3 x 120ms finish / 100ms gaps / no DONE repeat", testThreeFinishPulses},
   {"sound: B2 cancels start and completion cues", testResetCancelsCue},
   {"sound: flip replaces completion with new 120ms start", testFlipReplacesFinishCue},
   {"sound: paused flip and resume do not sound a start", testPausedFlipNoStartCue},
   {"sound: same-loop B2 wins over B1, flip and DONE", testSameLoopAudioPriorities},
   {"sound: B1 starts again from DONE", testDoneRestartFromB1},
   {"sound: start, finish and 60s timer survive millis wrap", testSoundAndTimerWrap}
  };
 }
 for (const auto& c : cases) {
  try { c.second(); std::cout << "PASS: " << c.first << '\n'; }
  catch (const std::exception& e) { ++failures; std::cout << "FAIL: " << c.first << ": " << e.what() << '\n'; }
 }
 std::cout << "RESULT " << cases.size()-failures << '/' << cases.size() << " scenario groups passed\n";
 return failures ? 1 : 0;
}
