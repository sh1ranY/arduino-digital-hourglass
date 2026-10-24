#include "Arduino.h"
#include "Wire.h"
#include <cstring>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <map>

uint32_t fakeNow = 0;
uint8_t fakePins[32] = {};
uint8_t fakeModes[32] = {};
FakeSerial Serial;
TwoWire Wire;
static std::vector<uint8_t> shiftBits;
static uint8_t maxRegs[2][16] = {};
static std::vector<uint8_t> clockShiftBits;
static uint8_t clockRegs[4][16] = {};
static unsigned malformedClockTransactions = 0;
static unsigned clockDisplayLatches = 0;
static unsigned malformedTransactions = 0;
static unsigned displayLatches = 0;
static unsigned toneCalls = 0;
static unsigned noToneCalls = 0;
static unsigned long blockingDelayMs = 0;
static unsigned long gpioDelayUs = 0;
struct AudioEdge { uint32_t at; bool on; };
static std::vector<AudioEdge> audioEdges;
static bool buzzerLevel = false;
static uint32_t serialReportLatencyMs = 0;
static unsigned serialLatencyEvents = 0;

// The complete candidate is the system under test. All state transitions,
// physical input debounce, IMU register parsing and drawing execute unchanged.
#ifndef HOURGLASS_TEST_SOURCE
#include "../../firmware/Hourglass_v39/Hourglass_v39.ino"
#else
#include HOURGLASS_TEST_SOURCE
#endif

unsigned long millis() { return fakeNow; }
void fakeSerialBlock(const std::string& text) {
  if(serialReportLatencyMs && (text=="Started" || text=="Completed")) {
    fakeNow+=serialReportLatencyMs;
    ++serialLatencyEvents;
  }
}
unsigned long micros() { return uint32_t(fakeNow * 1000UL); }
static bool externalBusLow[2] = {false, false};
static unsigned busRiseCount = 0, releaseSdaAfterClocks = 0, unsafeBusHigh = 0;
void pinMode(uint8_t p, uint8_t mode) {
  const uint8_t old = fakeModes[p];
  fakeModes[p] = mode;
  if ((p == SDA || p == SCL) && mode == OUTPUT && fakePins[p] == HIGH) ++unsafeBusHigh;
  if (p == SCL && old == OUTPUT && mode == INPUT && !externalBusLow[1]) {
    ++busRiseCount;
    if (releaseSdaAfterClocks && busRiseCount >= releaseSdaAfterClocks) externalBusLow[0] = false;
  }
}
int digitalRead(uint8_t p) {
  if (p == SDA || p == SCL) {
    if (externalBusLow[p == SDA ? 0 : 1]) return LOW;
    return fakeModes[p] == OUTPUT ? fakePins[p] : HIGH;
  }
  return fakePins[p];
}
void delay(unsigned long ms) { blockingDelayMs += ms; fakeNow += uint32_t(ms); }
void delayMicroseconds(unsigned int us) { gpioDelayUs += us; }
void tone(uint8_t, unsigned int, unsigned long) { ++toneCalls; }
void noTone(uint8_t) { ++noToneCalls; }
void digitalWrite(uint8_t p, uint8_t value) {
  if ((p == SDA || p == SCL) && value == HIGH) ++unsafeBusHigh;
  const uint8_t old = fakePins[p]; fakePins[p] = value;
  if (p == 5 && (value == HIGH) != buzzerLevel) {
    buzzerLevel = value == HIGH;
    audioEdges.push_back({fakeNow, buzzerLevel});
  }
  if (p == 10 && value == LOW && old != LOW) shiftBits.clear();
  if (p == 13 && value == HIGH && old != HIGH && fakePins[10] == LOW)
    shiftBits.push_back(fakePins[11]);
  if (p == 10 && value == HIGH && old == LOW) {
    if (shiftBits.size() != 32) { ++malformedTransactions; return; }
    uint8_t b[4] = {};
    for (unsigned i = 0; i < 32; ++i)
      b[i / 8] = uint8_t((b[i / 8] << 1) | shiftBits[i]);
    maxRegs[1][b[0] & 15] = b[1];
    maxRegs[0][b[2] & 15] = b[3];
    ++displayLatches;
  }
  if (p == 9 && value == LOW && old != LOW) clockShiftBits.clear();
  if (p == 8 && value == HIGH && old != HIGH && fakePins[9] == LOW)
    clockShiftBits.push_back(fakePins[7]);
  if (p == 9 && value == HIGH && old == LOW) {
    if (clockShiftBits.size() != 64) { ++malformedClockTransactions; return; }
    uint8_t b[8] = {};
    for (unsigned i = 0; i < 64; ++i)
      b[i / 8] = uint8_t((b[i / 8] << 1) | clockShiftBits[i]);
    for (unsigned i = 0; i < 4; ++i)
      clockRegs[3-i][b[2*i] & 15] = b[2*i+1];
    ++clockDisplayLatches;
  }
}

static void require(bool ok, const std::string& reason) {
  if (!ok) throw std::runtime_error(reason);
}
static unsigned onCount() {
  unsigned n = 0; for (const auto& e : audioEdges) n += e.on; return n;
}
static unsigned pixels(unsigned module) {
  unsigned n = 0;
  for (unsigned r = 1; r <= 8; ++r)
    for (unsigned bit = 0; bit < 8; ++bit) n += (maxRegs[module][r] >> bit) & 1;
  return n;
}
static void invariants() {
  require(grainCount(0) + grainCount(1) == 36, "grain storage lost or created particles");
  require(malformedTransactions == 0, "MAX7219 latch had other than 32 clock bits");
  for (unsigned m = 0; m < 2; ++m) {
    require(maxRegs[m][0x0f] == 0, "MAX7219 display-test unexpectedly ON");
    require(maxRegs[m][0x0a] == 0, "MAX7219 brightness is not minimum");
    for (unsigned r = 0; r < 8; ++r)
      require(maxRegs[m][r+1] == frame[m][r], "SPI-latched rows differ from candidate frame buffer");
  }
  require(malformedClockTransactions == 0, "clock MAX7219 latch had other than 64 clock bits");
  for (unsigned m = 0; m < 4; ++m) {
    require(clockRegs[m][0x0f] == 0, "clock MAX7219 display-test unexpectedly ON");
    require(clockRegs[m][0x0a] == 0, "clock MAX7219 brightness is not minimum");
  }
  require(Wire.protocolErrors == 0, "invalid/over-read I2C transaction");
  require(toneCalls == 0 && noToneCalls == 0, "real active-buzzer build must use D5 levels, not tone/noTone");
}
static void tick(uint32_t dt = 1) { fakeNow += dt; loop(); invariants(); }
static void advance(uint32_t ms, uint32_t step = 1) {
  while (ms) { const uint32_t d = std::min(ms, step); tick(d); ms -= d; }
}
static void input(uint8_t pin, uint8_t level) { fakePins[pin] = level; tick(0); }
static void buttonDown(uint8_t pin) { input(pin, LOW); advance(31); }
static void buttonUp(uint8_t pin) { input(pin, HIGH); advance(31); }
static void press(uint8_t pin) { buttonDown(pin); buttonUp(pin); }
static void acceleration(float ax, float ay, float az) { Wire.setAcceleration(ax, ay, az); }
static void boot(float ax = 1, float ay = 0, float az = 0, uint32_t at = 0) {
  fakeNow = at;
  std::memset(fakePins, HIGH, sizeof(fakePins));
  acceleration(ax, ay, az);
  setup();
  advance(1500);
  require(state == RunState::READY, "boot must remain READY, not auto-start");
  require(!buzzerLevel && onCount() == 0, "power-on/baseline must stay silent");
  require(fakeModes[3] == INPUT_PULLUP && fakeModes[4] == INPUT_PULLUP,
          "B1 and B2 need internal pull-ups");
  require(fakeModes[5] == OUTPUT, "D5 is not a driver output");
  invariants();
}
static void running() {
  press(3); require(state == RunState::RUNNING, "green button did not start RUNNING");
}
static void paused() {
  press(3); require(state == RunState::PAUSED, "green button did not manually pause");
}
static bool wrote(uint8_t reg, uint8_t value) {
  for (const auto& t : Wire.transactions)
    if (!t.result && t.bytes.size() == 2 && t.bytes[0] == reg && t.bytes[1] == value) return true;
  return false;
}
static uint32_t finishCycle() {
  const uint32_t remaining = 60000UL - elapsedMs;
  const uint32_t startAt = fakeNow;
  advance(remaining);
  require(state == RunState::DONE && elapsedMs == 60000,
          "cycle must finish at 60000 active ms");
  require(uint32_t(fakeNow - startAt) == remaining, "blocking call changed elapsed wall time");
  require(grainCount(upperModule()) == 0 && grainCount(1-upperModule()) == 36,
          "DONE does not contain all 36 grains in lower chamber");
  return fakeNow;
}
static void exactFinishSequence(uint32_t onset, unsigned expectedTotalOn) {
  advance(560);
  std::vector<AudioEdge> tail;
  for (const auto& e : audioEdges)
    if (uint32_t(e.at - onset) <= 560) tail.push_back(e);
  const uint32_t offsets[] = {0, 120, 220, 340, 440, 560};
  require(tail.size() == 6, "finish must contain 3 complete on/off pairs");
  for (unsigned i = 0; i < 6; ++i) {
    require(uint32_t(tail[i].at - onset) == offsets[i], "finish pulse/gap boundary incorrect");
    require(tail[i].on == (i%2 == 0), "finish edges do not alternate on/off");
  }
  require(onCount() == expectedTotalOn && !buzzerLevel, "incorrect total beep count");
  advance(1500);
  require(onCount() == expectedTotalOn, "DONE repeats completion beeps");
}

static void startup() {
  boot();
  require(imu.ready() && imu.healthy() && !sensorHold, "valid upright IMU did not become ready");
  require(Wire.begun && Wire.readRequests > 10, "candidate did not really read the I2C register model");
  require(wrote(0x6b, 1) && wrote(0x1a, 3) && wrote(0x1b, 0) &&
          wrote(0x1c, 0) && wrote(0x19, 4), "MPU6050 initialization registers incorrect/missing");
  require((Wire.registers[0x6b] & 0x40) == 0, "MPU6050 remains asleep");
  require(imu.ax() == 16384 && imu.ay() == 0 && imu.az() == 0,
          "big-endian signed accelerometer register interpretation incorrect");
  require(grainCount(upperModule()) == 36, "READY did not fill upper module");
  require(pixels(0) + pixels(1) >= 36, "external MAX frames omitted initial grains");
}
static void cycle(bool inverted) {
  boot(inverted ? -1.0f : 1.0f);
  const unsigned top = upperModule(); running();
  const uint32_t e = elapsedMs; const unsigned reads = Wire.readRequests;
  advance(30000-e);
  require(elapsedMs == 30000 && transferred == 18 && grainCount(1-top) == 18,
          "30s active time must transfer exactly 18 of 36 grains");
  require(Wire.readRequests > reads + 500, "IMU servicing blocked during timing");
  const uint32_t onset = finishCycle();
  exactFinishSequence(onset, 4);
  require(blockingDelayMs == 0, "runtime uses blocking millisecond delays");
}
static void startBoundary() {
  boot(); buttonDown(3);
  require(state == RunState::RUNNING && onCount() == 1 && buzzerLevel, "start needs exactly one beep");
  const uint32_t onset = audioEdges.front().at;
  advance(119-uint32_t(fakeNow-onset)); require(buzzerLevel, "start ended before 120ms");
  tick(1); require(!buzzerLevel && uint32_t(audioEdges.back().at-onset) == 120,
                    "start beep must stop at 120ms");
  const uint32_t countElapsed = elapsedMs;
  advance(2000); require(onCount() == 1, "holding B1 repeats start");
  require(elapsedMs == countElapsed+2000, "beep scheduling blocks active timer");
}
static void manualPause() {
  boot(); running(); advance(5000); paused();
  const uint32_t frozenTime = elapsedMs;
  bool saved[2][8][8]; std::memcpy(saved, grains, sizeof(saved));
  const unsigned n = onCount(); advance(1500);
  require(elapsedMs == frozenTime && std::memcmp(saved, grains, sizeof(saved)) == 0,
          "manual pause must freeze time and grains");
  press(3); require(state == RunState::RUNNING && onCount() == n, "resume must stay quiet");
  const uint32_t before = elapsedMs; advance(1234);
  require(elapsedMs == before+1234, "resume counts paused wall time or loses running time");
  exactFinishSequence(finishCycle(), n+3);
}
static void flip() {
  boot(); running(); advance(2000);
  const unsigned oldTop = upperModule(); const unsigned oldN = onCount();
  acceleration(-1,0,0); advance(100); acceleration(1,0,0); advance(800);
  require(upperModule() == oldTop && onCount() == oldN, "brief transient was accepted as stable flip");
  acceleration(-1,0,0); advance(900);
  require(upperModule() == 1-oldTop && state == RunState::RUNNING && elapsedMs < 900,
          "stable flip must change top and begin a fresh running cycle");
  require(onCount() == oldN+1, "stable flip needs one new-cycle beep");
  const uint32_t e = elapsedMs; advance(2500);
  require(elapsedMs == e+2500 && onCount() == oldN+1, "held inverted pose repeatedly restarted/beeped");
  acceleration(1,0,0); advance(900);
  require(upperModule() == oldTop && onCount() == oldN+2, "reverse flip not accepted exactly once");
}
static void pausedFlip() {
  boot(); running(); advance(1000); paused();
  const unsigned oldTop = upperModule(), n = onCount();
  acceleration(-1,0,0); advance(1200);
  require(state == RunState::PAUSED && elapsedMs == 0 && upperModule() == 1-oldTop,
          "paused flip must refill opposite chamber but remain manually paused");
  require(onCount() == n && !buzzerLevel, "paused flip must be quiet");
  acceleration(0,1,0); advance(900); acceleration(-1,0,0); advance(1000);
  require(state == RunState::PAUSED && elapsedMs == 0, "autoHold recovery overrides manual pause");
  press(3); require(state == RunState::RUNNING && onCount() == n,
                   "resume after paused flip must not produce start beep");
}
static void horizontalHold(bool faceFlat) {
  boot(); running(); advance(4000); const unsigned n = onCount();
  if(faceFlat) acceleration(0,0,1); else acceleration(0,1,0);
  advance(1200); require(sensorHold, "flat/horizontal orientation must hold");
  const uint32_t frozen = elapsedMs;
  bool saved[2][8][8]; std::memcpy(saved, grains, sizeof(saved));
  advance(2000);
  require(elapsedMs == frozen && std::memcmp(saved,grains,sizeof(saved)) == 0,
          "autoHold must freeze timer and grain animation");
  acceleration(1,0,0); advance(1200);
  require(state == RunState::RUNNING && !sensorHold && elapsedMs > frozen,
          "restored stable upright pose must resume existing running cycle");
  require(onCount() == n && elapsedMs >= frozen && elapsedMs < frozen+1200,
          "hold recovery restarted/beeped or counted all blocked wall time");
}
static void holdHysteresis() {
  boot(); running(); advance(2000); const unsigned n = onCount();
  acceleration(.20f,.979796f,0); advance(1200);
  require(sensorHold, "below hold threshold must engage hold");
  for(unsigned i=0;i<15;i++) {
    const float x = (i%2) ? .28f : .32f;
    acceleration(x,std::sqrt(1-x*x),0); advance(80);
    require(sensorHold, "dead-band noise releases hold/chatter");
  }
  acceleration(.8f,.6f,0); advance(1200);
  require(!sensorHold && state == RunState::RUNNING && onCount() == n,
          "stable above-release pose must quietly resume");
}
static void resetPriorities() {
  boot();
  input(3,LOW); input(4,LOW); advance(31);
  require(state == RunState::READY && elapsedMs == 0 && onCount() == 0,
          "B2 must win simultaneous B1/B2 without transient start");
  buttonUp(3); buttonUp(4); advance(600); running(); advance(1000);
  acceleration(-1,0,0); advance(80); press(4); advance(1400);
  require(state == RunState::READY && elapsedMs == 0 && onCount() == 1,
          "pending flip must not undo reset or beep");
  require(grainCount(upperModule()) == 36, "reset must restore 36 grains in current upper chamber");
}
static void faultRecovery(bool shortRead) {
  boot(); running(); advance(2000); const unsigned n = onCount();
  if (shortRead) Wire.shortReadCount = 5; else Wire.present = false;
  advance(80);
  require(sensorHold, "I2C NACK/short read must immediately engage fail-safe hold");
  const uint32_t frozen = elapsedMs; advance(250);
  require(elapsedMs == frozen, "timer runs with failed IMU");
  Wire.present = true; acceleration(1,0,0); advance(1900);
  require(imu.healthy() && imu.ready() && !sensorHold && state == RunState::RUNNING,
          "valid IMU must recover without power-cycle");
  require(elapsedMs >= frozen && elapsedMs < frozen+1900 && onCount() == n,
          "recovery must preserve current cycle and stay quiet");
}
static void missingStartup(bool badWho) {
  if(badWho) Wire.whoAmI = 0x71; else Wire.present = false;
  boot();
  require(sensorHold && !imu.ready(), "absent/wrong-identity sensor must not become ready");
  press(3); require(state == RunState::READY && onCount() == 0,
                  "B1 must not start with missing/wrong MPU");
  Wire.present = true; Wire.whoAmI = 0x68; acceleration(1,0,0); advance(2200);
  require(imu.ready() && imu.healthy() && !sensorHold && state == RunState::READY,
          "sensor recovery must establish READY baseline without auto-start");
  running(); require(onCount() == 1, "B1 must work after missing-sensor recovery");
}
static void alternateAddress() {
  Wire.deviceAddress = 0x69; boot();
  require(imu.ready() && imu.healthy() && !sensorHold, "0x69 address fallback did not initialize MPU");
  running(); advance(2000); require(elapsedMs >= 2000, "0x69 MPU does not sustain running");
}
static void unsafeAcceleration() {
  boot(); running(); advance(1500);
  acceleration(0,0,0); advance(600); require(sensorHold, "free-fall/zero-g must hold");
  const uint32_t e = elapsedMs; advance(1000); require(elapsedMs == e, "zero-g counts active time");
  acceleration(1.7f,0,0); advance(600); require(sensorHold, "large dynamic acceleration must hold");
  acceleration(1,0,0); advance(1200); require(!sensorHold, "valid static gravity did not recover");
}
static void continuousGravity() {
  boot(); running(); advance(800);
  acceleration(.8f,.6f,0); advance(2000);
  const float leftRow = gravityRow, leftCol = gravityCol;
  require(!sensorHold && state == RunState::RUNNING, "valid left tilt must keep running");
  acceleration(.8f,-.6f,0); advance(2000);
  require(!sensorHold && state == RunState::RUNNING, "valid right tilt must keep running");
  require(std::fabs(gravityRow-leftRow) > .1f || std::fabs(gravityCol-leftCol) > .1f,
          "continuous XY tilt does not change display gravity");
  require(onCount() == 1, "left/right tilts that preserve upper chamber must not restart/beep");
}
static std::pair<double,double> displayedCentroid(unsigned module) {
  double rowSum=0, colSum=0; unsigned count=0;
  for(unsigned r=0;r<8;r++) for(unsigned c=0;c<8;c++)
    if(maxRegs[module][r+1] & (1U << (7-c))) {
      rowSum+=r; colSum+=c; ++count;
    }
  require(count == 36, "DONE lower matrix must display exactly 36 sand pixels");
  return {rowSum/count,colSum/count};
}
static void displayMovesWithGravity() {
  boot(); running(); finishCycle(); advance(800);
  const unsigned lower=1-upperModule(), before=onCount();
  acceleration(.6f,.8f,0); advance(2400);
  const auto a=displayedCentroid(lower);
  acceleration(.6f,-.8f,0); advance(2400);
  const auto b=displayedCentroid(lower);
  // A lower-left versus upper-right pile is visible in SPI-latched pixels,
  // independent of the candidate's gravity variables or proposed movement rule.
  require(a.first > b.first+1.0 && a.second+1.0 < b.second,
          "latched matrix pixels do not move their centre with opposite XY slopes");
  require(state == RunState::DONE && onCount() == before,
          "small tilt at DONE restarted the cycle or repeated completion beeps");
  std::cout << "centroid_left="<<a.first<<'/'<<a.second
    <<" centroid_right="<<b.first<<'/'<<b.second<<'\n';
}
static void resetCancelsFinish() {
  boot(); running(); finishCycle();
  require(buzzerLevel, "finish must have begun before cancellation");
  press(4); const unsigned n=onCount(); advance(1500);
  require(state==RunState::READY && elapsedMs==0 && !buzzerLevel && onCount()==n,
          "B2 reset did not cancel remaining completion pulses");
}
static void resetAtExactCompletion() {
  boot(); running(); advance(60000-elapsedMs-30);
  input(4,LOW); advance(30);
  require(state==RunState::READY && elapsedMs==0 && onCount()==1 && !buzzerLevel,
          "same-loop B2 and DONE emitted finish or failed to reset");
}
static void readFaultCancelsStart() {
  boot(); buttonDown(3); require(buzzerLevel,"start cue needed for fault cancellation");
  Wire.present=false; advance(25);
  require(sensorHold && !buzzerLevel,"read fault must cancel active beep and freeze timing");
  const unsigned n=onCount(); advance(200); Wire.present=true; acceleration(1,0,0); advance(1900);
  require(onCount()==n && !buzzerLevel,"read recovery replayed cancelled start cue");
}
static void faultRecoveryOpposite() {
  boot(); running(); advance(2500); const unsigned oldTop=upperModule(), n=onCount();
  Wire.present=false; advance(80); const uint32_t frozen=elapsedMs;
  advance(600); require(elapsedMs==frozen,"fault before opposite recovery kept running");
  Wire.present=true; acceleration(-1,0,0); advance(2000);
  uint32_t freshStart=0; for(const auto& edge:audioEdges) if(edge.on) freshStart=edge.at;
  require(upperModule()==1-oldTop && state==RunState::RUNNING && elapsedMs==uint32_t(fakeNow-freshStart),
          "opposite recovery did not establish one fresh flipped cycle");
  require(onCount()==n+1,"opposite recovery must produce exactly one new-cycle beep");
  const uint32_t e=elapsedMs; advance(2000);
  require(elapsedMs==e+2000 && onCount()==n+1,"opposite recovery repeatedly restarts");
}
static void initialFaceHold() {
  boot(0,0,1); require(sensorHold && !imu.ready(),"face-flat boot must await valid pose");
  press(3); require(state==RunState::READY && onCount()==0,"flat boot B1 starts without baseline");
  acceleration(1,0,0); advance(1200);
  require(!sensorHold && imu.ready() && state==RunState::READY && elapsedMs==0,
          "first valid pose after face-flat boot must establish quiet READY baseline");
}
static void pauseDuringAutomaticHold() {
  boot(); running(); advance(1700); acceleration(0,1,0); advance(600);
  require(sensorHold,"test requires automatic hold");
  paused(); const unsigned n=onCount(); const uint32_t e=elapsedMs;
  acceleration(1,0,0); advance(1000);
  require(state==RunState::PAUSED && !sensorHold && elapsedMs==e && onCount()==n,
          "valid pose recovery overrides deliberate manual pause");
}
static void noResetFromUntrustedMotion() {
  boot(); running(); advance(1700); const unsigned oldTop=upperModule(),n=onCount();
  acceleration(-1.7f,0,0); advance(1000);
  require(sensorHold && upperModule()==oldTop && onCount()==n,
          "untrusted dynamic vector reset cycle or changed accepted orientation");
  const uint32_t e=elapsedMs; acceleration(1,0,0); advance(1000);
  require(!sensorHold && elapsedMs>=e && elapsedMs<e+1000 && onCount()==n,
          "return after rejected dynamic inversion restarted cycle");
}
static void heldAtPowerUp() {
  // This scenario owns its boot so the actual input baseline sees LOW.
  fakeNow=0; std::memset(fakePins,HIGH,sizeof(fakePins));
  fakePins[3]=LOW; acceleration(1,0,0); setup(); advance(2000);
  require(state==RunState::READY && onCount()==0,"held startup button generated a phantom press");
  buttonUp(3); running(); require(onCount()==1,"fresh press after release did not start once");
}
static void gyroMotionCandidate() {
  boot(); running(); advance(1700); const unsigned top=upperModule(),n=onCount();
  acceleration(-1,0,0);
  Wire.registers[0x43]=uint8_t(uint16_t(int16_t(-200*131))>>8);
  Wire.registers[0x44]=uint8_t(int16_t(-200*131));
  advance(1000);
  require(sensorHold && upperModule()==top && onCount()==n,
          "rapid negative gyro candidate bypassed motion hold/signed decode");
  Wire.registers[0x43]=0; Wire.registers[0x44]=0; advance(1000);
  require(!sensorHold && upperModule()==1-top && onCount()==n+1,
          "stopped rotation did not accept one stable inversion");
}
static void blockedSerialStartWidth() {
  boot(); serialReportLatencyMs=25; buttonDown(3);
  require(serialLatencyEvents==0 && buzzerLevel && onCount()==1,
          "new telemetry used a legacy blocking print or start cue is missing");
  const uint32_t physicalOn=audioEdges.front().at;
  const uint32_t age=uint32_t(fakeNow-physicalOn);
  require(age<119,"unexpected start observation age");
  advance(119-age); require(buzzerLevel,"nonblocking telemetry shortened real D5 pulse");
  tick(1);
  require(!buzzerLevel && uint32_t(audioEdges.back().at-physicalOn)==120,
          "start pulse must last 120ms from actual D5 edge with nonblocking telemetry");
}
static void blockedSerialFinishWidths() {
  boot(); serialReportLatencyMs=25; running();
  advance(60000-elapsedMs);
  require(state==RunState::DONE && elapsedMs==60000 && buzzerLevel && serialLatencyEvents==0,
          "new telemetry used a legacy blocking print or finish cue is missing");
  uint32_t physicalOn=0; for(const auto& edge:audioEdges) if(edge.on)physicalOn=edge.at;
  exactFinishSequence(physicalOn,4);
}
static void wrap() {
  boot(1,0,0,0xfffffb00UL); running();
  advance(60000-elapsedMs); require(state == RunState::DONE, "60s timer fails across millis rollover");
  exactFinishSequence(fakeNow,4);
}
static void finishAcrossWrap() {
  // Start beep begins at boot + 1500ms baseline + 30ms contact debounce.
  // Therefore completion onset is 50ms before rollover in this scenario.
  boot(1,0,0,uint32_t(0xffffffffUL-60000UL-1530UL-50UL));
  running(); finishCycle();
  require(fakeNow>=0xffffff00UL,"completion cue did not begin near rollover");
  exactFinishSequence(fakeNow,4);
}
static void calibration() {
  boot();
  require(pixels(0) == 3 && pixels(1) == 3, "calibration must output 3 corner pixels per matrix");
  press(3); acceleration(-1,0,0); advance(800); press(4); advance(70000);
  require(state == RunState::READY && elapsedMs == 0 && onCount() == 0 && !buzzerLevel,
          "calibration responds to operating inputs or sounds cues");
  require(pixels(0) == 3 && pixels(1) == 3, "calibration corner image changed");
}

int main(int argc, char** argv) {
  const std::map<std::string,std::function<void()>> scenarios = {
    {"startup_registers",startup}, {"start_120ms",startBoundary},
    {"cycle_upright",[]{cycle(false);}}, {"cycle_inverted",[]{cycle(true);}},
    {"manual_pause_resume",manualPause}, {"stable_flip_no_repeat",flip},
    {"paused_flip_quiet",pausedFlip}, {"horizontal_hold_resume",[]{horizontalHold(false);}},
    {"face_flat_hold_resume",[]{horizontalHold(true);}}, {"hold_hysteresis",holdHysteresis},
    {"reset_priority",resetPriorities}, {"i2c_nack_recovery",[]{faultRecovery(false);}},
    {"i2c_short_read_recovery",[]{faultRecovery(true);}},
    {"missing_startup_recovery",[]{missingStartup(false);}},
    {"wrong_whoami_recovery",[]{missingStartup(true);}},
    {"i2c_address_0x69",alternateAddress}, {"invalid_acceleration_hold",unsafeAcceleration},
    {"continuous_xy_gravity",continuousGravity}, {"millis_rollover",wrap},
    {"finish_beeps_millis_rollover",finishAcrossWrap},
    {"display_centroid_tracks_tilt",displayMovesWithGravity},
    {"reset_cancels_finish",resetCancelsFinish}, {"reset_exact_done",resetAtExactCompletion},
    {"read_fault_cancels_start",readFaultCancelsStart},
    {"fault_recovery_opposite_once",faultRecoveryOpposite},
    {"initial_face_flat_hold",initialFaceHold}, {"manual_pause_during_auto_hold",pauseDuringAutomaticHold},
    {"untrusted_inversion_no_reset",noResetFromUntrustedMotion},
    {"button_held_at_power_up",heldAtPowerUp}, {"gyro_motion_candidate",gyroMotionCandidate},
    {"serial_block_start_120ms",blockedSerialStartWidth},
    {"serial_block_finish_3x120ms",blockedSerialFinishWidths},
    {"calibration_silent",calibration}
  };
  if(argc != 2) { for(const auto& s:scenarios) std::cout<<s.first<<'\n'; return 2; }
  try {
    auto it=scenarios.find(argv[1]); require(it!=scenarios.end(),"unknown scenario");
    it->second();
    std::cout << "PASS " << argv[1] << " | I2C_reads=" << Wire.readRequests
      << " MAX_latches=" << displayLatches << " D5_ons=" << onCount()
      << " elapsed_ms=" << elapsedMs << " blocking_delay_ms=" << blockingDelayMs << '\n';
    return 0;
  } catch(const std::exception& e) {
    std::cerr << "FAIL " << argv[1] << ": " << e.what() << " | now=" << fakeNow
      << " elapsed=" << elapsedMs << " state=" << int(state) << " hold=" << sensorHold
      << " IMU=" << imu.healthName() << '\n';
    std::cerr << Serial.str().substr(Serial.str().size()>6000?Serial.str().size()-6000:0);
    return 1;
  }
}
