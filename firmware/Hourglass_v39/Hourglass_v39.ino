#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ClockDisplay.h"

// ELEC281 v3.9: UNO R4 WiFi, two MAX7219 modules, GY-87 MPU6050, active buzzer.
// Main Wire bus: A4 SDA / A5 SCL. No third-party sensor/display library.
// D2 and the old two-pin tilt switch are retired.
// v3.9: bounded I2C/controller + MPU recovery and paced optional telemetry.
// Separate four-module 8x32 preset and countdown display remains on D7/D8/D9.
// Preserves calibrated supported sand, gravity holds, and synchronized finish flashes.
// 0 = hourglass; 1 = low-brightness corner calibration (no timer).
#ifndef HOURGLASS_CORNER_CALIBRATION
#define HOURGLASS_CORNER_CALIBRATION 0
#endif
#ifndef AUX_CLOCK_CALIBRATION
#define AUX_CLOCK_CALIBRATION 0
#endif
// Temporary bench fault injection is compiled out of the delivered program.
#ifndef HOURGLASS_RECOVERY_SELF_TEST
#define HOURGLASS_RECOVERY_SELF_TEST 0
#endif
const bool AUX_CLOCK_ENABLED = true;

// Default: confirmed TMB12A05 active buzzer, controlled through an NPN driver.
// 1: HIGH/LOW control; 0: optional passive buzzer with tone().
// D5 is a HIGH-active control signal to an NPN low-side driver, not buzzer power.
#ifndef HOURGLASS_ACTIVE_BUZZER
#define HOURGLASS_ACTIVE_BUZZER 1
#endif

const uint8_t PIN_DIN = 11;
const uint8_t PIN_CLK = 13;
const uint8_t PIN_CS = 10;
const uint8_t PIN_START_PAUSE = 3;
const uint8_t PIN_RESET = 4;
const uint8_t PIN_BUZZER = 5;
const unsigned int BUZZER_FREQUENCY_HZ = 2000;
const uint32_t BUZZER_ON_MS = 120UL;
const uint32_t BUZZER_GAP_MS = 100UL;
const uint8_t MODULE_COUNT = 2;
const uint8_t GRAIN_COUNT = 36;
const uint32_t TIME_PRESETS_MS[6] = {30000UL, 60000UL, 120000UL, 180000UL, 300000UL, 600000UL};
const uint8_t TIME_PRESET_COUNT = 6;
uint8_t selectedDurationIndex = 1;  // Power-up default: one minute.
uint8_t editingDurationIndex = 1;
uint32_t DURATION_MS = 60000UL;
bool settingMode = false;
const uint32_t SETTINGS_HOLD_MS = 2000UL;
const uint32_t TIME_PREVIEW_MS = 1200UL;
const uint32_t FRAME_MS = 60UL;
const uint32_t BUTTON_DEBOUNCE_MS = 30UL;
const uint32_t IMU_STABLE_MS = 300UL;
const uint32_t IMU_POLL_MS = 20UL;
const uint32_t IMU_RETRY_MS = 1000UL;
// +X must point along M2 -> M1 on the moving backplate. At rest +ax then means M1 up.
// Reverse this sign if the physical +X arrow points from M1 towards M2.
const int8_t IMU_ALONG_SIGN = 1;
// Change this sign after a physical left/right mapping test if required.
// At ax>0, positive ay maps gravity towards increasing row / decreasing column.
const int8_t IMU_TRANSVERSE_SIGN = 1;
static_assert(sizeof(TIME_PRESETS_MS) / sizeof(TIME_PRESETS_MS[0]) == TIME_PRESET_COUNT, "Every preset needs a duration.");

// Chain address 0 is the module whose DIN connects to UNO D11.
// Address 1 is the module whose DIN connects to address 0 DOUT.
// With +X pointing M2 -> M1 and positive ax, M1 is above M2.
// Change this if the module chain order differs from the mechanical assembly.
const uint8_t TOP_MODULE_WHEN_POSITIVE_X = 0;

// Per-module physical calibration: clockwise 90-degree turns, mirror X/Y.
// Logical (row 0, col 0) is the top corner; (7, 7) is the bottom corner.
// IMG_6793 corner calibration: near M0 raw(0,0) is the lower tip;
// far M1 raw(0,0) is the upper tip in the photographed assembly.
// Turn M0 by 180 degrees so both logical necks follow the assembly axis.
const uint8_t ROTATION_QUARTER_TURNS[MODULE_COUNT] = {2, 0};
const bool MIRROR_X[MODULE_COUNT] = {false, false};
const bool MIRROR_Y[MODULE_COUNT] = {false, false};

enum class RunState : uint8_t { READY, RUNNING, PAUSED, DONE };
RunState state = RunState::READY;
// Explicit prototype avoids Arduino auto-prototype ordering for the enum type.
void restartCycle(RunState newState, uint32_t now, const char* reason);

class DebouncedInput {
 public:
  DebouncedInput(uint8_t pin, uint32_t interval)
      : pin_(pin), interval_(interval) {}

  void begin(uint32_t now) {
    pinMode(pin_, INPUT_PULLUP);
    candidate_ = digitalRead(pin_);
    stable_ = candidate_;
    changedAt_ = now;
    ready_ = false;
  }

  // Returns true once per accepted change, after the initial baseline settles.
  bool update(uint32_t now) {
    const uint8_t raw = digitalRead(pin_);
    if (raw != candidate_) {
      candidate_ = raw;
      changedAt_ = now;
    }
    if (uint32_t(now - changedAt_) < interval_) return false;
    if (!ready_) {
      stable_ = candidate_;
      ready_ = true;
      return false;  // A button held at power-up does not trigger a press.
    }
    if (stable_ == candidate_) return false;
    stable_ = candidate_;
    return true;
  }

  bool ready() const { return ready_; }
  uint8_t value() const { return stable_; }

 private:
  uint8_t pin_;
  uint32_t interval_;
  uint8_t candidate_ = HIGH;
  uint8_t stable_ = HIGH;
  uint32_t changedAt_ = 0;
  bool ready_ = false;
};

// IMU health and automatic hold are independent of the manual timer state.
// A short read / NACK immediately stops active-time accumulation and cancels sound.
// Only a continuous, physically plausible sample sequence may release a hold.
class MpuOrientation {
 public:
  void begin(uint32_t now) {
    startWire();
    configured_ = false;
    healthy_ = false;
    ready_ = false;
    hold_ = true;
    recoveryPending_ = true;
    candidateValid_ = false;
    lastAttempt_ = now - IMU_RETRY_MS;
    health_ = "MISSING";
  }

  bool update(uint32_t now) {
    if (!configured_) {
      if (resetWaiting_) {
        // DEVICE_RESET is checked after a conservative delay; no delay() in loop.
        if (uint32_t(now - resetAt_) < 100UL) return false;
        resetWaiting_ = false;
        uint8_t power = 0, who = 0;
        if (!readRegisters(0x6B, &power, 1) || (power & 0x80) ||
            !readRegisters(0x75, &who, 1) || who != 0x68 || !applyConfiguration()) {
          lastAttempt_ = now;
          fail("RESET_ERROR");
          return false;
        }
      } else {
        if (uint32_t(now - lastAttempt_) < IMU_RETRY_MS) return false;
        lastAttempt_ = now;
        if (needsRecovery_) {
          ++recoveryAttempts_;
          const bool released = recoverBus();
          startWire();
          if (!released) { fail(busState_); return false; }
          if (!findDevice()) { fail("MISSING"); return false; }
          // Do not reset other devices on the GY-87: positively identify MPU first.
          if (!writeRegister(0x6B, 0x80)) { fail("RESET_ERROR"); return false; }
          resetWaiting_ = true;
          resetAt_ = uint32_t(millis());
          health_ = "RESET_WAIT";
          return false;
        }
        if (!findDevice() || !applyConfiguration()) { fail("MISSING"); return false; }
      }
      configured_ = true;
      needsRecovery_ = false;
      warmUntil_ = uint32_t(millis());
      lastPoll_ = warmUntil_;
      healthy_ = false;
      health_ = "WARMUP";
      return false;
    }
    if (uint32_t(now - warmUntil_) < 100UL) return false;
    if (uint32_t(now - lastPoll_) < IMU_POLL_MS) return false;
    lastPoll_ = now;
    uint8_t sample[14];
    if (!readRegisters(0x3B, sample, sizeof(sample))) {
      lastAttempt_ = now;
      fail("READ_ERROR");
      return false;
    }
    ax_ = word(sample);
    ay_ = word(sample + 2);
    az_ = word(sample + 4);
    gx_ = word(sample + 8);
    gy_ = word(sample + 10);
    gz_ = word(sample + 12);
    healthy_ = true;
    lastSampleAt_ = now;
    const float x = float(ax_) * IMU_ALONG_SIGN / 16384.0f;
    const float y = float(ay_) * IMU_TRANSVERSE_SIGN / 16384.0f;
    const float z = float(az_) / 16384.0f;
    const float totalSq = x*x + y*y + z*z;
    const float planeSq = x*x + y*y;
    // Use the measured in-plane gravity; ignore impossible / highly dynamic samples.
    const bool plausible = totalSq >= 0.75f*0.75f && totalSq <= 1.25f*1.25f;
    const bool enoughPlane = planeSq >= 0.35f*0.35f;
    const float ratio = enoughPlane ? fabsf(x) / sqrtf(planeSq) : 0.0f;
    const bool invalidPlane = !plausible || !enoughPlane;
    // Enter hold at 0.25, leave only above 0.40 for at least IMU_STABLE_MS.
    const bool enterHold = invalidPlane || ratio <= 0.25f;
    if (enterHold) {
      hold_ = true;
      candidateValid_ = false;
      health_ = !plausible ? "MOTION_HOLD" : (!enoughPlane ? "FACE_HOLD" : "HORIZONTAL_HOLD");
      return false;
    }
    if (hold_ && ratio < 0.40f) {
      candidateValid_ = false;
      health_ = "HORIZONTAL_HOLD";
      return false;
    }
    const bool proposedHigh = x >= 0.0f;
    const bool needsSettle = !ready_ || recoveryPending_ || hold_ || proposedHigh != high_;
    if (needsSettle) {
      // The gyro is not integrated: it only rejects a rapidly moving flip candidate.
      const int32_t gyroLimit = 120L * 131L;
      if (abs32(gx_) > gyroLimit || abs32(gy_) > gyroLimit || abs32(gz_) > gyroLimit) {
        candidateValid_ = false;
        hold_ = true;
        health_ = "MOTION_HOLD";
        return false;
      }
      hold_ = true;
      health_ = "SETTLING";
      if (!candidateValid_ || proposedHigh != candidateHigh_) {
        candidateValid_ = true;
        candidateHigh_ = proposedHigh;
        candidateAt_ = now;
        return false;
      }
      if (uint32_t(now - candidateAt_) < IMU_STABLE_MS) return false;
      const bool changed = ready_ && proposedHigh != high_;
      high_ = proposedHigh;
      ready_ = true;
      recoveryPending_ = false;
      candidateValid_ = false;
      hold_ = false;
      health_ = "OK";
      x_ = x;
      y_ = y;
      return changed;
    }
    x_ = x;
    y_ = y;
    hold_ = false;
    candidateValid_ = false;
    health_ = "OK";
    return false;
  }

  void rebaseline() {
    // Reset discards a pending flip, then accepts the current attitude as a quiet baseline.
    ready_ = false;
    candidateValid_ = false;
    recoveryPending_ = true;
    hold_ = true;
    if (healthy_) health_ = "SETTLING";
  }
  bool ready() const { return ready_; }
  bool healthy() const { return healthy_; }
  bool hold() const { return hold_ || !ready_ || !healthy_; }
  bool high() const { return high_; }
  float along() const { return x_; }
  float transverse() const { return y_; }
  int16_t ax() const { return ax_; }
  int16_t ay() const { return ay_; }
  int16_t az() const { return az_; }
  uint8_t address() const { return address_; }
  const char* healthName() const { return health_; }
  uint32_t recoveryAttempts() const { return recoveryAttempts_; }
  uint32_t busClearPulses() const { return busClearPulses_; }
  uint32_t faults() const { return faults_; }
  uint32_t lastSampleAt() const { return lastSampleAt_; }
  uint8_t transactionError() const { return transactionError_; }
  uint8_t receivedBytes() const { return receivedBytes_; }
  const char* busState() const { return busState_; }
#if HOURGLASS_RECOVERY_SELF_TEST
  void injectReadFaultForTest(uint32_t now) { testFault_ = true; testFaultAt_ = now; }
#endif

 private:
  static int32_t abs32(int16_t v) { return v < 0 ? -int32_t(v) : int32_t(v); }
  static int16_t word(const uint8_t* b) {
    return int16_t((uint16_t(b[0]) << 8) | uint16_t(b[1]));
  }
  bool readRegisters(uint8_t reg, uint8_t* target, uint8_t count) {
#if HOURGLASS_RECOVERY_SELF_TEST
    if (reg == 0x3B && testFault_) {
      if (uint32_t(millis() - testFaultAt_) < 1800UL) {
        transactionError_ = 0x82;
        receivedBytes_ = 0;
        return false;
      }
      testFault_ = false;
    }
#endif
    Wire.beginTransmission(address_);
    Wire.write(reg);
    receivedBytes_ = 0;
    transactionError_ = Wire.endTransmission(false);
    if (transactionError_ != 0) return false;
#if defined(ARDUINO_ARCH_AVR)
    // AVR Wire also has int overloads: match its uint8_t signature exactly.
    const size_t received = Wire.requestFrom(uint8_t(address_), uint8_t(count), uint8_t(true));
#else
    const size_t received = Wire.requestFrom(address_, size_t(count), true);
#endif
    receivedBytes_ = uint8_t(received);
    if (received != count) {
      transactionError_ = 0x80;  // Short read (distinct from Wire transaction codes).
      while (Wire.available()) Wire.read();
      return false;
    }
    for (uint8_t i = 0; i < count; ++i) {
      if (!Wire.available()) { transactionError_ = 0x80; return false; }
      target[i] = uint8_t(Wire.read());
    }
    return true;
  }
  bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(value);
    transactionError_ = Wire.endTransmission(true);
    return transactionError_ == 0;
  }
  void startWire() {
    Wire.begin();
    Wire.setClock(100000UL);
#if defined(ARDUINO_ARCH_RENESAS_UNO) || defined(WIRE_HAS_TIMEOUT)
    // Renesas 1.6.0 ignores reset_on_timeout. Recovery explicitly calls end/begin.
    Wire.setWireTimeout(25000U, false);
#endif
  }
  static void releaseLine(uint8_t pin) { pinMode(pin, INPUT); }
  static void pullLow(uint8_t pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  static bool waitHigh(uint8_t pin) {
    // Bounded clock-stretch wait: up to 1 ms, including a permanently held line.
    for (uint16_t i = 0; i < 200; ++i) {
      if (digitalRead(pin) == HIGH) return true;
      delayMicroseconds(5);
    }
    return digitalRead(pin) == HIGH;
  }
  bool recoverBus() {
    Wire.end();
    releaseLine(SDA);  // Override end() pullups; rely on existing module pullups.
    releaseLine(SCL);
    delayMicroseconds(5);
    if (!waitHigh(SCL)) { busState_ = "SCL_LOW"; return false; }
    if (digitalRead(SDA) == LOW) {
      // NXP UM10204 3.1.16: at most nine clocks when SDA is stuck LOW.
      // Open drain only: never drive either bus line HIGH.
      for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; ++i) {
        pullLow(SCL);
        delayMicroseconds(5);
        releaseLine(SCL);
        ++busClearPulses_;
        if (!waitHigh(SCL)) { busState_ = "SCL_LOW"; return false; }
        delayMicroseconds(5);
      }
      if (digitalRead(SDA) == LOW) { busState_ = "SDA_LOW"; return false; }
      // Generate STOP: SDA rises while SCL is HIGH.
      pullLow(SCL);
      pullLow(SDA);
      delayMicroseconds(5);
      releaseLine(SCL);
      if (!waitHigh(SCL)) { releaseLine(SDA); busState_ = "SCL_LOW"; return false; }
      delayMicroseconds(5);
      releaseLine(SDA);
      delayMicroseconds(5);
    }
    const bool free = digitalRead(SDA) == HIGH && digitalRead(SCL) == HIGH;
    busState_ = free ? "FREE" : "SDA_LOW";
    return free;
  }
  bool findDevice() {
    bool found = false;
    for (uint8_t i = 0; i < 2; ++i) {
      address_ = 0x68 + i;
      uint8_t who = 0;
      // MPU6050 WHO_AM_I is 0x68 even when its I2C address is 0x69.
      if (readRegisters(0x75, &who, 1) && who == 0x68) { found = true; break; }
    }
    return found;
  }
  bool applyConfiguration() {
    const bool written = writeRegister(0x6B, 0x01) &&  // Wake, PLL X gyro reference.
           writeRegister(0x1A, 0x03) &&  // DLPF: accelerometer 44Hz / gyro 42Hz.
           writeRegister(0x1B, 0x00) &&  // Gyro +/-250 degrees/s.
           writeRegister(0x1C, 0x00) &&  // Accelerometer +/-2g, 16384 LSB/g.
           writeRegister(0x19, 0x04);    // 1kHz / (1+4) = 200Hz sample rate.
    if (!written) return false;
    const uint8_t regs[5] = {0x6B, 0x1A, 0x1B, 0x1C, 0x19};
    const uint8_t values[5] = {0x01, 0x03, 0x00, 0x00, 0x04};
    for (uint8_t i = 0; i < 5; ++i) {
      uint8_t actual = 0;
      if (!readRegisters(regs[i], &actual, 1)) return false;
      if (actual != values[i]) { transactionError_ = 0x81; return false; }
    }
    return true;
  }
  void fail(const char* reason) {
    ++faults_;
    needsRecovery_ = true;
    resetWaiting_ = false;
    // Keep ready_ / high_: recovery must not become a new baseline or reset timer.
    configured_ = false;
    healthy_ = false;
    hold_ = true;
    recoveryPending_ = true;
    candidateValid_ = false;
    health_ = reason;
  }
  uint8_t address_ = 0x68;
#if HOURGLASS_RECOVERY_SELF_TEST
  bool testFault_ = false;
  uint32_t testFaultAt_ = 0;
#endif
  bool needsRecovery_ = false, resetWaiting_ = false;
  uint32_t resetAt_ = 0, recoveryAttempts_ = 0, busClearPulses_ = 0;
  uint32_t faults_ = 0, lastSampleAt_ = 0;
  uint8_t transactionError_ = 0, receivedBytes_ = 0;
  const char* busState_ = "UNCHECKED";
  bool configured_ = false, healthy_ = false, ready_ = false, high_ = true;
  bool hold_ = true, recoveryPending_ = true, candidateValid_ = false, candidateHigh_ = true;
  uint32_t lastAttempt_ = 0, warmUntil_ = 0, lastPoll_ = 0, candidateAt_ = 0;
  int16_t ax_ = 0, ay_ = 0, az_ = 0, gx_ = 0, gy_ = 0, gz_ = 0;
  float x_ = 1.0f, y_ = 0.0f;
  const char* health_ = "MISSING";
};

MpuOrientation imu;
bool sensorHold = true;
float gravityRow = 1.0f;
float gravityCol = 1.0f;

DebouncedInput startPause(PIN_START_PAUSE, BUTTON_DEBOUNCE_MS);
DebouncedInput resetButton(PIN_RESET, BUTTON_DEBOUNCE_MS);

bool orientationHigh = true;
bool grains[MODULE_COUNT][8][8] = {};
uint8_t frame[MODULE_COUNT][8] = {};
uint8_t transferred = 0;
uint32_t elapsedMs = 0;
uint32_t lastClockMs = 0;
uint32_t lastFrameMs = 0;
bool frameDirty = false;
bool redHoldArmed = false;
uint32_t redPressedAtMs = 0;
bool timePreviewActive = false;
uint32_t timePreviewStartedMs = 0;
uint8_t pileRelaxFrames = 8;
float pileReferenceRow = 0.0f;
float pileReferenceCol = 0.0f;
uint32_t lastStatusMs = 0;
uint32_t lastRestartMs = 0;
bool pausedFlipFeedback = false;
uint32_t rngState = 0x281036UL;

enum class BuzzerCue : uint8_t { NONE, START, FINISH };
BuzzerCue pendingBuzzerCue = BuzzerCue::NONE;
BuzzerCue playingBuzzerCue = BuzzerCue::NONE;
uint32_t buzzerStartedMs = 0;
bool buzzerIsOn = false;

void setBuzzerOutput(bool on) {
  if (buzzerIsOn == on) return;
  buzzerIsOn = on;
  if (playingBuzzerCue == BuzzerCue::FINISH) frameDirty = true;
  if (HOURGLASS_ACTIVE_BUZZER) {
    digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
  } else if (on) {
    // Do not use tone's duration argument: our unsigned time difference handles wrap.
    tone(PIN_BUZZER, BUZZER_FREQUENCY_HZ);
  } else {
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);
  }
}

void cancelBuzzer() {
  if (playingBuzzerCue == BuzzerCue::FINISH) frameDirty = true;
  pendingBuzzerCue = BuzzerCue::NONE;
  playingBuzzerCue = BuzzerCue::NONE;
  if (!HOURGLASS_ACTIVE_BUZZER) noTone(PIN_BUZZER);
  digitalWrite(PIN_BUZZER, LOW);
  buzzerIsOn = false;
}

void initialiseBuzzer() {
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  cancelBuzzer();
}

void serviceBuzzer(uint32_t now) {
  // Cues are serviced after all contacts so a same-loop B2 reset wins silently.
  if (HOURGLASS_CORNER_CALIBRATION) return;
  if (pendingBuzzerCue != BuzzerCue::NONE) {
    setBuzzerOutput(false);
    playingBuzzerCue = pendingBuzzerCue;
    pendingBuzzerCue = BuzzerCue::NONE;
    buzzerStartedMs = now;
  }
  if (playingBuzzerCue == BuzzerCue::NONE) return;
  const uint32_t age = uint32_t(now - buzzerStartedMs);
  const uint32_t interval = BUZZER_ON_MS + BUZZER_GAP_MS;
  const uint32_t duration = (playingBuzzerCue == BuzzerCue::START)
      ? BUZZER_ON_MS : (3UL * BUZZER_ON_MS + 2UL * BUZZER_GAP_MS);
  if (age >= duration) {
    setBuzzerOutput(false);
    playingBuzzerCue = BuzzerCue::NONE;
  } else {
    setBuzzerOutput((age % interval) < BUZZER_ON_MS);
  }
}

uint8_t upperModule() {
  return orientationHigh ? TOP_MODULE_WHEN_POSITIVE_X : (1 - TOP_MODULE_WHEN_POSITIVE_X);
}

bool randomChoice() {
  // Local deterministic generator; reproducible without floating analogue inputs.
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return (rngState & 1UL) != 0;
}

void sendByte(uint8_t value) {
  for (int8_t bit = 7; bit >= 0; --bit) {
    digitalWrite(PIN_CLK, LOW);
    digitalWrite(PIN_DIN, (value & (1U << bit)) ? HIGH : LOW);
    // Conservative timing; no dependency on the host CPU's GPIO speed.
    delayMicroseconds(1);
    digitalWrite(PIN_CLK, HIGH);
    delayMicroseconds(1);
  }
  digitalWrite(PIN_CLK, LOW);
}

void writeBoth(uint8_t reg, uint8_t nearValue, uint8_t farValue) {
  digitalWrite(PIN_CS, LOW);
  // The first 16 bits travel to the far module; the last stay in the near one.
  sendByte(reg);
  sendByte(farValue);
  sendByte(reg);
  sendByte(nearValue);
  digitalWrite(PIN_CS, HIGH);
}

void initialiseDisplays() {
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_DIN, LOW);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_DIN, OUTPUT);
  writeBoth(0x0C, 0, 0);  // Shutdown while configuring.
  writeBoth(0x0F, 0, 0);  // Display-test OFF: it otherwise forces full brightness.
  writeBoth(0x09, 0, 0);  // Decode OFF for a pixel matrix.
  writeBoth(0x0B, 7, 7);  // Keep all eight rows scanned.
  writeBoth(0x0A, 0, 0);  // Minimum intensity, 1/32 PWM, not an off setting.
  for (uint8_t row = 1; row <= 8; ++row) writeBoth(row, 0, 0);
  writeBoth(0x0C, 1, 1);  // Normal operation only after the blank configuration.
}

void clearFrame() {
  for (uint8_t module = 0; module < MODULE_COUNT; ++module)
    for (uint8_t row = 0; row < 8; ++row) frame[module][row] = 0;
}

void putPixel(uint8_t module, uint8_t row, uint8_t col, bool compensateFlip) {
  uint8_t x = col;
  uint8_t y = row;
  // Stable inversion rotates the same physical assembly through 180 degrees.
  if (compensateFlip && !orientationHigh) {
    x = 7 - x;
    y = 7 - y;
  }
  for (uint8_t turn = 0; turn < (ROTATION_QUARTER_TURNS[module] & 3); ++turn) {
    const uint8_t oldX = x;
    x = 7 - y;
    y = oldX;
  }
  if (MIRROR_X[module]) x = 7 - x;
  if (MIRROR_Y[module]) y = 7 - y;
  frame[module][y] |= uint8_t(1U << (7 - x));
}

void flushFrame() {
  for (uint8_t row = 0; row < 8; ++row)
    writeBoth(row + 1, frame[0][row], frame[1][row]);
}

#include "PresetDisplay.h"

void renderGrains() {
  if (!AUX_CLOCK_ENABLED && (settingMode || (timePreviewActive && state == RunState::READY))) {
    renderPresetTime(settingMode ? editingDurationIndex : selectedDurationIndex, settingMode);
    return;
  }
  clearFrame();
  for (uint8_t module = 0; module < MODULE_COUNT; ++module)
    for (uint8_t row = 0; row < 8; ++row)
      for (uint8_t col = 0; col < 8; ++col)
        if (grains[module][row][col]) putPixel(module, row, col, true);

  // Separate status pixels occupy the empty top corner of the upper chamber.
  // They are indicators, not grains; the internal grain count remains exactly 36.
  const uint8_t top = upperModule();
  const uint32_t now = uint32_t(millis());
  if (state == RunState::READY) {
    // Small play triangle: ready for B1 or a stable flip.
    putPixel(top, 0, 0, true);
    putPixel(top, 1, 0, true);
    putPixel(top, 1, 1, true);
    putPixel(top, 2, 0, true);
  } else if (state == RunState::RUNNING) {
    // A heartbeat dot distinguishes active timing even between grain transfers.
    if ((now % 1000UL) < 500UL) putPixel(top, 0, 0, true);
  } else if (state == RunState::PAUSED) {
    // When a paused flip replenishes the chamber, flash the sand twice.
    const uint32_t sinceRestart = uint32_t(now - lastRestartMs);
    if (pausedFlipFeedback && sinceRestart < 600UL && (sinceRestart % 300UL) < 90UL) clearFrame();
    for (uint8_t row = 0; row < 3; ++row) {
      putPixel(top, row, 0, true);
      putPixel(top, row, 2, true);
    }
  } else {
    // Check mark in the empty upper chamber: timer completed.
    putPixel(top, 1, 0, true);
    putPixel(top, 2, 1, true);
    putPixel(top, 1, 2, true);
    putPixel(top, 0, 3, true);
  }
  if (sensorHold) {
    // Two pause bars on healthy automatic holds; an X when sensor reads fail.
    // Status pixels are separate from the invariant 36 logical grains.
    for (uint8_t r = 0; r < 4; ++r) {
      putPixel(top, r, imu.healthy() ? 0 : r, true);
      putPixel(top, r, imu.healthy() ? 2 : 3 - r, true);
    }
  }
  // Three dark pulses expose the lower pile again between finish beeps.
  // Only the frame changes; all 36 logical grains are preserved.
  if (state == RunState::DONE && playingBuzzerCue == BuzzerCue::FINISH && buzzerIsOn) {
    const uint8_t bottom = 1 - top;
    for (uint8_t row = 0; row < 8; ++row) frame[bottom][row] = 0;
  }
  flushFrame();
}

void renderCalibration() {
  clearFrame();
  for (uint8_t module = 0; module < MODULE_COUNT; ++module) {
    // One pixel = logical top corner. Two adjacent pixels = logical bottom.
    putPixel(module, 0, 0, false);
    putPixel(module, 7, 7, false);
    putPixel(module, 7, 6, false);
  }
  flushFrame();
}

uint8_t grainCount(uint8_t module) {
  uint8_t count = 0;
  for (uint8_t row = 0; row < 8; ++row)
    for (uint8_t col = 0; col < 8; ++col)
      if (grains[module][row][col]) ++count;
  return count;
}

const char* stateName() {
  switch (state) {
    case RunState::READY: return "READY";
    case RunState::RUNNING: return "RUNNING";
    case RunState::PAUSED: return "PAUSED";
    case RunState::DONE: return "DONE";
  }
  return "UNKNOWN";
}

// Fixed-size optional telemetry. No waiting for a computer to connect/read.
// R4 WiFi uses an internal UART bridge; its inherited availableForWrite() is zero.
char telemetryBuffer[512] = {};
size_t telemetryLength = 0, telemetryOffset = 0;
uint32_t telemetryDropped = 0;
void serviceTelemetry() {
  if (!Serial) { telemetryLength = telemetryOffset = 0; return; }
  if (telemetryOffset >= telemetryLength) return;
  size_t count = telemetryLength - telemetryOffset;
#if defined(ARDUINO_UNOR4_WIFI) && defined(NO_USB)
  // Installed Renesas 1.6.0 UART has no FIFO-availability API. It has no CTS,
  // so the host cannot back-pressure this link. Send at most eight bytes
  // synchronously (~0.7ms at 115200), then return to inputs/sensor/animation.
  if (count > 8) count = 8;
  // Use the core's proven byte-at-a-time Print path; the non-const UART block
  // overload returns on DATA_EMPTY before the last byte completes.
  for (size_t i = 0; i < count; ++i)
    telemetryOffset += Serial.write(uint8_t(telemetryBuffer[telemetryOffset]));
#else
  const int available = Serial.availableForWrite();
  if (available <= 0) return;
  if (count > 64) count = 64;
  if (count > size_t(available)) count = size_t(available);
  // Native USB: single writer, never exceed currently available FIFO space.
  telemetryOffset += Serial.write(reinterpret_cast<const uint8_t*>(telemetryBuffer + telemetryOffset), count);
#endif
}
void logLine(const char* text) {
  serviceTelemetry();
  if (telemetryOffset < telemetryLength) { ++telemetryDropped; return; }
  const size_t count = strlen(text);
  if (count + 1 >= sizeof(telemetryBuffer)) { ++telemetryDropped; return; }
  memcpy(telemetryBuffer, text, count);
  telemetryBuffer[count] = '\n';
  telemetryLength = count + 1;
  telemetryOffset = 0;
  serviceTelemetry();
}
void reportStatus(const char* reason) {
  const uint32_t remainingMs = DURATION_MS - elapsedMs;
  char line[512];
  const int count = snprintf(line, sizeof(line),
      "%s | fw=3.9 state=%s duration_s=%lu setting=%u selected=%u editing=%u "
      "elapsed_ms=%lu remaining_s=%lu top_addr=%u grains_near/far=%u/%u "
      "imu=%s sensor_hold=%u ax/ay/az=%d/%d/%d D3/D4=%d/%d "
      "uptime_ms=%lu sample_age_ms=%lu faults=%lu recoveries=%lu pulses=%lu "
      "i2c_err=%u rx=%u bus=%s logs_dropped=%lu bench=%u",
      reason, stateName(), static_cast<unsigned long>(DURATION_MS / 1000UL),
      unsigned(settingMode), unsigned(selectedDurationIndex), unsigned(editingDurationIndex),
      static_cast<unsigned long>(elapsedMs), static_cast<unsigned long>((remainingMs + 999UL) / 1000UL),
      unsigned(upperModule()), unsigned(grainCount(0)), unsigned(grainCount(1)),
      imu.healthName(), unsigned(sensorHold), int(imu.ax()), int(imu.ay()), int(imu.az()),
      digitalRead(PIN_START_PAUSE), digitalRead(PIN_RESET), static_cast<unsigned long>(millis()),
      static_cast<unsigned long>(uint32_t(millis() - imu.lastSampleAt())),
      static_cast<unsigned long>(imu.faults()), static_cast<unsigned long>(imu.recoveryAttempts()),
      static_cast<unsigned long>(imu.busClearPulses()), unsigned(imu.transactionError()),
      unsigned(imu.receivedBytes()), imu.busState(), static_cast<unsigned long>(telemetryDropped), unsigned(HOURGLASS_RECOVERY_SELF_TEST));
  if (count > 0 && size_t(count) < sizeof(line)) logLine(line);
}

void restartCycle(RunState newState, uint32_t now, const char* reason) {
  cancelBuzzer();  // B2, baseline settling and every accepted flip discard old cues.
  if (!HOURGLASS_CORNER_CALIBRATION && newState == RunState::RUNNING)
    pendingBuzzerCue = BuzzerCue::START;
  state = newState;
  pausedFlipFeedback = (newState == RunState::PAUSED);
  lastRestartMs = now;
  elapsedMs = 0;
  transferred = 0;
  pileRelaxFrames = 8;
  pileReferenceRow = 0.0f;
  pileReferenceCol = 0.0f;
  lastClockMs = now;
  lastFrameMs = now - FRAME_MS;  // The next loop renders immediately, including wrap.
  for (uint8_t module = 0; module < MODULE_COUNT; ++module)
    for (uint8_t row = 0; row < 8; ++row)
      for (uint8_t col = 0; col < 8; ++col)
        grains[module][row][col] = false;
  const uint8_t top = upperModule();
  // Exactly 36 pixels form the lower half of a diagonally oriented 8x8 chamber.
  for (uint8_t row = 0; row < 8; ++row)
    for (uint8_t col = 0; col < 8; ++col)
      grains[top][row][col] = (row + col >= 7);
  reportStatus(reason);
}

void updateGravity() {
  // Accelerometer signs describe "up" in body coordinates. In our diamond mapping,
  // +ax corresponds to the logical lower-right diagonal when M1 is uppermost.
  const float flip = orientationHigh ? 1.0f : -1.0f;
  gravityRow = (imu.along() + imu.transverse()) * flip;
  gravityCol = (imu.along() - imu.transverse()) * flip;
}

void settleModule(uint8_t module, uint8_t maxMoves) {
  uint8_t moved = 0;
  // Each grain moves at most once. Visit downstream cells first for any gravity angle.
  uint8_t order[64];
  float projection[64];
  for (uint8_t i = 0; i < 64; ++i) {
    order[i] = i;
    projection[i] = (i / 8) * gravityRow + (i % 8) * gravityCol;
  }
  for (uint8_t i = 1; i < 64; ++i) {
    const uint8_t cell = order[i];
    const float value = projection[cell];
    int8_t j = int8_t(i) - 1;
    while (j >= 0 && projection[order[j]] < value) {
      order[j + 1] = order[j];
      --j;
    }
    order[j + 1] = cell;
  }
  const int8_t dr[8] = {-1,-1,-1,0,0,1,1,1};
  const int8_t dc[8] = {-1,0,1,-1,1,-1,0,1};
  for (uint8_t index = 0; index < 64; ++index) {
    const uint8_t row = order[index] / 8;
    const uint8_t col = order[index] % 8;
    if (!grains[module][row][col]) continue;
    int8_t selected = -1;
    float best = 0.0f;
    for (uint8_t d = 0; d < 8; ++d) {
      const int8_t r = int8_t(row) + dr[d], c = int8_t(col) + dc[d];
      if (r < 0 || r > 7 || c < 0 || c > 7 || grains[module][r][c]) continue;
      float score = dr[d] * gravityRow + dc[d] * gravityCol;
      if (dr[d] && dc[d]) score *= 0.70710678f;
      // Zero/uphill moves are disallowed; randomise exactly symmetric alternatives.
      if (score <= 0.0001f) continue;
      if (selected < 0 || score > best + 0.0001f ||
          (fabsf(score - best) < 0.0001f && randomChoice())) {
        selected = int8_t(d);
        best = score;
      }
    }
    if (selected >= 0) {
      grains[module][row][col] = false;
      grains[module][int8_t(row) + dr[selected]][int8_t(col) + dc[selected]] = true;
      if (++moved >= maxMoves) break;
    }
  }
}

bool transferOneGrain() {
  const uint8_t top = upperModule();
  const uint8_t bottom = 1 - top;
  int8_t sourceRow = -1;
  int8_t sourceCol = -1;
  int8_t targetRow = -1;
  int8_t targetCol = -1;
  // Prefer the physical neck. If settling is delayed, choose the nearest grain
  // and free entry cell; animation must not extend the independent timer.
  for (int8_t diagonal = 14; diagonal >= 0 && sourceRow < 0; --diagonal)
    for (uint8_t row = 0; row < 8; ++row) {
      const int8_t col = diagonal - row;
      if (col >= 0 && col < 8 && grains[top][row][col]) {
        sourceRow = row; sourceCol = col; break;
      }
    }
  for (uint8_t diagonal = 0; diagonal <= 14 && targetRow < 0; ++diagonal)
    for (uint8_t row = 0; row < 8; ++row) {
      const int8_t col = int8_t(diagonal) - row;
      if (col >= 0 && col < 8 && !grains[bottom][row][col]) {
        targetRow = row; targetCol = col; break;
      }
    }
  if (sourceRow < 0 || targetRow < 0) return false;
  grains[top][sourceRow][sourceCol] = false;
  grains[bottom][targetRow][targetCol] = true;
  ++transferred;
  return true;
}

void synchroniseTransfers() {
  const uint8_t target = uint8_t((uint64_t(elapsedMs) * GRAIN_COUNT) / DURATION_MS);
  while (transferred < target) {
    if (!transferOneGrain()) {
      // Unexpected invariant failure: do not spin forever or hide it.
      logLine("ERROR: grain transfer failed; reset and inspect.");
      state = RunState::PAUSED;
      return;
    }
  }
}

void advanceClock(uint32_t now) {
  const uint32_t delta = uint32_t(now - lastClockMs);
  lastClockMs = now;  // Also updated while idle/paused, so those intervals never count.
  if (state != RunState::RUNNING || sensorHold) return;
  const uint32_t remaining = DURATION_MS - elapsedMs;
  elapsedMs += (delta >= remaining) ? remaining : delta;
  synchroniseTransfers();
  if (elapsedMs == DURATION_MS && state == RunState::RUNNING) {
    state = RunState::DONE;
    if (!HOURGLASS_CORNER_CALIBRATION) pendingBuzzerCue = BuzzerCue::FINISH;
    lastFrameMs = now - FRAME_MS;
    reportStatus("Completed");
  }
}

void handleStartPause(uint32_t now) {
  timePreviewActive = false;
  frameDirty = true;
  // A user can still pause while an automatic hold is active. Starting or resuming
  // also sets the requested state, but the sensor hold continues to freeze time.
  if (!imu.ready() && state != RunState::RUNNING) {
    logLine("IMU baseline unavailable: wait for a stable valid attitude.");
    return;
  }
  switch (state) {
    case RunState::READY:
    case RunState::DONE:
      restartCycle(RunState::RUNNING, now, "Started");
      break;
    case RunState::RUNNING:
      pausedFlipFeedback = false;
      state = RunState::PAUSED;
      reportStatus("Paused");
      break;
    case RunState::PAUSED:
      state = RunState::RUNNING;
      lastClockMs = now;
      reportStatus("Resumed");
      break;
  }
}

void serviceClockDisplay() {
  if (!AUX_CLOCK_ENABLED || AUX_CLOCK_CALIBRATION) return;
  uint32_t seconds = 0;
  char mode = 'T';
  if (settingMode) {
    seconds = TIME_PRESETS_MS[editingDurationIndex] / 1000UL;
    mode = 'S';
  } else if (state == RunState::READY) {
    seconds = DURATION_MS / 1000UL;
    mode = imu.healthy() ? 'T' : 'E';
  } else {
    seconds = (DURATION_MS - elapsedMs + 999UL) / 1000UL;
    if (!imu.healthy()) mode = 'E';
    else if (state == RunState::DONE) mode = 'D';
    else if (state == RunState::PAUSED) mode = 'P';
    else if (sensorHold) mode = 'H';
    else mode = 'R';
  }
  AuxClock::service(uint16_t(seconds), mode);
}

void resetFromButton(uint32_t now) {
  timePreviewActive = false;
  imu.rebaseline();
  sensorHold = true;
  restartCycle(RunState::READY, now, "Reset: settling a fresh IMU baseline");
  frameDirty = true;
}

void enterSettings(uint32_t now) {
  settingMode = true;
  timePreviewActive = false;
  editingDurationIndex = selectedDurationIndex;
  redHoldArmed = false;  // One menu entry per hold; release cannot confirm it.
  cancelBuzzer();
  restartCycle(RunState::READY, now, "Time selection opened");
  frameDirty = true;
}

void confirmSettings(uint32_t now) {
  selectedDurationIndex = editingDurationIndex;
  DURATION_MS = TIME_PRESETS_MS[selectedDurationIndex];
  settingMode = false;
  redHoldArmed = false;  // The confirming press cannot become another long press.
  restartCycle(RunState::READY, now, "Time selection confirmed");
  timePreviewActive = true;
  timePreviewStartedMs = now;
  frameDirty = true;
}

void serviceButtons(uint32_t now) {
  const bool b1Changed = startPause.update(now);
  const bool b2Changed = resetButton.update(now);
  const bool b1Pressed = b1Changed && startPause.value() == LOW;
  const bool b2Pressed = b2Changed && resetButton.value() == LOW;
  if (b2Changed && resetButton.value() == HIGH) redHoldArmed = false;
  // Red takes priority over simultaneous contacts, including inside the menu.
  if (b2Pressed) {
    if (settingMode) {
      confirmSettings(now);
    } else {
      // Remember the state at the original press. Holding reset during a run
      // must not become a settings command after reset changes state to READY.
      redHoldArmed = state == RunState::READY || state == RunState::DONE;
      redPressedAtMs = now;
      resetFromButton(now);
    }
  } else if (b1Pressed && resetButton.value() != LOW) {
    if (settingMode) {
      editingDurationIndex = uint8_t((editingDurationIndex + 1) % TIME_PRESET_COUNT);
      frameDirty = true;
      reportStatus("Time selection changed");
    } else {
      redHoldArmed = false;
      handleStartPause(now);
    }
  }
  if (redHoldArmed && resetButton.value() == LOW &&
      (state == RunState::READY || state == RunState::DONE) &&
      uint32_t(now - redPressedAtMs) >= SETTINGS_HOLD_MS) {
    enterSettings(now);
  }
}

void setup() {
  Serial.begin(115200);  // USB serial is optional; never block waiting for a monitor.
  logLine("ELEC281 v3.9: MPU6050 gravity, horizontal hold, D5 active buzzer");
  const uint32_t now = uint32_t(millis());
  startPause.begin(now);
  resetButton.begin(now);
  initialiseBuzzer();
  initialiseDisplays();
  if (AUX_CLOCK_ENABLED) AuxClock::begin();
  imu.begin(now);
  orientationHigh = true;  // Provisional display only; no timing until an IMU baseline.
  sensorHold = true;
  restartCycle(RunState::READY, now, "Power-up: waiting for IMU");
  if (HOURGLASS_CORNER_CALIBRATION) {
    renderCalibration();
    logLine("CALIBRATION: 1 pixel = top; 2 pixels = bottom; timer and sound disabled.");
  } else {
    timePreviewActive = true;
    timePreviewStartedMs = now;
    renderGrains();
    logLine("B1 start/pause/resume; B2 reset; idle B2 hold 2s: set time; B1 next; B2 confirm.");
    serviceClockDisplay();
  }
  if (AUX_CLOCK_ENABLED && AUX_CLOCK_CALIBRATION) {
    cancelBuzzer();
    AuxClock::calibration();
    logLine("AUX CALIBRATION: 1234; timer and sound disabled.");
  }
}

void loop() {
  serviceTelemetry();
  const uint32_t now = uint32_t(millis());
  const bool wasReady = imu.ready();
  const bool wasHeld = sensorHold;
  const bool wasHealthy = imu.healthy();
  const bool flipped = imu.update(now);
  sensorHold = imu.hold();
  if (wasHeld != sensorHold) {
    lastClockMs = now;  // Never count the automatic pause or its settling interval.
    reportStatus(sensorHold ? "Automatic hold entered" : "Automatic hold released");
    lastFrameMs = now - FRAME_MS;
  }
  if (wasHealthy && !imu.healthy()) {
    cancelBuzzer();
    reportStatus("IMU read fault: timing frozen");
  }

  if (HOURGLASS_CORNER_CALIBRATION || AUX_CLOCK_CALIBRATION) {
    cancelBuzzer();
    if (uint32_t(now - lastStatusMs) >= 500UL) {
      lastStatusMs = now;
      reportStatus("Calibration inputs");
    }
    return;
  }

  if (!wasReady && imu.ready()) {
    orientationHigh = imu.high();
    restartCycle(RunState::READY, now, "Orientation baseline settled");
  } else if (flipped) {
    orientationHigh = imu.high();
    // A pending idle long-red command owns the interaction even before the
    // two-second threshold. Repositioning the sensor must not auto-start it.
    const bool selectingTime = settingMode || (redHoldArmed && resetButton.value() == LOW);
    const RunState next = selectingTime ? RunState::READY :
        ((state == RunState::PAUSED) ? RunState::PAUSED : RunState::RUNNING);
    restartCycle(next, now, "Stable IMU inversion");
  }
  updateGravity();
  advanceClock(now);
#if HOURGLASS_RECOVERY_SELF_TEST
  // One recoverable read fault after 5s of actual timing; not an electrical fault.
  static bool injected = false;
  if (!injected && state == RunState::RUNNING && elapsedMs >= 5000UL) {
    injected = true;
    imu.injectReadFaultForTest(now);
    reportStatus("DIAGNOSTIC injected read failure");
  }
#endif

  serviceButtons(now);
  if (timePreviewActive && uint32_t(now - timePreviewStartedMs) >= TIME_PREVIEW_MS) {
    timePreviewActive = false;
    frameDirty = true;
  }
  if (!imu.healthy()) cancelBuzzer();
  // Keep cue timing tied to the actual GPIO output time.
  serviceBuzzer(uint32_t(millis()));

  if (frameDirty || uint32_t(now - lastFrameMs) >= FRAME_MS) {
    frameDirty = false;
    lastFrameMs = now;
    if (!settingMode && !sensorHold && (state == RunState::RUNNING || state == RunState::DONE)) {
      // Compare with the last accepted direction, so slow continuous tilts also
      // trigger fast settling. Budgets advance only on actual, unpaused frames.
      const float magnitude = sqrtf(gravityRow * gravityRow + gravityCol * gravityCol);
      if (magnitude > 0.0001f) {
        const float row = gravityRow / magnitude;
        const float col = gravityCol / magnitude;
        if (row * pileReferenceRow + col * pileReferenceCol < 0.995f) {
          pileReferenceRow = row;
          pileReferenceCol = col;
          pileRelaxFrames = 8;
        }
      }
      const uint8_t top = upperModule();
      // Stable upper pile: show the void refilling one real adjacent grain per
      // frame. A changed attitude gets rapid settling without reserving a gap.
      settleModule(top, pileRelaxFrames ? 64 : 1);
      settleModule(1 - top, 64);
      if (pileRelaxFrames) --pileRelaxFrames;
    }
    renderGrains();
  }
  serviceClockDisplay();
  if (uint32_t(now - lastStatusMs) >= 1000UL) {
    lastStatusMs = now;
    reportStatus("Status");
  }
  serviceTelemetry();
}
