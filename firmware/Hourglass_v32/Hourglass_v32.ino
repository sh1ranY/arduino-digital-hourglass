#include <Arduino.h>

// ELEC281 v3.2 prototype: UNO R4 WiFi, two MAX7219 modules, passive switches.
// Standalone Arduino API only. No LedControl or AVR-specific dependencies.
// 0 = hourglass; 1 = low-brightness corner calibration (no timer).
#ifndef HOURGLASS_CORNER_CALIBRATION
#define HOURGLASS_CORNER_CALIBRATION 0
#endif

// Default: confirmed TMB12A05 active buzzer, controlled through an NPN driver.
// 1: HIGH/LOW control; 0: optional passive buzzer with tone().
// D5 is a HIGH-active control signal to an NPN low-side driver, not buzzer power.
#ifndef HOURGLASS_ACTIVE_BUZZER
#define HOURGLASS_ACTIVE_BUZZER 1
#endif

const uint8_t PIN_DIN = 11;
const uint8_t PIN_CLK = 13;
const uint8_t PIN_CS = 10;
const uint8_t PIN_TILT = 2;
const uint8_t PIN_START_PAUSE = 3;
const uint8_t PIN_RESET = 4;
const uint8_t PIN_BUZZER = 5;
const unsigned int BUZZER_FREQUENCY_HZ = 2000;
const uint32_t BUZZER_ON_MS = 120UL;
const uint32_t BUZZER_GAP_MS = 100UL;
const uint8_t MODULE_COUNT = 2;
const uint8_t GRAIN_COUNT = 36;
const uint32_t DURATION_MS = 60000UL;
const uint32_t FRAME_MS = 60UL;
const uint32_t BUTTON_DEBOUNCE_MS = 30UL;
const uint32_t TILT_DEBOUNCE_MS = 300UL;
static_assert(DURATION_MS > 0, "Timer duration must be positive.");

// Chain address 0 is the module whose DIN connects to UNO D11.
// Address 1 is the module whose DIN connects to address 0 DOUT.
// With the assembly in its reference orientation and D2 HIGH, this is the top.
// Change to 1 if the physical modules / switch mounting require it.
const uint8_t TOP_MODULE_WHEN_HIGH = 0;

// Per-module physical calibration: clockwise 90-degree turns, mirror X/Y.
// Logical (row 0, col 0) is the top corner; (7, 7) is the bottom corner.
// These defaults require verification on the two actual modules.
const uint8_t ROTATION_QUARTER_TURNS[MODULE_COUNT] = {0, 0};
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

DebouncedInput tilt(PIN_TILT, TILT_DEBOUNCE_MS);
DebouncedInput startPause(PIN_START_PAUSE, BUTTON_DEBOUNCE_MS);
DebouncedInput resetButton(PIN_RESET, BUTTON_DEBOUNCE_MS);

bool orientationHigh = true;
bool grains[MODULE_COUNT][8][8] = {};
uint8_t frame[MODULE_COUNT][8] = {};
uint8_t transferred = 0;
uint32_t elapsedMs = 0;
uint32_t lastClockMs = 0;
uint32_t lastFrameMs = 0;
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
  return orientationHigh ? TOP_MODULE_WHEN_HIGH : (1 - TOP_MODULE_WHEN_HIGH);
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
  // The same physical assembly is inverted 180 degrees when the switch changes.
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

void renderGrains() {
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

void reportStatus(const char* reason) {
  const uint32_t remainingMs = DURATION_MS - elapsedMs;
  Serial.print(reason);
  Serial.print(" | state="); Serial.print(stateName());
  Serial.print(" elapsed_ms="); Serial.print(elapsedMs);
  Serial.print(" remaining_s="); Serial.print((remainingMs + 999UL) / 1000UL);
  Serial.print(" top_addr="); Serial.print(upperModule());
  Serial.print(" grains_near/far="); Serial.print(grainCount(0));
  Serial.print('/'); Serial.print(grainCount(1));
  Serial.print(" D2/D3/D4="); Serial.print(digitalRead(PIN_TILT));
  Serial.print('/'); Serial.print(digitalRead(PIN_START_PAUSE));
  Serial.print('/'); Serial.println(digitalRead(PIN_RESET));
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

void settleModule(uint8_t module) {
  // Down is toward logical (7,7); each pixel moves at most once per frame.
  // Descending diagonals prevent a just-moved grain from being visited again.
  for (int8_t diagonal = 14; diagonal >= 0; --diagonal) {
    const bool reverse = randomChoice();
    for (uint8_t index = 0; index < 8; ++index) {
      const uint8_t row = reverse ? (7 - index) : index;
      const int8_t col = diagonal - row;
      if (col < 0 || col > 7 || !grains[module][row][col]) continue;
      uint8_t nextRow = row;
      uint8_t nextCol = uint8_t(col);
      if (row < 7 && col < 7 && !grains[module][row + 1][col + 1]) {
        nextRow = row + 1;
        nextCol = col + 1;
      } else {
        const bool canRow = row < 7 && !grains[module][row + 1][col];
        const bool canCol = col < 7 && !grains[module][row][col + 1];
        if (canRow && (!canCol || randomChoice())) ++nextRow;
        else if (canCol) ++nextCol;
        else continue;
      }
      grains[module][row][col] = false;
      grains[module][nextRow][nextCol] = true;
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
      Serial.println("ERROR: grain transfer failed; reset and inspect.");
      state = RunState::PAUSED;
      return;
    }
  }
}

void advanceClock(uint32_t now) {
  const uint32_t delta = uint32_t(now - lastClockMs);
  lastClockMs = now;  // Also updated while idle/paused, so those intervals never count.
  if (state != RunState::RUNNING) return;
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
  if (!tilt.ready()) {
    Serial.println("Orientation settling: wait briefly, then press B1 again.");
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

void setup() {
  Serial.begin(115200);  // USB serial is optional; never block waiting for a monitor.
  Serial.println("ELEC281 v3.2: state feedback / D5 start and completion cues");
  const uint32_t now = uint32_t(millis());
  tilt.begin(now);
  startPause.begin(now);
  resetButton.begin(now);
  orientationHigh = (tilt.value() == HIGH);
  initialiseBuzzer();
  initialiseDisplays();
  restartCycle(RunState::READY, now, "Power-up");
  if (HOURGLASS_CORNER_CALIBRATION) {
    renderCalibration();
    Serial.println("CALIBRATION: 1 pixel = top; 2 pixels = bottom; timer disabled.");
  } else {
    renderGrains();
    Serial.println("B1 start/pause/resume; B2 reset; stable flip restarts (paused stays paused).");
  }
}

void loop() {
  const uint32_t now = uint32_t(millis());
  if (HOURGLASS_CORNER_CALIBRATION) {
    if (uint32_t(now - lastStatusMs) >= 500UL) {
      lastStatusMs = now;
      reportStatus("Calibration inputs");
    }
    return;
  }

  advanceClock(now);
  const bool tiltWasReady = tilt.ready();
  const bool tiltChanged = tilt.update(now);
  const bool b1Changed = startPause.update(now);
  const bool b2Changed = resetButton.update(now);
  const bool b1Pressed = b1Changed && startPause.value() == LOW;
  const bool b2Pressed = b2Changed && resetButton.value() == LOW;

  if (!tiltWasReady && tilt.ready()) {
    orientationHigh = (tilt.value() == HIGH);
    restartCycle(RunState::READY, now, "Orientation baseline settled");
  } else if (tiltChanged) {
    orientationHigh = (tilt.value() == HIGH);
    const RunState next = (state == RunState::PAUSED) ? RunState::PAUSED : RunState::RUNNING;
    restartCycle(next, now, "Stable flip");
  }

  // Reset wins if both button presses are accepted in the same loop.
  if (b2Pressed) {
    // Cancel any not-yet-accepted orientation change and settle a fresh baseline.
    // Reset remains READY rather than being undone by a stale delayed flip.
    tilt.begin(now);
    orientationHigh = (tilt.value() == HIGH);
    restartCycle(RunState::READY, now, "Reset");
  } else if (b1Pressed) handleStartPause(now);

  serviceBuzzer(now);

  if (uint32_t(now - lastFrameMs) >= FRAME_MS) {
    lastFrameMs = now;
    if (state == RunState::RUNNING || state == RunState::DONE) {
      settleModule(0);
      settleModule(1);
    }
    renderGrains();  // PAUSED freezes the grains; DONE keeps all 36 in the bottom.
  }
  if (uint32_t(now - lastStatusMs) >= 1000UL) {
    lastStatusMs = now;
    reportStatus("Status");
  }
}
