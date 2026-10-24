// Full-sketch tests of physical controls, active-time accounting, and the
// separate MAX7219 time display. Read digits from captured serial-bus registers,
// rather than accepting the driver's cached seconds or software canvas.
#define main regression_main
#include "host_tests.cpp"
#undef main
#include <set>

static const uint32_t presetMs[6] = {
  30000UL, 60000UL, 120000UL, 180000UL, 300000UL, 600000UL
};

static bool clockPixel(unsigned x, unsigned y) {
  // Physical chain runs right to left when the input header is on the right.
  const unsigned module = 3 - x / 8;
  return ((clockRegs[module][y + 1] >> (7 - x % 8)) & 1) != 0;
}

static unsigned digitAt(unsigned x) {
  // Human-readable, independent reference pictures for the documented 5x7 font.
  static const char* const glyphs[10][7] = {
    {".###.", "#...#", "#..##", "#.#.#", "##..#", "#...#", ".###."},
    {"..#..", ".##..", "..#..", "..#..", "..#..", "..#..", ".###."},
    {".###.", "#...#", "....#", "...#.", "..#..", ".#...", "#####"},
    {"####.", "....#", "....#", ".###.", "....#", "....#", "####."},
    {"...#.", "..##.", ".#.#.", "#..#.", "#####", "...#.", "...#."},
    {"#####", "#....", "#....", "####.", "....#", "....#", "####."},
    {".###.", "#....", "#....", "####.", "#...#", "#...#", ".###."},
    {"#####", "....#", "...#.", "..#..", ".#...", ".#...", ".#..."},
    {".###.", "#...#", "#...#", ".###.", "#...#", "#...#", ".###."},
    {".###.", "#...#", "#...#", ".####", "....#", "....#", ".###."}
  };
  for (unsigned d = 0; d < 10; ++d) {
    bool equal = true;
    for (unsigned y = 0; y < 7; ++y)
      for (unsigned col = 0; col < 5; ++col)
        if (clockPixel(x + col, y) != (glyphs[d][y][col] == '#')) equal = false;
    if (equal) return d;
  }
  throw std::runtime_error("actual four-module MAX rows contain an unreadable time digit");
}

static char clockMode() {
  static const char modes[] = {'S', 'T', 'R', 'P', 'H', 'E', 'D'};
  static const char* const glyphs[7][5] = {
    {"###", "#..", "###", "..#", "###"},
    {"###", ".#.", ".#.", ".#.", ".#."},
    {"##.", "#.#", "##.", "#.#", "#.#"},
    {"##.", "#.#", "##.", "#..", "#.."},
    {"#.#", "#.#", "###", "#.#", "#.#"},
    {"###", "#..", "##.", "#..", "###"},
    {"##.", "#.#", "#.#", "#.#", "##."}
  };
  for (unsigned m = 0; m < 7; ++m) {
    bool equal = true;
    for (unsigned y = 0; y < 5; ++y)
      for (unsigned x = 0; x < 3; ++x)
        if (clockPixel(x, y + 1) != (glyphs[m][y][x] == '#')) equal = false;
    if (equal) return modes[m];
  }
  throw std::runtime_error("actual four-module MAX rows contain an unreadable mode glyph");
}

static uint16_t clockSeconds() {
  const unsigned minutes = 10 * digitAt(4) + digitAt(10);
  const unsigned seconds = 10 * digitAt(18) + digitAt(24);
  require(seconds < 60, "actual time display has a seconds field beyond 59");
  require(clockPixel(16, 2) && clockPixel(16, 4), "MM:SS separator is absent");
  for (unsigned y = 0; y < 8; ++y)
    require(clockPixel(16, y) == (y == 2 || y == 4), "colon has stray pixels");
  return uint16_t(minutes * 60 + seconds);
}

static void expectClock(uint16_t seconds, char mode) {
  require(clockDisplayLatches > 0, "new display never received a hardware latch");
  const uint16_t actual = clockSeconds();
  require(actual == seconds, "actual clock seconds " + std::to_string(actual) +
          " differ from expected " + std::to_string(seconds));
  const char actualMode = clockMode();
  require(actualMode == mode, std::string("actual clock mode ") + actualMode +
          " differs from expected " + mode);
}

static std::string clockImage() {
  std::string result;
  for (unsigned m = 0; m < 4; ++m)
    for (unsigned row = 1; row <= 8; ++row) result.push_back(char(clockRegs[m][row]));
  return result;
}

static uint16_t remainingSeconds() {
  return uint16_t((DURATION_MS - elapsedMs + 999UL) / 1000UL);
}

static void openSettings() {
  require(state == RunState::READY || state == RunState::DONE, "menu entry must start idle");
  buttonDown(PIN_RESET); advance(2050); buttonUp(PIN_RESET);
  require(settingMode && state == RunState::READY, "physical red hold did not enter settings");
}

static void choose(unsigned index) {
  openSettings();
  for (unsigned i = 0; editingDurationIndex != index && i < 6; ++i) press(PIN_START_PAUSE);
  require(editingDurationIndex == index, "six-choice green cycle cannot reach requested time");
  expectClock(uint16_t(presetMs[index] / 1000), 'S');
  press(PIN_RESET);
  require(!settingMode && selectedDurationIndex == index && DURATION_MS == presetMs[index],
          "red confirmation lost the selected time");
  expectClock(uint16_t(presetMs[index] / 1000), 'T');
}

static void allChoicesVisibleAndSandRetained() {
  boot(); expectClock(60, 'T');
  openSettings(); expectClock(60, 'S');
  bool original[2][8][8]; std::memcpy(original, grains, sizeof(original));
  std::set<std::string> distinct;
  for (unsigned n = 0; n < 6; ++n) {
    const unsigned index = (n + 1) % 6;
    require(editingDurationIndex == index, "display and six-choice UI diverged");
    expectClock(uint16_t(presetMs[index] / 1000), 'S');
    require(distinct.insert(clockImage()).second, "different presets have the same physical clock image");
    require(std::memcmp(original, grains, sizeof(original)) == 0,
            "setting time changed the logical sand instead of retaining its pile");
    require(pixels(upperModule()) >= 36 && pixels(1-upperModule()) == 0,
            "time selection replaced hourglass sand with numeric labels");
    const std::string frame = clockImage();
    advance(1250, 10);
    require(clockImage() == frame && elapsedMs == 0 && transferred == 0 && onCount() == 0,
            "idle setting label changed, counted time, or made a timer cue");
    press(PIN_START_PAUSE);
  }
  expectClock(60, 'S'); press(PIN_RESET); expectClock(60, 'T');
  advance(5000, 10); expectClock(60, 'T');
}

static void countdownCeilingAndExactEnd() {
  boot(); choose(0);
  // Stop exactly on the accepted contact edge: the ordinary press() helper
  // advances another 32ms after start and would miss the elapsed==0 boundary.
  input(PIN_START_PAUSE, LOW); advance(30);
  require(state == RunState::RUNNING && elapsedMs == 0, "green acceptance did not start exactly now");
  expectClock(30, 'R'); tick(1); expectClock(30, 'R');
  advance(999); require(elapsedMs == 1000, "scenario missed 1s active boundary"); expectClock(29, 'R');
  input(PIN_START_PAUSE, HIGH); advance(31);
  advance(29999 - elapsedMs, 10); expectClock(1, 'R');
  require(state == RunState::RUNNING && transferred == 35, "timer completed before the last millisecond");
  tick(1); expectClock(0, 'D');
  require(state == RunState::DONE && elapsedMs == 30000 && transferred == 36,
          "clock zero and hourglass completion did not share the exact active-time deadline");
}

static void pauseAndAutomaticHoldFreeze() {
  boot(); choose(0); running(); advance(3210); paused();
  const uint32_t pausedAt = elapsedMs;
  const unsigned n = onCount();
  expectClock(remainingSeconds(), 'P'); const std::string manual = clockImage();
  advance(5000, 10);
  require(elapsedMs == pausedAt && clockImage() == manual && onCount() == n,
          "manual pause did not freeze the countdown quietly");
  press(PIN_START_PAUSE); expectClock(remainingSeconds(), 'R');
  acceleration(0, 1, 0); advance(1200, 10);
  require(sensorHold && state == RunState::RUNNING, "scenario failed to enter horizontal hold");
  expectClock(remainingSeconds(), 'H');
  const uint32_t heldAt = elapsedMs; const std::string automatic = clockImage();
  advance(5000, 10);
  require(elapsedMs == heldAt && clockImage() == automatic && onCount() == n,
          "horizontal hold did not freeze countdown and show H quietly");
  paused(); expectClock(remainingSeconds(), 'P');
  acceleration(1, 0, 0); advance(1200, 10);
  require(state == RunState::PAUSED && elapsedMs == heldAt,
          "restoring upright bypassed manual pause");
  expectClock(remainingSeconds(), 'P');
  press(PIN_START_PAUSE); advance(1000); expectClock(remainingSeconds(), 'R');
  require(elapsedMs > heldAt && onCount() == n, "resume restarted or replayed a start cue");
}

static void resetAndFlipRestoreSelectedTotal() {
  boot(); choose(5); running(); advance(5310, 10); expectClock(remainingSeconds(), 'R');
  const unsigned oldTop = upperModule(), n = onCount();
  acceleration(-1, 0, 0); advance(1200, 10);
  require(upperModule() != oldTop && state == RunState::RUNNING && elapsedMs < 1200,
          "stable flip did not start a fresh cycle");
  require(DURATION_MS == 600000 && selectedDurationIndex == 5 && onCount() == n + 1,
          "stable flip changed the selected total or failed to cue the new cycle");
  expectClock(remainingSeconds(), 'R');
  press(PIN_RESET); advance(1200, 10);
  require(state == RunState::READY && elapsedMs == 0 && DURATION_MS == 600000,
          "red reset failed to restore the selected ten-minute total");
  expectClock(600, 'T'); advance(3000, 10); expectClock(600, 'T');
}

static void sensorFaultPreservesRemainingTime() {
  boot(); choose(0); running(); advance(5432, 10);
  Wire.present = false; advance(80);
  require(!imu.healthy() && sensorHold, "scenario failed to expose missing sensor");
  const uint32_t frozen = elapsedMs; const unsigned n = onCount();
  expectClock(remainingSeconds(), 'E'); const std::string failed = clockImage();
  advance(5000, 10);
  require(elapsedMs == frozen && clockImage() == failed && onCount() == n,
          "fault display lost remaining time or advanced the timer during missing IMU");
  Wire.present = true; acceleration(1, 0, 0); advance(1900, 10);
  require(imu.healthy() && imu.ready() && !sensorHold && state == RunState::RUNNING,
          "valid sensor did not recover the existing running cycle");
  require(elapsedMs >= frozen && elapsedMs < frozen + 1900 && onCount() == n,
          "sensor recovery restarted, counted all fault wall time, or replayed a start cue");
  expectClock(remainingSeconds(), 'R');
}

static void settingDisplaySurvivesMissingSensor() {
  boot(0, 0, 1);  // The clock remains usable while the IMU is face-up/held.
  openSettings(); expectClock(60, 'S');
  press(PIN_START_PAUSE); expectClock(120, 'S');
  Wire.present = false; advance(1500, 10);
  require(settingMode && !imu.healthy() && elapsedMs == 0 && onCount() == 0,
          "missing IMU abandoned the setting interaction");
  expectClock(120, 'S');
  press(PIN_START_PAUSE); expectClock(180, 'S');
  Wire.present = true; acceleration(-1, 0, 0); advance(2200, 10);
  acceleration(1, 0, 0); advance(1200, 10);
  require(settingMode && state == RunState::READY && elapsedMs == 0 && onCount() == 0,
          "sensor recovery/inversion started a timer during time selection");
  expectClock(180, 'S'); press(PIN_RESET); expectClock(180, 'T');
  require(selectedDurationIndex == 3 && DURATION_MS == 180000,
          "sensor failure/recovery discarded the edited three-minute time");
}

static void finishFlashesLeaveClockVisible() {
  boot(); choose(0); running(); advance(30000 - elapsedMs, 10);
  require(state == RunState::DONE, "scenario did not reach DONE");
  const uint32_t onset = fakeNow;
  const unsigned bottom = 1 - upperModule();
  expectClock(0, 'D'); const std::string complete = clockImage();
  const uint32_t samples[] = {0,119,120,219,220,339,340,439,440,559,560,1500};
  for (const uint32_t age : samples) {
    advance(age - uint32_t(fakeNow - onset));
    const bool on = age < 560 && age % 220 < 120;
    require(buzzerLevel == on && pixels(bottom) == (on ? 0U : 36U),
            "hourglass finish flash no longer follows the three buzzer pulses");
    require(clockImage() == complete, "completion blink incorrectly blanked or flashed the time display");
    expectClock(0, 'D');
  }
  require(onCount() == 4 && !buzzerLevel && blockingDelayMs == 0,
          "completion added an extra beep or blocking delay");
}

static void displayBusesAreIndependent() {
  boot(); expectClock(60, 'T');
  const std::string oldClock = clockImage();
  const unsigned clockLatches = clockDisplayLatches, sandLatches = displayLatches;
  advance(1500, 10);
  require(displayLatches > sandLatches && clockDisplayLatches == clockLatches && clockImage() == oldClock,
          "hourglass redraw traffic mutated or needlessly clocked the separate static time display");
  bool sand[2][8][8]; std::memcpy(sand, grains, sizeof(sand));
  const unsigned sandBeforeClock = displayLatches;
  AuxClock::service(599, 'P');
  expectClock(599, 'P');
  require(std::memcmp(sand, grains, sizeof(sand)) == 0 && displayLatches == sandBeforeClock,
          "clock drawing mutated sand or accidentally latched the hourglass bus");
  invariants();
  tick(0); expectClock(60, 'T');
}

static void rolloverCountdown() {
  boot(1, 0, 0, 0xfffff500UL); choose(0); running();
  const uint32_t start = fakeNow;
  advance(29999 - elapsedMs, 10);
  require(fakeNow < start || start < 10000, "scenario failed to cross the intended millis rollover");
  expectClock(1, 'R'); tick(1); expectClock(0, 'D');
  require(state == RunState::DONE && elapsedMs == 30000, "clock changes broke rollover-safe timing");
}

int main(int argc, char** argv) {
  const std::map<std::string, std::function<void()>> cases = {
    {"six_choices_and_sand", allChoicesVisibleAndSandRetained},
    {"countdown_ceil_exact_end", countdownCeilingAndExactEnd},
    {"manual_and_horizontal_freeze", pauseAndAutomaticHoldFreeze},
    {"reset_and_flip_selected_total", resetAndFlipRestoreSelectedTotal},
    {"fault_freezes_time_with_error", sensorFaultPreservesRemainingTime},
    {"settings_fault_recovery_stays_visible", settingDisplaySurvivesMissingSensor},
    {"finish_flash_clock_steady", finishFlashesLeaveClockVisible},
    {"independent_max_buses", displayBusesAreIndependent},
    {"countdown_millis_rollover", rolloverCountdown}
  };
  if (argc != 2) { for (const auto& c : cases) std::cout << c.first << '\n'; return 2; }
  try {
    const auto it = cases.find(argv[1]); require(it != cases.end(), "unknown clock scenario");
    it->second();
    std::cout << "PASS " << argv[1] << " | elapsed_ms=" << elapsedMs
              << " duration_ms=" << DURATION_MS << " D5_ons=" << onCount()
              << " sand_latches=" << displayLatches << " clock_latches=" << clockDisplayLatches << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL " << argv[1] << ": " << e.what() << " | now=" << fakeNow
              << " elapsed=" << elapsedMs << " state=" << int(state)
              << " menu=" << settingMode << " hold=" << sensorHold << '\n';
    return 1;
  }
}
