// Independent UI-to-hardware-output tests for all six timer presets.
// v3.9 migrates duration labels to the separate four-module clock.
// Every scenario runs the unchanged full sketch with physical active-low contacts,
// real debounce, a register-level MPU model and captured MAX7219/D5 outputs.
#define main regression_main
#include "host_tests.cpp"
#undef main
#include <set>

static const uint32_t expectedDurations[6] = {
  30000UL, 60000UL, 120000UL, 180000UL, 300000UL, 600000UL
};

static std::string matrixImage() {
  std::string image;
  for (unsigned module = 0; module < 4; ++module)
    for (unsigned row = 1; row <= 8; ++row) image.push_back(char(clockRegs[module][row]));
  return image;
}

static void enterSettings() {
  require(state == RunState::READY || state == RunState::DONE,
          "test must enter menu from an idle state");
  const unsigned n = onCount();
  buttonDown(PIN_RESET);
  advance(2050);
  require(settingMode && state == RunState::READY && elapsedMs == 0,
          "2s physical red hold from idle must enter a frozen settings menu");
  require(!buzzerLevel && onCount() == n, "settings entry must not beep");
  buttonUp(PIN_RESET);
}

static void editTo(unsigned index) {
  require(settingMode, "preset selection requires settings menu");
  unsigned attempts = 0;
  while (editingDurationIndex != index && attempts++ < 6) press(PIN_START_PAUSE);
  require(editingDurationIndex == index, "physical green presses did not reach requested preset");
  advance(80);  // Inspect a flushed MAX frame, rather than a not-yet-drawn buffer.
}

static void selectPreset(unsigned index) {
  enterSettings();
  editTo(index);
  press(PIN_RESET);
  require(!settingMode && state == RunState::READY && elapsedMs == 0,
          "red confirmation must return to READY without starting");
  require(selectedDurationIndex == index && DURATION_MS == expectedDurations[index],
          "committed timer does not match the physical UI selection");
  require(!buzzerLevel && onCount() == 0, "setting confirmation must stay quiet");
}

static void verifyPresetCompletion(unsigned index) {
  boot();
  selectPreset(index);
  running();
  require(DURATION_MS == expectedDurations[index] && onCount() == 1,
          "starting changed the preset or omitted the single start beep");
  const unsigned top = upperModule();
  advance(expectedDurations[index] / 2 - elapsedMs, 10);
  require(state == RunState::RUNNING && elapsedMs == expectedDurations[index] / 2,
          "selected preset reaches the wrong half-time");
  require(transferred == 18 && grainCount(top) == 18 && grainCount(1-top) == 18,
          "half of selected active duration must move exactly 18 grains");
  advance(expectedDurations[index] - elapsedMs - 1, 10);
  require(state == RunState::RUNNING && elapsedMs == expectedDurations[index] - 1 && transferred == 35,
          "preset finished early or did not preserve last grain until the deadline");
  tick(1);
  require(state == RunState::DONE && elapsedMs == expectedDurations[index] && transferred == 36,
          "preset did not finish at the exact active-time deadline");
  require(grainCount(top) == 0 && grainCount(1-top) == 36,
          "selected preset did not transfer all 36 grains");
  const uint32_t onset = fakeNow;
  const uint32_t boundaries[] = {0, 119, 120, 219, 220, 339, 340, 439, 440, 559, 560};
  for (const uint32_t age : boundaries) {
    advance(age - uint32_t(fakeNow-onset));
    const bool on = age < 560 && age % 220 < 120;
    require(buzzerLevel == on && pixels(1-top) == (on ? 0U : 36U),
            "selected-preset finish flashes do not follow the three physical beeps");
  }
  require(onCount() == 4 && !buzzerLevel, "selected preset needs one start plus three finish beeps");
  advance(1500, 10);
  require(onCount() == 4 && pixels(1-top) == 36 && blockingDelayMs == 0,
          "finish replayed, left a blank pile or introduced a blocking delay");
}

static void menuAndDisplay() {
  boot();
  require(selectedDurationIndex == 1 && DURATION_MS == 60000,
          "power-on default must remain one minute");
  enterSettings();
  require(editingDurationIndex == 1, "menu should initially show the committed duration");
  std::set<std::string> observed;
  for (unsigned i = 0; i < 6; ++i) {
    const unsigned expected = (1+i) % 6;
    require(editingDurationIndex == expected, "green preset order differs from the six offered choices");
    advance(80);
    const std::string image = matrixImage();
    require(clockDisplayLatches > 0 && image != std::string(32, char(0)),
            "time choice must be visible on the separate four-module MAX clock");
    require(grainCount(upperModule()) == 36 && grainCount(1-upperModule()) == 0,
            "settings must retain the supported sand pile instead of replacing it with labels");
    require(observed.insert(image).second, "two different time choices produced identical actual MAX frames");
    advance(1200, 10);
    require(matrixImage() == image, "settings time label must not animate or disappear while idle");
    require(elapsedMs == 0 && transferred == 0 && onCount() == 0,
            "settings display advanced grains/time or emitted a timer cue");
    press(PIN_START_PAUSE);
  }
  require(editingDurationIndex == 1 && selectedDurationIndex == 1 && DURATION_MS == 60000,
          "one full green cycle should wrap without committing interim choices");
  buttonDown(PIN_START_PAUSE);
  const unsigned heldIndex = editingDurationIndex;
  advance(3000, 10);
  require(editingDurationIndex == heldIndex, "holding green repeated preset changes");
  buttonUp(PIN_START_PAUSE);
}

static void longHoldBoundary() {
  boot();
  buttonDown(PIN_RESET);  // Accepted after 30ms; current accepted hold age is 1ms.
  advance(500);
  acceleration(-1,0,0);  // A stable inversion while the idle long press is pending.
  advance(1498);
  require(!settingMode && state == RunState::READY && onCount() == 0,
          "pending 2s idle long-press must suppress flip auto-start and not enter menu early");
  tick(1);
  require(settingMode, "red hold failed to enter at 2000ms across the actual debounce path");
  const unsigned index = editingDurationIndex;
  advance(4000, 10);
  require(settingMode && editingDurationIndex == index && onCount() == 0,
          "held entry red contact confirmed/re-entered/repeated menu actions");
  buttonUp(PIN_RESET);
  buttonDown(PIN_RESET);
  require(!settingMode && state == RunState::READY,
          "a fresh red press after release must confirm menu");
  advance(2500, 10);
  require(!settingMode && state == RunState::READY && onCount() == 0,
          "holding the confirming red press for >2s must not reopen settings");
  buttonUp(PIN_RESET);
  enterSettings();
  press(PIN_RESET);
  buttonDown(PIN_START_PAUSE);
  require(state == RunState::RUNNING && !settingMode && onCount() == 1,
          "post-confirmation label preview must not block an immediate green start");
}

static void noAccidentalRunningMenu() {
  boot(); running(); advance(1234);
  buttonDown(PIN_RESET);
  require(state == RunState::READY && elapsedMs == 0 && !buzzerLevel,
          "running red press must retain immediate quiet reset");
  advance(4000, 10);
  require(!settingMode, "the same red press that reset RUNNING must not subsequently open settings");
  buttonUp(PIN_RESET);
  enterSettings();
  press(PIN_RESET);
  running(); advance(1234); paused();
  buttonDown(PIN_RESET); advance(4000, 10);
  require(!settingMode && state == RunState::READY && elapsedMs == 0,
          "the same red press that reset PAUSED must not subsequently open settings");
}

static void pauseHoldResetFlipPreserve() {
  boot(); selectPreset(5); running(); advance(5123, 10);
  paused(); const uint32_t frozen = elapsedMs; const unsigned beeps = onCount();
  advance(2000, 10);
  require(elapsedMs == frozen && DURATION_MS == 600000 && selectedDurationIndex == 5,
          "manual pause changed time selection or counted paused wall time");
  press(PIN_START_PAUSE);
  require(state == RunState::RUNNING && onCount() == beeps, "preset resume replayed a start cue");
  acceleration(0,1,0); advance(1200, 10);
  require(sensorHold, "preset cycle lost horizontal automatic pause");
  const uint32_t held = elapsedMs; advance(2000, 10);
  require(elapsedMs == held, "horizontal hold advanced selected-duration clock");
  acceleration(1,0,0); advance(1200, 10);
  require(!sensorHold && state == RunState::RUNNING && elapsedMs > held && onCount() == beeps,
          "upright recovery must quietly continue selected duration");
  press(PIN_RESET); advance(1000, 10);
  require(state == RunState::READY && elapsedMs == 0 && DURATION_MS == 600000 && selectedDurationIndex == 5,
          "reset must retain committed ten-minute selection");
  running(); advance(1700, 10);
  const unsigned oldTop = upperModule(), n = onCount();
  acceleration(-1,0,0); advance(1200, 10);
  require(upperModule() == 1-oldTop && state == RunState::RUNNING && elapsedMs < 1200,
          "selected-duration stable flip did not create a fresh cycle");
  require(DURATION_MS == 600000 && selectedDurationIndex == 5 && onCount() == n+1,
          "flip changed duration or omitted one new-cycle start cue");
}

static void menuSensorIsolation() {
  boot(0,0,1);
  require(sensorHold && !imu.ready(), "scenario needs a flat initial sensor");
  enterSettings(); editTo(0);
  Wire.present = false; advance(1500, 10);
  require(settingMode && elapsedMs == 0 && onCount() == 0,
          "missing IMU must not exit settings, advance timer or make noise");
  Wire.present = true; acceleration(-1,0,0); advance(2000, 10);
  acceleration(1,0,0); advance(1200, 10);
  require(settingMode && state == RunState::READY && elapsedMs == 0 && onCount() == 0,
          "IMU baseline/flip while editing must not auto-start or abandon menu");
  press(PIN_RESET);
  require(!settingMode && DURATION_MS == 30000 && selectedDurationIndex == 0,
          "sensor recovery lost the edited menu choice");
  advance(1000, 10); running();
}

static void simultaneousAndBounce() {
  boot(); enterSettings(); editTo(4);
  input(PIN_START_PAUSE, LOW); input(PIN_RESET, LOW); advance(31);
  require(!settingMode && state == RunState::READY && selectedDurationIndex == 4 && DURATION_MS == 300000,
          "red must confirm original editing selection before simultaneous green");
  require(onCount() == 0, "simultaneous confirm/green accidentally started timer");
  advance(1000, 10);
  require(state == RunState::READY && !settingMode && onCount() == 0,
          "held simultaneous contacts triggered a delayed green start");
  buttonUp(PIN_START_PAUSE); buttonUp(PIN_RESET); enterSettings();
  const unsigned old = editingDurationIndex;
  for (unsigned i = 0; i < 6; ++i) {
    input(PIN_START_PAUSE, LOW); advance(6);
    input(PIN_START_PAUSE, HIGH); advance(6);
  }
  advance(40);
  require(editingDurationIndex == old, "sub-debounce contact chatter changed preset");
  press(PIN_START_PAUSE);
  require(editingDurationIndex == (old+1)%6, "one debounced green press did not make one selection step");
}

static void redHeldAtBoot() {
  fakeNow = 0; std::memset(fakePins, HIGH, sizeof(fakePins));
  fakePins[PIN_RESET] = LOW; acceleration(1,0,0); setup(); advance(5000, 10);
  require(!settingMode && state == RunState::READY && DURATION_MS == 60000 && onCount() == 0,
          "red held during power-on must not become a phantom long press");
  buttonUp(PIN_RESET); enterSettings();
}

static void tenMinuteTimerAcrossWrap() {
  boot(1,0,0,0xfffe0000UL);
  selectPreset(5);
  running();
  const uint32_t startedAt = fakeNow;
  advance(600000-elapsedMs-1, 10);
  require(fakeNow < startedAt && state == RunState::RUNNING && elapsedMs == 599999 && transferred == 35,
          "ten-minute preset did not safely count through millis rollover");
  tick(1);
  require(state == RunState::DONE && elapsedMs == 600000 && transferred == 36,
          "ten-minute preset finished at the wrong selected deadline after rollover");
  exactFinishSequence(fakeNow, 4);
}

static void settingsAndDurationAcrossWrap() {
  boot(1,0,0,0xfffff500UL);
  selectPreset(0);
  require(fakeNow < 10000, "scenario did not carry red long-press through millis rollover");
  running(); advance(30000-elapsedMs-1, 10);
  require(state == RunState::RUNNING && transferred == 35, "30s selected deadline fails after rollover");
  tick(1);
  require(state == RunState::DONE && elapsedMs == 30000 && transferred == 36,
          "30s selected timer did not complete after menu rollover");
  exactFinishSequence(fakeNow, 4);
}

int main(int argc, char** argv) {
  const std::map<std::string, std::function<void()>> cases = {
    {"preset_30_seconds", [] { verifyPresetCompletion(0); }},
    {"preset_1_minute", [] { verifyPresetCompletion(1); }},
    {"preset_2_minutes", [] { verifyPresetCompletion(2); }},
    {"preset_3_minutes", [] { verifyPresetCompletion(3); }},
    {"preset_5_minutes", [] { verifyPresetCompletion(4); }},
    {"preset_10_minutes", [] { verifyPresetCompletion(5); }},
    {"menu_actual_frames_and_wrap", menuAndDisplay},
    {"red_2000ms_and_immediate_start", longHoldBoundary},
    {"running_paused_long_red_only_resets", noAccidentalRunningMenu},
    {"pause_hold_reset_flip_preserve_10m", pauseHoldResetFlipPreserve},
    {"menu_sensor_missing_flat_and_flip", menuSensorIsolation},
    {"menu_simultaneous_and_bounce", simultaneousAndBounce},
    {"red_held_at_boot", redHeldAtBoot},
    {"menu_timer_millis_rollover", settingsAndDurationAcrossWrap},
    {"ten_minute_timer_millis_rollover", tenMinuteTimerAcrossWrap}
  };
  if (argc != 2) { for (const auto& c : cases) std::cout << c.first << '\n'; return 2; }
  try {
    const auto it = cases.find(argv[1]); require(it != cases.end(), "unknown preset scenario");
    it->second();
    std::cout << "PASS " << argv[1] << " | elapsed_ms=" << elapsedMs
              << " duration_ms=" << DURATION_MS << " D5_ons=" << onCount()
              << " MAX_latches=" << displayLatches << " I2C_reads=" << Wire.readRequests << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL " << argv[1] << ": " << e.what() << " | now=" << fakeNow
              << " elapsed=" << elapsedMs << " state=" << int(state)
              << " menu=" << settingMode << " selected=" << int(selectedDurationIndex)
              << " editing=" << int(editingDurationIndex) << " hold=" << sensorHold << '\n';
    std::cerr << Serial.str().substr(Serial.str().size()>2500?Serial.str().size()-2500:0);
    return 1;
  }
}
