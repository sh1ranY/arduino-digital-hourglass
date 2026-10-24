// Additional recovery regressions execute the complete sketch and inspect the
// latched MAX7219 mode glyph, real timer/grain state and actual buzzer edges.
#define main preexisting_regression_main
#include "host_tests.cpp"
#undef main

static char recoveryClockMode() {
  static const char modes[] = {'S', 'T', 'R', 'P', 'H', 'E', 'D'};
  static const char* const rows[7][5] = {
    {"###", "#..", "###", "..#", "###"},
    {"###", ".#.", ".#.", ".#.", ".#."},
    {"##.", "#.#", "##.", "#.#", "#.#"},
    {"##.", "#.#", "##.", "#..", "#.."},
    {"#.#", "#.#", "###", "#.#", "#.#"},
    {"###", "#..", "##.", "#..", "###"},
    {"##.", "#.#", "#.#", "#.#", "##."}
  };
  for (unsigned m = 0; m < 7; ++m) {
    bool same = true;
    for (unsigned y = 0; y < 5; ++y)
      for (unsigned x = 0; x < 3; ++x) {
        const bool physical = ((clockRegs[3][y + 2] >> (7 - x)) & 1) != 0;
        if (physical != (rows[m][y][x] == '#')) same = false;
      }
    if (same) return modes[m];
  }
  throw std::runtime_error("latched clock mode is unreadable after recovery");
}

static uint32_t enterControllerFault() {
  Wire.controllerStuck = true;
  advance(60);
  require(!imu.healthy() && sensorHold, "controller fault did not freeze immediately");
  require(imu.ready(), "communication fault erased the accepted orientation baseline");
  require(recoveryClockMode() == 'E', "fault is not visible as E on the real display bus");
  const uint32_t frozen = elapsedMs;
  const unsigned transferredBefore = transferred;
  advance(200);
  require(elapsedMs == frozen && transferred == transferredBefore,
          "failed sensor advanced countdown or grain transfer");
  return frozen;
}

static void controllerLatchRecovery() {
  boot(); running(); advance(8400);
  const unsigned cues = onCount(), oldTop = upperModule();
  const unsigned ends = Wire.endCalls, resets = Wire.resetCommands;
  const uint32_t frozen = enterControllerFault();
  advance(3500);
  require(imu.healthy() && !sensorHold && state == RunState::RUNNING,
          "latched controller did not recover autonomously");
  require(Wire.endCalls > ends && Wire.beginCalls > 1 && Wire.resetCommands > resets,
          "recovery did not reinitialize the controller and reset the MPU");
  require(elapsedMs >= frozen && elapsedMs < frozen + 3500 && upperModule() == oldTop,
          "recovery restarted a running cycle or counted the fault interval");
  require(onCount() == cues && selectedDurationIndex == 1 && DURATION_MS == 60000,
          "same-pose recovery changed the duration or replayed a start sound");
  require(recoveryClockMode() == 'R', "display did not leave E for R after recovery");
}

static void deviceResetRecovery() {
  boot(); running(); advance(2500);
  const unsigned resets = Wire.resetCommands, cues = onCount();
  Wire.requiresDeviceReset = true;
  advance(60);
  require(sensorHold, "reset-required sensor did not enter fail-safe hold");
  const uint32_t frozen = elapsedMs;
  advance(4500);
  require(Wire.resetCommands > resets && imu.healthy() && !sensorHold,
          "MPU reset-required fault cannot recover");
  require(state == RunState::RUNNING && elapsedMs >= frozen && onCount() == cues,
          "MPU reset changed the active cycle or replayed a cue");
}

static void pausedRecovery() {
  boot(); running(); advance(8250); paused();
  const uint32_t frozen = elapsedMs;
  const uint8_t grainsBefore = transferred;
  const unsigned cues = onCount();
  enterControllerFault(); advance(3500);
  require(imu.healthy() && !sensorHold && state == RunState::PAUSED,
          "bus recovery released deliberate manual pause");
  require(elapsedMs == frozen && transferred == grainsBefore && onCount() == cues,
          "paused recovery changed time, grains or sound");
  require(recoveryClockMode() == 'P', "paused recovery did not restore P");
  running(); advance(1000);
  require(elapsedMs >= frozen + 1000 && onCount() == cues,
          "resume after bus recovery restarted or stayed frozen");
}

static void settingsRecovery() {
  boot();
  buttonDown(PIN_RESET); advance(2050); buttonUp(PIN_RESET);
  require(settingMode, "test did not enter settings through physical controls");
  press(PIN_START_PAUSE); press(PIN_START_PAUSE); press(PIN_START_PAUSE);
  const uint8_t selected = selectedDurationIndex, editing = editingDurationIndex;
  Wire.controllerStuck = true; advance(4000);
  require(imu.healthy() && !sensorHold && settingMode && state == RunState::READY,
          "sensor recovery interrupted time selection");
  require(selectedDurationIndex == selected && editingDurationIndex == editing &&
          elapsedMs == 0 && onCount() == 0 && recoveryClockMode() == 'S',
          "settings index, mode or quiet state changed during recovery");
  press(PIN_RESET);
  require(!settingMode && selectedDurationIndex == editing &&
          DURATION_MS == TIME_PRESETS_MS[editing],
          "post-recovery red confirmation lost the chosen preset");
}

static void doneRecovery() {
  boot(); running(); finishCycle(); advance(700);
  const unsigned cues = onCount();
  require(cues == 4, "test did not finish with three completion pulses");
  enterControllerFault(); advance(3500);
  require(imu.healthy() && !sensorHold && state == RunState::DONE &&
          elapsedMs == DURATION_MS && transferred == GRAIN_COUNT,
          "finished timer was restarted or lost its sand during recovery");
  require(onCount() == cues && !buzzerLevel && recoveryClockMode() == 'D',
          "DONE recovery replayed the completion cue or kept E");
}

static void healthyHoldDoesNotResetBus() {
  boot(); running(); advance(2100);
  const unsigned ends = Wire.endCalls, resets = Wire.resetCommands;
  acceleration(0, 1, 0); advance(15000, 10);
  require(imu.healthy() && sensorHold && recoveryClockMode() == 'H',
          "healthy horizontal hold has incorrect health/display state");
  require(Wire.endCalls == ends && Wire.resetCommands == resets,
          "normal horizontal hold unnecessarily reset sensor/controller");
  const uint32_t frozen = elapsedMs;
  acceleration(1, 0, 0); advance(1200);
  require(!sensorHold && state == RunState::RUNNING && elapsedMs >= frozen,
          "normal horizontal-hold recovery stopped working");
}

static void persistentAbsenceKeepsControls() {
  boot(); running(); advance(3100);
  const unsigned cues = onCount();
  Wire.present = false; advance(60);
  const uint32_t frozen = elapsedMs;
  const uint8_t grainsBefore = transferred;
  advance(15000, 10);
  require(!imu.healthy() && sensorHold && elapsedMs == frozen && transferred == grainsBefore,
          "persistent missing sensor lost its fail-safe frozen cycle");
  require(Wire.endCalls > 0 && Wire.endCalls <= 20 && recoveryClockMode() == 'E',
          "persistent recovery is unbounded or failed to expose E");
  paused(); require(state == RunState::PAUSED && elapsedMs == frozen,
                    "green control is unresponsive during persistent E");
  press(PIN_RESET); require(state == RunState::READY && elapsedMs == 0,
                          "red reset is unresponsive during persistent E");
  buttonDown(PIN_RESET); advance(2050); buttonUp(PIN_RESET);
  require(settingMode, "settings cannot open during persistent missing sensor");
  const uint8_t edit = editingDurationIndex; press(PIN_START_PAUSE);
  require(editingDurationIndex == (edit + 1) % TIME_PRESET_COUNT && onCount() == cues,
          "green preset control or quiet failure behavior regressed");
}

static void oppositeRecoveryRestartsOnce() {
  boot(); running(); advance(3100);
  const unsigned oldTop = upperModule(), cues = onCount();
  enterControllerFault(); acceleration(-1, 0, 0); advance(3500);
  require(imu.healthy() && !sensorHold && upperModule() != oldTop &&
          state == RunState::RUNNING && onCount() == cues + 1,
          "opposite-pose bus recovery did not accept one stable flip");
  const uint32_t freshElapsed = elapsedMs; advance(1500);
  require(elapsedMs == freshElapsed + 1500 && onCount() == cues + 1,
          "bus recovery repeatedly accepted the same inversion");
}

static void fullSerialDoesNotBlock() {
  Serial.capacity = 0;
  boot();
  require(Serial.str().empty(), "serial full test unexpectedly accepted telemetry");
  running(); const uint32_t start = fakeNow;
  const uint32_t remaining = DURATION_MS - elapsedMs;
  advance(remaining);
  require(state == RunState::DONE && uint32_t(fakeNow - start) == remaining &&
          elapsedMs == DURATION_MS && blockingDelayMs == 0,
          "full USB telemetry buffer stalled timing or added a blocking delay");
  exactFinishSequence(fakeNow, 4);
  require(Serial.str().empty(), "telemetry bypassed zero-capacity admission");
}

static void recoveryAcrossMillisWrap() {
  boot(1, 0, 0, uint32_t(0xffffffffUL - 4500UL));
  running(); advance(1800);
  const unsigned cues = onCount();
  const uint32_t frozen = enterControllerFault();
  advance(4000);
  require(fakeNow < 10000UL && imu.healthy() && !sensorHold &&
          state == RunState::RUNNING && elapsedMs >= frozen && onCount() == cues,
          "retry, warmup or settling failed across millis rollover");
}

int main(int argc, char** argv) {
  const std::map<std::string, std::function<void()>> cases = {
    {"controller_latch_preserves_running", controllerLatchRecovery},
    {"mpu_device_reset_recovers", deviceResetRecovery},
    {"manual_pause_survives_recovery", pausedRecovery},
    {"settings_survive_recovery", settingsRecovery},
    {"done_survives_recovery_quiet", doneRecovery},
    {"healthy_horizontal_hold_no_bus_reset", healthyHoldDoesNotResetBus},
    {"persistent_missing_sensor_controls", persistentAbsenceKeepsControls},
    {"opposite_pose_recovery_flip_once", oppositeRecoveryRestartsOnce},
    {"usb_full_buffer_does_not_block", fullSerialDoesNotBlock},
    {"recovery_across_millis_wrap", recoveryAcrossMillisWrap}
  };
  if (argc != 2) {
    for (const auto& c : cases) std::cout << c.first << '\n';
    return 2;
  }
  try {
    auto c = cases.find(argv[1]); require(c != cases.end(), "unknown recovery scenario");
    c->second(); invariants();
    std::cout << "PASS " << argv[1] << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL " << argv[1] << ": " << e.what() << " now=" << fakeNow
              << " elapsed=" << elapsedMs << " state=" << int(state)
              << " health=" << imu.healthName() << '\n';
    return 1;
  }
}
