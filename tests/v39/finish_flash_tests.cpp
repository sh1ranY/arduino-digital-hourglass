// Full-firmware tests: observe the actual MAX7219 row registers and D5 edges.
// Each command-line scenario runs in its own process to isolate Arduino globals.
#define main regression_main
#include "host_tests.cpp"
#undef main

static uint32_t lastPhysicalOn() {
  for (auto it = audioEdges.rbegin(); it != audioEdges.rend(); ++it)
    if (it->on) return it->at;
  throw std::runtime_error("no physical D5 rising edge to align completion flash");
}

static void requireFinishFrame(unsigned top, uint32_t age) {
  const bool expectedBlank = age < 560UL && (age % 220UL) < 120UL;
  require(state == RunState::DONE, "completion flash changed timer state");
  require(upperModule() == top, "completion flash changed accepted orientation");
  require(pixels(top) == 4, "completion flash also blanked/changed upper check mark");
  require(pixels(1-top) == (expectedBlank ? 0U : 36U),
          "lower MAX7219 rows must be blank during each 120ms beep, 36 pixels otherwise; age=" +
          std::to_string(age) + " pixels=" + std::to_string(pixels(1-top)));
  require(buzzerLevel == expectedBlank, "lower flash does not align with actual D5 level");
}

static void examineEntireFinish(unsigned top, uint32_t onset) {
  require(fakeNow == onset, "test must observe actual physical finish onset");
  for (uint32_t age = 0; age <= 560; ++age) {
    if (age) tick(1);
    requireFinishFrame(top, age);
  }
  const uint32_t offsets[] = {0, 120, 220, 340, 440, 560};
  std::vector<AudioEdge> tail;
  for (const auto& e : audioEdges)
    if (uint32_t(e.at - onset) <= 560) tail.push_back(e);
  require(tail.size() == 6, "finish needs exactly three complete physical beep pairs");
  for (unsigned i = 0; i < 6; ++i) {
    require(uint32_t(tail[i].at - onset) == offsets[i], "finish sound timing changed");
    require(tail[i].on == (i % 2 == 0), "finish physical edges do not alternate");
  }
  const unsigned n = onCount();
  for (unsigned i = 0; i < 1500; ++i) {
    tick(1);
    requireFinishFrame(top, 561 + i);
  }
  require(onCount() == n, "DONE replayed an old completion cue");
}

static void completionFlash(bool inverted) {
  boot(inverted ? -1.0f : 1.0f);
  const unsigned top = upperModule();
  running();
  const uint32_t onset = finishCycle();
  require(buzzerLevel, "completion did not open physical D5 output");
  examineEntireFinish(top, onset);
  require(onCount() == 4 && blockingDelayMs == 0,
          "completion changed total beep count or added blocking delays");
}

static void resetRestoresBlankedDisplay() {
  boot(); running(); finishCycle();
  const unsigned oldTop = upperModule();
  requireFinishFrame(oldTop, 0);
  advance(50); press(4);
  require(state == RunState::READY && pixels(oldTop) >= 36 && pixels(1-oldTop) == 0,
          "reset left old completion blank active or failed to clear old lower sand");
  require(!buzzerLevel, "reset did not cancel completion sound");
  const unsigned n = onCount();
  for (unsigned i = 0; i < 1800; ++i) {
    tick(1);
    require(pixels(oldTop) >= 36 && pixels(1-oldTop) == 0,
            "cancelled completion flash replayed after reset");
  }
  require(onCount() == n, "cancelled completion sound replayed after reset");
}

static void faultRestoresBlankedDisplay() {
  boot(); running(); finishCycle();
  const unsigned top = upperModule();
  requireFinishFrame(top, 0);
  advance(50); Wire.present = false; advance(25);
  require(sensorHold && !imu.healthy() && !buzzerLevel,
          "fault did not cancel completion sound and hold");
  require(pixels(1-top) == 36, "fault left completed lower chamber permanently blank");
  const unsigned n = onCount();
  Wire.present = true; acceleration(1,0,0);
  for (unsigned i = 0; i < 2200; ++i) {
    tick(1);
    require(pixels(1-top) == 36, "fault recovery replayed old completion flash");
  }
  require(state == RunState::DONE && imu.healthy() && !sensorHold && onCount() == n,
          "fault recovery restarted completion or replayed sound");
}

static void flipDiscardsOldFlash() {
  boot(); running(); finishCycle();
  const unsigned oldTop = upperModule();
  requireFinishFrame(oldTop, 0);
  acceleration(-1,0,0);
  unsigned waited = 0;
  while (upperModule() == oldTop && waited < 1000) { tick(1); ++waited; }
  require(upperModule() == 1-oldTop && state == RunState::RUNNING,
          "stable opposite pose did not start a fresh inverted cycle");
  require(pixels(1-oldTop) >= 36 && pixels(oldTop) == 0,
          "old finish blank leaked into the newly filled upper chamber");
  const unsigned n = onCount();
  for (unsigned i = 0; i < 1000; ++i) {
    tick(1);
    require(pixels(1-oldTop) >= 36 && pixels(oldTop) == 0,
            "old completion flash replayed after accepted inversion");
  }
  require(onCount() == n && !buzzerLevel, "flip replayed old completion pulses after fresh start");
}

static void finishFlashAcrossWrap() {
  boot(1,0,0,uint32_t(0xffffffffUL-60000UL-1530UL-50UL));
  running();
  const unsigned top = upperModule();
  const uint32_t onset = finishCycle();
  require(onset >= 0xffffff00UL, "test completion did not begin near millis rollover");
  examineEntireFinish(top, onset);
}

static void finishFlashWithBlockedSerial() {
  boot(); serialReportLatencyMs = 25; Serial.capacity = 0; running();
  const unsigned top = upperModule();
  advance(60000-elapsedMs);
  require(state == RunState::DONE && elapsedMs == 60000 && serialLatencyEvents == 0,
          "full USB FIFO reached a blocking serial print or finish state is incorrect");
  const uint32_t onset = lastPhysicalOn();
  examineEntireFinish(top, onset);
}

int main(int argc, char** argv) {
  const std::map<std::string,std::function<void()>> scenarios = {
    {"finish_flash_upright", [] { completionFlash(false); }},
    {"finish_flash_inverted", [] { completionFlash(true); }},
    {"finish_flash_reset_cancel", resetRestoresBlankedDisplay},
    {"finish_flash_fault_cancel", faultRestoresBlankedDisplay},
    {"finish_flash_flip_cancel", flipDiscardsOldFlash},
    {"finish_flash_millis_rollover", finishFlashAcrossWrap},
    {"finish_flash_serial_latency", finishFlashWithBlockedSerial}
  };
  if (argc != 2) { for (const auto& s : scenarios) std::cout << s.first << '\n'; return 2; }
  try {
    auto it = scenarios.find(argv[1]); require(it != scenarios.end(), "unknown flash scenario");
    it->second();
    std::cout << "PASS " << argv[1] << " | I2C_reads=" << Wire.readRequests
              << " MAX_latches=" << displayLatches << " D5_ons=" << onCount()
              << " elapsed_ms=" << elapsedMs << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL " << argv[1] << ": " << e.what() << " | now=" << fakeNow
              << " elapsed=" << elapsedMs << " state=" << int(state)
              << " lower_pixels=" << pixels(1-upperModule()) << " buzzer=" << buzzerLevel << '\n';
    return 1;
  }
}
