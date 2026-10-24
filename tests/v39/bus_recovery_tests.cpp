#define main previous_main
#include "host_tests.cpp"
#undef main

static void sdaReleases() {
  boot(); running(); advance(2500);
  Wire.controllerStuck = true;
  externalBusLow[0] = true; releaseSdaAfterClocks = 3;
  advance(60); const uint32_t frozen = elapsedMs;
  advance(2600);
  require(imu.healthy() && !sensorHold && state == RunState::RUNNING, "SDA release did not recover");
  require(imu.busClearPulses() == 3 && busRiseCount == 4, "expected three clear clocks and STOP rising edge");
  require(unsafeBusHigh == 0, "bus recovery drove HIGH");
  require(elapsedMs >= frozen && elapsedMs < frozen + 2600 && onCount() == 1, "bus clear restarted timer/cue");
  require(std::string(imu.busState()) == "FREE", "cleared bus not reported FREE");
}
static void sdaPersists() {
  boot(); running(); advance(2500);
  Wire.controllerStuck = true; externalBusLow[0] = true;
  advance(60); const uint32_t frozen = elapsedMs;
  advance(1100);
  require(imu.busClearPulses() == 9 && !imu.healthy() && elapsedMs == frozen, "persistent SDA was not bounded to nine clocks");
  require(std::string(imu.busState()) == "SDA_LOW" && unsafeBusHigh == 0, "SDA safety/report incorrect");
  externalBusLow[0] = false;
  advance(2100);
  require(imu.healthy() && !sensorHold && elapsedMs >= frozen, "released SDA did not recover on next retry");
}
static void sclPersists() {
  boot(); running(); advance(2500);
  Wire.controllerStuck = true; externalBusLow[1] = true;
  advance(60); const uint32_t frozen = elapsedMs;
  const auto delayBefore = gpioDelayUs;
  advance(1100);
  require(!imu.healthy() && elapsedMs == frozen && imu.busClearPulses() == 0, "SCL stuck did not safely hold");
  require(std::string(imu.busState()) == "SCL_LOW" && unsafeBusHigh == 0, "SCL safety/report incorrect");
  // Most GPIO delay is ordinary display traffic. No unbounded clock-stretch wait.
  require(gpioDelayUs - delayBefore < 100000UL, "stuck SCL wait excessive");
  press(3); require(state == RunState::PAUSED, "SCL stuck blocked button service");
  externalBusLow[1] = false; advance(2500);
  require(imu.healthy() && !sensorHold && state == RunState::PAUSED && elapsedMs == frozen,
          "SCL release lost manual pause/time");
}
static void partialSerial() {
  boot(); Serial.capacity = 1; running();
  advance(60000 - elapsedMs); exactFinishSequence(fakeNow, 4);
  require(state == RunState::DONE && elapsedMs == 60000 && Serial.bytesWritten > 0,
          "one-byte FIFO stalled timer or cues");
}
static void repeatedRecovery() {
  boot(); running();
  for (unsigned n = 0; n < 100; ++n) {
    Wire.controllerStuck = true; advance(60); const uint32_t frozen = elapsedMs;
    advance(2000);
    require(imu.healthy() && !sensorHold && elapsedMs >= frozen && state != RunState::READY,
            "repeated recovery reset state or stopped recovering");
    invariants();
  }
  require(imu.recoveryAttempts() == 100 && Wire.resetCommands == 100 && unsafeBusHigh == 0,
          "recovery attempt/reset counts inconsistent");
}
static void slowConfigurationWaits() {
  boot(); running(); advance(1500);
  Wire.transactionLatencyMs = 10;
  Wire.controllerStuck = true; advance(60);
  while (!Wire.resetCommands) tick(1);
  const unsigned readsAtReset = Wire.readRequests;
  advance(99);
  require(Wire.readRequests == readsAtReset, "reset wait starts before actual reset write completes");
  tick(1);
  require(Wire.registers[0x6b] == 1 && !imu.healthy(), "reset verification/configuration did not finish");
  const unsigned readsAtConfig = Wire.readRequests;
  advance(99);
  require(Wire.readRequests == readsAtConfig, "warmup starts before slow configuration completes");
  advance(600);
  require(imu.healthy() && !sensorHold && onCount() == 1, "slow valid bus did not settle/recover quietly");
}
int main(int argc, char** argv) {
  const std::map<std::string, std::function<void()>> cases = {
    {"sda_release_three_clocks",sdaReleases},{"sda_stuck_nine_clocks",sdaPersists},
    {"scl_stuck_bounded",sclPersists},{"serial_one_byte_fifo",partialSerial},
    {"one_hundred_recoveries",repeatedRecovery}, {"slow_config_full_waits",slowConfigurationWaits}};
  if (argc == 1) { for (const auto& c: cases) std::cout << c.first << '\n'; return 0; }
  try { cases.at(argv[1])(); std::cout << "PASS " << argv[1] << '\n'; }
  catch (const std::exception& e) { std::cerr << "FAIL " << argv[1] << ": " << e.what() << '\n'; return 1; }
}
