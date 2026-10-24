# Arduino Digital Hourglass — v3.9

An Arduino UNO R4 WiFi hourglass with two sand matrices, GY-87 orientation sensing, six timer presets, a separate four-module countdown display and synchronized buzzer/flash feedback. v3.9 adds bounded I2C/controller and sensor recovery that preserves countdown progress.

**Current version:** [v3.9 firmware](firmware/Hourglass_v39) · [Build, wiring and operation](docs/v3.9.md) · [Validation and limitations](docs/validation-v3.9.md)

Run the current software checks with `python3 tests/v39/run.py`. Hardware observations date from 23 October 2026; see the validation document for the distinction between diagnostic and formal firmware. Long-duration reliability and the original intermittent fault remain unresolved.

This portfolio release documents the local prototype and its evidence. Development and documentation used AI assistance; school handouts, group records and third-party reference projects are not bundled.

## Version status

| Version | Included | Evidence and remaining work |
| --- | --- | --- |
| **v3.9 current** | [Firmware and headers](firmware/Hourglass_v39), full-sketch host tests, selected hardware records | Software-induced recovery exercised on real hardware; formal five-minute observation. Not long-duration or electrical-fault validation. |
| v3.1 baseline (historical) | [Firmware](firmware/Hourglass_v3/Hourglass_v3.ino), [Wokwi model](simulation/wokwi), archived MCU logs | Physical UNO R4 compilation/upload, buttons, bidirectional tilt and display operation recorded on 9 October 2026. Independent timing accuracy and electrical margins remain unmeasured. |
| v3.2 buzzer (historical) | [Firmware](firmware/Hourglass_v32/Hourglass_v32.ino), host tests | Active-buzzer logic tested with simulated Arduino APIs. This version still needs physical R4 upload, driver pinout confirmation and audio acceptance. |
| v3.3 sensor proposal (historical) | [Circuit proposal](hardware/v3.3-proposal/circuit.svg), PDF and netlist | GY-87/buzzer hardware design only. No v3.3 firmware or completed system validation is claimed. |

## Historical v3.1 setup

Open `firmware/Hourglass_v3/Hourglass_v3.ino` in Arduino IDE and select UNO R4 WiFi. The sketch uses the Arduino API without external libraries. Follow the [wiring and operation guide](docs/wiring-and-operation.md), calibrate matrix orientation, then use the serial monitor at 115200 baud.

For simulation, import the three files from `simulation/wokwi` into a new classic-UNO Wokwi project. Its slide switch is a proxy for the physical tilt switch. This simulation does not model the UNO R4 processor or validate physical power delivery.

## Historical v3.1/v3.2 software checks

With a C++11 compiler installed:

```sh
sh tests/run.sh
```

Tests include the actual sketches and replace Arduino I/O and time with host stubs. They check state transitions, display output, grain conservation, pause/flip/reset interactions and buzzer timing. They do not test hardware electrically. See [validation and evidence](docs/validation.md).

## Historical v3.1 engineering tradeoff

Animation and timing are separate: the target number of grains in the lower chamber is `floor(active_ms * 36 / 60000)`. If the animation blocks the neck, a fallback moves a grain to preserve timing. This can produce visible jumps; the prototype prioritizes elapsed-time progress over physical sand simulation.

## Attribution and reuse

This is a student prototype, not an official university release. It does not claim every design idea is novel. Third-party reference repositories and course documents remain outside this repository. No additional reuse license is granted in this release; contact the maintainer before redistribution beyond applicable permissions.
