# Arduino Digital Hourglass

A dual-MAX7219 LED hourglass prototype developed during ELEC281, using an Arduino UNO R4 WiFi, a tilt switch and two buttons. It displays 36 grains over 60 seconds of active running time, with start, pause, resume, reset and flip handling.

This portfolio release documents the local prototype and its evidence. Development and documentation used AI assistance; school handouts, group records and third-party reference projects are not bundled.

## Version status

| Version | Included | Evidence and remaining work |
| --- | --- | --- |
| v3.1 baseline | [Firmware](firmware/Hourglass_v3/Hourglass_v3.ino), [Wokwi model](simulation/wokwi), archived MCU logs | Physical UNO R4 compilation/upload, buttons, bidirectional tilt and display operation recorded on 9 October 2026. Independent timing accuracy and electrical margins remain unmeasured. |
| v3.2 buzzer | [Firmware](firmware/Hourglass_v32/Hourglass_v32.ino), host tests | Active-buzzer logic tested with simulated Arduino APIs. This version still needs physical R4 upload, driver pinout confirmation and audio acceptance. |
| v3.3 sensor extension | [Circuit proposal](hardware/v3.3-proposal/circuit.svg), PDF and netlist | GY-87/buzzer hardware design only. No v3.3 firmware or completed system validation is claimed. |

## Start with v3.1

Open `firmware/Hourglass_v3/Hourglass_v3.ino` in Arduino IDE and select UNO R4 WiFi. The sketch uses the Arduino API without external libraries. Follow the [wiring and operation guide](docs/wiring-and-operation.md), calibrate matrix orientation, then use the serial monitor at 115200 baud.

For simulation, import the three files from `simulation/wokwi` into a new classic-UNO Wokwi project. Its slide switch is a proxy for the physical tilt switch. This simulation does not model the UNO R4 processor or validate physical power delivery.

## Run software checks

With a C++11 compiler installed:

```sh
sh tests/run.sh
```

Tests include the actual sketches and replace Arduino I/O and time with host stubs. They check state transitions, display output, grain conservation, pause/flip/reset interactions and buzzer timing. They do not test hardware electrically. See [validation and evidence](docs/validation.md).

## Engineering tradeoff

Animation and timing are separate: the target number of grains in the lower chamber is `floor(active_ms * 36 / 60000)`. If the animation blocks the neck, a fallback moves a grain to preserve timing. This can produce visible jumps; the prototype prioritizes elapsed-time progress over physical sand simulation.

## Attribution and reuse

This is a student prototype, not an official university release. It does not claim every design idea is novel. Third-party reference repositories and course documents remain outside this repository. No additional reuse license is granted in this release; contact the maintainer before redistribution beyond applicable permissions.
