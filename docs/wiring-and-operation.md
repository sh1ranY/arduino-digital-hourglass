# Wiring and operation

## Baseline v3.1

| Pin | Connection |
| --- | --- |
| D11 | First MAX7219 DIN |
| D13 | Both MAX7219 CLK |
| D10 | Both MAX7219 CS/LOAD |
| First module DOUT | Second module DIN |
| 5V / GND | Common module supply and ground |
| D2 | Tilt switch to GND, INPUT_PULLUP |
| D3 | Start/pause button to GND, INPUT_PULLUP |
| D4 | Reset button to GND, INPUT_PULLUP |

Confirm button contact groups, power polarity and common ground before powering. Brightness is set to the lowest MAX7219 intensity. Set `HOURGLASS_CORNER_CALIBRATION` to 1 for corner alignment, adjust per-module rotations/mirroring and the top-module assignment, then restore it to 0 for normal operation. Final mechanical orientation remains assembly-dependent.

- Start/pause: READY or DONE starts; RUNNING pauses; PAUSED resumes.
- Reset: returns to READY and clears elapsed time; it wins over simultaneous inputs.
- A stable flip restarts a cycle, retaining PAUSED if already paused.
- Buttons debounce for 30 ms; tilt settles for 300 ms.
- Paused time does not count toward the 60-second interval.

## v3.2 buzzer

D5 is a control signal for a HIGH-active external NPN low-side driver, not a buzzer power supply. Default firmware targets an active buzzer: one 120 ms start cue and three completion cues with 100 ms gaps. Confirm the actual transistor E/B/C pinout and driver wiring. Physical audio acceptance remains outstanding.

## v3.3 proposal

The GY-87 circuit files describe a future hardware extension. Confirm the module supply/level-shifting design and transistor pinout before assembly. Baseline firmware does not implement the sensor; do not assume that uploading v3.1 or v3.2 completes v3.3.
