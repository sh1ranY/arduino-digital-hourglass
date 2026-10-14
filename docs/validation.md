# Validation and evidence

## Archived hardware session — 9 October 2026

The files in [evidence/2026-10-09](../evidence/2026-10-09) are historical records, not hardware tests rerun for this publication.

- `board-check-results.json`: eight serial-command checks on the physical UNO R4, including READY, start/pause/resume/reset, grain conservation and 60000 ms completion.
- `board-state-machine.log`: serial state trace supporting those checks.
- `physical-controls-final.log`: separate mechanical button events.
- `physical-tilt-final.log`: bidirectional stable-flip events.

The local session also recorded successful compilation/upload and user-confirmed display behavior. A reported elapsed_ms=60000 is the firmware clock, not an independent calibrated stopwatch measurement. Electrical margins, final mounting and flash readback were not established. v3.2 and v3.3 have no completed physical acceptance in this release.

## Publication checks — 14 October 2026

Host-test results are stored in `host-test-results.txt` alongside this document. These tests compile the published firmware through Arduino stubs and exercise software behavior. They do not replace the archived physical evidence. The simulation sketch is checked byte-for-byte against v3.1. No new hardware upload is performed during publication.

Publication host checks: v3.1 **5/5**, v3.2 default active-buzzer mode **14/14**. Calibration-mode historical results are not counted as fresh checks.
