# v3.9 validation: evidence and limits

## Historical hardware evidence — 23 October 2026

The selected [evidence files](../evidence/2026-10-23) are copied unchanged and checked against the local archive's SHA-256 manifest. They are not tests performed again during publication.

**Temporary diagnostic build:** a software-induced sample-read failure exercised the real board's Wire/controller restart, MPU reset, register verification and sensor settling. Timing froze at 5011 ms with 33/3 grains, then resumed in the same pose. The diagnostic trace reached 60000 ms effective running time and 0/36 grains after 62711 ms wall time, excluding 2711 ms of fault hold. This was not an electrical disconnection, nor a reproduction of the original long-idle failure. The operator reported normal automatic recovery and completion sound/flashes. See `physical-recovery-results.json` and `diagnostic-r3-runtime.jsonl`.

**Formal build:** the local record reports successful native R4 compilation/upload, with self-test disabled. Source SHA-256 is `3af95fb69432b124e86e9b76cae52046d89de360e44ddf313c216919600a1900`. Formal binary SHA-256 is `38e412824e98fbab86238b1092a87b43fa07c12d3e3cd9b6bf0ab020999b2cee`; flash readback was not verified. Machine-specific build/upload scripts and binaries are not published here.

Formal telemetry spans **300.22 seconds / 343 records**, with bench=0, fault count 0 and recovery count 0. Manual pause/resume and automatic hold/release were captured. **No final Completed event appears in the formal trace.** Completion sound/flashes were reported by the operator; the complete 60-second serial result belongs to the separate diagnostic build. See `formal-runtime-summary.json` and `formal-runtime.jsonl`.

## Host software checks

The archived 23 October report records **159 scenario executions**: 97 baseline/configuration executions, 10 recovery cases and 52 extended executions. These are executions across configurations, not 159 distinct physical tests. The actual complete sketch runs against Arduino, GPIO, display and register-backed I2C test doubles.

To reproduce the publication checks with Python 3 and a C++11 compiler:

```sh
python3 tests/v39/run.py
```

Default compiler is `clang++`; set `CXX` to another compatible compiler if needed. Binaries are temporary. Results go to ignored `build/v39-host-results.json`. The runner checks nonempty scenario discovery, compilation, exit codes and input hashes. Original test assertions are preserved; include paths and the runner are adapted for this repository. AVR/R4 preprocessor variants are host emulations, not native builds for those processors.

## What remains unproven

- The original long-idle failure has not been reproduced; its trigger and root cause remain unresolved.
- Five minutes of formal observation does not establish hours/days reliability.
- Software doubles do not validate power delivery, contact quality, acoustic output or actual electrical bus faults.
- Firmware time is not an independently calibrated timer measurement.
- No fresh hardware upload or physical acceptance was performed for this publication.

The older [v3.1 validation](validation.md) remains historical and does not validate v3.9.

## Publication rerun — 24 October 2026

**159/159 host scenario executions passed** against the published source: 32 native, 1 calibration, 32 AVR API emulation, 32 R4 bridge emulation, 10 recovery and 52 extended executions. Firmware and test inputs remained unchanged during execution. [Machine-readable results and input hashes](v39-host-results-2026-10-24.json). No fresh native R4 build or board upload was performed.
