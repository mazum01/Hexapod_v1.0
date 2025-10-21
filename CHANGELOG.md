# Changelog
All notable changes to this project will be documented in this file. This mirrors the changelog header in `Hexapod_v1.0.ino` and should remain in sync with code comments and `HELP_TEXT`.
## 2025-10-19

- Safety and guards
  - Config-backed thresholds with sane clamps: `safety.over_temp_c`, `safety.low_bus_mv`, `safety.min_valid_mv` in `/config.txt`.
  - Sticky trip with console controls: `safety show|set|clear`. When tripped, gait stops and all joints are disabled until re-enabled.
  - Clears PID integrators on warning (hot/low-V) even before a trip; status shows last `tempC` and `mV` readings.
  - Prints and clears Teensy `CrashReport` at boot for post-mortem visibility.
- Gait and motion pipeline (WIP)
  - Added a parametric foot trajectory generator (mm) with stance/swing durations, stride, lift, and per-leg lateral offsets.
  - Integrated the user-provided `calculateIK` (centideg) with home offsets and the project’s axis mapping.
  - Restored `gait run|stop`; desired joint angles (`q_des`) now come from Trajectory → IK → clamps.
  - Actuation remains guarded: servo `move_time(...)` calls are still commented pending validation on hardware.
- Telemetry and timing
  - Enforced exactly one joint sensor read per tick (round-robin); removed the duplicate RR block to keep the 166 Hz loop deterministic.
  - Improved loop dt tracking and guards; NaN/Inf safety checks throughout.
- Logging
  - CSV schema v1.1 header (includes `q_est`/`dq_est`) and cycle-based cadence: `log every <n>` persisted to `/config.txt`.
  - Supports `log off` and safe deletion tools: `log del <path>`, `log delall` (skips current file).
- Home tools and config
  - Homes are persisted as `home_cdeg` (18 centidegree ints) in `/config.txt`, with migration from legacy `/home_angles.csv`.
  - Added `home read <leg>` to capture current angles into RAM homes (leg must be disabled); `home move` documented and clamped.
  - Config lifecycle: Ensure factory defaults file at boot; added `cfg factory` (auto-backup then reset), `cfg backup`, and `cfg restore` commands.
- Cleanup and docs
  - Removed unused legacy `legIK_3dof`; synchronized help text with implemented commands.
  - Console parser refactor: renamed local argv/argc to tokens/ntokens for clarity and to appease IntelliSense.
  - Default logging mode clarified as SD Only at boot; help/status reflect SD availability.
- Version
  - Firmware version stamped as v1.9.0.

### Boot stability and Safe Mode
- Added Safe Mode via pin 33 (INPUT_PULLUP). Hold low at boot to skip SD/ticker/logging for recovery.
- Implemented EEPROM-backed boot markers and a software watchdog:
  - Marks boot "in-progress" early in setup and clears on successful completion.
  - If a reboot occurs without clearing, next boot forces Safe Mode with a notice.
  - A simple software watchdog pets on healthy loop progress; on timeout, it sets a force-safe flag in EEPROM and reboots.
  - If a Teensy CrashReport is present, it’s printed, the force-safe flag is set, and the device reboots into Safe Mode for investigation.
  - Help text updated; status and boot logs reflect Safe Mode reasons.

## 2025-10-17

- Unified SD configuration
  - Added header-only `Config.h` with a simple key=value store in `/config.txt`.
  - Homes are now persisted as `home_cdeg` (18 comma-separated centidegree ints).
  - Logging cadence persisted as `log.every=<n>` cycles.
  - Boot-time migration: if `home_cdeg` missing but legacy `HOME_CFG_PATH` exists, automatically import.
  - Updated help text and inline comments to point to `/config.txt`.
## 2025-10-16

- Console
  - Added `s`/`status` with loop timing (Hz, Ts, last dt) and system snapshot.
  - Added `stance` to stop gait and hold home joint angles; resets `phase_t` per leg.
  - Wired `R`/`r` to `teensyReboot()` (Cortex-M AIRCR software reset) with splash/notice.
  - Per-leg enable/disable
    - `le <leg>` now enables the specified leg only and torques on its servos.
    - `ld <leg>` disables the specified leg only and torques off its servos.
- Logging
  - Mode controls: `log show|serial|sd|both`.
  - SD tools: `log ls`, `log cat <path>`.
  - Default startup logging mode changed to SD Only (was Both).
  - Added `log off` mode to fully disable logging (no Serial, no SD).
- Help text / Config
  - Synchronized `HELP_TEXT` with implemented commands and 166 Hz docs; removed unimplemented items for now.
  - Added home tools docs: `home show`, `home defaults`, `home set`, `home deg`, `home move`.
  - Unified SD config: added `/config.txt` (key=value). Homes stored as `home_cdeg=...`; log cadence stored as `log.every=...`.
  - `home move <leg> [ms] [off|disable]` documented (optional torque-off & disable after move).
  - Added `home read <leg>` to capture current angles into RAM homes (leg must be disabled).
  - Added `motors_on` and `last_dt_loop` to `ControllerState`; status includes loop dt.
  - Improved dt guards; clarified and enforced one-joint-per-tick read path.
- Style/Docs
  - Introduced `STYLE.md`; expanded comments and whitespace; added critical trailing comments.

Files touched (recent): `Hexapod_v1.0.ino`, `Hexapod_v1.0.h`, `Logging.h`, `ControllerState.h`, `HomeCfg.ino`, `STYLE.md`.
## 2025-10-13

- Refactoring and infrastructure
  - Non-blocking console with fixed buffers + `strtok_r` tokenization.
  - Memory gauges: `freeHeapGap()`, `stackFreeNow()`, `maxHeapAllocTest()`.
  - Enabled `CrashReport` printing at boot for fault diagnosis.
  - 166 Hz loop via `IntervalTimer`; ISR sets `tickFlag`; main loop processes on tick.
  - Line-by-line SD file reader (no dynamic String accumulation).
