# Changelog
## 2025-10-17

- Unified SD configuration
  - Added header-only `Config.h` with a simple key=value store in `/config.txt`.
  - Homes are now persisted as `home_cdeg` (18 comma-separated centidegree ints).
  - Logging cadence persisted as `log.every=<n>` cycles.
  - Boot-time migration: if `home_cdeg` missing but legacy `HOME_CFG_PATH` exists, automatically import.
  - Updated help text and inline comments to point to `/config.txt`.


All notable changes to this project will be documented in this file. This mirrors the changelog header in `Hexapod_v1.0.ino` and should remain in sync with code comments and `HELP_TEXT`.

## 2025-10-16

- Console
  - Added `s`/`status` with loop timing (Hz, Ts, last dt) and system snapshot.
  - Added `stance` to stop gait and hold home joint angles; resets `phase_t` per leg.
  - Wired `R`/`r` to `teensyReboot()` (Cortex-M AIRCR software reset) with splash/notice.
  - Implemented per-leg enable/disable with torque control:
    - `le <leg>` now enables the specified leg only and torques on its servos.
    - `ld <leg>` disables the specified leg only and torques off its servos.
- Logging
  - Mode controls: `log show|serial|sd|both`.
  - SD tools: `log ls`, `log cat <path>`.
  - Safe deletion: `log del <path>` and `log delall` (protects current log; only `LOG*.CSV`).
  - Default startup logging mode changed to SD Only (was Both).
  - Added `log off` mode to fully disable logging (no Serial, no SD).
- Help text / Config
  - Synchronized `HELP_TEXT` with implemented commands and 166 Hz docs; removed unimplemented items for now.
  - Added home tools docs: `home show`, `home defaults`, `home set`, `home deg`, `home move`.
  - Unified SD config: added `/config.txt` (key=value). Homes stored as `home_cdeg=...`; log cadence stored as `log.every=...`.
  - Legacy reference to `/home_angles.csv` kept in code as constant but no longer used for persistence.
  - `home move <leg> [ms] [off|disable]` documented (optional torque-off & disable after move).
  - Added `home read <leg>` to capture current angles into RAM homes (leg must be disabled).
- State/Timing
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
