# Code Style Guide (Hexapod_v1.0)

Goal: maximize readability and maintainability under embedded constraints.

Core principles
- Generous vertical whitespace
  - Separate logical blocks (init, checks, actions) with blank lines.
  - Keep functions visually chunked: early returns, short paragraphs.
- Very generous comments
  - Above each function: high-level intent, inputs/outputs/side-effects.
  - Within functions: short comments for non-obvious steps and rationale.
  - Document safety checks, assumptions, and units.
- Clear, stable names
  - Prefer descriptive names over abbreviations (unless widely understood).
  - Keep units in variable names when helpful (e.g., `* _us`, `*_rad`).
- Deterministic hot paths
  - Avoid dynamic allocation and heavy string ops in control loop.
  - Explain any timing tradeoffs or ISR interactions.
- Consistent layout
  - Braces on the same line as control statements.
  - Space around operators; aligned initializer lists when practical.

Examples (C++)
- Function header comment
  ```cpp
  // printStatus
  // ------------------------------------------------------------------
  // Print current runtime state: gait, motors, loop timing, logging,
  // per-leg enable summary, and memory snapshot. Lightweight and safe
  // to call from console context (not ISR).
  ```
- Inline rationale
  ```cpp
  // Reset per-leg phase timers so resume is deterministic from phase 0
  // (important for tripod sequence symmetry after stopping).
  ```

CLI conventions
- Each command prints a one-line summary of the action taken.
- `usage:` lines shown when arguments are missing/invalid.
- All user-facing units are explicit (Hz, s, rad, deg, mV, °C).

Documentation
- Keep banner comments in `*.ino` up to date (loop rate, safety notes).
- Update `Hexapod::HELP_TEXT` when adding/removing commands.

Testing notes
- Prefer staged enablement: add feature, test in isolation, then integrate.
- When debugging lockups, temporarily guard new features behind `#if 0` or block comments and re-enable incrementally.

Trailing comments for critical code
- Use end-of-line comments for operations that alter global state, timing, safety, or hardware I/O.
- Keep them brief but explicit; prefer units and rationale when helpful.
- Examples:
  ```cpp
  s.tickFlag = true;             // ISR: signal 166 Hz control tick
  ticker.begin(isr, us_per_tick);// start periodic ISR at control_loop_Hz
  if (!s.tickFlag) return;       // tick-gated: deterministic loop cadence
  SCB_AIRCR = 0x05FA0004;        // NVIC software reset (Teensy reboot)
  cs->GAIT_RUN = false;          // stop gait before entering stance
  ```
