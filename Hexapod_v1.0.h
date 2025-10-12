#pragma once
#include <Arduino.h>

namespace Hexapod {

// -----------------------------------------------------------------------------
// Splash banner (moved from .ino)
// -----------------------------------------------------------------------------
inline const char SPLASH_BANNER[] =
R"(+----------------------------------+
|  __  __    _    ____  ____       |
| |  \/  |  / \  |  _ \/ ___|      |
| | |\/| | / _ \ | |_) \___ \      |
| | |  | |/ ___ \|  _ < ___) |     |
| |_|  |_/_/   \_\_| \_\____/      |
|                                  |
+----------------------------------+
)";

inline void printSplash(Stream& out = Serial) {
  out.print(SPLASH_BANNER);
}

// -----------------------------------------------------------------------------
// Help text (generated from commands implemented in handleCommandLine())
// -----------------------------------------------------------------------------
inline const char HELP_TEXT[] =
R"(================================================================
Hexapod Console — Help
================================================================
Basics
  h | ?                        : Show this help
  s                            : Status (servo enable, gait, logging)
  e / d                        : Enable/disable ALL legs
  le <leg>                     : Enable ONLY one leg (0..5)
  ld <leg>                     : Disable one leg (0..5)
  R | r                        : Software reboot

Gait / Stance
  gait run                     : Run tripod gait (advance phases)
  gait stop                    : Freeze phases (no stepping)
  stance                       : Hold all feet at home position (no stepping)

Logging (tri-state) + SD tools
  log show                     : Show current logging mode and SD status
  log ls [path]                : List SD directory (default "/")
  log cat <path> [max_bytes]   : Print SD file to Serial (0 = whole file)
  log serial                   : Set destination → Serial only
  log sd                       : Set destination → SD only (if SD ready)
  log both                     : Set destination → Serial + SD

VSD (Virtual Spring-Damper; per DOF or overrides)
  vsd show                     : Dump VSD bases and any overrides
  vsd stance <dof> <ks> <b> <tau>
  vsd swing  <dof> <ks> <b> <tau>
                                Set base VSD for a DOF in STANCE/SWING
                                DOF: 0|1|2 or coxa|femur|tibia
  vsd ov  <leg> <dof> <ks> <b> <tau>
                                Per-leg override (enable) for a DOF
  vsd clr <leg> <dof>          : Clear per-leg override

PID tuning
  pid show                     : Print PID for all (leg,dof)
  pid <leg> <dof> <kp> <ki> <kd>
                                Set PID for one leg & DOF
  pidg <dof> <kp> <ki> <kd>    : Set PID for ALL legs on a DOF

Slew limits (rad/s by default)
  slew show                    : Print current slew by DOF
  slew <dof|all> <value> [deg] : Set slew (optionally in degrees/sec)

IK home angles (persisted on SD: /home_angles.csv)
  home show                    : List current homes (centideg) per leg
  home load                    : Load homes from SD (if present)
  home save                    : Save current homes to SD
  home load defaults           : Restore baked-in defaults (RAM only)
  home set <leg> <c> <f> <t>   : Set one leg homes (centideg)
  home deg <leg> <c> <f> <t>   : Set one leg homes (degrees → centideg)

Notes
  • Deterministic 100 Hz loop; one joint read per tick (others predicted).
  • Tri-state logging respects SD availability; 'log sd' may fall back.
  • Use 'stance' for stable tuning without gait motion.
================================================================
)";

// -----------------------------------------------------------------------------
// Version + helpers
// -----------------------------------------------------------------------------
inline void printHelp(Stream& out = Serial) {
  out.print(HELP_TEXT);
}

inline void printVersion(Stream& out = Serial) {
#ifdef FW_NAME
  out.print("[SYS] "); out.print(FW_NAME);
  out.print(" v");
# ifdef FW_VERSION
  out.print(FW_VERSION);
# else
  out.print("?.?.?");
# endif
# ifdef FW_BUILD_DT
  out.print(" (built "); out.print(FW_BUILD_DT); out.print(')');
# endif
  out.println();
#else
  out.println("Hexapod — version info not compiled into header.");
#endif
}

} // namespace Hexapod
