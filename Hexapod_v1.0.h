#pragma once
#include <cstring>   // for strtok_r, strcasecmp
#include <Arduino.h>
#include <lx16a-servo.h>

// =====================================================================
//                              LOOP TIMING
// =====================================================================

#define LOOP_HZ 166
//#define Ts      (1.0f / LOOP_HZ)

// =====================================================================
//                       FIRMWARE ID / VERSION STAMP
// =====================================================================

#define FW_NAME     "Hexapod Controller"
#define FW_VERSION  "1.8.0"   // IK + VSD/PID + SD-persisted IK homes + String console + tri-logging
#define FW_BUILD_DT __DATE__ " " __TIME__

// =====================================================================
//                  ROBOT TOPOLOGY / CONSTANTS (LEAVE EARLY)
// =====================================================================

const int N_LEGS       = 6;
const int DOF_PER_LEG  = 3;
const int N_JOINTS     = N_LEGS * DOF_PER_LEG;

#define NUM_LEGS N_LEGS

enum { COXA = 0, FEMUR = 1, TIBIA = 2 };

// =====================================================================
//                      SD CONFIG FILE PATHS (CENTRALIZED)
// =====================================================================
// Keep all SD-backed config filenames here to avoid drift across files.
static const char HOME_CFG_PATH[] = "/home_angles.csv";   // Legacy (now stored in /config.txt: key 'home_cdeg')

// =====================================================================
//                          PHYSICAL GEOMETRY (IK)
// =====================================================================
// NOTE: Units for IK are millimeters, angles are radians/deg/centideg as noted.
// Adjust these lengths to your hardware.

#define COXA_LENGTH_MM   41.70f
#define FEMUR_LENGTH_MM  80.00f
#define TIBIA_LENGTH_MM 133.78f

// =====================================================================
//                   SERIAL BUSES / SERVO IDS / BUFFERS
// =====================================================================

HardwareSerial* SERVO_PORTS[N_LEGS] = {&Serial7, &Serial6, &Serial2, &Serial5, &Serial3, &Serial8};

uint8_t SERIAL_TX_PINS[N_LEGS] = {28,25,8,21,14,34};

uint8_t SERVO_ID[N_LEGS][DOF_PER_LEG] = {
  {1,2,3}, {1,2,3}, {1,2,3}, {1,2,3}, {1,2,3}, {1,2,3}
};

// Optional 74HC126 buffer OE pins per bus (adjust to wiring)
const int bufferEnablePins[N_LEGS] = {32, 9, 6, 22, 16, 36};

// LX-16A servo bus objects
LX16ABus* legBus[N_LEGS];

// Array of LX16AServo objects for each servo
LX16AServo* servos[N_JOINTS]; // Pointers to servo objects

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
  help | h | ?                 : Show this help
  s | status                   : Status (motors, loop, logging, per-leg enable)
  mem                          : Memory snapshot (heap/stack/alloc)
  e / d                        : Enable/disable ALL servos
  le <leg>                     : Enable one leg (leaves others unchanged) and torque-on
  ld <leg>                     : Disable one leg and torque-off
  safety clear                 : Clear safety latch (after over-temp/low-V)
  safety show                  : Show thresholds and last readings
  safety set over_temp_c <n>   : Set trip temp (40..100 C)
  safety set low_mv <n>        : Set low bus voltage (5000..12000 mV)
  safety set min_mv <n>        : Set ignore-bogus limit (1000..low_mv-500)
  R | r                        : Software reboot
  gait run | gait stop         : Start/stop tripod gait
  gait show                    : Show gait parameters (stance height)
  gait stance <mm>             : Set base stance height (mm, negative down) and persist
  gait stride <mm>             : Set stride length (mm) and persist
  gait lift <mm>               : Set swing lift height (mm) and persist
  gait dur <stance_ms> <swing_ms>
                               : Set stance/swing durations (milliseconds) and persist

Stance
  stance                       : Hold all joints at home position (no stepping)

Logging (tri-state) + SD tools
  log show                     : Show current logging mode, SD status, and file
  log ls                       : List SD root directory
  log cat <path>               : Print SD file to Serial
  log serial                   : Set destination → Serial only
  log sd                       : Set destination → SD only (if SD ready)
  log both                     : Set destination → Serial + SD
  log off                      : Turn logging off (no Serial, no SD)
  log del <path>               : Delete a log file (LOG*.CSV), not the current one
  log delall                   : Delete all log files (LOG*.CSV) except current
  log every <n>                : Emit one log row every n control cycles (n>=1)

Home angles (SD: /config.txt → key 'home_cdeg')
  home show                    : Print homes per leg (centideg and deg)
  home defaults                : Restore default homes to RAM (use 'home save' to persist)
  home read <leg>              : Capture current angles as homes (leg must be disabled)
  home move <leg> [ms] [off|disable]
                               : Move one leg to its home; add flag to torque-off & disable
  home set <leg> <c> <f> <t>   : Set homes in centideg (clamped to joint limits)
  home deg <leg> <c> <f> <t>   : Set homes in degrees (clamped to joint limits)
  home load                    : Load homes from SD (if present)
  home save                    : Save current homes to SD

Notes
  * Deterministic 166 Hz loop; one joint read per tick (others predicted).
  * Tri-state logging respects SD availability; 'log sd' may fall back.
  * Use 'stance' for stable tuning without gait motion.
  * Safety: If over-temp (>=70C) or low bus voltage (<=7.0V) is detected, the
    controller clears integrators, stops gait, and disables all joints. Use
    'safety clear' to clear the latch, then re-enable with 'e' or 'le <leg>'.
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
