/*
   Teensy 4.1 Hexapod Controller (LX16A buses, HTS-35S-style servos)
   ==================================================================

   BIG PICTURE
   -----------
   • 6 legs × 3 DOF LX16A serial-bus servos (centidegrees API).
  • 166 Hz deterministic loop via IntervalTimer ISR (dt computed once).
   • Round-robin: read exactly 1 joint/tick; predict others for continuity.
   • Control stack (per joint):
       Foot trajectory (mm) → IK → q_des  --->  VSD outer loop (compliant target)
                                                → PID inner loop (angle, DoM, LPF-D, anti-windup)
                                                → slew limit & joint limits
                                                → move_time(centideg, ms)
   • Safety: NaN/Inf guards, slew clamps, joint limits, integrator clear on hot/low-V.
   • Global + per-leg enable; serial commands for tuning (VSD, PID, slew), gait, stance, logging.
  • Tri-state logging: Serial / SD / Both / Off (default SD Only), with `log ls`, `log cat`.
  • IK home angles and log settings are persisted in /config.txt on SD; auto-created if missing.

   SERIAL COMMANDS (abridged)
   --------------------------
   h / ?                         : help
   e / d                         : enable/disable ALL legs
   le <leg> / ld <leg>           : enable ONLY one leg / disable leg (0..5)
   s                             : status (enable, gait, logging)
   gait run|stop                 : run tripod (advance phases) / freeze phases
   stance                        : set all legs to home foot position (no stepping)
   vsd ...                       : VSD base & per-leg overrides (see help)
   pid / pidg ...                : PID tuning (leg/DOF or per-DOF all legs)
   slew ...                      : slew rate per DOF or all
   home show|load|save|defaults  : IK home tools (persist to SD)
   home set <leg> c f t          : set one leg homes (centideg)
   home deg <leg> c f t          : set one leg homes (degrees)
   log show|serial|sd|both       : logging destinations
   log ls [path]                 : list SD directory (default '/')
   log cat <path> [max_bytes]    : print SD file to Serial (0 = whole file)
   R / r                         : software reboot (splash + info)

   WHY ROUND-ROBIN READ?
   ---------------------
   • LX16A telemetry can be slow/variable; reading 1/tick keeps timing deterministic.
   • Others are predicted forward from last state + command, corrected on their read turn.

   KEY SAFETY POINTS
   -----------------
   • dt_loop computed once/guarded; NaN/Inf guards everywhere.
   • Derivatives LPF + spike clamp by slew.
   • Integrator cleared on over-temp (>70C) or low bus voltage (<7.0V).

   TUNING BLOCK (starter values)
   -----------------------------
   • Foot path: STANCE_HEIGHT = -80 mm, STRIDE_LEN = 80 mm, LIFT = 20 mm
   • VSD (base): stance/swing per-DOF (see DEFAULT_VSD_*). These set compliance feel.
   • PID per-DOF defaults: COXA(24,32,0.5), FEMUR(28,36,0.6), TIBIA(22,32,0.5)
   • Slew default: 300 deg/s per-DOF (converted to rad/s internally)

   REFRACTORING NOTES (2025-10-13)
   --------------------------------
   • Eliminated dynamic String usage in hot paths (console/file parsing).
   • Added live memory gauges: freeHeapGap(), stackFreeNow(), maxHeapAllocTest().
   • Enabled CrashReport printing at boot to diagnose hard faults.
   • Non-blocking serial console with fixed buffers + strtok_r tokenization.
   • Line-by-line SD file reader (no String accumulation).
  • 166 Hz loop via IntervalTimer; set tickFlag in ISR; main loop processes on tick.
*/

/* (removed duplicate banner; single authoritative banner above) */

#include <Streaming.h>
#include <string.h>  // for strtok_r
#include <ctype.h>   // for tolower
#include <malloc.h>
#include "Hexapod_v1.0.h"
#include "Logging.h"
#include <CrashReport.h>
#include "ControllerState.h"
#include "Config.h"

// IntelliSense-only fallback for strtok_r to silence parser squiggles.
// Teensy/newlib provides strtok_r at build time; this shim is ignored by the compiler.
#ifdef __INTELLISENSE__
extern "C" inline char* strtok_r(char* s, const char* delim, char** saveptr) {
  (void)saveptr; // not used in fallback
  return strtok(s, delim);
}
#endif

// ───────────────────────────────────────────────────────────────────────────────
// SAFETY LIMITS (configurable via /config.txt)
// ───────────────────────────────────────────────────────────────────────────────
// Trip conditions default conservatively for hobby bus servos. If exceeded,
// we clear integrators, stop gait, disable all joints (torque off), and require
// operator intervention (re-enable) to resume. Thresholds can be overridden via
// /config.txt keys: safety.over_temp_c, safety.low_bus_mv, safety.min_valid_mv
static const int16_t DEF_SAFETY_OVER_TEMP_C = 70;     // deg C
static const int16_t DEF_SAFETY_LOW_V_MV    = 7000;   // mV (7.0 V)
static const int16_t DEF_SAFETY_MIN_VALID_MV= 3000;   // mV (ignore bogus early reads)

static int16_t g_over_temp_c = DEF_SAFETY_OVER_TEMP_C;
static int16_t g_low_v_mv    = DEF_SAFETY_LOW_V_MV;
static int16_t g_min_valid_mv= DEF_SAFETY_MIN_VALID_MV;

static bool   g_safety_tripped = false;
static char   g_safety_reason[64] = {0};
static int16_t g_last_tempC = 0;
static int16_t g_last_mV    = 0;

static void safetyLoadConfig() {
  Config::ensureFile();
  long tC = Config::getInt("safety.over_temp_c", g_over_temp_c);
  long low = Config::getInt("safety.low_bus_mv", g_low_v_mv);
  long min = Config::getInt("safety.min_valid_mv", g_min_valid_mv);
  // Clamp to sane ranges
  if (tC < 40) tC = 40; if (tC > 100) tC = 100;
  if (low < 5000) low = 5000; if (low > 12000) low = 12000;
  if (min < 1000) min = 1000; if (min > low-500) min = low-500;
  g_over_temp_c = (int16_t)tC;
  g_low_v_mv    = (int16_t)low;
  g_min_valid_mv= (int16_t)min;
}

static bool safetySaveInt(const char* key, long v) {
  Config::ensureFile();
  return Config::setInt(key, v);
}

static void safetyClearIntegrators(ControllerState* cs) {
  for (int j = 0; j < ControllerState::N_JOINTS; ++j) cs->J[j].pid.i_state = 0.0f;
}

static void safetyTrip(ControllerState* cs, const char* reason) {
  if (g_safety_tripped) return;  // sticky until user intervention
  g_safety_tripped = true;
  strncpy(g_safety_reason, reason ? reason : "unknown", sizeof(g_safety_reason)-1);
  g_safety_reason[sizeof(g_safety_reason)-1] = '\0';

  // Clear integrators first, then disable motion
  safetyClearIntegrators(cs);
  cs->GAIT_RUN = false;

  for (int j = 0; j < ControllerState::N_JOINTS; ++j) {
    cs->J[j].enabled = false;
    if (cs->J[j].srv) cs->J[j].srv->disable();
  }

  Serial.println("\n[SAFETY] TRIPPED → Motors disabled and gait stopped.");
  Serial.print  ("         Reason: "); Serial.println(g_safety_reason);
  Serial.print  ("         Last tempC="); Serial.print(g_last_tempC);
  Serial.print  ("  bus mV=");         Serial.println(g_last_mV);
  Serial.println("         Use 'e' or 'le <leg>' to re-enable once safe.");
}

/*
   CHANGELOG
   ---------
   2025-10-19
     - Safety & guards: config-backed thresholds (over_temp_c, low_bus_mv, min_valid_mv) in /config.txt
       • Sticky trip with console: 'safety show|set|clear'; stops gait + disables joints until re-enabled
       • Clear PID integrators on warning (hot/low-V) before a trip; status shows last tempC/mV
       • Print and clear Teensy CrashReport at boot for post-mortem
     - Gait/IK (WIP): parametric foot trajectory; integrated user 'calculateIK' (centideg) with home offsets and axis mapping
       • 'gait run|stop' restored; q_des derived from Trajectory → IK → clamps
       • Actuation guarded: move_time(...) remains commented pending HW validation
     - Telemetry/timing: enforce 1 read/tick (RR); removed duplicate block; improved dt guards
     - Logging: schema v1.1; cycle-based 'log every <n>' persisted to /config.txt; supports 'log off' and safe deletions
     - Home tools: 'home read <leg>' added; homes persisted as 'home_cdeg' with legacy migration
     - Cleanup/docs: removed unused legIK_3dof; help/status aligned; default boot logging clarified as SD Only

   2025-10-17
     - Config: Introduced unified key=value config at /config.txt (see Config.h).
       • Homes now stored as 'home_cdeg' (18 comma-separated centidegree ints).
       • Log cadence stored as 'log.every=<n>' cycles; used by 'log every <n>'.
       • Migration: on boot, if home_cdeg missing but legacy HOME_CFG_PATH exists, import it.
       • Help/docs updated to reference /config.txt.
   2025-10-16
     - Console: added 's' status with loop timing and system snapshot; aligned docs to 166 Hz.
     - Console: 'stance' stops gait and holds home joint angles; resets per-leg phase_t.
     - Logging: mode controls 'log show|serial|sd|both' and SD tools 'log ls' / 'log cat'.
     - Logging: safe deletion 'log del <path>' and 'log delall' (excludes current log).
     - Logging: cycle-based interval via 'log every <n>' (emit every n control cycles); removed ms-based request.
     - Per-leg control: 'le <leg>' enables the leg (others unchanged); 'ld <leg>' disables the leg.
     - Reboot: wired 'R'/'r' to teensyReboot() (Cortex-M AIRCR software reset) with splash/notice.
  - Help: HELP_TEXT synchronized with current command set; updated 'home move' semantics (optional off/disable).
  - Per-leg enable/disable now also toggles servo torque via srv->enable()/disable().
     - State: added motors_on and last_dt_loop; status prints now include loop dt.
     - Loop: improved dt guards; store last_dt_loop; clarified one-joint-per-tick read path.
     - Style: added STYLE.md; expanded comments and whitespace; critical trailing comments.

   2025-10-13
     - Refactoring notes: non-blocking console, memory gauges, CrashReport, 166 Hz ISR, SD readers.
*/

// ─── Forward declarations for moved namespace (so this file can call them) ───
namespace HomeCfg {
bool save(const long* homes, const char* path);
bool load(long* outHomes, const char* path);
void ensureFile(const long* defaults, const char* path);
}


// =====================================================================
//                                 HELPERS
// =====================================================================

static inline int     legOf(int jointIdx)         { return jointIdx / DOF_PER_LEG; }
static inline float   deg2rad(float d)            { return d * (PI / 180.0f); }
static inline float   rad2deg(float r)            { return r * (180.0f / PI); }
static inline int32_t rad_to_cdeg(float r)        { return (int32_t)lroundf(rad2deg(r) * 100.0f); }
static inline float   cdeg_to_rad(int32_t cdeg)   { return deg2rad(cdeg / 100.0f); }
static inline float   sat(float x, float a, float b){ return x < a ? a : (x > b ? b : x); }
static inline bool    isFinite(float x)           { return isfinite(x); }
static inline float   finite_or(float x, float f) { return isFinite(x) ? x : f; }
static inline float   lpf1(float y_prev, float x, float a){
  if (!isFinite(y_prev)) y_prev = 0.0f;
  if (!isFinite(x))      x      = 0.0f;
  if (a < 0.0f) a = 0.0f; else if (a > 1.0f) a = 1.0f;
  return (1.0f - a)*y_prev + a*x;
}

// ───────────────────────────────────────────────────────────────────────────────
// KINEMATICS AND TRAJECTORY HELPERS
// ───────────────────────────────────────────────────────────────────────────────
struct Vector3 { float x, y, z; };

// IK function to calculate servo angles from foot position (adapted from user code)
// Inputs:
//  - leg: [0..5]
//  - x=lateral(mm), y=vertical(mm, up+), z=forward(mm)
//  - homeAngles: centidegrees array of size N_JOINTS
// Outputs:
//  - angles: centidegrees [coxa, femur, tibia], clamped to 0..24000
// Note: Using primitive params avoids Arduino's auto-prototype issue with custom types.
static bool calculateIK(int leg, float x, float y, float z, int* angles, const long* homeAngles) {
  const int LEG_SERVOS = ControllerState::DOF_PER_LEG; // 3
  const int servoIdxBase = leg * LEG_SERVOS;

  // x: lateral (left +), y: vertical (up +), z: forward (+)

  // Link lengths (mm)
  const float COXA_LENGTH  = COXA_LENGTH_MM;
  const float FEMUR_LENGTH = FEMUR_LENGTH_MM;
  const float TIBIA_LENGTH = TIBIA_LENGTH_MM;

  // Coxa angle (lateral movement, 0° = forward, positive left)
  float coxaAngleRad = atan2f(x, z);
  float coxaAngleDeg = rad2deg(coxaAngleRad);
  // Apply servo home offset; original code subtracts 90° baseline
  float coxaAngleCentideg = (coxaAngleDeg * 100.0f) + (float)homeAngles[servoIdxBase] - 9000.0f;

  // Distances in sagittal plane
  float L = sqrtf((x * x) + (z * z)); // horizontal distance hip axis to foot
  float Dx = (L - COXA_LENGTH);
  float D  = sqrtf(Dx * Dx + (y * y));

  // Workspace guard
  if (D > (FEMUR_LENGTH + TIBIA_LENGTH) || D < fabsf(FEMUR_LENGTH - TIBIA_LENGTH)) {
    // out of reach -> clamp D into workspace but report false
    float Dmax = FEMUR_LENGTH + TIBIA_LENGTH - 1e-3f;
    float Dmin = fabsf(FEMUR_LENGTH - TIBIA_LENGTH) + 1e-3f;
    D = sat(D, Dmin, Dmax);
    // keep computing but return false after angles are formed
  }

  // Femur angle
  float alpha1 = (D > 1e-6f) ? asinf(Dx / D) : 0.0f; // angle of D from horizontal
  float alpha2 = acosf(sat((D * D + FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH) / (2.0f * FEMUR_LENGTH * D), -1.0f, 1.0f));
  float alpha  = alpha1 + alpha2; // femur relative to horizontal
  float femurAngleDeg = rad2deg(radians((homeAngles[servoIdxBase + 1] / 100.0f) - 90.0f) + alpha);
  float femurAngleCentideg = femurAngleDeg * 100.0f;

  // Tibia angle
  float beta  = acosf(sat((TIBIA_LENGTH * TIBIA_LENGTH + FEMUR_LENGTH * FEMUR_LENGTH - D * D) / (2.0f * FEMUR_LENGTH * TIBIA_LENGTH), -1.0f, 1.0f));
  float gamma = PI - beta; // complement for servo control
  float tibiaAngleDeg = rad2deg(radians(homeAngles[servoIdxBase + 2] / 100.0f) + gamma);
  float tibiaAngleCentideg = tibiaAngleDeg * 100.0f;

  // Clamp to servo range
  angles[0] = constrain((int)lroundf(coxaAngleCentideg), 0, 24000);
  angles[1] = constrain((int)lroundf(femurAngleCentideg), 0, 24000);
  angles[2] = constrain((int)lroundf(tibiaAngleCentideg), 0, 24000);

  // Return false if unreachable but still provide best-effort angles
  return !( (sqrtf(Dx*Dx + y*y) > (FEMUR_LENGTH + TIBIA_LENGTH)) || (sqrtf(Dx*Dx + y*y) < fabsf(FEMUR_LENGTH - TIBIA_LENGTH)) );
}

// Parametric foot trajectory along x with lift on swing; y set by leg lateral offset.
static void footTrajectory_mm(const ControllerState* cs, int leg,
                              float* x_mm, float* y_mm, float* z_mm) {
  const auto& lp = cs->L[leg];
  const float stride = cs->STRIDE_LEN_MM;
  const float lift   = cs->LIFT_MM;
  const float z0     = cs->STANCE_HEIGHT_MM; // negative (down)
  const float y0     = cs->LATERAL_OFFSET_MM[leg];

  float x = 0, z = z0;
  if (lp.phase == CS_LegPhase::STANCE) {
    float u = (lp.stance_dur > 1e-6f) ? (lp.phase_t / lp.stance_dur) : 0.0f;
    u = sat(u, 0.0f, 1.0f);
    x = (stride * 0.5f) - u * stride;  // +S/2 -> -S/2
    z = z0;
  } else {
    float u = (lp.swing_dur > 1e-6f) ? (lp.phase_t / lp.swing_dur) : 0.0f;
    u = sat(u, 0.0f, 1.0f);
    x = (-stride * 0.5f) + u * stride; // -S/2 -> +S/2
    // Smooth lift arc; zero at ends, peak at mid-swing
    z = z0 + lift * sinf(PI * u);
  }

  if (x_mm) *x_mm = x;
  if (y_mm) *y_mm = y0;
  if (z_mm) *z_mm = z;
}

// ───────────────────────────────────────────────────────────────────────────────
// MEMORY GAUGES (same as before)
// ───────────────────────────────────────────────────────────────────────────────

extern "C" char* sbrk(int incr);
extern unsigned long _ebss;
extern unsigned long _estack;

static uint32_t* __canary_start         = nullptr;
static uint32_t* __canary_end           = nullptr;
static bool      __canary_ok            = false;
static uint32_t  __canary_painted_bytes = 0;

static uint32_t freeHeapGap() {
  volatile uint32_t sp_snap = 0;
  uint8_t* sp = (uint8_t*)&sp_snap;

  uint8_t* heap_base = (uint8_t*)&_ebss;
  struct mallinfo mi = mallinfo();
  size_t heap_used   = (size_t)mi.uordblks;
  uint8_t* heap_top  = heap_base + heap_used;

  return (sp > heap_top) ? (uint32_t)(sp - heap_top) : 0u;
}

static void stackCanaryInit() {
  __canary_ok            = false;
  __canary_painted_bytes = 0;

  uint8_t* heap_base = (uint8_t*)&_ebss;
  struct mallinfo mi = mallinfo();
  size_t heap_used   = (size_t)mi.uordblks;
  uint8_t* heap_top  = heap_base + heap_used;

  volatile uint32_t sp_snap = 0;
  uint8_t* sp = (uint8_t*)&sp_snap;

  const size_t HEAP_MARGIN = 512;
  const size_t STACK_GUARD = 2048;
  const size_t MIN_WINDOW  = 2048;

  uint32_t* base = (uint32_t*)(heap_top + HEAP_MARGIN);
  uint32_t* end  = (uint32_t*)(sp - STACK_GUARD);

  if (end <= base) {
    __canary_start = __canary_end = nullptr;
    return;
  }

  size_t window = (size_t)((uint8_t*)end - (uint8_t*)base);
  if (window < MIN_WINDOW) {
    __canary_start = __canary_end = nullptr;
    return;
  }

  __canary_start = base;
  __canary_end   = end;

  for (uint32_t* p = __canary_start; p < __canary_end; ++p) {
    *p = 0xDEADBEEF;
  }

  __canary_ok            = true;
  __canary_painted_bytes = (uint32_t)window;
}

static uint32_t stackFreeNow() {
  if (!__canary_ok || !__canary_start || !__canary_end) return 0u;

  uint32_t* p = __canary_start;
  while (p < __canary_end && *p == 0xDEADBEEF) ++p;

  // FREE bytes = intact prefix length
  return (uint32_t)((uint8_t*)p - (uint8_t*)__canary_start);
}

static uint32_t maxHeapAllocTest() {
  uint32_t sz = 1024;
  for (;;) {
    void* q = malloc(sz);
    if (!q) break;
    free(q);
    sz <<= 1;
    if (sz > (1u << 24)) break;
  }
  return sz >> 1;
}

#ifndef MEM_GAUGES
#define MEM_GAUGES 0
#endif

// ───────────────────────────────────────────────────────────────────────────────
// GLOBAL STATE (unchanged)
// ───────────────────────────────────────────────────────────────────────────────

static ControllerState s;

// ───────────────────────────────────────────────────────────────────────────────
// TICKER / REBOOT / CONSOLE (unchanged logic)
// ───────────────────────────────────────────────────────────────────────────────

IntervalTimer ticker;
void isr() {
  s.tickFlag = true;  // ISR: signal main loop to process one 166 Hz tick
}

uint8_t  rr            = 0;
uint32_t loop_stamp_us = 0;

void teensyReboot() {
  Serial.println("[SYS] Reboot requested...");
  Serial.flush();
  delay(50);
  Hexapod::printSplash();
  Serial.print("[SYS] *** REBOOTING "); Serial.print(FW_NAME);
  Serial.print(" v"); Serial.print(FW_VERSION); Serial.println(" ***");
  SCB_AIRCR = 0x05FA0004; // Software reset
}

// Console (fixed buffer)
static char   cmdBuf[256];
static size_t cmdLen = 0;

static void printStartupInfo(ControllerState* cs) {
  Serial << "Hexapod Controller - Teensy 4.1" << endl;
  Serial << "Type 'help' for commands." << endl;
  Log::begin(); Log::setMode(Log::SD_ONLY); Log::setLevel(Log::DETAIL);
  // Load persisted logging cadence if available
  cs->log_every_cycles = Log::loadEvery(cs->log_every_cycles);
}

// Pretty status readout
// -----------------------------------------------------------------------------
// Print current runtime state: gait, motors, loop timing, logging, per-leg
// enable summary, and a light memory snapshot. Safe to invoke from the
// interactive console context (not from ISR).
static void printStatus(ControllerState* cs) {
  Serial.println("[STATUS]");

  // High-level toggles
  Serial.print("  GAIT: "); Serial.println(cs->GAIT_RUN ? "RUNNING" : "STOPPED");
  Serial.print("  MOTORS: "); Serial.println(cs->motors_on ? "ON" : "OFF");

  // Loop timing (commanded vs last measured)
  Serial.print("  LOOP: "); Serial.print(cs->control_loop_Hz, 1); Serial.print(" Hz");
  Serial.print(" (Ts="); Serial.print(cs->Ts, 6); Serial.print(" s)  dt(last)=");
  Serial.print(cs->last_dt_loop, 6); Serial.println(" s");

  // Logging destination and SD availability
  Serial.print("  LOG: mode="); Serial.print(Log::modeName());
  Serial.print("; SD="); Serial.println(Log::sdReady() ? "OK" : "NOT AVAILABLE");
  Serial.print("  LOG cadence: every "); Serial.print(cs->log_every_cycles); Serial.println(" cycles");

  // Safety
  Serial.print("  SAFETY: ");
  if (g_safety_tripped) {
    Serial.print("TRIPPED ("); Serial.print(g_safety_reason); Serial.println(")");
  } else {
    Serial.println("OK");
  }
  Serial.print("    last tempC="); Serial.print(g_last_tempC);
  Serial.print("  bus mV=");      Serial.println(g_last_mV);

  // Per-leg enable summary
  for (int leg = 0; leg < ControllerState::N_LEGS; ++leg) {
    int en = 0;
    for (int dof = 0; dof < ControllerState::DOF_PER_LEG; ++dof) {
      const int idx = leg * ControllerState::DOF_PER_LEG + dof;
      if (cs->J[idx].enabled) ++en;
    }
    Serial.print("  leg "); Serial.print(leg); Serial.print(": ");
    Serial.print(en); Serial.print("/"); Serial.println(ControllerState::DOF_PER_LEG);
  }

  // Memory snapshot (optional)
  // Note: lightweight probes; safe to call here. Use MEM_GAUGES for periodic prints.
  Serial.printf("  MEM: freeGap=%u stackFree=%u maxAlloc=%u\n",
                freeHeapGap(), stackFreeNow(), maxHeapAllocTest());
}

static inline bool streqi(const char* a, const char* b) {
  if (!a || !b) return false;
  while (*a && *b) {
    char ca = (char)tolower((unsigned char) * a++);
    char cb = (char)tolower((unsigned char) * b++);
    if (ca != cb) return false;
  }
  return *a == *b;
}

static void handleCommandLineC(ControllerState* cs, char* line) {
  static char* argv[16];
  int   argc = 0;
  char* save = nullptr;

  for (char* tok = strtok_r(line, " \t\r\n", &save);
       tok && argc < (int)(sizeof(argv) / sizeof(argv[0]));
       tok = strtok_r(nullptr, " \t\r\n", &save)) {
    argv[argc++] = tok;
  }
  if (argc == 0) return;

  if (streqi(argv[0], "help") || streqi(argv[0], "h") || streqi(argv[0], "?")) {
    Serial << Hexapod::HELP_TEXT;
    return;
  }

  if (streqi(argv[0], "s") || streqi(argv[0], "status")) {
    printStatus(cs);
    return;
  }

  // Acknowledge and clear sticky safety latch (does not re-enable motors)
  if (streqi(argv[0], "safety") && argc >= 2 && streqi(argv[1], "clear")) {
    g_safety_tripped = false;
    g_safety_reason[0] = '\0';
    Serial.println("[SAFETY] latch cleared. Use 'e' or 'le <leg>' to re-enable when safe.");
    return;
  }

  // Show safety thresholds and last readings
  if (streqi(argv[0], "safety") && argc >= 2 && streqi(argv[1], "show")) {
    Serial.println("[SAFETY] thresholds and last readings:");
    Serial.print("  over_temp_c="); Serial.print(g_over_temp_c);
    Serial.print("  low_bus_mv=");  Serial.print(g_low_v_mv);
    Serial.print("  min_valid_mv="); Serial.println(g_min_valid_mv);
    Serial.print("  last tempC="); Serial.print(g_last_tempC);
    Serial.print("  bus mV=");      Serial.println(g_last_mV);
    Serial.print("  state="); Serial.println(g_safety_tripped ? "TRIPPED" : "OK");
    return;
  }

  // Set and persist a safety threshold: safety set <over_temp_c|low_mv|min_mv> <value>
  if (streqi(argv[0], "safety") && argc >= 4 && streqi(argv[1], "set")) {
    const char* which = argv[2];
    long val = strtol(argv[3], nullptr, 10);
    if (streqi(which, "over_temp_c")) {
      if (val < 40 || val > 100) { Serial.println("[SAFETY] over_temp_c out of range (40..100)"); return; }
      g_over_temp_c = (int16_t)val;
      if (!safetySaveInt("safety.over_temp_c", val)) Serial.println("[SAFETY] persist failed");
      Serial.print("[SAFETY] over_temp_c="); Serial.println(g_over_temp_c);
      return;
    } else if (streqi(which, "low_mv")) {
      if (val < 5000 || val > 12000) { Serial.println("[SAFETY] low_mv out of range (5000..12000)"); return; }
      g_low_v_mv = (int16_t)val;
      if (!safetySaveInt("safety.low_bus_mv", val)) Serial.println("[SAFETY] persist failed");
      Serial.print("[SAFETY] low_bus_mv="); Serial.println(g_low_v_mv);
      return;
    } else if (streqi(which, "min_mv")) {
      if (val < 1000 || val > g_low_v_mv - 500) { Serial.println("[SAFETY] min_mv out of range (1000..low_mv-500)"); return; }
      g_min_valid_mv = (int16_t)val;
      if (!safetySaveInt("safety.min_valid_mv", val)) Serial.println("[SAFETY] persist failed");
      Serial.print("[SAFETY] min_valid_mv="); Serial.println(g_min_valid_mv);
      return;
    } else {
      Serial.println("[SAFETY] usage: safety set over_temp_c <40..100> | safety set low_mv <5000..12000> | safety set min_mv <1000..(low_mv-500)>");
      return;
    }
  }

  // Software reboot command (R / r)
  if (streqi(argv[0], "r") || streqi(argv[0], "R")) {
    teensyReboot();
    return;
  }

  if (streqi(argv[0], "mem")) {
    Serial.printf("[MEM] freeGap=%u stackFree=%u maxAlloc=%u\n",
                  freeHeapGap(), stackFreeNow(), maxHeapAllocTest());
    return;
  }

  // Logging controls and SD browsing
  if (streqi(argv[0], "log")) {
    if (argc == 1 || streqi(argv[1], "show")) {
      Serial.print("[LOG] mode="); Serial.print(Log::modeName());
      Serial.print("; SD="); Serial.print(Log::sdReady() ? "OK" : "NOT AVAILABLE");
      Serial.print("; file="); Serial.print(Log::currentFile());
      Serial.print("; every="); Serial.print(cs->log_every_cycles); Serial.println(" cycles");
      return;
    }
    if (streqi(argv[1], "serial")) { Log::setMode(Log::SERIAL_ONLY); return; }
    if (streqi(argv[1], "sd"))     { Log::setMode(Log::SD_ONLY);     return; }
    if (streqi(argv[1], "both"))   { Log::setMode(Log::BOTH);         return; }
    if (streqi(argv[1], "off"))    { Log::setMode(Log::NONE);         return; }

  if (streqi(argv[1], "ls"))  { Log::list("/"); return; }
  if (argc >= 3 && streqi(argv[1], "cat")) { Log::cat(argv[2]); return; }
  if (argc >= 3 && streqi(argv[1], "del")) { Log::del(argv[2]); return; }
  if (streqi(argv[1], "delall"))           { Log::delAll(true);  return; }

  if (argc >= 3 && streqi(argv[1], "every")) {
      long n = strtol(argv[2], nullptr, 10);
      if (n < 1) { Serial.println("[LOG] 'every' must be >= 1 cycles"); return; }
      cs->log_every_cycles = (uint32_t)n;
      cs->log_cycle_counter = 0; // apply immediately
    // Persist to SD if available
    Log::saveEvery(cs->log_every_cycles);
      Serial.print("[LOG] logging every "); Serial.print(cs->log_every_cycles); Serial.println(" cycles");
      return;
  }

  Serial.println("[LOG] usage: log show|serial|sd|both|off | log ls | log cat <path> | log del <path> | log delall | log every <n>");
    return;
  }

  // Legacy alias 'lo' removed. Use 'log ...' commands.

  // Stance: stop gait and hold home joint angles (pre-IK implementation)
  if (streqi(argv[0], "stance")) {
    cs->GAIT_RUN = false;

    // Convert home angles (centideg) to radians and clamp to joint limits
    // Set desired joint angles to homes (centideg -> radians), clamped to limits
    for (int j = 0; j < ControllerState::N_JOINTS; ++j) {
      float qh = cdeg_to_rad((int32_t)cs->home_cdeg[j]);
      if (!isFinite(qh)) qh = 0.0f;
      cs->J[j].q_des = sat(qh, cs->J[j].qmin, cs->J[j].qmax);
    }
    
    // Reset phase timers for determinism when resuming gait later
    for (int leg = 0; leg < ControllerState::N_LEGS; ++leg) cs->L[leg].phase_t = 0.0f;

    Serial.println("[STANCE] GAIT STOP; holding home joint angles.");
    return;
  }

  // ---------------------------------------------------------------------------
  // Per-leg enable/disable
  // le <leg> : enable the specified leg (leave others unchanged) and torque-on
  // ld <leg> : disable the specified leg (leave others unchanged) and torque-off
  // ---------------------------------------------------------------------------
  if (streqi(argv[0], "le") || streqi(argv[0], "ld")) {
    if (argc < 2) {
      Serial.println("[SERVOS] usage: le <leg> | ld <leg>");
      return;
    }

    // Parse leg index with bounds check
    const int leg = atoi(argv[1]);
    if (leg < 0 || leg >= ControllerState::N_LEGS) {
      Serial.print("[SERVOS] leg out of range (0..");
      Serial.print(ControllerState::N_LEGS - 1);
      Serial.println(")");
      return;
    }

    if (streqi(argv[0], "le")) {
      // Enable the specified leg only (do not change other legs); torque ON
      for (int d = 0; d < ControllerState::DOF_PER_LEG; ++d) {
        const int j = leg * ControllerState::DOF_PER_LEG + d;
        s.J[j].enabled = true;
        if (s.J[j].srv) s.J[j].srv->enable();
      }
      Serial.print("[SERVOS] enabled leg "); Serial.println(leg);
      return;
    } else {
      // Disable the specified leg only; torque OFF
      for (int d = 0; d < ControllerState::DOF_PER_LEG; ++d) {
        const int j = leg * ControllerState::DOF_PER_LEG + d;
        s.J[j].enabled = false;
        if (s.J[j].srv) s.J[j].srv->disable();
      }
      Serial.print("[SERVOS] disabled leg "); Serial.println(leg);
      return;
    }
  }
  if (streqi(argv[0], "gait")) {
    if (argc >= 2 && streqi(argv[1], "run")) {
      cs->GAIT_RUN = true;
      Serial.println("[GAIT] RUN");
      return;
    }
    if (argc >= 2 && streqi(argv[1], "stop")) {
      cs->GAIT_RUN = false;
      for (int leg = 0; leg < ControllerState::N_LEGS; ++leg) {
        cs->L[leg].phase_t = 0.0f;
      }
      Serial.println("[GAIT] STOP");
      return;
    }
    Serial.println("[GAIT] usage: gait run | gait stop");
    return;
  }

  if (streqi(argv[0], "home")) {
    // Capture current joint angles as homes (RAM only): home read <leg>
    if (argc >= 3 && streqi(argv[1], "read")) {
      const int leg = atoi(argv[2]);
      if (leg < 0 || leg >= ControllerState::N_LEGS) {
        Serial.print("[HOME] leg out of range (0.."); Serial.print(ControllerState::N_LEGS - 1); Serial.println(")");
        return;
      }
      const int j0 = leg * ControllerState::DOF_PER_LEG;
      // Require leg disabled to avoid fighting torque
      for (int d = 0; d < ControllerState::DOF_PER_LEG; ++d) {
        if (cs->J[j0 + d].enabled) {
          Serial.println("[HOME] leg must be disabled first (use 'ld <leg>')");
          return;
        }
      }
      // Torque-off for safety, then read
      for (int d = 0; d < ControllerState::DOF_PER_LEG; ++d) {
        const int j = j0 + d;
        if (cs->J[j].srv) cs->J[j].srv->disable();
      }
      for (int d = 0; d < ControllerState::DOF_PER_LEG; ++d) {
        const int j = j0 + d;
        int32_t cdeg = cs->J[j].srv ? cs->J[j].srv->pos_read() : 0;
        float q = cdeg_to_rad(cdeg);
        if (!isFinite(q)) q = 0.0f;
        q = sat(q, cs->J[j].qmin, cs->J[j].qmax);
        cs->home_cdeg[j] = rad_to_cdeg(q);                    // store in RAM only
      }
      {
        // Echo captured values (centideg and deg) for visibility
        long c = cs->home_cdeg[j0 + ControllerState::COXA];
        long f = cs->home_cdeg[j0 + ControllerState::FEMUR];
        long t = cs->home_cdeg[j0 + ControllerState::TIBIA];
        Serial.print("[HOME] leg "); Serial.print(leg);
        Serial.print(" cdeg=("); Serial.print(c); Serial.print(','); Serial.print(f); Serial.print(','); Serial.print(t); Serial.print(")  deg=(");
        Serial.print(c / 100.0f); Serial.print(','); Serial.print(f / 100.0f); Serial.print(','); Serial.print(t / 100.0f); Serial.println(")");
      }
      return;
    }
    // Show current homes (centideg and deg)
    if (argc >= 2 && streqi(argv[1], "show")) {
      for (int leg = 0; leg < ControllerState::N_LEGS; ++leg) {
        const int j0 = leg * ControllerState::DOF_PER_LEG;
        long c = cs->home_cdeg[j0 + ControllerState::COXA];
        long f = cs->home_cdeg[j0 + ControllerState::FEMUR];
        long t = cs->home_cdeg[j0 + ControllerState::TIBIA];
        Serial.print("[HOME] leg "); Serial.print(leg);
        Serial.print(" cdeg=("); Serial.print(c); Serial.print(','); Serial.print(f); Serial.print(','); Serial.print(t); Serial.print(")  deg=(");
        Serial.print(c / 100.0f); Serial.print(','); Serial.print(f / 100.0f); Serial.print(','); Serial.print(t / 100.0f); Serial.println(")");
      }
      return;
    }

    // Restore baked-in defaults (assumption: 0 cdeg for all joints)
    if (argc >= 2 && streqi(argv[1], "defaults")) {
      for (int j = 0; j < ControllerState::N_JOINTS; ++j) cs->home_cdeg[j] = 0;
      Serial.println("[HOME] defaults restored in RAM (0 cdeg for all joints). Use 'home save' to persist.");
      return;
    }

    // Move one leg to its home joint angles, wait, then torque-off & disable:
    //   home move <leg> [ms]
    if (argc >= 3 && streqi(argv[1], "move")) {
      const int leg = atoi(argv[2]);
      if (leg < 0 || leg >= ControllerState::N_LEGS) {
        Serial.print("[HOME] leg out of range (0.."); Serial.print(ControllerState::N_LEGS - 1); Serial.println(")");
        return;
      }
      int ms = 800;                                           // default 800 ms
      if (argc >= 4 && (isdigit((unsigned char)argv[3][0]) || argv[3][0] == '-')) {
        ms = atoi(argv[3]);
      }
      
      if (ms < 50)  ms = 50;                                  // clamp 50..5000 ms
      if (ms > 5000) ms = 5000;

      const int j0 = leg * ControllerState::DOF_PER_LEG;
      for (int d = 0; d < ControllerState::DOF_PER_LEG; ++d) {
        const int j = j0 + d;
        float qh = cdeg_to_rad((int32_t)cs->home_cdeg[j]);
        if (!isFinite(qh)) qh = 0.0f;
        qh = sat(qh, cs->J[j].qmin, cs->J[j].qmax);           // guard to joint limits
        const int32_t cdeg_cmd = rad_to_cdeg(qh);

        cs->J[j].q_des = qh;                                  // keep controller target in sync
        if (cs->J[j].srv) {
          cs->J[j].srv->move_time(cdeg_cmd, ms);              // direct actuation to home
        }
      }
      // Wait for the commanded duration to finish motion, then torque-off & disable
      delay(ms);
      for (int d = 0; d < ControllerState::DOF_PER_LEG; ++d) {
        const int j = j0 + d;
        cs->J[j].enabled = false;
        if (cs->J[j].srv) cs->J[j].srv->disable();
      }
      Serial.print("[HOME] moved leg "); Serial.print(leg);
      Serial.print(" to home in "); Serial.print(ms); Serial.println(" ms, then disabled torque");
      return;
    }

    // Set one leg homes in centidegrees: home set <leg> <c> <f> <t>
    if (argc >= 6 && streqi(argv[1], "set")) {
      const int leg = atoi(argv[2]);
      if (leg < 0 || leg >= ControllerState::N_LEGS) {
        Serial.print("[HOME] leg out of range (0.."); Serial.print(ControllerState::N_LEGS - 1); Serial.println(")");
        return;
      }
      long c_cdeg = strtol(argv[3], nullptr, 10);
      long f_cdeg = strtol(argv[4], nullptr, 10);
      long t_cdeg = strtol(argv[5], nullptr, 10);

      // Clamp to joint limits via rad conversion
      const int j0 = leg * ControllerState::DOF_PER_LEG;
      float qc = sat(cdeg_to_rad((int32_t)c_cdeg), cs->J[j0 + ControllerState::COXA].qmin,  cs->J[j0 + ControllerState::COXA].qmax);
      float qf = sat(cdeg_to_rad((int32_t)f_cdeg), cs->J[j0 + ControllerState::FEMUR].qmin, cs->J[j0 + ControllerState::FEMUR].qmax);
      float qt = sat(cdeg_to_rad((int32_t)t_cdeg), cs->J[j0 + ControllerState::TIBIA].qmin, cs->J[j0 + ControllerState::TIBIA].qmax);

      cs->home_cdeg[j0 + ControllerState::COXA]  = rad_to_cdeg(qc);
      cs->home_cdeg[j0 + ControllerState::FEMUR] = rad_to_cdeg(qf);
      cs->home_cdeg[j0 + ControllerState::TIBIA] = rad_to_cdeg(qt);

      Serial.print("[HOME] leg "); Serial.print(leg); Serial.println(" homes updated (cdeg, clamped to limits)");
      return;
    }

    // Set one leg homes in degrees: home deg <leg> <c> <f> <t>
    if (argc >= 6 && streqi(argv[1], "deg")) {
      const int leg = atoi(argv[2]);
      if (leg < 0 || leg >= ControllerState::N_LEGS) {
        Serial.print("[HOME] leg out of range (0.."); Serial.print(ControllerState::N_LEGS - 1); Serial.println(")");
        return;
      }
      const float c_deg = atof(argv[3]);
      const float f_deg = atof(argv[4]);
      const float t_deg = atof(argv[5]);

      const int j0 = leg * ControllerState::DOF_PER_LEG;
      float qc = sat(deg2rad(c_deg), cs->J[j0 + ControllerState::COXA].qmin,  cs->J[j0 + ControllerState::COXA].qmax);
      float qf = sat(deg2rad(f_deg), cs->J[j0 + ControllerState::FEMUR].qmin, cs->J[j0 + ControllerState::FEMUR].qmax);
      float qt = sat(deg2rad(t_deg), cs->J[j0 + ControllerState::TIBIA].qmin, cs->J[j0 + ControllerState::TIBIA].qmax);

      cs->home_cdeg[j0 + ControllerState::COXA]  = rad_to_cdeg(qc);
      cs->home_cdeg[j0 + ControllerState::FEMUR] = rad_to_cdeg(qf);
      cs->home_cdeg[j0 + ControllerState::TIBIA] = rad_to_cdeg(qt);

      Serial.print("[HOME] leg "); Serial.print(leg); Serial.println(" homes updated from degrees (clamped)");
      return;
    }

    // Save / Load remain as before
  if (argc >= 2 && streqi(argv[1], "save")) { HomeCfg::save(cs->home_cdeg, HOME_CFG_PATH); return; }
  if (argc >= 2 && streqi(argv[1], "load")) { HomeCfg::load(cs->home_cdeg, HOME_CFG_PATH); return; }

    Serial.println("[HOME] usage: home show | home defaults | home read <leg> | home move <leg> [ms] | home set <leg> <c> <f> <t> | home deg <leg> <c> <f> <t> | home load | home save");
    return;
  }

  if (streqi(argv[0], "e")) {
    for (int i = 0; i < ControllerState::N_JOINTS; ++i) cs->J[i].enabled = true;
    Serial.println("[SERVOS] enabled all");
    return;
  }
  if (streqi(argv[0], "d")) {
    for (int i = 0; i < ControllerState::N_JOINTS; ++i) cs->J[i].enabled = false;
    Serial.println("[SERVOS] disabled all");
    return;
  }

  Serial.print("[ERR] unknown cmd: ");
  Serial.println(argv[0]);
}

// Small VSD tweak (unchanged)
static void set_vsd_for_leg(ControllerState* cs, int leg_index) {
  auto& lp = cs->L[leg_index];
  if (lp.phase == CS_LegPhase::STANCE) {
    const float u = (lp.stance_dur > 1e-6f) ? (lp.phase_t / lp.stance_dur) : 0.0f;
    cs->J[leg_index * ControllerState::DOF_PER_LEG + ControllerState::FEMUR]
    .vsd.tau_des += 0.15f * sinf(PI * u);
  }
}

// ───────────────────────────────────────────────────────────────────────────────
// SETUP (unchanged behavior)
// ───────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis(); while (!Serial && (millis() - t0) < 5000) {}
  delay(1000);
  Hexapod::printSplash();

  // CrashReport (Teensy): print any prior crash info for diagnostics
  if (CrashReport) {
    Serial.println("\n[CrashReport] Previous crash detected:\n-----------------------------------");
    Serial.print(CrashReport);
    Serial.println("-----------------------------------\n(End CrashReport)\n");
    CrashReport.clear();
  }

  stackCanaryInit();
  Serial.printf("[MEM] canary window = %u bytes\n", __canary_painted_bytes);
  Serial.printf("[MEM] freeGap=%u stackFree=%u maxAlloc=%u\n", freeHeapGap(), stackFreeNow(), maxHeapAllocTest());

  s.initDefaults();
  printStartupInfo(&s);

  // Load safety thresholds from /config.txt (with sane clamps)
  safetyLoadConfig();

  // Per-leg serial buses; enable 74HC126 buffers
  for (int leg = 0; leg < ControllerState::N_LEGS; ++leg) {
    pinMode(bufferEnablePins[leg], OUTPUT);
    digitalWrite(bufferEnablePins[leg], HIGH);

    s.legBus[leg] = new LX16ABus();
    s.legBus[leg]->begin(SERVO_PORTS[leg], 0);

    for (int dof = 0; dof < ControllerState::DOF_PER_LEG; ++dof) {
      int idx = leg * ControllerState::DOF_PER_LEG + dof;
      s.servos[idx]  = new LX16AServo(s.legBus[leg], SERVO_ID[leg][dof]);
      s.J[idx].srv   = s.servos[idx];
    }
  }

  // Home angles
  HomeCfg::ensureFile(s.home_cdeg, HOME_CFG_PATH);
  HomeCfg::load(s.home_cdeg, HOME_CFG_PATH);

  // Initial per-leg VSD
  for (int leg = 0; leg < ControllerState::N_LEGS; ++leg) set_vsd_for_leg(&s, leg);

  s.loop_stamp_us = micros();
  s.last_dt_loop = s.Ts;
  ticker.begin(isr, (uint32_t)(1000000.0f / s.control_loop_Hz));  // start periodic ISR
  Log::header();
}

// ───────────────────────────────────────────────────────────────────────────────
// LOOP (unchanged behavior)
// ───────────────────────────────────────────────────────────────────────────────

void loop() {
#if MEM_GAUGES
  static elapsedMillis _memT;
  if (_memT > 5000) {
    _memT = 0;
    Serial.printf("\n[MEM] freeGap=%u stackFree=%u maxAlloc=%u\n", freeHeapGap(), stackFreeNow(), maxHeapAllocTest());
  }
#endif

  float log_temp = 0;
  float log_voltage = 0;
  
  // Non-blocking console line assembly
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmdLen) {
        cmdBuf[cmdLen] = '\0';
        handleCommandLineC(&s, cmdBuf);
        cmdLen = 0;
      }
    } else if ((unsigned char)c >= 0x20 && c != 0x7F) {
      if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = c;
      } else {
        Serial.println("[ERR] command too long; discarded");
        cmdLen = 0;
      }
    } else {
      // ignore other control chars
    }
  }

  // Tick-gated control work
  if (!s.tickFlag) return;  // tick-gated: maintain deterministic cadence
  s.tickFlag = false;

  const uint32_t now_us = micros();
  const uint32_t loop_us = (uint32_t)(now_us - s.loop_stamp_us);
  float dt_loop = (float)loop_us * 1e-6f;
  if (!(dt_loop > 0.0f) || !isFinite(dt_loop) || dt_loop > 0.2f) dt_loop = s.Ts;  // guard bad/slow ticks
  s.loop_stamp_us = now_us;
  s.last_dt_loop = dt_loop;


  {
    // Actual measurement this tick (single joint only)
    // Enforce deterministic 1-read-per-tick via s.read_joint and incrementReadJoint().
    // Guard against unexpected null pointers.
    if (s.servos[s.read_joint]) {
      log_temp = s.servos[s.read_joint]->temp();
      log_voltage = s.servos[s.read_joint]->vin();
      s.J[s.read_joint].temp = log_temp;
      s.J[s.read_joint].vin = log_voltage;
      // Update last seen bus telemetry for safety logic
      g_last_tempC = (int16_t)log_temp;
      g_last_mV    = (int16_t)log_voltage;
      // Non-sticky hygiene: clear integrators if hot/low-V is observed
      if ((g_last_tempC >= g_over_temp_c) || (g_last_mV > g_min_valid_mv && g_last_mV <= g_low_v_mv)) {
        safetyClearIntegrators(&s);
      }

      int32_t cdeg = s.servos[s.read_joint]->pos_read();
      s.J[s.read_joint].q_meas = cdeg_to_rad(cdeg);
      if (!isFinite(s.J[s.read_joint].q_meas)) s.J[s.read_joint].q_meas = s.J[s.read_joint].q_est;
    }

    if (!s.J[s.read_joint].has_meas) {
      s.J[s.read_joint].has_meas = true;
      s.J[s.read_joint].q_est    = s.J[s.read_joint].q_meas;
      s.J[s.read_joint].dq_est   = 0.0f;
      s.J[s.read_joint].dq_meas  = 0.0f;
      s.J[s.read_joint].q_prev   = s.J[s.read_joint].q_meas;  // keep q_prev in sync for legacy paths
    } else {
      float dq_raw = (s.J[s.read_joint].q_meas - s.J[s.read_joint].q_est) / dt_loop;
      if (!isFinite(dq_raw) || fabsf(dq_raw) > 200.0f) dq_raw = 0.0f;
      s.J[s.read_joint].dq_est = lpf1(s.J[s.read_joint].dq_est, dq_raw, 0.20f);
      s.J[s.read_joint].q_est  = s.J[s.read_joint].q_meas;
      s.J[s.read_joint].dq_meas= s.J[s.read_joint].dq_est;
      s.J[s.read_joint].q_prev = s.J[s.read_joint].q_meas;    // update q_prev here as well
    }
  s.incrementReadJoint();  // advance RR read pointer (1 joint/tick)
  }

  // Advance gait and assign VSD bases
  for (int leg = 0; leg < ControllerState::N_LEGS; ++leg) {
    auto& lp = s.L[leg];
    if (s.GAIT_RUN) {
      float ph_dur = (lp.phase == CS_LegPhase::STANCE) ? lp.stance_dur : lp.swing_dur;
      lp.phase_t += dt_loop;
      if (lp.phase_t >= ph_dur) {
        lp.phase_t -= ph_dur;
        lp.phase = (lp.phase == CS_LegPhase::STANCE) ? CS_LegPhase::SWING : CS_LegPhase::STANCE;
      }
    }
    for (int dof = 0; dof < ControllerState::DOF_PER_LEG; ++dof) {
      CS_VSD base = (lp.phase == CS_LegPhase::STANCE) ? s.VSD_STANCE_BASE[dof] : s.VSD_SWING_BASE[dof];
      if (s.VSD_OVERRIDE_EN[dof]) base = s.VSD_OVERRIDE[dof];
      s.J[leg * ControllerState::DOF_PER_LEG + dof].vsd = base;
    }
  }

  // NOTE: Removed duplicate RR read block; all joint telemetry reads are handled
  // via s.read_joint above to guarantee exactly one read per tick.

  // Trajectory → IK → q_des (using provided IK with home offsets)
  if (s.GAIT_RUN) {
    for (int leg = 0; leg < ControllerState::N_LEGS; ++leg) {
      float xf, yf, zf;
      footTrajectory_mm(&s, leg, &xf, &yf, &zf); // our frame: x=forward, y=lateral-left, z=up
      int angles_cdeg[3];
      // Map to user IK frame: x=lateral, y=vertical, z=forward
      (void)calculateIK(leg, /*x*/ yf, /*y*/ zf, /*z*/ xf, angles_cdeg, s.home_cdeg);
      const int j0 = leg * ControllerState::DOF_PER_LEG;
      float qc = cdeg_to_rad(angles_cdeg[0]);
      float qf = cdeg_to_rad(angles_cdeg[1]);
      float qt = cdeg_to_rad(angles_cdeg[2]);
      s.J[j0 + ControllerState::COXA ].q_des = sat(qc, s.J[j0 + ControllerState::COXA ].qmin, s.J[j0 + ControllerState::COXA ].qmax);
      s.J[j0 + ControllerState::FEMUR].q_des = sat(qf, s.J[j0 + ControllerState::FEMUR].qmin, s.J[j0 + ControllerState::FEMUR].qmax);
      s.J[j0 + ControllerState::TIBIA].q_des = sat(qt, s.J[j0 + ControllerState::TIBIA].qmin, s.J[j0 + ControllerState::TIBIA].qmax);
    }
  }

  // Controllers → command
  for (int j = 0; j < ControllerState::N_JOINTS; ++j) {
    auto& X = s.J[j];
    if (!X.enabled) continue;
    if (g_safety_tripped) continue; // do not compute/emit commands when tripped

    float err = X.q_des - X.q_meas;
    float tau = X.vsd.ks * err - X.vsd.b * X.dq_meas + X.vsd.tau_des;

    float derr = (X.q_meas - X.pid.x_prev) / dt_loop;
    X.pid.x_prev = X.q_meas;
  float u = X.pid.kp * err + X.pid.ki * X.pid.i_state - X.pid.kd * derr + tau;

    float i_next = X.pid.i_state + err * dt_loop;
    const float ICLAMP = 0.5f;
    X.pid.i_state = sat(i_next, -ICLAMP, ICLAMP);

  float q_cmd = sat(X.q_meas + sat(u, -X.dqmax * dt_loop, X.dqmax * dt_loop), X.qmin, X.qmax);
  const int32_t cdeg = rad_to_cdeg(q_cmd);
  X.q_cmd = q_cmd;
  X.u_out = u;

    // NOTE: still commented per your code
    // X.srv->move_time(cdeg, (int)(s.Ts * 1000));
  }

  // Evaluate sticky safety trip after reading latest telemetry
  if (!g_safety_tripped) {
    if (g_last_tempC >= g_over_temp_c) {
      safetyTrip(&s, "over-temp");
    } else if (g_last_mV > g_min_valid_mv && g_last_mV <= g_low_v_mv) {
      safetyTrip(&s, "low-bus-voltage");
    }
  }

  // create a log row (cycle-based cadence)
  if (++s.log_cycle_counter >= s.log_every_cycles) {

    //    float dt_loop,
    //    int read_idx, int leg_idx, uint8_t joint_id,
    //    float q_meas, float dq_meas,
    //    int16_t tempC, int16_t mV,
    //    float q_cmd, float q_ref, float e,
    //    const char* phase_str, float u
    const int jidx = (int)((s.read_joint == 0) ? ControllerState::N_JOINTS - 1 : s.read_joint - 1);
    const int leg_idx = jidx / ControllerState::DOF_PER_LEG;
    const uint8_t joint_id = (uint8_t)(jidx % ControllerState::DOF_PER_LEG);
    Log::row(
      s.loop_stamp_us,
      loop_us,
      dt_loop,
      jidx,
      leg_idx,
      joint_id,
      s.J[jidx].q_meas,
      s.J[jidx].dq_meas,
      s.J[jidx].q_est,
      s.J[jidx].dq_est,
      (int16_t)s.J[jidx].temp,
      (int16_t)s.J[jidx].vin,
      s.J[jidx].q_cmd,
      s.J[jidx].q_des,
      s.J[jidx].q_des - s.J[jidx].q_meas,
      (s.L[leg_idx].phase == CS_LegPhase::STANCE ? "STANCE" : "SWING"),
      s.J[jidx].u_out);
    s.log_cycle_counter = 0;
  }
}
