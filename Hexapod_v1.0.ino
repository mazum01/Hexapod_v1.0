/*
   Teensy 4.1 Hexapod Controller (LX16A buses, HTS-35S-style servos)
   ==================================================================

   BIG PICTURE
   -----------
   • 6 legs × 3 DOF LX16A serial-bus servos (centidegrees API).
   • 100 Hz deterministic loop via IntervalTimer ISR (dt computed once).
   • Round-robin: read exactly 1 joint/tick; predict others for continuity.
   • Control stack (per joint):
       Foot trajectory (mm) → IK → q_des  --->  VSD outer loop (compliant target)
                                                → PID inner loop (angle, DoM, LPF-D, anti-windup)
                                                → slew limit & joint limits
                                                → move_time(centideg, ms)
   • Safety: NaN/Inf guards, slew clamps, joint limits, integrator clear on hot/low-V.
   • Global + per-leg enable; serial commands for tuning (VSD, PID, slew), gait, stance, logging.
   • Tri-state logging: Serial / SD / Both (default Both), with `log ls`, `log cat`.
   • IK home angles are persisted on SD card (/home_angles.csv); file auto-created if missing.

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

   Nova (GPT-5) + Mark M collaboration
*/

//#include <Arduino.h>
#include <Streaming.h>
#include <lx16a-servo.h>
#include "Logging.h"   // Tri-state logging (Serial / SD / Both), plus log ls/cat

// =====================================================================
//                              LOOP TIMING
// =====================================================================

#define LOOP_HZ 100
#define Ts      (1.0f / LOOP_HZ)

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
//                USER-DEFINED DATA STRUCTS (DECLARED EARLY)
// =====================================================================

struct VSD {
  float ks;        // virtual spring "stiffness"
  float b;         // virtual damping (on measurement)
  float tau_des;   // per-tick command (bias + target following)
};

struct PID {
  float kp  = 0.0f;
  float ki  = 0.0f;
  float kd  = 0.0f;

  float umin = radians(-1.0f);   // per-step clamp (~±1 deg/tick)
  float umax = radians(+1.0f);

  float i_term = 0.0f;
  float d_filt = 0.0f;
  float beta   = 0.35f;          // LPF coefficient for derivative
  bool  init   = false;
};

struct Joint {
  uint8_t id;      // servo ID on its bus
  float   q_min;   // [rad] joint lower limit
  float   q_max;   // [rad] joint upper limit
  float   q_cmd;   // [rad] commanded joint angle (integrated)
  float   slew;    // [rad/s] per-DOF max speed
  PID     pid;     // inner PID controller
  VSD     vsd;     // outer VSD parameters (updated per-leg-phase)
};

struct JointState {
  float q_est  = 0.0f;  // [rad] estimated position
  float dq_est = 0.0f;  // [rad/s] estimated velocity
  bool  has_meas = false;
};

struct LegPhase {
  enum Phase { STANCE, SWING } phase;
  float phase_t;       // [s] time within current phase
  float stance_dur;    // [s]
  float swing_dur;     // [s]
  float phase_offset;  // [s] initial offset (tripod group)
};

// Simple vector for IK (mm coordinates, coxa base frame)
struct Vector3 { float x, y, z; };

// =====================================================================
//                          PHYSICAL GEOMETRY (IK)
// =====================================================================
// NOTE: Units for IK are millimeters, angles are radians/deg/centideg as noted.
// Adjust these lengths to your hardware.

#define COXA_LENGTH_MM   50.0f
#define FEMUR_LENGTH_MM  80.0f
#define TIBIA_LENGTH_MM 120.0f

// =====================================================================
//                   SERIAL BUSES / SERVO IDS / BUFFERS
// =====================================================================

HardwareSerial* SERVO_PORTS[N_LEGS] = {
  &Serial7, &Serial6, &Serial2, &Serial5, &Serial3, &Serial8
};

uint8_t SERIAL_TX_PINS[N_LEGS] = {28,25,8,21,14,34};

uint8_t SERVO_ID[N_LEGS][DOF_PER_LEG] = {
  {1,2,3}, {1,2,3}, {1,2,3}, {1,2,3}, {1,2,3}, {1,2,3}
};

// Optional 74HC126 buffer OE pins per bus (adjust to wiring)
const int bufferEnablePins[N_LEGS] = {32, 9, 6, 22, 16, 36};

// LX16A handles
LX16ABus    legBus[N_LEGS];
LX16AServo* servo[N_JOINTS];

// =====================================================================
//                           GLOBAL ARRAYS / STATE
// =====================================================================

Joint      J[N_JOINTS];
JointState S[N_JOINTS];
LegPhase   L[N_LEGS];

float Q_MIN [N_JOINTS];
float Q_MAX [N_JOINTS];
float Q_SLEW[N_JOINTS];

volatile bool SERVOS_ENABLED = false;
bool LEG_ENABLED[N_LEGS]     = {false,false,false,false,false,false};

// =====================================================================
//                      IK HOME ANGLES (PERSISTED ON SD)
// =====================================================================
// Defaults (centideg) provided by user, per leg 0..5, joints 0..2 (coxa,femur,tibia)

const long HOME_DEFAULTS[N_JOINTS] = {
  /* leg 0 */ 12336, 11184, 10896,
  /* leg 1 */ 12544, 11736, 11352,
  /* leg 2 */ 10392, 10968, 10056,
  /* leg 3 */ 11232, 11976, 12192,
  /* leg 4 */ 11712, 11400, 13944,
  /* leg 5 */ 11544, 11112, 10968
};

// Working copy used by IK (can be changed at runtime by commands)
long homeAngles[N_JOINTS];

// SD config path for home angles
const char* HOME_CFG_PATH = "/home_angles.csv";

// =====================================================================
//                    GAIT FLAGS / VSD & PID DEFAULTS
// =====================================================================

bool GAIT_RUN     = true;   // if false → phases freeze (no stepping)
bool STANCE_HOLD  = false;  // if true → legs are held at home foot pose

struct VSDParams { float ks, b, tau; };

const VSDParams DEFAULT_VSD_STANCE[DOF_PER_LEG] = {
  {26.0f, 0.55f, +0.20f},   // coxa
  {36.0f, 0.70f, +0.70f},   // femur
  {30.0f, 0.60f, -0.35f}    // tibia
};
const VSDParams DEFAULT_VSD_SWING[DOF_PER_LEG] = {
  {18.0f, 0.35f, 0.0f},     // coxa
  {20.0f, 0.40f, 0.0f},     // femur
  {18.0f, 0.35f, 0.0f}      // tibia
};

VSDParams VSD_STANCE_BASE[DOF_PER_LEG];
VSDParams VSD_SWING_BASE[DOF_PER_LEG];

bool      VSD_OVERRIDE_EN [N_LEGS][DOF_PER_LEG] = {0};
VSDParams VSD_OVERRIDE_VAL[N_LEGS][DOF_PER_LEG];

struct PIDGains { float kp, ki, kd; };
const PIDGains DEFAULT_PID[DOF_PER_LEG] = {
  {24.0f, 32.0f, 0.5f},  // coxa
  {28.0f, 36.0f, 0.6f},  // femur
  {22.0f, 32.0f, 0.5f}   // tibia
};

const float DEFAULT_SLEW[DOF_PER_LEG] = {
  radians(300), radians(300), radians(300)
};

// =====================================================================
//                             IK TRAJECTORY KNOBS
// =====================================================================
// These shape the tripod foot path (units in mm).

float STANCE_HEIGHT_MM = -80.0f;   // nominal foot vertical (negative = down)
float STRIDE_LEN_MM     =  80.0f;  // total fore-aft travel over a full step
float LIFT_MM           =  20.0f;  // peak swing lift

// Lateral offsets (per leg, in mm) so feet aren't collinear; tweak as needed
float LEG_X_OFFSET_MM[N_LEGS] = { +25, -25, +25, -25, +25, -25 };
// Forward/back offsets per leg (mm) to spread the base
float LEG_Z_OFFSET_MM[N_LEGS] = { +30, +30,  0,  0, -30, -30 };

// =====================================================================
//                                 HELPERS
// =====================================================================

inline int     legOf(int jointIdx)         { return jointIdx / DOF_PER_LEG; }
inline float   deg2rad(float d)            { return d * (PI / 180.0f); }
inline float   rad2deg(float r)            { return r * (180.0f / PI); }
inline int32_t rad_to_cdeg(float r)        { return (int32_t)lroundf(rad2deg(r) * 100.0f); }
inline float   cdeg_to_rad(int32_t cdeg)   { return deg2rad(cdeg / 100.0f); }
inline float   sat(float x, float a, float b){ return x < a ? a : (x > b ? b : x); }
inline bool    isFinite(float x)           { return isfinite(x); }
inline float   finite_or(float x, float f) { return isFinite(x) ? x : f; }
inline float   lpf1(float y_prev, float x, float a){
  if (!isFinite(y_prev)) y_prev = 0.0f;
  if (!isFinite(x))      x      = 0.0f;
  if (a < 0.0f) a = 0.0f; else if (a > 1.0f) a = 1.0f;
  return (1.0f - a)*y_prev + a*x;
}

// =====================================================================
//                         SPLASH + STARTUP INFO
// =====================================================================

const char* SPLASH_BANNER =
"+----------------------------------+\n"
"|  __  __    _    ____  ____       |\n"
"| |  \\/  |  / \\  |  _ \\/ ___|      |\n"
"| | |\\/| | / _ \\ | |_) \\___ \\      |\n"
"| | |  | |/ ___ \\|  _ < ___) |     |\n"
"| |_|  |_/_/   \\_\\_| \\_\\____/      |\n"
"|                                  |\n"
"+----------------------------------+\n";

void printStartupInfo() {
  Serial << SPLASH_BANNER << endl;


  Serial << "[SYS] " << FW_NAME << " v" << FW_VERSION << " (built " << FW_BUILD_DT << ")" << endl;
  Serial << "[SYS] CPU @ " << F_CPU_ACTUAL / 1e6 << " MHz" << endl;
  Serial << "[SYS] Loop rate: " << LOOP_HZ << " Hz (" << Ts * 1000 << " ms)" << endl;
  Serial << "[SYS] Legs: " << N_LEGS << " × " << DOF_PER_LEG << " DOF = " << N_JOINTS << " joints" << endl;

  Serial << "[BUS] Mapping (leg → Serial port, buffer pin):" << endl;
  for (int leg = 0; leg < N_LEGS; ++leg) {
    Serial << "   Leg " << leg << " → "
           << ((leg==0) ? "Serial7" :
               (leg==1) ? "Serial6" :
               (leg==2) ? "Serial2" :
               (leg==3) ? "Serial5" :
               (leg==4) ? "Serial3" : "Serial8")
           << ", buffer pin = " << bufferEnablePins[leg] << endl;
  }

  Serial << "[SERVOS] ID mapping per leg (coxa,femur,tibia):" << endl;
  for (int leg = 0; leg < N_LEGS; ++leg) {
    Serial << "   Leg " << leg << " → IDs {"
           << (int)SERVO_ID[leg][0] << ", "
           << (int)SERVO_ID[leg][1] << ", "
           << (int)SERVO_ID[leg][2] << "}" << endl;
  }
}

// =====================================================================
//                   SD CONFIG FOR IK HOME ANGLES
// =====================================================================

namespace HomeCfg {

  // Write a single CSV line with 18 centidegree integers
  bool save(const long* homes, const char* path) {
    if (!Log::sdReady()) {
      Serial.println("[HOME] SD not available; cannot save.");
      return false;
    }
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
      Serial.print("[HOME] Cannot open for write: "); Serial.println(path);
      return false;
    }

    // Overwrite: rewind and write a single CSV line
    f.seek(0);
    for (int i = 0; i < N_JOINTS; ++i) {
      f.print(homes[i]);
      if (i != N_JOINTS - 1) f.print(',');
    }
    f.println();
    f.flush();
    f.close();

    Serial.print("[HOME] Saved "); Serial.print(N_JOINTS);
    Serial.print(" angles to "); Serial.println(path);
    return true;
  }

  // Tolerant CSV/whitespace parser: reads first 18 integers it finds
  bool load(long* outHomes, const char* path) {
    if (!Log::sdReady()) {
      Serial.println("[HOME] SD not available; cannot load.");
      return false;
    }
    File f = SD.open(path, FILE_READ);
    if (!f) {
      Serial.print("[HOME] Cannot open for read: "); Serial.println(path);
      return false;
    }

    const size_t BUFSZ = 512;
    char buf[BUFSZ+1];
    int  found = 0;
    String acc;

    while (f.available() && found < N_JOINTS) {
      int n = f.readBytes(buf, BUFSZ);
      buf[n] = '\0';
      acc += buf;

      // Parse integers in acc
      int start = 0;
      while (found < N_JOINTS && start < (int)acc.length()) {
        // Skip non-number chars until a digit or sign
        while (start < (int)acc.length() &&
               !(isDigit(acc[start]) || acc[start]=='-' || acc[start]=='+')) start++;

        if (start >= (int)acc.length()) break;

        // Collect token until next non-digit
        int end = start + 1;
        while (end < (int)acc.length() && isDigit(acc[end])) end++;

        long val = acc.substring(start, end).toInt();
        outHomes[found++] = val;
        start = end;
      }

      // Keep leftover tail (if any) for next read
      if (start < (int)acc.length()) acc = acc.substring(start);
      else                           acc = "";
    }

    f.close();

    if (found == N_JOINTS) {
      Serial.print("[HOME] Loaded "); Serial.print(found);
      Serial.print(" angles from "); Serial.println(path);
      return true;
    } else {
      Serial.print("[HOME] Found only "); Serial.print(found);
      Serial.println(" integers; load failed.");
      return false;
    }
  }

  // Ensure file exists; if missing, create it with defaults
  void ensureFile(const long* defaults, const char* path) {
    if (!Log::sdReady()) {
      Serial.println("[HOME] SD not available; skipping ensureFile().");
      return;
    }
    if (!SD.exists(path)) {
      Serial.print("[HOME] Config not found; creating "); Serial.println(path);
      save(defaults, path);
    } else {
      Serial.print("[HOME] Config exists: "); Serial.println(path);
    }
  }

} // namespace HomeCfg

// =====================================================================
//                         LIMITS/SLEW FILL FUNCTIONS
// =====================================================================

void fill_limits_for_all_legs(float coxa_min, float coxa_max,
                              float femur_min, float femur_max,
                              float tibia_min, float tibia_max) {
  for (int leg = 0; leg < N_LEGS; ++leg) {
    int base = leg * DOF_PER_LEG;
    Q_MIN[base + COXA]  = coxa_min;  Q_MAX[base + COXA]  = coxa_max;
    Q_MIN[base + FEMUR] = femur_min; Q_MAX[base + FEMUR] = femur_max;
    Q_MIN[base + TIBIA] = tibia_min; Q_MAX[base + TIBIA] = tibia_max;
  }
}

void fill_slew_for_all_legs(float coxa_slew, float femur_slew, float tibia_slew) {
  for (int leg = 0; leg < N_LEGS; ++leg) {
    int base = leg * DOF_PER_LEG;
    Q_SLEW[base + COXA]  = coxa_slew;
    Q_SLEW[base + FEMUR] = femur_slew;
    Q_SLEW[base + TIBIA] = tibia_slew;
  }
}

// =====================================================================
//                       ENABLE/DISABLE UTILITIES
// =====================================================================

void applyServoEnableState() {
  for (int i = 0; i < N_JOINTS; ++i) {
    const int leg = legOf(i);
    if (!servo[i]) continue;
    if (SERVOS_ENABLED && LEG_ENABLED[leg]) servo[i]->enable();
    else                                    servo[i]->disable();
  }
}

void enableAllLegs(bool en) {
  for (int l = 0; l < N_LEGS; ++l) LEG_ENABLED[l] = en;
  SERVOS_ENABLED = en;
  applyServoEnableState();
}

void enableOnlyOneLeg(int leg) {
  if (leg < 0 || leg >= N_LEGS) return;
  for (int l = 0; l < N_LEGS; ++l) LEG_ENABLED[l] = false;
  LEG_ENABLED[leg] = true;
  SERVOS_ENABLED = true;
  applyServoEnableState();
}

// =====================================================================
//                     CONTROL SUBROUTINES (VSD + PID)
// =====================================================================

float vsd_qref(float q_est, float dq_est, const VSD& v, float qmin, float qmax){
  // q_ref = q + (tau_des - b*dq)/ks
  float q_ref = q_est + (v.tau_des - v.b * dq_est) / max(v.ks, 1e-6f);
  return sat(q_ref, qmin, qmax);
}

float pid_step_predictAware(PID &c, float q_ref, float q_est, float dq_est){
  if(!c.init){ c.i_term=0.0f; c.d_filt=0.0f; c.init=true; }

  q_ref  = finite_or(q_ref,  q_est);
  q_est  = finite_or(q_est,  0.0f);
  dq_est = finite_or(dq_est, 0.0f);

  const float e     = q_ref - q_est;
  const float d_raw = -dq_est; // D on measurement
  c.d_filt = lpf1(finite_or(c.d_filt,0.0f), d_raw, finite_or(c.beta,0.35f));

  float u_unsat = c.kp*e + c.i_term + c.kd*c.d_filt;
  if (!isFinite(u_unsat)) u_unsat = 0.0f;

  const float u = sat(u_unsat, c.umin, c.umax);

  const float Kaw = finite_or(c.ki, 0.0f);
  c.i_term = finite_or(c.i_term, 0.0f);
  c.i_term += c.ki*Ts*e + Kaw*(u - u_unsat);
  if (!isFinite(c.i_term)) c.i_term = 0.0f;

  return finite_or(u, 0.0f);
}

// =====================================================================
//                        GAIT FSM SUBROUTINES
// =====================================================================

void advance_leg_phase(LegPhase &lp, float dt_loop){
  if (!GAIT_RUN) return;     // when stopped, freeze phase_t
  lp.phase_t += dt_loop;
  const float dur = (lp.phase == LegPhase::STANCE) ? lp.stance_dur : lp.swing_dur;
  if (lp.phase_t >= dur){
    lp.phase = (lp.phase == LegPhase::STANCE) ? LegPhase::SWING : LegPhase::STANCE;
    lp.phase_t = 0.0f;
  }
}

// VSD bases per leg/phase; per-leg overrides apply at the end
void set_vsd_for_leg(int leg_index){
  for (int dof = 0; dof < DOF_PER_LEG; ++dof) {
    VSDParams base = (L[leg_index].phase == LegPhase::STANCE)
                       ? VSD_STANCE_BASE[dof]
                       : VSD_SWING_BASE[dof];
    if (VSD_OVERRIDE_EN[leg_index][dof]) base = VSD_OVERRIDE_VAL[leg_index][dof];
    J[leg_index*DOF_PER_LEG + dof].vsd = { base.ks, base.b, base.tau };
  }
}

// Optional stance-time femur bias modulation (kept from your original)
void modulate_stance_bias_for_leg(int leg_index){
  LegPhase &lp = L[leg_index];
  if (lp.phase == LegPhase::STANCE){
    const float u = (lp.stance_dur > 1e-6f) ? (lp.phase_t / lp.stance_dur) : 0.0f;
    J[leg_index*DOF_PER_LEG + FEMUR].vsd.tau_des += 0.15f * sinf(PI * u);
  }
}

// =====================================================================
//                              IK SOLVER
// =====================================================================
// Adapted from your function. It computes servo angles (centideg) given
// a foot target (mm) in the local coxa frame. Uses homeAngles as offsets.

const int LEG_SERVOS = 3;

bool calculateIK(int leg, Vector3 target, int* angles_cd, const long* home_cd)
{
  // Index base into the home angle array
  const int servoIdxBase = leg * LEG_SERVOS;

  // Rename inputs for clarity; units in mm
  const float x = target.x; // lateral (+ right)
  const float y = target.y; // vertical (+ up)  [negative down in our path]
  const float z = target.z; // forward (+ forward)

  // --- Coxa angle (centideg), 0° forward, + to the right
  const float coxaAngleRad = atan2f(x, z);
  const float coxaAngleDeg = rad2deg(coxaAngleRad);
  // Offset by home; empirical + legacy offset of 90°
  int coxaAngleCentideg = (int)lroundf(coxaAngleDeg * 100.0f
                                + (float)home_cd[servoIdxBase]
                                - 9000.0f);

  // --- Distances in sagittal plane from coxa tip to foot tip
  const float L = sqrtf(x*x + z*z);                         // horizontal distance
  const float D = sqrtf((L - COXA_LENGTH_MM)*(L - COXA_LENGTH_MM) + y*y);

  // Workspace reach check
  if (D > (FEMUR_LENGTH_MM + TIBIA_LENGTH_MM) ||
      D < fabsf(FEMUR_LENGTH_MM - TIBIA_LENGTH_MM)) {
    return false; // out of reach
  }

  // --- Femur angle
  const float alpha1 = asinf((L - COXA_LENGTH_MM) / D);
  const float alpha2 = acosf((D*D + FEMUR_LENGTH_MM*FEMUR_LENGTH_MM - TIBIA_LENGTH_MM*TIBIA_LENGTH_MM) / (2.0f * FEMUR_LENGTH_MM * D));
  const float alpha  = alpha1 + alpha2;
  const float femurAngleDeg = rad2deg( deg2rad((home_cd[servoIdxBase+1] / 100.0f) - 90.0f) + alpha );
  int femurAngleCentideg = (int)lroundf(femurAngleDeg * 100.0f);

  // --- Tibia angle (relative to femur)
  const float beta  = acosf((TIBIA_LENGTH_MM*TIBIA_LENGTH_MM + FEMUR_LENGTH_MM*FEMUR_LENGTH_MM - D*D) / (2.0f * FEMUR_LENGTH_MM * TIBIA_LENGTH_MM));
  const float gamma = PI - beta; // interior knee angle
  const float tibiaAngleDeg = rad2deg( deg2rad(home_cd[servoIdxBase+2] / 100.0f) + gamma );
  int tibiaAngleCentideg = (int)lroundf(tibiaAngleDeg * 100.0f);

  // --- Clamp to servo centidegree range [0 .. 24000]
  angles_cd[0] = constrain(coxaAngleCentideg,  0, 24000);
  angles_cd[1] = constrain(femurAngleCentideg, 0, 24000);
  angles_cd[2] = constrain(tibiaAngleCentideg, 0, 24000);

  return true;
}

// =====================================================================
//                     FOOT TRAJECTORY (TRIPOD GAIT)
// =====================================================================
// Returns foot target (mm) in leg-local frame. When STANCE_HOLD=true, we
// return the home foot position for a “no motion” pose.

Vector3 homeFootPosForLeg(int leg) {
  // Neutral foot location per leg (use offsets to form a stable support polygon)
  Vector3 p;
  p.x = LEG_X_OFFSET_MM[leg];
  p.y = STANCE_HEIGHT_MM;            // vertical height
  p.z = LEG_Z_OFFSET_MM[leg];
  return p;
}

Vector3 footTargetForLeg(int leg, const LegPhase& lp) {
  // If stance-hold is requested, keep feet at home pose (no stepping)
  if (STANCE_HOLD) return homeFootPosForLeg(leg);

  // Base position (center of stride)
  Vector3 p = homeFootPosForLeg(leg);

  // Define stride travel: forward (+z), backward (−z)
  const float halfStride = 0.5f * STRIDE_LEN_MM;

  if (lp.phase == LegPhase::STANCE) {
    // Stance: foot slides backward along ground (push phase), constant height
    const float u = (lp.stance_dur > 1e-6f) ? (lp.phase_t / lp.stance_dur) : 0.0f; // 0→1
    p.z -= (1.0f - 2.0f*u) * halfStride;    // from +half → -half
    // height held at STANCE_HEIGHT_MM
  } else {
    // Swing: foot lifts and moves forward in the air
    const float u = (lp.swing_dur > 1e-6f) ? (lp.phase_t / lp.swing_dur) : 0.0f;   // 0→1
    p.z += (-1.0f + 2.0f*u) * halfStride;   // from -half → +half
    p.y += LIFT_MM * sinf(PI * u);          // up and down arc
  }

  return p;
}

// =====================================================================
//                        TICKER / REBOOT SUPPORT
// =====================================================================

IntervalTimer ticker;
volatile bool tickFlag=false;
void isr(){ tickFlag=true; }

uint8_t  rr = 0;
uint32_t loop_stamp_us = 0;

void teensyReboot() {
  Serial.println("[SYS] Reboot requested...");
  Serial.flush();
  delay(50);
  Serial.println(SPLASH_BANNER);
  Serial.print("[SYS] *** REBOOTING "); Serial.print(FW_NAME);
  Serial.print(" v"); Serial.print(FW_VERSION); Serial.println(" ***");
  SCB_AIRCR = 0x05FA0004; // Software reset via Cortex-M7 AIRCR
}

// =====================================================================
//                         STRING-BASED CONSOLE PARSER
// =====================================================================

String lineBuf;

void splitTokens(const String& line, String tokens[], int& count, int maxTokens) {
  count = 0;
  int i = 0;
  while (i < (int)line.length() && count < maxTokens) {
    while (i < (int)line.length() && isspace(line[i])) i++;   // skip leading spaces
    if (i >= (int)line.length()) break;
    int start = i;
    while (i < (int)line.length() && !isspace(line[i])) i++;  // token chars
    tokens[count++] = line.substring(start, i);
  }
}

int dofFromToken(const String& t) {
  if (t.length() == 0) return -1;
  bool numeric = true;
  for (unsigned i=0; i<t.length(); ++i) {
    if (!isDigit(t[i]) && !(i==0 && (t[i]=='-'||t[i]=='+'))) { numeric=false; break; }
  }
  if (numeric) {
    int v = t.toInt();
    return (v>=0 && v<DOF_PER_LEG) ? v : -1;
  }
  String lower = t; lower.toLowerCase();
  if (lower == "coxa")  return COXA;
  if (lower == "femur") return FEMUR;
  if (lower == "tibia") return TIBIA;
  return -1;
}

// Forward decls
void printHelp();
void printStatus();
void printPIDAll();
void printVSDAll();
void printSlew();

// =====================================================================
//                                  SETUP
// =====================================================================

void setup(){
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 5000) {}
  delay(1000);

  printStartupInfo();

  // --- Tri-state logging: SD probe and default to BOTH ----------------
  Log::begin();
  Log::setMode(Log::BOTH);
  Serial.print("[LOG] Mode: "); Serial.println(Log::modeName());
  Serial.print("[LOG] SD status: "); Serial.println(Log::sdReady() ? "OK" : "NOT AVAILABLE");

  // --- Initialize IK home angles from SD (or defaults) ----------------
  for (int i = 0; i < N_JOINTS; ++i) homeAngles[i] = HOME_DEFAULTS[i];

  if (Log::sdReady()) {
    HomeCfg::ensureFile(HOME_DEFAULTS, HOME_CFG_PATH);   // create if missing
    if (!HomeCfg::load(homeAngles, HOME_CFG_PATH)) {
      Serial.println("[HOME] Using baked-in defaults (SD load failed).");
    }
  } else {
    Serial.println("[HOME] SD not ready; using baked-in defaults.");
  }
 
  // --- Start per-leg serial buses; enable 74HC126 buffers -------------
  
//HardwareSerial* SERVO_PORTS[N_LEGS] = {
//  &Serial7, &Serial6, &Serial2, &Serial5, &Serial3, &Serial8
//};
//
//uint8_t SERIAL_TX_PINS[N_LEGS] = {28,25,8,21,14,34};
  HardwareSerial *port;
  for (int leg = 0; leg < N_LEGS; ++leg) {
    switch (leg) {
      case 0:
        port = &Serial7;
        break;
      case 1:
        port = &Serial6;
        break;
      case 2:
        port = &Serial2;
        break;
      case 3:
        port = &Serial5;
        break;
      case 4:
        port = &Serial3;
        break;
      case 5:
        port = &Serial8;
        break;
    }
    pinMode(bufferEnablePins[leg], OUTPUT);
    digitalWrite(bufferEnablePins[leg], HIGH);
    //legBus[leg].begin(port, 0);
  }
  Serial << "[INIT] Buses started." << endl;

  // --- Limits & default slew ------------------------------------------
  fill_limits_for_all_legs(
    radians(-90),  radians( 90),
    radians(-120), radians( 60),
    radians(-140), radians(  0)
  );
  for (int leg=0; leg<N_LEGS; ++leg) {
    Q_SLEW[leg*DOF_PER_LEG + COXA ] = DEFAULT_SLEW[COXA];
    Q_SLEW[leg*DOF_PER_LEG + FEMUR] = DEFAULT_SLEW[FEMUR];
    Q_SLEW[leg*DOF_PER_LEG + TIBIA] = DEFAULT_SLEW[TIBIA];
  }

  // --- Initialize VSD bases & overrides -------------------------------
  //for (int d=0; d<DOF_PER_LEG; ++d) {
  //  VSD_STANCE_BASE[d] = DEFAULT_VSD_STANCE[d];
  //  VSD_SWING_BASE[d]  = DEFAULT_VSD_SWING[d];
  //}
  //memset(VSD_OVERRIDE_EN,  0, sizeof(VSD_OVERRIDE_EN));

  // --- Create servos and seed joint/estimator state -------------------
//  for (int leg = 0; leg < N_LEGS; ++leg) {
//    for (int dof = 0; dof < DOF_PER_LEG; ++dof) {
//      const int idx = leg*DOF_PER_LEG + dof;
//
//      servo[idx] = new LX16AServo(&legBus[leg], SERVO_ID[leg][dof]);
//
//      // Read current position → seed q0 (do NOT overwrite homeAngles)
//      const int32_t cdeg = servo[idx]->pos_read();
//      const float q0  = sat(cdeg_to_rad(cdeg), Q_MIN[idx], Q_MAX[idx]);
//      // NOTE: homeAngles are loaded from SD (or defaults) and are NOT overwritten here
//
//      J[idx].id    = SERVO_ID[leg][dof];
//      J[idx].q_min = Q_MIN[idx];
//      J[idx].q_max = Q_MAX[idx];
//      J[idx].slew  = Q_SLEW[idx];
//      J[idx].q_cmd = q0;
//
//      J[idx].pid.kp = DEFAULT_PID[dof].kp;
//      J[idx].pid.ki = DEFAULT_PID[dof].ki;
//      J[idx].pid.kd = DEFAULT_PID[dof].kd;
//
//      S[idx].q_est    = q0;
//      S[idx].dq_est   = 0.0f;
//      S[idx].has_meas = false;
//    }
//  }
//  Serial << "[INIT] Joints initialized (IK homes loaded)." << endl;
//
//  // --- Tripod gait phases ---------------------------------------------
//  for (int leg = 0; leg < N_LEGS; ++leg) {
//    L[leg].stance_dur   = 0.40f;
//    L[leg].swing_dur    = 0.20f;
//    const float Tstride = L[leg].stance_dur + L[leg].swing_dur;
//    const bool groupA   = (leg == 0 || leg == 2 || leg == 4);
//    L[leg].phase_offset = groupA ? 0.0f : 0.30f;
//
//    const float off = fmodf(L[leg].phase_offset, Tstride);
//    if (off < L[leg].stance_dur) { L[leg].phase = LegPhase::STANCE; L[leg].phase_t = off; }
//    else                         { L[leg].phase = LegPhase::SWING;  L[leg].phase_t = off - L[leg].stance_dur; }
//
//    set_vsd_for_leg(leg);
//  }
//  Serial << "[INIT] Gait initialized (tripod)." << endl;
//
//  Serial << "[SERVOS] DISABLED at boot (send 'e' to enable all, 'le <leg>' to enable one)" << endl;
//
//  enableAllLegs(false);
//
  loop_stamp_us = micros();
//  ticker.begin(isr, 1e6/LOOP_HZ);
//  Log::header();
//printHelp();
}

// =====================================================================
//                              SERIAL CONSOLE
// =====================================================================

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  h|?                            : help");
  Serial.println("  e / d                          : enable/disable ALL legs");
  Serial.println("  le <leg> / ld <leg>            : enable ONLY leg / disable leg");
  Serial.println("  s                              : status");
  Serial.println("  gait run|stop                  : run tripod (advance phases) / freeze");
  Serial.println("  stance                         : set all legs to home foot position (no stepping)");
  Serial.println("  R                              : reboot");
  Serial.println("  vsd stance <dof> <ks> <b> <tau>: set VSD STANCE base for DOF");
  Serial.println("  vsd swing  <dof> <ks> <b> <tau>: set VSD SWING  base for DOF");
  Serial.println("  vsd ov  <leg> <dof> <ks> <b> <tau>: set/enable per-leg VSD override");
  Serial.println("  vsd clr <leg> <dof>            : clear per-leg VSD override");
  Serial.println("  vsd show                       : show base + overrides");
  Serial.println("  pid  <leg> <dof> <kp> <ki> <kd>: set PID for one leg/DOF");
  Serial.println("  pidg <dof> <kp> <ki> <kd>      : set PID for ALL legs (DOF)");
  Serial.println("  pid show                       : show PID gains for all joints");
  Serial.println("  slew <dof|all> <value> [deg]   : set slew rad/s (or deg/s if 'deg')");
  Serial.println("  slew show                      : show per-DOF slew");
  Serial.println("  home show                      : print IK home angles (centideg)");
  Serial.println("  home load                      : load IK homes from SD");
  Serial.println("  home save                      : save current IK homes to SD");
  Serial.println("  home load defaults             : restore baked-in defaults (RAM only)");
  Serial.println("  home set <leg> <coxa> <femur> <tibia>  : set one leg homes (centideg)");
  Serial.println("  home deg <leg> <coxa> <femur> <tibia>  : set one leg homes (degrees)");
  Serial.println("  log show                       : show logging mode & SD status");
  Serial.println("  log serial|sd|both             : set logging destination");
  Serial.println("  log ls [path]                  : list SD directory (default '/')");
  Serial.println("  log cat <path> [max_bytes]     : print file to Serial (0 = full file)");
}

void printStatus() {
  Serial << "[STATUS] SERVOS_ENABLED=" << (SERVOS_ENABLED ? "true":"false") << "; LEG_ENABLED={";
  for (int i=0;i<N_LEGS;i++){ Serial << (LEG_ENABLED[i]?"1":"0"); if(i<N_LEGS-1) Serial<<','; }
  Serial << "}\n";
  Serial << "[GAIT] run=" << (GAIT_RUN ? "true":"false") << "; stance_hold=" << (STANCE_HOLD?"true":"false") << "\n";
  Serial << "[LOG] mode=" << Log::modeName() << "; SD=" << (Log::sdReady() ? "OK":"NOT AVAILABLE") << "\n";
}

void printPIDAll() {
  for (int leg=0; leg<N_LEGS; ++leg) {
    for (int dof=0; dof<DOF_PER_LEG; ++dof) {
      const int idx = leg*DOF_PER_LEG + dof;
      Serial << "PID leg " << leg << " dof " << dof << " : "
             << "kp=" << J[idx].pid.kp << ", ki=" << J[idx].pid.ki << ", kd=" << J[idx].pid.kd << '\n';
    }
  }
}

void printVSDAll() {
  Serial << "[VSD BASE] STANCE (coxa,femur,tibia):\n";
  Serial << "  coxa:  ks=" << VSD_STANCE_BASE[COXA].ks  << " b=" << VSD_STANCE_BASE[COXA].b  << " tau=" << VSD_STANCE_BASE[COXA].tau  << '\n';
  Serial << "  femur: ks=" << VSD_STANCE_BASE[FEMUR].ks << " b=" << VSD_STANCE_BASE[FEMUR].b << " tau=" << VSD_STANCE_BASE[FEMUR].tau << '\n';
  Serial << "  tibia: ks=" << VSD_STANCE_BASE[TIBIA].ks << " b=" << VSD_STANCE_BASE[TIBIA].b << " tau=" << VSD_STANCE_BASE[TIBIA].tau << '\n';
  Serial << "[VSD BASE] SWING  (coxa,femur,tibia):\n";
  Serial << "  coxa:  ks=" << VSD_SWING_BASE[COXA].ks  << " b=" << VSD_SWING_BASE[COXA].b  << " tau=" << VSD_SWING_BASE[COXA].tau  << '\n';
  Serial << "  femur: ks=" << VSD_SWING_BASE[FEMUR].ks << " b=" << VSD_SWING_BASE[FEMUR].b << " tau=" << VSD_SWING_BASE[FEMUR].tau << '\n';
  Serial << "  tibia: ks=" << VSD_SWING_BASE[TIBIA].ks << " b=" << VSD_SWING_BASE[TIBIA].b << " tau=" << VSD_SWING_BASE[TIBIA].tau << '\n';

  Serial << "[VSD OVERRIDES] (leg,dof) if enabled:\n";
  for (int leg=0; leg<N_LEGS; ++leg) {
    for (int dof=0; dof<DOF_PER_LEG; ++dof) {
      if (VSD_OVERRIDE_EN[leg][dof]) {
        VSDParams &v = VSD_OVERRIDE_VAL[leg][dof];
        Serial << "  leg " << leg << " dof " << dof << " → ks=" << v.ks << " b=" << v.b << " tau=" << v.tau << '\n';
      }
    }
  }
}

void printSlew() {
  Serial << "[SLEW] (rad/s) by DOF (applied per-leg current values):\n";
  Serial << "  coxa:  " << Q_SLEW[COXA]  << '\n';
  Serial << "  femur: " << Q_SLEW[FEMUR] << '\n';
  Serial << "  tibia: " << Q_SLEW[TIBIA] << '\n';
}

// ----------- Handle one complete command line -----------
void handleCommandLine(const String& rawLine) {
  String line = rawLine; line.trim();
  if (line.length() == 0) return;

  Serial.print("[CMD] "); Serial.println(line);

  const int MAXTOK = 16;
  String tok[MAXTOK];
  int argc = 0;
  splitTokens(line, tok, argc, MAXTOK);
  if (argc == 0) return;

  tok[0].toLowerCase();

  // Basics
  if (tok[0] == "h" || tok[0] == "?") { printHelp(); return; }
  if (tok[0] == "s") { printStatus(); return; }
  if (tok[0] == "e") { enableAllLegs(true);  Serial.println("[SERVOS] ENABLED (all legs)"); return; }
  if (tok[0] == "d") { enableAllLegs(false); Serial.println("[SERVOS] DISABLED (all legs)"); return; }
  if (tok[0] == "le" && argc>=2) {
    int leg = tok[1].toInt();
    if (leg<0 || leg>=N_LEGS) { Serial.println("[ERR] leg out of range"); return; }
    enableOnlyOneLeg(leg); Serial.print("[SERVOS] ONLY leg "); Serial.print(leg); Serial.println(" enabled"); return;
  }
  if (tok[0] == "ld" && argc>=2) {
    int leg = tok[1].toInt();
    if (leg<0 || leg>=N_LEGS) { Serial.println("[ERR] leg out of range"); return; }
    LEG_ENABLED[leg] = false; applyServoEnableState(); Serial.print("[SERVOS] leg "); Serial.print(leg); Serial.println(" disabled"); return;
  }
  if (tok[0] == "r" || tok[0] == "R") { teensyReboot(); return; }

  // Gait control
  if (tok[0] == "gait" && argc>=2) {
    if (tok[1].equalsIgnoreCase("run"))  { GAIT_RUN = true;  STANCE_HOLD = false; Serial.println("[GAIT] run=true, stance_hold=false"); return; }
    if (tok[1].equalsIgnoreCase("stop")) { GAIT_RUN = false; Serial.println("[GAIT] run=false (phases frozen)"); return; }
    Serial.println("[ERR] usage: gait run|stop");
    return;
  }

  // Stance (home) hold
  if (tok[0] == "stance") {
    STANCE_HOLD = true;           // keep foot targets at home
    GAIT_RUN    = false;          // freeze phases for determinism
    Serial.println("[STANCE] All legs set to home foot position; gait frozen.");
    return;
  }

  // Logging (tri-state) + ls + cat
  if (tok[0] == "log") {
    if (argc==2 && tok[1].equalsIgnoreCase("show")) {
      Serial.print("[LOG] mode="); Serial.print(Log::modeName());
      Serial.print("; SD="); Serial.println(Log::sdReady() ? "OK" : "NOT AVAILABLE");
      return;
    }
    if (argc>=2 && tok[1].equalsIgnoreCase("ls")) {
      const char* path = "/";
      if (argc >= 3) {
        char pathBuf[128];
        tok[2].toCharArray(pathBuf, sizeof(pathBuf));
        path = pathBuf;
      }
      Log::list(path);
      return;
    }
    if ((argc==3 || argc==4) && tok[1].equalsIgnoreCase("cat")) {
      char pathBuf[128];
      tok[2].toCharArray(pathBuf, sizeof(pathBuf));
      uint32_t maxBytes = 4096;
      if (argc == 4) {
        long v = tok[3].toInt(); if (v < 0) v = 0; maxBytes = (uint32_t)v;
      }
      Log::cat(pathBuf, maxBytes);
      return;
    }
    if (argc==2 && tok[1].equalsIgnoreCase("serial")) {
      Log::setMode(Log::SERIAL_ONLY);
      Serial.println("[LOG] mode → Serial only");
      return;
    }
    if (argc==2 && tok[1].equalsIgnoreCase("sd")) {
      Log::setMode(Log::SD_ONLY);
      Serial.print("[LOG] requested mode → SD only; effective: ");
      Serial.println(Log::modeName());
      return;
    }
    if (argc==2 && tok[1].equalsIgnoreCase("both")) {
      Log::setMode(Log::BOTH);
      Serial.print("[LOG] requested mode → Both; effective: ");
      Serial.println(Log::modeName());
      return;
    }
    Serial.println("[ERR] log usage: log show | log ls [path] | log cat <path> [max_bytes] | log serial | log sd | log both");
    return;
  }

  // VSD
  if (tok[0] == "vsd") {
    if (argc>=2) {
      tok[1].toLowerCase();

      if (tok[1] == "show") { printVSDAll(); return; }

      if ((tok[1] == "stance" || tok[1] == "swing") && argc>=6) {
        int dof = dofFromToken(tok[2]);
        if (dof<0) { Serial.println("[ERR] bad DOF"); return; }
        float ks  = tok[3].toFloat();
        float b   = tok[4].toFloat();
        float tau = tok[5].toFloat();
        if (tok[1] == "stance") VSD_STANCE_BASE[dof] = {ks,b,tau};
        else                    VSD_SWING_BASE[dof]  = {ks,b,tau};
        Serial.print("[VSD] "); Serial.print(tok[1]); Serial.print(" base set dof ");
        Serial.print(dof); Serial.print(" → ks="); Serial.print(ks);
        Serial.print(" b="); Serial.print(b); Serial.print(" tau="); Serial.println(tau);
        return;
      }

      if (tok[1] == "ov" && (argc==7 || argc==6)) {
        int leg = tok[2].toInt();
        int dof = dofFromToken(tok[3]);
        if (leg<0 || leg>=N_LEGS || dof<0) { Serial.println("[ERR] vsd ov <leg> <dof> <ks> <b> <tau>"); return; }
        float ks  = tok[4].toFloat();
        float b   = tok[5].toFloat();
        float tau = (argc==7) ? tok[6].toFloat() : 0.0f;
        VSD_OVERRIDE_EN [leg][dof] = true;
        VSD_OVERRIDE_VAL[leg][dof] = {ks,b,tau};
        Serial.print("[VSD] override leg "); Serial.print(leg); Serial.print(" dof "); Serial.print(dof);
        Serial.print(" → ks="); Serial.print(ks); Serial.print(" b="); Serial.print(b); Serial.print(" tau="); Serial.println(tau);
        return;
      }

      if (tok[1] == "clr" && argc>=4) {
        int leg = tok[2].toInt();
        int dof = dofFromToken(tok[3]);
        if (leg<0 || leg>=N_LEGS || dof<0) { Serial.println("[ERR] vsd clr <leg> <dof>"); return; }
        VSD_OVERRIDE_EN[leg][dof] = false;
        Serial.print("[VSD] override cleared for leg "); Serial.print(leg); Serial.print(" dof "); Serial.println(dof);
        return;
      }
    }
    Serial.println("[ERR] vsd usage: vsd show | vsd stance|swing <dof> <ks> <b> <tau> | vsd ov <leg> <dof> <ks> <b> <tau> | vsd clr <leg> <dof>");
    return;
  }

  // PID (single)
  if (tok[0] == "pid") {
    if (argc>=2 && tok[1].equalsIgnoreCase("show")) { printPIDAll(); return; }
    if (argc>=6) {
      int leg = tok[1].toInt();
      int dof = dofFromToken(tok[2]);
      if (leg<0 || leg>=N_LEGS || dof<0) { Serial.println("[ERR] pid <leg> <dof> <kp> <ki> <kd>"); return; }
      float kp = tok[3].toFloat();
      float ki = tok[4].toFloat();
      float kd = tok[5].toFloat();
      const int idx = leg*DOF_PER_LEG + dof;
      J[idx].pid.kp = kp; J[idx].pid.ki = ki; J[idx].pid.kd = kd;
      Serial.print("[PID] leg "); Serial.print(leg); Serial.print(" dof "); Serial.print(dof);
      Serial.print(" → kp="); Serial.print(kp); Serial.print(" ki="); Serial.print(ki); Serial.print(" kd="); Serial.println(kd);
      return;
    }
    Serial.println("[ERR] pid usage: pid show | pid <leg> <dof> <kp> <ki> <kd>");
    return;
  }

  // PID (group per DOF)
  if (tok[0] == "pidg") {
    if (argc>=5) {
      int dof = dofFromToken(tok[1]); if (dof<0) { Serial.println("[ERR] pidg <dof> <kp> <ki> <kd>"); return; }
      float kp = tok[2].toFloat();
      float ki = tok[3].toFloat();
      float kd = tok[4].toFloat();
      for (int leg=0; leg<N_LEGS; ++leg) {
        const int idx = leg*DOF_PER_LEG + dof;
        J[idx].pid.kp = kp; J[idx].pid.ki = ki; J[idx].pid.kd = kd;
      }
      Serial.print("[PID] ALL legs dof "); Serial.print(dof);
      Serial.print(" → kp="); Serial.print(kp); Serial.print(" ki="); Serial.print(ki); Serial.print(" kd="); Serial.println(kd);
      return;
    }
    Serial.println("[ERR] pidg usage: pidg <dof> <kp> <ki> <kd>");
    return;
  }

  // Slew
  if (tok[0] == "slew") {
    if (argc>=2 && tok[1].equalsIgnoreCase("show")) { printSlew(); return; }

    if (argc>=3) {
      bool isDeg = (argc>=4 && tok[3].equalsIgnoreCase("deg"));
      float val = tok[2].toFloat();
      if (isDeg) val = deg2rad(val);

      if (tok[1].equalsIgnoreCase("all")) {
        for (int leg=0; leg<N_LEGS; ++leg) {
          Q_SLEW[leg*DOF_PER_LEG + COXA ] = val;
          Q_SLEW[leg*DOF_PER_LEG + FEMUR] = val;
          Q_SLEW[leg*DOF_PER_LEG + TIBIA] = val;
        }
        Serial.print("[SLEW] ALL DOFs → "); Serial.print(val); Serial.println(" rad/s");
        return;
      }

      int dof = dofFromToken(tok[1]); if (dof<0) { Serial.println("[ERR] slew <dof|all> <value> [deg]"); return; }
      for (int leg=0; leg<N_LEGS; ++leg) {
        Q_SLEW[leg*DOF_PER_LEG + dof] = val;
      }
      Serial.print("[SLEW] DOF "); Serial.print(dof); Serial.print(" → "); Serial.print(val); Serial.println(" rad/s");
      return;
    }
    Serial.println("[ERR] slew usage: slew show | slew <dof|all> <value> [deg]");
    return;
  }

  // IK home angles on SD
  if (tok[0] == "home") {
    // home show
    if (argc == 2 && tok[1].equalsIgnoreCase("show")) {
      for (int leg = 0; leg < N_LEGS; ++leg) {
        int base = leg * DOF_PER_LEG;
        Serial.print("leg "); Serial.print(leg); Serial.print(": ");
        Serial.print(homeAngles[base + 0]); Serial.print(", ");
        Serial.print(homeAngles[base + 1]); Serial.print(", ");
        Serial.println(homeAngles[base + 2]);
      }
      return;
    }
    // home load
    if (argc == 2 && tok[1].equalsIgnoreCase("load")) {
      if (HomeCfg::load(homeAngles, HOME_CFG_PATH)) {
        Serial.println("[HOME] Loaded from SD.");
      } else {
        Serial.println("[HOME] Load failed; homes unchanged.");
      }
      return;
    }
    // home save
    if (argc == 2 && tok[1].equalsIgnoreCase("save")) {
      if (HomeCfg::save(homeAngles, HOME_CFG_PATH)) {
        Serial.println("[HOME] Saved to SD.");
      } else {
        Serial.println("[HOME] Save failed.");
      }
      return;
    }
    // home load defaults
    if (argc == 3 && tok[1].equalsIgnoreCase("load") && tok[2].equalsIgnoreCase("defaults")) {
      for (int i = 0; i < N_JOINTS; ++i) homeAngles[i] = HOME_DEFAULTS[i];
      Serial.println("[HOME] IK homes restored to baked-in defaults (RAM only). Use 'home save' to persist.");
      return;
    }
    // home set <leg> c f t
    if (argc == 6 && tok[1].equalsIgnoreCase("set")) {
      int leg = tok[2].toInt();
      if (leg < 0 || leg >= N_LEGS) { Serial.println("[ERR] leg 0..5"); return; }
      long c = tok[3].toInt();
      long f = tok[4].toInt();
      long t = tok[5].toInt();
      int base = leg * DOF_PER_LEG;
      homeAngles[base+0] = c;
      homeAngles[base+1] = f;
      homeAngles[base+2] = t;
      Serial.print("[HOME] leg "); Serial.print(leg); Serial.print(" set to ");
      Serial.print(c); Serial.print(", "); Serial.print(f); Serial.print(", "); Serial.println(t);
      Serial.println("       (Use 'home save' to persist to SD.)");
      return;
    }
    // home deg <leg> c f t (degrees → centideg)
    if (argc == 6 && tok[1].equalsIgnoreCase("deg")) {
      int leg = tok[2].toInt();
      if (leg < 0 || leg >= N_LEGS) { Serial.println("[ERR] leg 0..5"); return; }
      float c = tok[3].toFloat();
      float f = tok[4].toFloat();
      float t = tok[5].toFloat();
      int base = leg * DOF_PER_LEG;
      homeAngles[base+0] = (long)lroundf(c * 100.0f);
      homeAngles[base+1] = (long)lroundf(f * 100.0f);
      homeAngles[base+2] = (long)lroundf(t * 100.0f);
      Serial.print("[HOME] leg "); Serial.print(leg); Serial.print(" set (deg) to ");
      Serial.print(c); Serial.print(", "); Serial.print(f); Serial.print(", "); Serial.println(t);
      Serial.println("       (Use 'home save' to persist to SD.)");
      return;
    }
    Serial.println("[ERR] home usage: home show | home load | home save | home load defaults | home set <leg> <coxa> <femur> <tibia> | home deg <leg> <coxa> <femur> <tibia>");
    return;
  }

  Serial.println("[ERR] Unknown command. Type 'h' for help.");
}

// =====================================================================
//                                   LOOP
// =====================================================================

void loop(){
  // Build command line (non-blocking)
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (lineBuf.length() > 0) {
        handleCommandLine(lineBuf);
        lineBuf = "";
      }
    } else {
      if (lineBuf.length() < 240) lineBuf += c;
      else { Serial.println("[ERR] command too long; discarded"); lineBuf = ""; }
    }
  }

  if(!tickFlag) return;
  tickFlag=false;

  // dt computed once
  const uint32_t now_us = micros();
  float dt_loop = (float)((uint32_t)(now_us - loop_stamp_us)) * 1e-6f;
  if (!(dt_loop > 0.0f) || !isFinite(dt_loop) || dt_loop > 0.2f) dt_loop = Ts;
  loop_stamp_us = now_us;

  // --- Advance gait phases (unless GAIT_RUN==false) and set VSD base ---
  for (int leg = 0; leg < N_LEGS; ++leg) {
    advance_leg_phase(L[leg], dt_loop);
    set_vsd_for_leg(leg);
    modulate_stance_bias_for_leg(leg);   // optional mild femur bias in stance
  }

  // --- IK: compute q_des per joint from desired foot targets ----------
  for (int leg = 0; leg < N_LEGS; ++leg) {
    const Vector3 target_mm = footTargetForLeg(leg, L[leg]);

    int angles_cd[3];  // IK output in centidegrees
    if (calculateIK(leg, target_mm, angles_cd, homeAngles)) {
      // Convert centideg → radians for desired joint angles
      const float q_des[3] = {
        cdeg_to_rad(angles_cd[COXA]),
        cdeg_to_rad(angles_cd[FEMUR]),
        cdeg_to_rad(angles_cd[TIBIA])
      };

      // Set VSD tau_des so that q_ref ≈ q_des while keeping compliance.
      // q_ref = q + (tau_des - b*dq)/ks  ⇒ choose tau_des = ks*(q_des - q) + b*dq + tau_base
      for (int dof = 0; dof < DOF_PER_LEG; ++dof) {
        const int idx = leg*DOF_PER_LEG + dof;
        const float ks  = J[idx].vsd.ks;
        const float b   = J[idx].vsd.b;
        const float tau_base = J[idx].vsd.tau_des;  // currently holds base tau from set_vsd_for_leg
        J[idx].vsd.tau_des = ks * (q_des[dof] - S[idx].q_est) + b * S[idx].dq_est + tau_base;
      }
    } else {
      // IK failed (out of workspace). Hold previous commands, no special action.
      // You could also clamp target_mm or shrink stride dynamically.
    }
  }

  // --- Round-robin read & control update ------------------------------
  const int read_idx = rr; rr++; if (rr >= N_JOINTS) rr = 0;
  const uint32_t t_start = now_us;

  int16_t minTempSeen = 32767, maxTempSeen = -32768, minMVSeen = 32767;

  for (int idx=0; idx<N_JOINTS; ++idx){
    Joint      &j = J[idx];
    JointState &s = S[idx];

    float  q_meas = s.q_est;
    float  dq_meas = s.dq_est;
    int16_t tempC_row = 0;
    int16_t mV_row    = 0;

    if (idx == read_idx) {
      // Actual measurement this tick
      int32_t cdeg = servo[idx]->pos_read();
      q_meas = cdeg_to_rad(cdeg);
      if (!isFinite(q_meas)) q_meas = s.q_est;

      if (!s.has_meas) {
        s.has_meas = true;
        s.q_est    = q_meas;
        s.dq_est   = 0.0f;
        dq_meas    = 0.0f;
      } else {
        float dq_raw = (q_meas - s.q_est) / dt_loop;
        if (!isFinite(dq_raw) || fabsf(dq_raw) > 200.0f) dq_raw = 0.0f;
        s.dq_est = lpf1(s.dq_est, dq_raw, 0.20f);
        s.q_est  = q_meas;
        dq_meas  = s.dq_est;
      }

      // Health check (on read joint only)
      uint8_t  tC  = servo[idx]->temp();
      uint16_t mV  = servo[idx]->vin();
      tempC_row = (int16_t)tC;
      mV_row    = (int16_t)mV;

      if ((tempC_row > 70) || (mV_row > 0 && mV_row < 7000)) {
        j.pid.i_term = 0.0f;  // shed integrator on stress
      }
      if (tempC_row && tempC_row < minTempSeen) minTempSeen = tempC_row;
      if (tempC_row && tempC_row > maxTempSeen) maxTempSeen = tempC_row;
      if (mV_row    && mV_row    < minMVSeen)   minMVSeen   = mV_row;

    } else {
      // Predict forward for non-read joints
      float dq_cmd = (j.q_cmd - s.q_est) / dt_loop;
      if (!isFinite(dq_cmd) || fabsf(dq_cmd) > 200.0f) dq_cmd = 0.0f;
      s.dq_est = lpf1(s.dq_est, dq_cmd, 0.30f);
      s.q_est  = s.q_est + s.dq_est * dt_loop;
      if (!isFinite(s.q_est)) s.q_est = j.q_cmd;
    }

    // VSD → PID update
    const float q_ref  = vsd_qref(s.q_est, s.dq_est, j.vsd, j.q_min, j.q_max);
    float       dq_pid = pid_step_predictAware(j.pid, q_ref, s.q_est, s.dq_est);

    // Slew limit (per DOF)
    float dq_max = j.slew * dt_loop;
    if (!isFinite(dq_max) || dq_max <= 0) dq_max = radians(5.0f) * Ts;
    dq_pid  = finite_or(dq_pid, 0.0f);
    dq_pid  = sat(dq_pid, -dq_max, dq_max);

    // Integrate to new command and clamp to joint limits
    j.q_cmd = sat(finite_or(j.q_cmd, s.q_est) + dq_pid, j.q_min, j.q_max);
    if (!isFinite(j.q_cmd)) j.q_cmd = s.q_est;

    // Send to servo
    if (SERVOS_ENABLED && LEG_ENABLED[legOf(idx)]) {
      int32_t  cdeg_cmd = rad_to_cdeg(j.q_cmd);
      uint16_t t_ms     = (uint16_t)max(1.0f, 1000.0f * dt_loop);
      servo[idx]->move_time(cdeg_cmd, t_ms);
    }

    // Log on the read joint (CSV)
    if (idx == read_idx) {
      float e = q_ref - s.q_est;
      uint32_t loop_us = micros() - t_start;

      const LegPhase& lp = L[legOf(idx)];
      const bool isStance = (lp.phase == LegPhase::STANCE);
      const char* phase_str = isStance ? "STANCE" : "SWING";
      float u = (isStance && lp.stance_dur > 1e-6f) ? constrain(lp.phase_t / lp.stance_dur, 0.0f, 1.0f) : 0.0f;

      Log::row(
        t_start, loop_us, dt_loop,
        read_idx, legOf(idx), J[idx].id,
        q_meas, dq_meas,
        tempC_row, mV_row,
        j.q_cmd, q_ref, e,
        phase_str, u
      );
    }
  }
}
