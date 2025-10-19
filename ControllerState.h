#pragma once
#include <Arduino.h>

// Forward declarations (actual headers included in the .ino where hardware is set up)
class LX16ABus;
class LX16AServo;

struct CS_PID { float kp, ki, kd, x_prev, i_state; };
struct CS_VSD { float ks, b, tau_des; };
struct CS_Joint {
  LX16AServo* srv = nullptr;
  bool  enabled = false;
  bool has_meas = false;
  float q_des = 0, q_meas = 0, dq_meas = 0, q_prev = 0, dq_est = 0, q_est = 0;
  float qmin = 0, qmax = 0, dqmax = 0, vin = 0, temp = 0;
  float q_cmd = 0;   // last commanded position (rad)
  float u_out = 0;   // last control effort (arbitrary units)
  CS_PID pid{};
  CS_VSD vsd{};
};
struct CS_LegPhase {
  enum Phase : uint8_t { STANCE=0, SWING=1 } phase = STANCE;
  float phase_t = 0, stance_dur = 0.30f, swing_dur = 0.15f, phase_offset = 0;
};

class ControllerState {
public:
  // Sizes (kept same as your project)
  static constexpr int N_LEGS = 6;
  static constexpr int DOF_PER_LEG = 3;
  static constexpr int N_JOINTS = N_LEGS * DOF_PER_LEG;

  // Helpers
  static constexpr int COXA=0, FEMUR=1, TIBIA=2;
  
  // Loop timing
  volatile bool tickFlag = false;     // set by ISR
  uint32_t loop_stamp_us = 0;
  float control_loop_Hz = 166.0;      // main control loop frequency (Hz)
  float Ts = 1.0f / control_loop_Hz;  // derived sample time; source of truth is control_loop_Hz
  float last_dt_loop = 0.0f;          // last measured loop dt (s)
  bool  motors_on = false;            // software guard for issuing servo commands

  // Gait
  bool  GAIT_RUN = true;
  CS_LegPhase L[N_LEGS]{};

  // Joints
  CS_Joint J[N_JOINTS]{};

  // Servo buses & objects (created in setup)
  LX16ABus*  legBus[N_LEGS] = {nullptr};
  LX16AServo* servos[N_JOINTS] = {nullptr};


  // Home angles (centidegrees)
  long home_cdeg[N_JOINTS] = {0};

  // Coefficient tables / limits / slew
  CS_VSD DEFAULT_VSD_STANCE[DOF_PER_LEG]{};
  CS_VSD DEFAULT_VSD_SWING [DOF_PER_LEG]{};
  CS_PID DEFAULT_PID       [DOF_PER_LEG]{};
  float  Q_MIN[N_JOINTS]{};
  float  Q_MAX[N_JOINTS]{};
  float  DEFAULT_SLEW[DOF_PER_LEG]{};

  // VSD per-DOF bases and optional overrides
  CS_VSD VSD_STANCE_BASE[DOF_PER_LEG]{};
  CS_VSD VSD_SWING_BASE [DOF_PER_LEG]{};
  bool   VSD_OVERRIDE_EN[DOF_PER_LEG]{false,false,false};
  CS_VSD VSD_OVERRIDE   [DOF_PER_LEG]{};

  // Trajectory / geometry knobs
  float STANCE_HEIGHT_MM = -80.0f;
  float STRIDE_LEN_MM    =  80.0f;
  float LIFT_MM          =  20.0f;
  float LATERAL_OFFSET_MM[N_LEGS] = {-25,-25,-25, 25,25,25};
  float STANCE_DUR = 0.30f;
  float SWING_DUR  = 0.15f;

  // timer trackers
  elapsedMillis LOG_TIMER;
  uint8_t read_joint = 0;
  
  // Logging cadence (cycle-based)
  uint32_t log_every_cycles = 166;   // default: ~1 Hz at 166 Hz loop
  uint32_t log_cycle_counter = 0;    // increments each loop tick
  
  void initDefaults();  // fill tables & bases; does NOT touch hardware
  void incrementReadJoint() {read_joint = (read_joint == N_JOINTS - 1 ? 0 : read_joint + 1 );}  
};
