#include "ControllerState.h"

void ControllerState::initDefaults() {
  // Defaults copied from your sketch
  DEFAULT_VSD_STANCE[COXA] = {3.0f, 0.10f, 0.0f};
  DEFAULT_VSD_STANCE[FEMUR]= {4.0f, 0.10f, 0.0f};
  DEFAULT_VSD_STANCE[TIBIA]= {2.0f, 0.10f, 0.0f};

  DEFAULT_VSD_SWING[COXA]  = {1.0f, 0.05f, 0.0f};
  DEFAULT_VSD_SWING[FEMUR] = {1.5f, 0.05f, 0.0f};
  DEFAULT_VSD_SWING[TIBIA] = {1.0f, 0.05f, 0.0f};

  DEFAULT_PID[COXA]  = {7.5f, 0.8f, 0.02f, 0, 0};
  DEFAULT_PID[FEMUR] = {8.5f, 0.8f, 0.02f, 0, 0};
  DEFAULT_PID[TIBIA] = {7.0f, 0.8f, 0.02f, 0, 0};

  DEFAULT_SLEW[COXA] = DEFAULT_SLEW[FEMUR] = DEFAULT_SLEW[TIBIA] = radians(300.0f);

  // Limits: keep your same flat values (-2.5..2.5 for all)
  for (int j=0;j<N_JOINTS;++j) { Q_MIN[j] = -2.5f; Q_MAX[j] = 2.5f; }

  // Copy base VSD per-DOF
  for (int d=0; d<DOF_PER_LEG; ++d) {
    VSD_STANCE_BASE[d] = DEFAULT_VSD_STANCE[d];
    VSD_SWING_BASE [d] = DEFAULT_VSD_SWING [d];
  }

  // Initialize gait durations
  STANCE_DUR = 0.30f;
  SWING_DUR  = 0.15f;

  // Initialize per-leg phases (tripod groups A:0..2, B:3..5)
  for (int leg=0; leg<N_LEGS; ++leg) {
    bool groupA = (leg < 3);
    L[leg].stance_dur = STANCE_DUR;
    L[leg].swing_dur  = SWING_DUR;
    float off = groupA ? 0.0f : (STANCE_DUR + SWING_DUR) * 0.5f;
    if (off < STANCE_DUR) { L[leg].phase = CS_LegPhase::STANCE; L[leg].phase_t = off; }
    else                  { L[leg].phase = CS_LegPhase::SWING;  L[leg].phase_t = off - L[leg].stance_dur; }
  }

  // Initialize joints table from defaults
  for (int leg=0; leg<N_LEGS; ++leg) {
    for (int dof=0; dof<DOF_PER_LEG; ++dof) {
      int j = leg*DOF_PER_LEG + dof;
      J[j].enabled = false;
      J[j].qmin = Q_MIN[j];
      J[j].qmax = Q_MAX[j];
      J[j].dqmax = DEFAULT_SLEW[dof];
      J[j].q_prev = J[j].q_meas = J[j].dq_meas = 0;
      J[j].pid = DEFAULT_PID[dof];
      J[j].vsd = DEFAULT_VSD_STANCE[dof];
    }
  }
}
