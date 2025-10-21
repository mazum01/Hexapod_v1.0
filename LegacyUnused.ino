/*
  LegacyUnused.ino
  ----------------
  This file collects previously defined helpers, structs, and globals that are no longer used
  in the main sketch but retained here (commented) for reference during the refactor.
  These are not compiled as-is and can be reintroduced if needed.
*/

// --- Legacy helper: legOf (unused) ---
// static inline int legOf(int jointIdx) { return jointIdx / DOF_PER_LEG; }

// --- Legacy type: Vector3 (unused in current IK path) ---
// struct Vector3 { float x, y, z; };

// --- Legacy globals: rr, loop_stamp_us (replaced by s.read_joint and s.loop_stamp_us) ---
// uint8_t  rr            = 0;
// uint32_t loop_stamp_us = 0;

// Place other retired functions here as needed, keep them commented out.
