// HomeCfg.ino
// -----------------------------------------------------------------------------
// SD-backed persistence for IK home angles.
// Streaming integer parser to avoid heap churn.
// -----------------------------------------------------------------------------
//
// Depends on:
//   - Arduino core
//   - SD (pulled in by Logging.h in your project)
//   - Logging.h (Log::sdReady, Log::cat/list if you reuse them elsewhere)
//   - ControllerState.h (for N_JOINTS sizing only)
//
// This file only contains the HomeCfg namespace implementations moved
// out of Hexapod_v1.0.ino for readability. No logic changes.
//

#include <Arduino.h>
#include "ControllerState.h"
#include "Logging.h"
#include "Config.h"

namespace HomeCfg {

  // Save 18 centidegree integers under key 'home_cdeg' in /config.txt
  bool save(const long* homes, const char* /*path*/) {
    if (!Log::sdReady()) { Serial.println("[HOME] SD not available; cannot save."); return false; }
    Config::ensureFile();
    bool ok = Config::setIntList("home_cdeg", homes, ControllerState::N_JOINTS);
    Serial.println(ok ? "[HOME] Saved homes to /config.txt (home_cdeg)." : "[HOME] Failed to save homes to /config.txt.");
    return ok;
  }

  // Load homes from key 'home_cdeg' in /config.txt; fallback to legacy CSV path if provided
  bool load(long* outHomes, const char* path) {
    if (!Log::sdReady()) { Serial.println("[HOME] SD not available; cannot load."); return false; }
    int cnt = 0;
    bool ok = Config::getIntList("home_cdeg", outHomes, ControllerState::N_JOINTS, &cnt);
    if (ok && cnt == ControllerState::N_JOINTS) {
      Serial.println("[HOME] Loaded homes from /config.txt (home_cdeg). ");
      return true;
    }
    // Fallback: try legacy CSV path
    if (path && *path && SD.exists(path)) {
      File f = SD.open(path, FILE_READ);
      if (f) {
        const size_t BUFSZ = 256; static char buf[BUFSZ];
        int found = 0; bool inTok = false; bool neg = false; long val = 0;
        auto flushTok = [&](){ if (inTok) { if (neg) val = -val; outHomes[found++] = val; inTok = false; neg = false; val = 0; } };
        while (f.available() && found < ControllerState::N_JOINTS) {
          int n = f.read(buf, BUFSZ); if (n <= 0) break;
          for (int i = 0; i < n && found < ControllerState::N_JOINTS; ++i) {
            char c = buf[i];
            if ((c == '-' || c == '+') && !inTok) { inTok = true; neg = (c == '-'); val = 0; continue; }
            if (c >= '0' && c <= '9') { if (!inTok) { inTok = true; neg = false; val = 0; } val = val * 10 + (c - '0'); continue; }
            flushTok();
          }
        }
        if (found < ControllerState::N_JOINTS) flushTok();
        f.close();
        if (found == ControllerState::N_JOINTS) {
          Serial.print("[HOME] Loaded homes from legacy "); Serial.println(path);
          return true;
        }
      }
    }
    Serial.println("[HOME] home_cdeg missing and no legacy CSV available; load failed.");
    return false;
  }

  // Ensure /config.txt exists and contains a 'home_cdeg' entry; if missing, create and write defaults
  void ensureFile(const long* defaults, const char* path) {
    if (!Log::sdReady()) { Serial.println("[HOME] SD not available; skipping ensureFile()."); return; }
    Config::ensureFile();
    long tmp[ControllerState::N_JOINTS]; int cnt = 0;
    bool ok = Config::getIntList("home_cdeg", tmp, ControllerState::N_JOINTS, &cnt);
    if (!(ok && cnt == ControllerState::N_JOINTS)) {
      // Try to migrate from legacy CSV if present
      bool migrated = false;
      if (path && *path && SD.exists(path)) {
        Serial.print("[HOME] Migrating homes from legacy "); Serial.println(path);
        if (load(tmp, path)) {
          migrated = Config::setIntList("home_cdeg", tmp, ControllerState::N_JOINTS);
        }
      }
      if (!migrated) {
        Serial.println("[HOME] Writing default homes to /config.txt (home_cdeg).");
        save(defaults, "");
      } else {
        Serial.println("[HOME] Migration complete: wrote home_cdeg to /config.txt.");
      }
    } else {
      Serial.println("[HOME] home_cdeg exists in /config.txt");
    }
  }

} // namespace HomeCfg
