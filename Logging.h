#pragma once
/*
   Logging.h — Tri-state CSV logging + SD browsing for Teensy 4.1
   Upgrades:
     • Public modeName() (fixes linker err when called from main)
     • Versioned CSV header line with schema meta
     • Log levels: BASIC(0), DETAIL(1), DEBUG(2)  → filter row verbosity
     • Size-based rollover: auto-rotate to next /LOGxxx.CSV once MAX_BYTES reached
*/

#include <Arduino.h>
#include <Streaming.h>
#include <SD.h>
#include <strings.h> // for strcasecmp (POSIX)
#include "Config.h"

namespace Log {

// ----------------------- Modes & Levels -----------------------------
enum Mode  : uint8_t { SERIAL_ONLY = 0, SD_ONLY = 1, BOTH = 2, NONE = 3 };
enum Level : uint8_t { BASIC = 0, DETAIL = 1, DEBUG = 2 };

// ------------------ Internal state & configuration ------------------
static File     logFile;
static bool     sd_ok        = false;
static char     fname[32]    = {0};
static uint32_t line_count   = 0;
static const uint32_t FLUSH_EVERY = 50;
static const int      CHIP_SELECT = BUILTIN_SDCARD;

static Mode     currentMode  = SD_ONLY;   // sketch may override on boot
static Level    currentLevel = DETAIL;    // default verbosity

// rollover at ~5 MB by default
static const uint32_t MAX_BYTES = 5 * 1024UL * 1024UL;
// Unified configuration now stored in /config.txt (see Config.h)

// ---------------------- Private helpers -----------------------------
static void pickLogName() {
  for (int i = 0; i < 1000; ++i) {
    snprintf(fname, sizeof(fname), "/LOG%03d.CSV", i);
    if (!SD.exists(fname)) return;
  }
  snprintf(fname, sizeof(fname), "/LOG999.CSV");
}

static const char* modeNameStr() {
  switch (currentMode) {
    case SERIAL_ONLY: return "Serial only";
    case SD_ONLY:     return "SD only";
    case BOTH:        return "Both";
    case NONE:        return "Off";
    default:          return "Unknown";
  }
}

static void startNewFile() {
  if (!sd_ok) return;
  if (logFile) { logFile.close(); }
  pickLogName();
  logFile = SD.open(fname, FILE_WRITE);
  if (!logFile) {
    sd_ok = false;
    currentMode = SERIAL_ONLY;
    Serial << "[LOG] SD open failed; switching to Serial only." << endl;
    return;
  }
  // Write versioned header preamble and CSV header (includes kinematics)
  logFile.print("# schema=v1.2, units: t_us,loop_us,dt[s],angles[rad],vin[mV],temp[C],foot[mm]\n");
  logFile.print("t_us,loop_us,dt_loop,read_idx,leg,joint_id,"
                "q_meas_rad,dq_meas_rads,q_est_rad,dq_est_rads,tempC,mV,"
                "q_cmd_rad,q_ref_rad,e_rad,phase,u,"
                "fx_mm,fy_mm,fz_mm,ik_ok,phase_u\n");
  logFile.flush();
  line_count = 0;
}

// ------------------------- Public API -------------------------------

// Initialize SD; open a new /LOGxxx.CSV if possible.
static void begin() {
  Serial << "[LOG] Testing SD..." << endl;

  if (SD.begin(BUILTIN_SDCARD)) {
    sd_ok = true;
    startNewFile();
    if (sd_ok) Serial << "[LOG] SD ready: " << fname << endl;
  } else {
    sd_ok = false;
    Serial << "[LOG] SD init failed; SD logging disabled." << endl;
  }

  // If mode requires SD but SD isn't OK, degrade to Serial
  if (!sd_ok && (currentMode == SD_ONLY || currentMode == BOTH)) {
    currentMode = SERIAL_ONLY;
    Serial << "[LOG] SD unavailable → logging mode forced to Serial only." << endl;
  }
}

static void setMode(Mode m) {
  // Close file if turning logging off
  if (m == NONE) {
    if (logFile) { logFile.close(); }
    fname[0] = 0;
    currentMode = NONE;
    Serial << "[LOG] Mode set to: " << modeNameStr() << endl;
    return;
  }

  currentMode = m;
  if ((m == SD_ONLY || m == BOTH) && !sd_ok) {
    currentMode = SERIAL_ONLY;
    Serial << "[LOG] SD not available → using Serial only." << endl;
  } else {
    Serial << "[LOG] Mode set to: " << modeNameStr() << endl;
  }
}

static Mode  getMode()   { return currentMode; }
static bool  sdReady()   { return sd_ok; }
static Level getLevel()  { return currentLevel; }
static void  setLevel(Level L) { currentLevel = L; }

// Public-facing name (fixes main calling Log::modeName())
static const char* modeName() { return modeNameStr(); }
// Human-readable logging level name
static const char* levelName() {
  switch (currentLevel) {
    case BASIC: return "basic";
    case DETAIL: return "detail";
    case DEBUG: return "debug";
    default: return "unknown";
  }
}
// Set level by name or numeric string ("0".."2")
static bool setLevelByName(const char* name) {
  if (!name) return false;
  if (strcasecmp(name, "basic") == 0 || strcmp(name, "0") == 0) { currentLevel = BASIC; return true; }
  if (strcasecmp(name, "detail") == 0 || strcmp(name, "1") == 0) { currentLevel = DETAIL; return true; }
  if (strcasecmp(name, "debug") == 0 || strcmp(name, "2") == 0) { currentLevel = DEBUG; return true; }
  return false;
}
// Current log file path (if any); returns "(none)" if not opened yet.
static const char* currentFile() { return fname[0] ? fname : "(none)"; }

// ------------------------- Delete utilities ------------------------
// Safety notes:
//  - Only delete files that match LOG*.CSV (case-insensitive .CSV ok).
//  - Never delete the current file in use.
//  - Paths may be absolute ("/LOG000.CSV") or bare ("LOG000.CSV").

static bool isLogName(const char* name) {
  if (!name) return false;
  // Accept forms: LOG###.CSV with 3+ digits; be lenient on length
  if (strncmp(name, "LOG", 3) != 0) return false;
  const char* dot = strrchr(name, '.');
  if (!dot) return false;
  // Case-insensitive compare for extension
  return strcasecmp(dot, ".CSV") == 0;
}

static void normalizePath(const char* in, char* out, size_t outsz) {
  if (!in || !out || outsz == 0) return;
  if (in[0] == '/') snprintf(out, outsz, "%s", in);
  else              snprintf(out, outsz, "/%s", in);
}

// Delete a single log file; returns true if removed.
static bool del(const char* path) {
  if (!sd_ok) { Serial.println(R"([LOG] SD not available.)"); return false; }
  if (!path || !*path) { Serial.println(R"([LOG] del: missing path)"); return false; }

  char pbuf[32]; normalizePath(path, pbuf, sizeof(pbuf));

  // Extract name portion for validation
  const char* name = pbuf;
  if (pbuf[0] == '/') name = pbuf + 1;

  if (!isLogName(name)) { Serial.println(R"([LOG] del: only LOG*.CSV may be deleted)"); return false; }

  // Do not delete the current file in use
  if (fname[0] && strcasecmp(pbuf, fname) == 0) {
    Serial.print("[LOG] del: refusing to delete current file: "); Serial.println(fname);
    return false;
  }

  if (!SD.exists(pbuf)) { Serial.print("[LOG] del: not found: "); Serial.println(pbuf); return false; }

  bool ok = SD.remove(pbuf);
  Serial.print("[LOG] del "); Serial.print(pbuf); Serial.println(ok ? " : OK" : " : FAILED");
  return ok;
}

// Delete all LOG*.CSV files in root, excluding current file if excludeCurrent=true.
// Returns the count of files successfully deleted.
static int delAll(bool excludeCurrent = true) {
  if (!sd_ok) { Serial.println(R"([LOG] SD not available.)"); return 0; }
  File root = SD.open("/");
  if (!root || !root.isDirectory()) { Serial.println(R"([LOG] Cannot open root directory.)"); return 0; }

  int count = 0;
  File entry;
  root.rewindDirectory();
  while ((entry = root.openNextFile())) {
    if (!entry.isDirectory()) {
      const char* name = entry.name();
      // Build absolute path for compare/remove
      char apath[32]; snprintf(apath, sizeof(apath), "/%s", name);

      bool isLog = isLogName(name);
      bool isCurrent = (excludeCurrent && fname[0] && strcasecmp(apath, fname) == 0);
      entry.close();

      if (isLog && !isCurrent) {
        if (SD.remove(apath)) { ++count; }
      }
    } else {
      entry.close();
    }
  }
  root.close();
  Serial.print(R"([LOG] delall: deleted )"); Serial.print(count); Serial.println(R"( file(s))");
  return count;
}

// Manual rotate (close current and open next)
static void rotate() {
  if (!sd_ok) { Serial.println(R"([LOG] SD not available; cannot rotate.)"); return; }
  startNewFile();
  if (sd_ok) Serial << "[LOG] Rotated to " << fname << endl;
}

// Print CSV header respecting the current mode (kept for compatibility)
static void header() {
  const char* h =
    "t_us,loop_us,dt_loop,read_idx,leg,joint_id,"
    "q_meas_rad,dq_meas_rads,q_est_rad,dq_est_rads,tempC,mV,"
    "q_cmd_rad,q_ref_rad,e_rad,phase,u,"
    "fx_mm,fy_mm,fz_mm,ik_ok,phase_u\n";

  if (currentMode == SERIAL_ONLY || currentMode == BOTH) {
    Serial << "# schema=v1.2, units: t_us,loop_us,dt[s],angles[rad],vin[mV],temp[C],foot[mm]\n";
    Serial << h;
  }

  if ((currentMode == SD_ONLY || currentMode == BOTH) && sd_ok && logFile) {
    // startNewFile() already wrote the header, but keep this idempotent
    logFile.flush();
  }
}

// Write one CSV row according to current mode and level
static void row(uint32_t t_us, uint32_t loop_us, float dt_loop,
                int read_idx, int leg_idx, uint8_t joint_id,
                float q_meas, float dq_meas, float q_est, float dq_est,
                int16_t tempC, int16_t mV,
                float q_cmd, float q_ref, float e,
                const char* phase_str, float u,
                float fx_mm, float fy_mm, float fz_mm, int ik_ok, float phase_u)
{
  if (currentMode == NONE) {
    return; // logging fully disabled
  }
  // Level filtering (BASIC drops some columns on Serial to stay light)
  const bool verbose = (currentLevel >= DETAIL);

  // Serial path
  if (currentMode == SERIAL_ONLY || currentMode == BOTH) {
   if (verbose) {
        Serial << t_us << ',' << loop_us << ',' << _FLOAT(dt_loop, 8) << ','
             << read_idx << ',' << leg_idx << ',' << (int)joint_id << ','
                << _FLOAT(q_meas, 8) << ',' << _FLOAT(dq_meas, 8) << ','
             << _FLOAT(q_est, 8) << ',' << _FLOAT(dq_est, 8) << ','
             << tempC << ',' << mV << ','
           << _FLOAT(q_cmd, 8) << ',' << _FLOAT(q_ref, 8) << ',' << _FLOAT(e, 8) << ','
         << phase_str << ',' << u << ','
         << _FLOAT(fx_mm, 3) << ',' << _FLOAT(fy_mm, 3) << ',' << _FLOAT(fz_mm, 3) << ','
         << ik_ok << ',' << _FLOAT(phase_u, 3) << '\n';
    } else {
      // BASIC: timestamp, leg/joint, q_meas, q_cmd
      Serial << t_us << ',' << leg_idx << ',' << (int)joint_id << ','
               << _FLOAT(q_meas, 8) << ',' << _FLOAT(q_cmd, 8) << '\n';
    }
  }

  // SD path
  if ((currentMode == SD_ONLY || currentMode == BOTH) && sd_ok && logFile) {
    // Always write full schema to SD
    logFile.print(t_us); logFile.print(',');
    logFile.print(loop_us); logFile.print(',');
      logFile.print(dt_loop, 8); logFile.print(',');
    logFile.print(read_idx); logFile.print(',');
    logFile.print(leg_idx); logFile.print(',');
    logFile.print((int)joint_id); logFile.print(',');
      logFile.print(q_meas, 8); logFile.print(',');
      logFile.print(dq_meas, 8); logFile.print(',');
      logFile.print(q_est, 8); logFile.print(',');
      logFile.print(dq_est, 8); logFile.print(',');
    logFile.print(tempC); logFile.print(',');
    logFile.print(mV); logFile.print(',');
      logFile.print(q_cmd, 8); logFile.print(',');
      logFile.print(q_ref, 8); logFile.print(',');
      logFile.print(e, 8); logFile.print(',');
    logFile.print(phase_str); logFile.print(',');
      logFile.print(u, 8); logFile.print(',');
      logFile.print(fx_mm, 3); logFile.print(',');
      logFile.print(fy_mm, 3); logFile.print(',');
      logFile.print(fz_mm, 3); logFile.print(',');
    logFile.print(ik_ok); logFile.print(',');
      logFile.print(phase_u, 3); logFile.print('\n');

    // Periodic flush + rollover + error degrade
    if ((++line_count % FLUSH_EVERY) == 0) {
      logFile.flush();
      if (!logFile) {
        sd_ok = false;
        currentMode = SERIAL_ONLY;
        Serial << "[LOG] SD write error → switching to Serial only." << endl;
      } else if (logFile.size() >= MAX_BYTES) {
        Serial << "[LOG] Rollover at " << logFile.size() << " bytes." << endl;
        rotate();
      }
    }
  }
}

// ------------------------- SD browsing utils ------------------------
static void list(const char* path = "/") {
  if (!sd_ok) { Serial.println(R"([LOG] SD not available. (Tip: run 'log show'))"); return; }
  File dir = SD.open(path);
  if (!dir) { Serial.print("[LOG] Cannot open path: "); Serial.println(path); return; }
  if (!dir.isDirectory()) { Serial.print("[LOG] Not a directory: "); Serial.println(path); dir.close(); return; }

  Serial.print(R"([LOG] Listing ')"); Serial.print(path); Serial.println(R"(':")");
  dir.rewindDirectory();
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    Serial.print(entry.isDirectory() ? " <DIR> " : "       ");
    Serial.print(entry.name());
    if (!entry.isDirectory()) { Serial.print("  "); Serial.print((uint32_t)entry.size()); Serial.print(" bytes"); }
    Serial.println();
    entry.close();
  }
  dir.close();
}

static void cat(const char* path, uint32_t max_bytes = 4096) {
  if (!sd_ok) { Serial.println(R"([LOG] SD not available. (Tip: run 'log show'))"); return; }
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.print(R"([LOG] Cannot open file: )"); Serial.println(path); return; }
  if (f.isDirectory()) { Serial.print(R"([LOG] Path is a directory, not a file: )"); Serial.println(path); f.close(); return; }

  const uint32_t fsize = f.size();
  uint32_t to_read = (max_bytes == 0) ? fsize : min(max_bytes, fsize);

  Serial.print(R"([LOG] cat ')"); Serial.print(path); Serial.print(R"(' ()");
  Serial.print(to_read); Serial.print(R"( of )"); Serial.print(fsize); Serial.println(R"( bytes):)" );

  static const size_t BUFSZ = 256;
  static uint8_t buf[BUFSZ];
  while (to_read > 0) {
    size_t chunk = (to_read < BUFSZ) ? to_read : BUFSZ;
    int n = f.read(buf, chunk);
    if (n <= 0) break;
    Serial.write(buf, n);
    to_read -= n;
    if (to_read % 4096 == 0) yield();
  }
  Serial.println();
  f.close();
}

// ------------------------- Simple config helpers ---------------------
// Persist and retrieve the 'every' cycles setting via the unified /config.txt.
// Key: log.every
// Example lines:
//   # Hexapod config
//   home_cdeg=...
//   log.every=166
static uint32_t loadEvery(uint32_t defaultVal = 166) {
  if (!sd_ok) return defaultVal;
  Config::ensureFile();
  long v = Config::getInt("log.every", (long)defaultVal);
  if (v < 1) v = defaultVal;
  return (uint32_t)v;
}

static bool saveEvery(uint32_t every) {
  if (!sd_ok) { Serial.println(R"([LOG] SD not available; cannot persist 'every'.)" ); return false; }
  Config::ensureFile();
  bool ok = Config::setInt("log.every", (long)every);
  if (!ok) Serial.println(R"([LOG] Failed to persist log.every to /config.txt)" );
  return ok;
}

// Persist and retrieve the log level (BASIC=0, DETAIL=1, DEBUG=2)
// Key: log.level
static Level loadLevel(Level defaultVal = DETAIL) {
  if (!sd_ok) return defaultVal;
  Config::ensureFile();
  long v = Config::getInt("log.level", (long)defaultVal);
  if (v < 0) v = 0; if (v > 2) v = 2;
  return (Level)v;
}

static bool saveLevel(Level level) {
  if (!sd_ok) { Serial.println(R"([LOG] SD not available; cannot persist 'level'.)"); return false; }
  Config::ensureFile();
  bool ok = Config::setInt("log.level", (long)level);
  if (!ok) Serial << R"([LOG] Failed to persist log.level to /config.txt)";
  return ok;
}

// Convenience wrappers preserved for compatibility with your main
static bool listLogs() {
  if (!sd_ok) { Serial.println(R"([LOG] SD not available.)"); return false; }
  File root = SD.open("/");
  if (!root) { Serial.println(R"([LOG] Failed to open root.)"); return false; }
  Serial.println(R"([LOG] Files on SD (LOG*.CSV):)");
  File entry;
  root.rewindDirectory();
  while ((entry = root.openNextFile())) {
    if (!entry.isDirectory()) {
      const char* name = entry.name();
      if (strlen(name) == 11 && strncmp(name, "LOG", 3) == 0 && strcasecmp(name + 7, ".CSV") == 0) {
        Serial.print("  "); Serial.print(name); Serial.print("  ");
  Serial.print((uint32_t)entry.size()); Serial.println(R"( bytes)");
      }
    }
    entry.close();
  }
  root.close();
  return true;
}

static bool printLog(const char* path, uint32_t max_bytes = 0) {
  if (!sd_ok) { Serial.println(R"([LOG] SD not available.)"); return false; }
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.print(R"([LOG] Cannot open )"); Serial.println(path); return false; }

  Serial.print(R"([LOG] Dumping )"); Serial.print(path);
  if (max_bytes) { Serial.print(R"( ()"); Serial.print(max_bytes); Serial.println(R"( bytes)…)"); }
  else           { Serial.println(R"( (all bytes)…)" ); }

  const size_t CHUNK = 256;
  static uint8_t buf[CHUNK];
  uint32_t remaining = max_bytes;
  while (true) {
    size_t want = CHUNK;
    if (max_bytes && remaining < want) want = remaining;
    int n = f.read(buf, want);
    if (n <= 0) break;
    Serial.write(buf, n);
    if (max_bytes) { remaining -= n; if (remaining == 0) break; }
    yield();
  }
  f.close();
  Serial.println(R"(
[LOG] End of file.)");
  return true;
}

static bool printLogIndex(int idx, uint32_t max_bytes = 0) {
  if (idx < 0 || idx > 999) { Serial.println(R"([LOG] Index out of range (0..999))"); return false; }
  char path[16]; snprintf(path, sizeof(path), "/LOG%03d.CSV", idx);
  return printLog(path, max_bytes);
}

} // namespace Log
