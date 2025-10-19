#pragma once
// Config.h — Minimal key=value config on SD (header-only)
// -----------------------------------------------------
// Stores configuration in a single text file on SD: /config.txt
// Format:
//   # comments allowed
//   key = value
// Keys are case-sensitive. Values are raw strings; helpers provided for int and int-list.
// No dynamic allocation; merges are done via a temporary file (/config.tmp).

#include <Arduino.h>
#include <SD.h>
#include <string.h>

namespace Config {

static const char* CONFIG_PATH = "/config.txt";
static const char* CONFIG_TMP   = "/config.tmp";
static const char* FACTORY_PATH = "/config.factory.txt";
static const char* BACKUP_PATH  = "/config.backup.txt";

// Trim helpers (in-place)
static inline void rtrim(char* s) {
  if (!s) return;
  size_t n = strlen(s);
  while (n > 0) {
    char c = s[n - 1];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') { s[--n] = '\0'; }
    else break;
  }
}

static inline char* lskip(const char* s) {
  while (*s == ' ' || *s == '\t') ++s;
  return (char*)s;
}

// Read a single logical line (up to bufSz-1 chars). Returns length or -1 on EOF.
static int readLine(File& f, char* buf, size_t bufSz) {
  if (!buf || bufSz == 0) return -1;
  size_t n = 0;
  while (f.available()) {
    int c = f.read();
    if (c < 0) break;
    if (c == '\n') break;
    if (n < bufSz - 1) buf[n++] = (char)c;
  }
  if (n == 0 && !f.available()) return -1;
  buf[n] = '\0';
  return (int)n;
}

// Find a key in CONFIG_PATH and copy its (trimmed) value into outBuf.
static bool getString(const char* key, char* outBuf, size_t outSz) {
  if (!key || !outBuf || outSz == 0) return false;
  File f = SD.open(CONFIG_PATH, FILE_READ);
  if (!f) return false;

  const size_t BUFSZ = 256;
  char line[BUFSZ];
  bool found = false;
  while (true) {
    int len = readLine(f, line, sizeof(line));
    if (len < 0) break;
    char* p = lskip(line);
    if (*p == '\0' || *p == '#' || *p == ';') continue;
    // Find '='
    char* eq = strchr(p, '=');
    if (!eq) continue;
    // Key is p..(eq-1), trim right
    *eq = '\0';
    rtrim(p);
    if (strcmp(p, key) != 0) continue;
    // Value is (eq+1) trimmed on both sides
    char* v = lskip(eq + 1);
    rtrim(v);
    // safe copy
    size_t vl = strlen(v);
    if (vl >= outSz) vl = outSz - 1;
    memcpy(outBuf, v, vl);
    outBuf[vl] = '\0';
    found = true;
    break;
  }
  f.close();
  return found;
}

static long getInt(const char* key, long defaultVal = 0) {
  char buf[32];
  if (!getString(key, buf, sizeof(buf))) return defaultVal;
  char* endp = nullptr;
  long v = strtol(buf, &endp, 10);
  if (endp == buf) return defaultVal;
  return v;
}

// Parse comma-separated integers into outArr. Returns true if at least one parsed and <= maxCount.
static bool getIntList(const char* key, long* outArr, int maxCount, int* outCount = nullptr) {
  if (!key || !outArr || maxCount <= 0) return false;
  const size_t VSZ = 256;
  char vbuf[VSZ];
  if (!getString(key, vbuf, sizeof(vbuf))) return false;

  int n = 0;
  char* s = vbuf;
  while (*s && n < maxCount) {
    // Skip separators/space
    while (*s == ' ' || *s == '\t' || *s == ',') ++s;
    if (*s == '\0') break;
    char* endp = nullptr;
    long v = strtol(s, &endp, 10);
    if (endp == s) break; // no progress
    outArr[n++] = v;
    s = endp;
    while (*s == ' ' || *s == '\t') ++s;
    if (*s == ',') ++s;
  }
  if (outCount) *outCount = n;
  return n > 0;
}

// Write a single key=value (merging if key exists). Value provided as C string.
static bool setString(const char* key, const char* value) {
  if (!key || !*key || !value) return false;

  // Open existing for read (optional) and tmp for write
  File in = SD.open(CONFIG_PATH, FILE_READ);
  SD.remove(CONFIG_TMP);
  File out = SD.open(CONFIG_TMP, FILE_WRITE);
  if (!out) {
    if (in) in.close();
    Serial.println("[CFG] Failed to open temp file for write.");
    return false;
  }

  bool replaced = false;
  const size_t BUFSZ = 256;
  char line[BUFSZ];

  if (in) {
    while (true) {
      int len = readLine(in, line, sizeof(line));
      if (len < 0) break;
      char* p = lskip(line);
      if (*p == '\0') { out.println(); continue; }
      if (*p == '#' || *p == ';') { out.println(line); continue; }
      char* eq = strchr(p, '=');
      if (!eq) { out.println(line); continue; }
      *eq = '\0';
      rtrim(p);
      if (strcmp(p, key) == 0) {
        out.print(key); out.print('='); out.println(value);
        replaced = true;
      } else {
        // restore original
        *eq = '=';
        out.println(line);
      }
    }
    in.close();
  }

  if (!replaced) {
    out.print(key); out.print('='); out.println(value);
  }
  out.flush(); out.close();

  // Replace original with temp: copy temp -> final
  SD.remove(CONFIG_PATH);
  File tmp = SD.open(CONFIG_TMP, FILE_READ);
  File fin = SD.open(CONFIG_PATH, FILE_WRITE);
  if (!tmp || !fin) {
    if (tmp) tmp.close();
    if (fin) fin.close();
    SD.remove(CONFIG_TMP);
    Serial.println("[CFG] Failed to finalize config write.");
    return false;
  }
  const size_t COPYBUF = 128;
  uint8_t buf[COPYBUF];
  while (true) {
    int n = tmp.read(buf, COPYBUF);
    if (n <= 0) break;
    fin.write(buf, n);
  }
  fin.close();
  tmp.close();
  SD.remove(CONFIG_TMP);
  return true;
}

static bool setInt(const char* key, long v) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%ld", v);
  return setString(key, buf);
}

static bool setIntList(const char* key, const long* arr, int count) {
  if (!key || !arr || count <= 0) return false;
  // Build CSV into a small local buffer
  const size_t VSZ = 8 * 32; // enough for 18 ints with commas
  char vbuf[VSZ]; size_t pos = 0; vbuf[0] = '\0';
  for (int i = 0; i < count; ++i) {
    int n = snprintf(vbuf + pos, (pos < VSZ ? VSZ - pos : 0), (i == 0 ? "%ld" : ",%ld"), arr[i]);
    if (n <= 0) break; pos += (size_t)n; if (pos >= VSZ) { vbuf[VSZ - 1] = '\0'; break; }
  }
  return setString(key, vbuf);
}

static bool exists() { return SD.exists(CONFIG_PATH); }
static bool factoryExists() { return SD.exists(FACTORY_PATH); }

// Ensure config file exists (create empty if missing)
static void ensureFile() {
  if (exists()) return;
  File f = SD.open(CONFIG_PATH, FILE_WRITE);
  if (!f) { Serial.println("[CFG] Cannot create /config.txt"); return; }
  f.println("# Hexapod config (key=value)\n# home_cdeg=18 comma-separated centidegree ints\n# log.every=166");
  f.close();
  Serial.println("[CFG] Created /config.txt");
}

// Ensure a factory defaults file exists with sane defaults, including original home angles.
// This file can be copied over /config.txt to reset configuration.
static void ensureFactoryFile() {
  if (factoryExists()) return;
  File f = SD.open(FACTORY_PATH, FILE_WRITE);
  if (!f) { Serial.println("[CFG] Cannot create /config.factory.txt"); return; }
  f.println("# Hexapod factory defaults — do not edit on robot if you want to keep a clean baseline");
  f.println("# Copy this over /config.txt to reset settings (see 'cfg factory')\n");
  // Original hard-coded home angles (centideg) from legacy codebase
  f.println("home_cdeg=12336,11184,10896,12544,11736,11352,10392,10968,10056,11232,11976,12192,11712,11400,13944,11544,11112,10968");
  // Logging defaults
  f.println("log.every=166");
  f.println("log.level=1");
  // Safety defaults
  f.println("safety.over_temp_c=70");
  f.println("safety.low_bus_mv=7000");
  f.println("safety.min_valid_mv=3000");
  // Gait defaults
  f.println("gait.stance_mm=-120");
  f.println("gait.stride_mm=80");
  f.println("gait.lift_mm=20");
  f.println("gait.stance_ms=300");
  f.println("gait.swing_ms=150");
  f.close();
  Serial.println("[CFG] Created /config.factory.txt");
}

// Copy a file on SD from src to dst, overwriting dst. Returns true on success.
static bool copyFile(const char* src, const char* dst) {
  if (!src || !dst) return false;
  File in = SD.open(src, FILE_READ);
  if (!in) { Serial.print("[CFG] copy: cannot open src: "); Serial.println(src); return false; }
  SD.remove(dst);
  File out = SD.open(dst, FILE_WRITE);
  if (!out) { Serial.print("[CFG] copy: cannot open dst: "); Serial.println(dst); in.close(); return false; }
  uint8_t buf[256];
  while (true) {
    int n = in.read(buf, sizeof(buf));
    if (n <= 0) break;
    if (out.write(buf, n) != n) { Serial.println("[CFG] copy: write error"); in.close(); out.close(); return false; }
  }
  out.flush(); out.close(); in.close();
  return true;
}

// Reset /config.txt to factory defaults by copying FACTORY_PATH over CONFIG_PATH
static bool resetToFactory() {
  ensureFactoryFile();
  if (!factoryExists()) { Serial.println("[CFG] Factory defaults missing."); return false; }
  bool ok = copyFile(FACTORY_PATH, CONFIG_PATH);
  Serial.println(ok ? "[CFG] /config.txt reset to factory defaults." : "[CFG] Reset failed.");
  return ok;
}

// Backup current /config.txt to BACKUP_PATH. Returns true if copied.
static bool backupCurrent() {
  if (!SD.exists(CONFIG_PATH)) { Serial.println("[CFG] No /config.txt to back up."); return false; }
  bool ok = copyFile(CONFIG_PATH, BACKUP_PATH);
  Serial.println(ok ? "[CFG] Backed up /config.txt to /config.backup.txt" : "[CFG] Backup failed.");
  return ok;
}

// Restore /config.txt from BACKUP_PATH. Returns true if copied.
static bool restoreFromBackup() {
  if (!SD.exists(BACKUP_PATH)) { Serial.println("[CFG] No /config.backup.txt to restore from."); return false; }
  bool ok = copyFile(BACKUP_PATH, CONFIG_PATH);
  Serial.println(ok ? "[CFG] Restored /config.txt from /config.backup.txt" : "[CFG] Restore failed.");
  return ok;
}

} // namespace Config
