// MemoryGauges.ino — Stack/heap diagnostics for Teensy 4.x
// Extracted from main sketch for clarity and reuse.

#include <Arduino.h>
#include <malloc.h>
#include "MemoryGauges.h"

extern "C" char* sbrk(int incr);
extern unsigned long _ebss;
extern unsigned long _estack;

static uint32_t* __canary_start         = nullptr;
static uint32_t* __canary_end           = nullptr;
static bool      __canary_ok            = false;
static uint32_t  __canary_painted_bytes = 0;
static uintptr_t __canary_heap_top_addr = 0;
static uintptr_t __canary_sp_init_addr  = 0;

static uint32_t freeHeapGap_impl() {
  volatile uint32_t sp_snap = 0;
  uint8_t* sp = (uint8_t*)&sp_snap;

  uint8_t* heap_base = (uint8_t*)&_ebss;
  struct mallinfo mi = mallinfo();
  size_t heap_used   = (size_t)mi.uordblks;
  uint8_t* heap_top  = heap_base + heap_used;
  __canary_heap_top_addr = (uintptr_t)heap_top;

  return (sp > heap_top) ? (uint32_t)(sp - heap_top) : 0u;
}

// Set MEM_GAUGES_CANARY to 1 to enable writing a canary between heap and stack.
// Default is 0 to avoid touching allocator-managed memory at boot (safer for USB).
#ifndef MEM_GAUGES_CANARY
#define MEM_GAUGES_CANARY 0
#endif

void stackCanaryInit() {
  __canary_ok            = false;
  __canary_painted_bytes = 0;

  uint8_t* heap_base = (uint8_t*)&_ebss;
  struct mallinfo mi = mallinfo();
  size_t heap_used   = (size_t)mi.uordblks;
  uint8_t* heap_top  = heap_base + heap_used;
  // Record current heap_top for early diagnostics printing
  __canary_heap_top_addr = (uintptr_t)heap_top;

  volatile uint32_t sp_snap = 0;
  uint8_t* sp = (uint8_t*)&sp_snap;
  __canary_sp_init_addr = (uintptr_t)sp;

  const size_t HEAP_MARGIN = 512;
  const size_t STACK_GUARD = 1024;
  const size_t MIN_WINDOW  = 256;

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

#if MEM_GAUGES_CANARY
  // Paint only a limited portion to avoid long blocking or touching reserved areas.
  const size_t PAINT_LIMIT = 16 * 1024; // 16 KB max to paint for safety
  size_t paint_bytes = window;
  if (paint_bytes > PAINT_LIMIT) paint_bytes = PAINT_LIMIT;
  __canary_end = (uint32_t*)((uint8_t*)__canary_start + paint_bytes);

  for (uint32_t* p = __canary_start; p < __canary_end; ++p) {
    *p = 0xDEADBEEF;
  }

  __canary_ok            = true;
  __canary_painted_bytes = (uint32_t)paint_bytes;
#else
  // Canary disabled: don't write into RAM. Report zero window and keep canary off.
  __canary_start = nullptr;
  __canary_end   = nullptr;
  __canary_ok    = false;
  __canary_painted_bytes = 0;
#endif
}

static uint32_t stackFreeNow_impl() {
  if (!__canary_ok || !__canary_start || !__canary_end) return 0u;

  uint32_t* p = __canary_start;
  while (p < __canary_end && *p == 0xDEADBEEF) ++p;

  // FREE bytes = intact prefix length
  return (uint32_t)((uint8_t*)p - (uint8_t*)__canary_start);
}

static uint32_t maxHeapAllocTest_impl() {
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

// Accessors for other modules
bool mem_canary_ok() { return __canary_ok; }
uint32_t mem_canary_window() { return __canary_painted_bytes; }
uintptr_t mem_heap_top_addr() { return __canary_heap_top_addr; }
uintptr_t mem_sp_init_addr() { return __canary_sp_init_addr; }
uint32_t mem_freeHeapGap() { return freeHeapGap_impl(); }
uint32_t mem_stackFreeNow() { return stackFreeNow_impl(); }
uint32_t mem_maxHeapAllocTest() { return maxHeapAllocTest_impl(); }
