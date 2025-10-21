#pragma once
#include <stdint.h>
#include <stddef.h>

// Memory gauge API (exposed for use from other .ino files)
void     stackCanaryInit();
bool     mem_canary_ok();
uint32_t mem_canary_window();
uintptr_t mem_heap_top_addr();
uintptr_t mem_sp_init_addr();
uint32_t mem_freeHeapGap();
uint32_t mem_stackFreeNow();
uint32_t mem_maxHeapAllocTest();
