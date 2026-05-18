#pragma once
#include <stdint.h>

#define FLASH_PAGE_SIZE 4096
// Safe config area: firmware is ~47KB (ends ~0x0800BFFF).
// Use end-of-flash to avoid overlapping with code.
#define CONFIG_FLASH_BASE     0x0800E000
#define CONFIG_FLASH_PAGES    2
#define CONFIG_MAGIC          0x574C4346
#define CONFIG_HEADER_SIZE    12

extern bool Flash_saves(void *buf, uint32_t length, uint32_t address);
bool Flash_ConfigSave(void *buf, uint16_t length);
bool Flash_ConfigLoad(void *buf, uint16_t max_len, uint16_t *out_len);
bool Flash_ConfigEraseAll(void);