#pragma once
#include <stdint.h>

#define FLASH_PAGE_SIZE 4096
#define CONFIG_FLASH_BASE     0x08008000
#define CONFIG_FLASH_PAGES    8
#define CONFIG_MAGIC          0x574C4346
#define CONFIG_HEADER_SIZE    12

extern bool Flash_saves(void *buf, uint32_t length, uint32_t address);
bool Flash_ConfigSave(void *buf, uint16_t length);
bool Flash_ConfigLoad(void *buf, uint16_t max_len, uint16_t *out_len);
bool Flash_ConfigEraseAll(void);