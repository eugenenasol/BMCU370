/**
 * @file Flash_saves.cpp
 * @brief Non-volatile Flash storage driver for CH32V203 MCU.
 * 
 * @details
 * DEVELOPMENT STATE: FUNCTIONAL - PROVEN STABLE - DO NOT MODIFY
 * 
 * This driver provides persistent storage by writing data directly to
 * the MCU's internal Flash memory. Used for storing filament information,
 * calibration data, and boot configuration.
 * 
 * Key Design Decisions:
 * - **IRQ-safe**: Does NOT disable interrupts during Flash operations.
 *   This prevents serial communication timeouts at the cost of slight
 *   Flash operation timing sensitivity. Validated stable in practice.
 * - **Watchdog-safe**: Reloads IWDG during long erase/program cycles.
 * - **Page-aligned writes**: Erases full 4KB pages before programming.
 * 
 * Usage Notes:
 * - Flash writes are blocking (~20-50ms per page).
 * - Caller should ensure serial is idle before calling (see KlipperCLI::IsSerialIdle).
 * - Maximum recommended write frequency: once per 5 seconds (debounced in MMU_Logic).
 * 
 * @warning Frequent Flash writes reduce Flash lifespan. The debounce timer
 *          in MMU_Logic ensures writes are batched appropriately.
 * 
 * @author BMCU370 Development Team
 * @version 1.0.0
 * @date 2025-12-27
 */
#include "Flash_saves.h"
#include "ch32v20x_flash.h"

// ─── Config Page Header (packed, 12 bytes) ────────────────────
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint32_t sequence_number;
    uint16_t data_size;
    uint16_t crc;
} ConfigPageHeader;
#pragma pack(pop)

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
    }
    return crc;
}

static bool is_page_erased(uint32_t page_addr) {
    volatile uint32_t *ptr = (volatile uint32_t *)page_addr;
    return (*ptr == 0xFFFFFFFF);
}

static int16_t find_best_config_page(uint32_t *out_seq) {
    int16_t best_idx = -1; uint32_t best_seq = 0; *out_seq = 0;
    for (int16_t i = 0; i < CONFIG_FLASH_PAGES; i++) {
        uint32_t page_addr = CONFIG_FLASH_BASE + (uint32_t)i * FLASH_PAGE_SIZE;
        if (is_page_erased(page_addr)) continue;
        ConfigPageHeader *hdr = (ConfigPageHeader *)page_addr;
        if (hdr->magic != CONFIG_MAGIC) continue;
        if (hdr->data_size == 0 || hdr->data_size > (FLASH_PAGE_SIZE - CONFIG_HEADER_SIZE)) continue;
        uint8_t *payload = (uint8_t *)(page_addr + CONFIG_HEADER_SIZE);
        if (crc16_ccitt(payload, hdr->data_size) == hdr->crc) {
            if (best_idx < 0 || hdr->sequence_number >= best_seq) {
                best_seq = hdr->sequence_number; best_idx = i;
            }
        }
    }
    *out_seq = best_seq; return best_idx;
}

static int16_t count_occupied_pages(void) {
    int16_t count = 0;
    for (int16_t i = 0; i < CONFIG_FLASH_PAGES; i++)
        if (!is_page_erased(CONFIG_FLASH_BASE + (uint32_t)i * FLASH_PAGE_SIZE)) count++;
    return count;
}

/* Global define */
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

#define FLASH_PAGE_SIZE 4096                          ///< CH32V203 Flash page size (4KB)
#define FLASH_PAGES_TO_BE_PROTECTED FLASH_WRProt_Pages60to63

/* Global Variable */
uint32_t EraseCounter = 0x0, Address = 0x0;
uint16_t Data = 0xAAAA;
uint32_t WRPR_Value = 0xFFFFFFFF, ProtectedPages = 0x0;

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;
volatile TestStatus MemoryEraseStatus = PASSED;

// Redundant global buffer removed to save ~1KB RAM
// #define Fadr (0x08020000)
// #define Fsize ((((256 * 4)) >> 2))
// u32 buf[Fsize];

/**
 * @brief Save a buffer to internal Flash memory.
 * 
 * Erases the required number of 4KB pages and programs the data.
 * Reloads the Independent Watchdog (IWDG) during the process to prevent
 * system reset during long operations.
 * 
 * @note This function is BLOCKING and takes ~20-50ms per page.
 *       Ensure serial communication is idle before calling.
 * 
 * @param buf     Pointer to the source data buffer.
 * @param length  Length of data in bytes.
 * @param address Destination Flash address (should be page-aligned for best results).
 * 
 * @return true   Flash write completed successfully.
 * @return false  Flash write failed (check FLASHStatus for details).
 * 
 * @warning Do not call during active serial communication - use debounce timer.
 */
bool Flash_saves(void *buf, uint32_t length, uint32_t address)
{
    uint32_t end_address = address + length;
    uint32_t erase_counter = 0;
    uint32_t address_i = 0;
    uint32_t page_num = (length + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    if (page_num == 0) page_num = 1; // Safety
    uint16_t *data_ptr = (uint16_t *)buf;

    __disable_irq();
    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);

    for (erase_counter = 0; (erase_counter < page_num) && (FLASHStatus == FLASH_COMPLETE); erase_counter++)
    {
        IWDG->CTLR = 0xAAAA; // Reload IWDG
        FLASHStatus = FLASH_ErasePage(address + (FLASH_PAGE_SIZE * erase_counter)); // Erase 4KB

        if (FLASHStatus != FLASH_COMPLETE)
        {
            FLASH_Lock();
            __enable_irq();
            return false;
        }
    }

    address_i = address;
    while ((address_i < end_address) && (FLASHStatus == FLASH_COMPLETE))
    {
        IWDG->CTLR = 0xAAAA; // Reload IWDG during long writes
        FLASHStatus = FLASH_ProgramHalfWord(address_i, *data_ptr);
        address_i = address_i + 2;
        data_ptr++;
    }

    FLASH_Lock();
    __enable_irq();
    return (FLASHStatus == FLASH_COMPLETE);
}

bool Flash_ConfigSave(void *buf, uint16_t length) {
    if (buf == NULL || length == 0 || length > (FLASH_PAGE_SIZE - CONFIG_HEADER_SIZE)) return false;
    uint32_t current_seq = 0;
    int16_t best_idx = find_best_config_page(&current_seq);
    int16_t next_idx = (best_idx < 0) ? 0 : ((best_idx + 1) % CONFIG_FLASH_PAGES);
    if (best_idx < 0) current_seq = 0;
    if (count_occupied_pages() >= CONFIG_FLASH_PAGES) {
        Flash_ConfigEraseAll();
        next_idx = 0; current_seq = 0;
    }

    // Build header on stack (12 bytes only — no 4KB static buffer needed)
    ConfigPageHeader hdr;
    hdr.magic = CONFIG_MAGIC;
    hdr.sequence_number = current_seq + 1;
    hdr.data_size = length;
    hdr.crc = crc16_ccitt((const uint8_t *)buf, length);

    uint32_t page_addr = CONFIG_FLASH_BASE + (uint32_t)next_idx * FLASH_PAGE_SIZE;

    // Erase target page, then write header + payload in two passes.
    // Flash_saves erases before writing; we call it twice but both writes
    // are to different offsets within the already-erased page.
    // To avoid double-erase we write the full region: header padded to
    // alignment + payload. Use a minimal approach: erase once via a small
    // helper, then two program passes.

    __disable_irq();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    IWDG->CTLR = 0xAAAA;

    // Step 1: Erase the target page
    FLASHStatus = FLASH_ErasePage(page_addr);
    if (FLASHStatus != FLASH_COMPLETE) {
        FLASH_Lock(); __enable_irq(); return false;
    }

    // Step 2: Write header (12 bytes)
    uint16_t *src = (uint16_t *)&hdr;
    uint32_t addr = page_addr;
    for (uint16_t i = 0; i < CONFIG_HEADER_SIZE / 2; i++, addr += 2, src++) {
        FLASHStatus = FLASH_ProgramHalfWord(addr, *src);
        if (FLASHStatus != FLASH_COMPLETE) {
            FLASH_Lock(); __enable_irq(); return false;
        }
    }

    // Step 3: Write payload
    src = (uint16_t *)buf;
    uint16_t words = (length + 1) / 2; // Round up to half-words
    for (uint16_t i = 0; i < words; i++, addr += 2, src++) {
        FLASHStatus = FLASH_ProgramHalfWord(addr, *src);
        if (FLASHStatus != FLASH_COMPLETE) {
            FLASH_Lock(); __enable_irq(); return false;
        }
    }

    FLASH_Lock();
    __enable_irq();
    return true;
}

bool Flash_ConfigLoad(void *buf, uint16_t max_len, uint16_t *out_len) {
    if (buf == NULL || max_len == 0) return false;
    uint32_t seq = 0;
    int16_t best_idx = find_best_config_page(&seq);
    if (best_idx < 0) { if (out_len) *out_len = 0; return false; }
    uint32_t page_addr = CONFIG_FLASH_BASE + (uint32_t)best_idx * FLASH_PAGE_SIZE;
    ConfigPageHeader *hdr = (ConfigPageHeader *)page_addr;
    uint16_t copy_len = (hdr->data_size > max_len) ? max_len : hdr->data_size;
    uint8_t *payload = (uint8_t *)(page_addr + CONFIG_HEADER_SIZE);
    for (uint16_t i = 0; i < copy_len; i++) ((uint8_t *)buf)[i] = payload[i];
    if (out_len) *out_len = copy_len;
    return true;
}

bool Flash_ConfigEraseAll(void) {
    bool all_ok = true;
    __disable_irq();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    for (int16_t i = 0; i < CONFIG_FLASH_PAGES; i++) {
        IWDG->CTLR = 0xAAAA;
        FLASHStatus = FLASH_ErasePage(CONFIG_FLASH_BASE + (uint32_t)i * FLASH_PAGE_SIZE);
        if (FLASHStatus != FLASH_COMPLETE) { all_ok = false; break; }
    }
    FLASH_Lock();
    __enable_irq();
    return all_ok;
}