/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file main.c
 * @brief STM32L476RG Dedicated Dual-Bank SD Card Bootloader for WildSonic.
 *
 * Implements authoritative dual-bank firmware update and handoff:
 *  - Bank 1 (0x08000000 - 0x0807FFFF, 512 KB): Immutable Bootloader
 *  - Bank 2 (0x08080000 - 0x080F7FFF, 480 KB): Production Application
 *  - Bank 2 Top (0x080F8000 - 0x080FFFFF, 32 KB): Config Partition
 *
 * Update Process:
 *  1. Mounts SD card via SDMMC / FatFs.
 *  2. Scans root for 'wildsonic_stm32_l476_v*.bin' and selects newest version.
 *  3. Validates 64-byte release header: magic 0x57533332, format v1, MCU ID 0x00004760, header CRC32.
 *  4. Pre-flash validation: streams payload, verifies full payload CRC32, verifies vector table SP and Reset Vector.
 *  5. If valid, erases Bank 2 pages, writes 64-bit double-words, readback verifies CRC32.
 *  6. Renames/removes SD package on success.
 *  7. Sets SCB->VTOR to 0x08080000, updates MSP, and jumps to application Reset Handler.
 */

#include "bootloader.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/fs/fs.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/crc.h>

#if defined(CONFIG_SOC_SERIES_STM32L4X)
#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_flash.h>
#endif

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define BOOTLOADER_READ_CHUNK 1024

static uint8_t s_chunk_buffer[BOOTLOADER_READ_CHUNK] __aligned(8);

static uint32_t bootloader_crc32(const uint8_t *data, size_t len)
{
	return crc32_ieee(data, len);
}

static bool is_valid_sram_address(uint32_t addr)
{
	return (addr >= SRAM_START_ADDR && addr <= SRAM_END_ADDR);
}

static bool is_valid_bank2_app_address(uint32_t addr)
{
	/* Must be within Bank 2 application space and odd (Thumb mode) */
	return ((addr & 1) != 0 &&
	        addr >= APP_BANK2_START_ADDR &&
	        addr < (APP_BANK2_START_ADDR + APP_BANK2_MAX_SIZE));
}

static bool parse_firmware_filename(const char *name, uint8_t *major, uint8_t *minor, uint8_t *patch)
{
	/* Expected format: wildsonic_stm32_l476_v<maj>.<min>.<patch>.bin */
	const char prefix[] = "wildsonic_stm32_l476_v";
	if (strncmp(name, prefix, sizeof(prefix) - 1) != 0) {
		return false;
	}
	const char *p = name + sizeof(prefix) - 1;
	unsigned int maj = 0, min = 0, pat = 0;
	if (sscanf(p, "%u.%u.%u.bin", &maj, &min, &pat) == 3) {
		if (major) *major = (uint8_t)maj;
		if (minor) *minor = (uint8_t)min;
		if (patch) *patch = (uint8_t)pat;
		return true;
	}
	return false;
}

static bool scan_sd_for_update(char *out_path, size_t max_len, uint8_t *best_maj, uint8_t *best_min, uint8_t *best_pat)
{
	struct fs_dir_t dir;
	fs_dir_t_init(&dir);

	int rc = fs_opendir(&dir, "/SD:");
	if (rc != 0) {
		return false;
	}

	bool found = false;
	uint32_t best_version_packed = 0;

	struct fs_dirent entry;
	while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
		if (entry.type == FS_DIR_ENTRY_FILE) {
			uint8_t maj = 0, min = 0, pat = 0;
			if (parse_firmware_filename(entry.name, &maj, &min, &pat)) {
				uint32_t packed = ((uint32_t)maj << 16) | ((uint32_t)min << 8) | (uint32_t)pat;
				if (!found || packed >= best_version_packed) {
					best_version_packed = packed;
					found = true;
					if (best_maj) *best_maj = maj;
					if (best_min) *best_min = min;
					if (best_pat) *best_pat = pat;
					snprintf(out_path, max_len, "/SD:/%s", entry.name);
				}
			}
		}
	}
	fs_closedir(&dir);
	return found;
}

static bool validate_and_install_update(const char *file_path)
{
	struct fs_file_t file;
	fs_file_t_init(&file);

	int rc = fs_open(&file, file_path, FS_O_READ);
	if (rc != 0) {
		printk("[BOOTLOADER] Error: Failed to open %s (%d)\n", file_path, rc);
		return false;
	}

	struct wildsonic_release_header hdr;
	ssize_t bytes_read = fs_read(&file, &hdr, sizeof(hdr));
	if (bytes_read != sizeof(hdr)) {
		printk("[BOOTLOADER] Error: Header read failed (%d)\n", (int)bytes_read);
		fs_close(&file);
		return false;
	}

	if (hdr.magic != RELEASE_HEADER_MAGIC) {
		printk("[BOOTLOADER] Error: Invalid magic 0x%08X (expected 0x%08X)\n",
		       hdr.magic, RELEASE_HEADER_MAGIC);
		fs_close(&file);
		return false;
	}

	if (hdr.format_version != HEADER_FORMAT_VERSION) {
		printk("[BOOTLOADER] Error: Unsupported header format version %u\n", hdr.format_version);
		fs_close(&file);
		return false;
	}

	if (hdr.target_mcu_id != STM32L476_TARGET_MCU_ID) {
		printk("[BOOTLOADER] Error: Target MCU ID mismatch: 0x%08X\n", hdr.target_mcu_id);
		fs_close(&file);
		return false;
	}

	uint32_t computed_hdr_crc = bootloader_crc32((const uint8_t *)&hdr, 60);
	if (computed_hdr_crc != hdr.header_crc32) {
		printk("[BOOTLOADER] Error: Header CRC mismatch (calc 0x%08X vs hdr 0x%08X)\n",
		       computed_hdr_crc, hdr.header_crc32);
		fs_close(&file);
		return false;
	}

	if (hdr.payload_size == 0 || hdr.payload_size > APP_BANK2_MAX_SIZE) {
		printk("[BOOTLOADER] Error: Payload size %u exceeds Bank 2 limit (%u B)\n",
		       hdr.payload_size, APP_BANK2_MAX_SIZE);
		fs_close(&file);
		return false;
	}

	/* Seek to payload */
	rc = fs_seek(&file, hdr.payload_offset, FS_SEEK_SET);
	if (rc != 0) {
		printk("[BOOTLOADER] Error: Seek to payload failed (%d)\n", rc);
		fs_close(&file);
		return false;
	}

	/* Stream through payload to verify CRC32 and check vector table */
	uint32_t remaining = hdr.payload_size;
	uint32_t computed_payload_crc = 0xFFFFFFFF;
	bool vector_checked = false;

	while (remaining > 0) {
		size_t to_read = (remaining > BOOTLOADER_READ_CHUNK) ? BOOTLOADER_READ_CHUNK : remaining;
		ssize_t r = fs_read(&file, s_chunk_buffer, to_read);
		if (r <= 0) {
			printk("[BOOTLOADER] Error: Premature EOF reading payload\n");
			fs_close(&file);
			return false;
		}

		if (!vector_checked && r >= 8) {
			uint32_t initial_sp = *(uint32_t *)&s_chunk_buffer[0];
			uint32_t reset_vector = *(uint32_t *)&s_chunk_buffer[4];

			if (!is_valid_sram_address(initial_sp)) {
				printk("[BOOTLOADER] Pre-flash check failed: SP 0x%08X not in SRAM\n", initial_sp);
				fs_close(&file);
				return false;
			}
			if (!is_valid_bank2_app_address(reset_vector)) {
				printk("[BOOTLOADER] Pre-flash check failed: Reset vector 0x%08X invalid for Bank 2\n", reset_vector);
				fs_close(&file);
				return false;
			}
			vector_checked = true;
		}

		/* Update running CRC32 */
		computed_payload_crc = crc32_ieee_update(computed_payload_crc, s_chunk_buffer, (size_t)r);
		remaining -= (uint32_t)r;
	}

	computed_payload_crc ^= 0xFFFFFFFF;
	if (computed_payload_crc != hdr.payload_crc32) {
		printk("[BOOTLOADER] Error: Payload CRC mismatch (calc 0x%08X vs hdr 0x%08X)\n",
		       computed_payload_crc, hdr.payload_crc32);
		fs_close(&file);
		return false;
	}

	printk("[BOOTLOADER] Image pre-flash validation PASSED. Version: %u.%u.%u (%u bytes)\n",
	       hdr.version_major, hdr.version_minor, hdr.version_patch, hdr.payload_size);

	/* Flash Programming: Erase Bank 2 pages */
	uint32_t pages_needed = (hdr.payload_size + STM32L476_FLASH_PAGE_SIZE - 1) / STM32L476_FLASH_PAGE_SIZE;
	printk("[BOOTLOADER] Programming Bank 2 Flash: %u pages (%u KB)...\n",
	       pages_needed, (pages_needed * STM32L476_FLASH_PAGE_SIZE) / 1024);

#if defined(CONFIG_SOC_SERIES_STM32L4X)
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef erase_init;
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	erase_init.Banks = FLASH_BANK_2;
	erase_init.Page = 256; /* Bank 2 starts at page 256 on 1MB STM32L476 */
	erase_init.NbPages = pages_needed;
	uint32_t page_error = 0;

	HAL_StatusTypeDef hres = HAL_FLASHEx_Erase(&erase_init, &page_error);
	if (hres != HAL_OK) {
		printk("[BOOTLOADER] Error: Flash erase failed (HAL status %d, page error 0x%08X)\n",
		       (int)hres, page_error);
		HAL_FLASH_Lock();
		fs_close(&file);
		return false;
	}

	/* Rewind file to payload start */
	fs_seek(&file, hdr.payload_offset, FS_SEEK_SET);
	uint32_t flash_addr = APP_BANK2_START_ADDR;
	remaining = hdr.payload_size;

	while (remaining > 0) {
		size_t to_read = (remaining > BOOTLOADER_READ_CHUNK) ? BOOTLOADER_READ_CHUNK : remaining;
		/* Ensure double-word (8 bytes) alignment */
		size_t aligned_read = (to_read + 7) & ~7;
		memset(s_chunk_buffer, 0xFF, aligned_read);

		ssize_t r = fs_read(&file, s_chunk_buffer, to_read);
		if (r <= 0) {
			break;
		}

		for (size_t offset = 0; offset < (size_t)r; offset += 8) {
			uint64_t dword = *(uint64_t *)&s_chunk_buffer[offset];
			hres = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_addr + offset, dword);
			if (hres != HAL_OK) {
				printk("[BOOTLOADER] Error: Program failed at 0x%08X (%d)\n",
				       flash_addr + offset, (int)hres);
				HAL_FLASH_Lock();
				fs_close(&file);
				return false;
			}
		}

		flash_addr += (uint32_t)r;
		remaining -= (uint32_t)r;
	}

	HAL_FLASH_Lock();
#endif

	fs_close(&file);

	/* Readback Verification */
	printk("[BOOTLOADER] Verifying programmed flash against payload CRC32...\n");
	uint32_t flash_crc = bootloader_crc32((const uint8_t *)APP_BANK2_START_ADDR, hdr.payload_size);
	if (flash_crc != hdr.payload_crc32) {
		printk("[BOOTLOADER] Error: Readback CRC mismatch (calc 0x%08X vs expected 0x%08X)\n",
		       flash_crc, hdr.payload_crc32);
		return false;
	}

	printk("[BOOTLOADER] Flash readback verification PASSED (CRC 0x%08X)\n", flash_crc);

	/* Mark update file as completed by renaming */
	char done_path[128];
	snprintf(done_path, sizeof(done_path), "%s.done", file_path);
	(void)fs_rename(file_path, done_path);

	return true;
}

#ifndef WILDSONIC_GIT_COMMIT
#define WILDSONIC_GIT_COMMIT "unknown"
#endif

static void boot_application(void)
{
	uint32_t initial_sp = *(volatile uint32_t *)APP_BANK2_START_ADDR;
	uint32_t reset_handler = *(volatile uint32_t *)(APP_BANK2_START_ADDR + 4);

	if (!is_valid_sram_address(initial_sp) || !is_valid_bank2_app_address(reset_handler)) {
		printk("[BOOTLOADER] No valid application found at 0x%08X (SP=0x%08X, Reset=0x%08X)\n",
		       APP_BANK2_START_ADDR, initial_sp, reset_handler);
		return;
	}

	printk("Jumping to the main firmware...\n");

	/* Disable interrupts */
	__disable_irq();

	/* Relocate Vector Table to Bank 2 */
	SCB->VTOR = APP_BANK2_START_ADDR;

	/* Set Main Stack Pointer */
	__set_MSP(initial_sp);

	/* Jump to Application */
	void (*app_entry)(void) = (void (*)(void))reset_handler;
	app_entry();
}

int main(void)
{
	printk("Bootloader SHA: %s\n", WILDSONIC_GIT_COMMIT);
	printk("Bootloader Build Date: %s %s\n", __DATE__, __TIME__);

	char update_path[128] = {0};
	uint8_t maj = 0, min = 0, pat = 0;

	if (scan_sd_for_update(update_path, sizeof(update_path), &maj, &min, &pat)) {
		(void)validate_and_install_update(update_path);
	}

	boot_application();

	/* If boot_application returns, enter safe hang */
	printk("[BOOTLOADER] System halted.\n");
	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}

