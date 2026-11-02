/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RELEASE_HEADER_MAGIC      0x57533332U /* "WS32" */
#define HEADER_FORMAT_VERSION     1U
#define STM32L476_TARGET_MCU_ID   0x00004760U
#define APP_BANK2_START_ADDR      0x08080000U
#define APP_BANK2_MAX_SIZE        (480U * 1024U) /* 480 KB */
#define STM32L476_FLASH_PAGE_SIZE 2048U
#define SRAM_START_ADDR           0x20000000U
#define SRAM_END_ADDR             0x20020000U /* 128 KB SRAM1+SRAM2 */

struct __attribute__((packed)) wildsonic_release_header {
	uint32_t magic;           /* 0x57533332 */
	uint16_t format_version;  /* 1 */
	uint16_t flags;           /* 0 */
	uint32_t target_mcu_id;   /* 0x00004760 */
	uint8_t  version_major;
	uint8_t  version_minor;
	uint8_t  version_patch;
	uint8_t  reserved1;
	uint32_t payload_offset;  /* 64 */
	uint32_t payload_size;    /* Size in bytes */
	uint32_t target_load_addr;/* 0x08080000 */
	uint32_t entry_point;     /* Reset vector */
	uint32_t payload_crc32;   /* CRC32 of payload */
	uint32_t config_version;  /* 1 */
	uint8_t  reserved2[20];
	uint32_t header_crc32;    /* CRC32 of first 60 bytes */
};

#endif /* BOOTLOADER_H_ */
