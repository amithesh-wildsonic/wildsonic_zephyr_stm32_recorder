/* SPDX-License-Identifier: Apache-2.0 */
/*
 * One-shot exFAT formatter for STM32L476RG with configurable card capacity check.
 * WARNING: Destructive operation! Destroys all data on the target SD card.
 * Do not leave this firmware installed: it formats again on every boot.
 */
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/printk.h>
#include <ff.h>

#ifdef CONFIG_FORMAT_EXPECTED_SECTOR_COUNT
#define EXPECTED_SECTOR_COUNT ((uint32_t)CONFIG_FORMAT_EXPECTED_SECTOR_COUNT)
#else
/* Default to 32 GB microSD card: 62,333,952 sectors x 512 bytes = ~31.91 GB */
#define EXPECTED_SECTOR_COUNT 62333952U
#endif

#ifdef CONFIG_FORMAT_EXPECTED_SECTOR_SIZE
#define EXPECTED_SECTOR_SIZE ((uint32_t)CONFIG_FORMAT_EXPECTED_SECTOR_SIZE)
#else
#define EXPECTED_SECTOR_SIZE  512U
#endif

static void halt_forever(void)
{
    while (true) {
        k_sleep(K_SECONDS(1));
    }
}

int main(void)
{
    int ret;
    uint32_t sector_count = 0;
    uint32_t sector_size = 0;

    printk("\n=========================================================================\n");
    printk("                    WILDSONIC SD exFAT FORMATTER                         \n");
    printk("=========================================================================\n");
    printk("WARNING: This firmware will completely ERASE the target SD card.\n\n");

    const uint64_t exp_total_bytes = (uint64_t)EXPECTED_SECTOR_COUNT * EXPECTED_SECTOR_SIZE;
    printk("Target Configuration:\n");
    printk("  Filesystem Type    : exFAT (FM_EXFAT | FM_SFD)\n");
    printk("  Expected Card Model: %u sectors x %u bytes\n", EXPECTED_SECTOR_COUNT, EXPECTED_SECTOR_SIZE);
    printk("  Expected Capacity  : %llu bytes (~%.2f GB / ~%.2f GiB)\n\n",
           exp_total_bytes,
           (double)exp_total_bytes / 1000000000.0,
           (double)exp_total_bytes / (1024.0 * 1024.0 * 1024.0));

    ret = disk_access_init("SD");
    if (ret != 0) {
        printk("ERROR: disk_access_init(SD) failed: %d; refusing to format.\n", ret);
        halt_forever();
    }

    ret = disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
    if (ret != 0) {
        printk("ERROR: could not read sector count: %d; refusing to format.\n", ret);
        halt_forever();
    }

    ret = disk_access_ioctl("SD", DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
    if (ret != 0) {
        printk("ERROR: could not read sector size: %d; refusing to format.\n", ret);
        halt_forever();
    }

    const uint64_t det_total_bytes = (uint64_t)sector_count * sector_size;
    printk("Detected Card:\n");
    printk("  Detected Sectors   : %u sectors x %u bytes\n", sector_count, sector_size);
    printk("  Detected Capacity  : %llu bytes (~%.2f GB / ~%.2f GiB)\n",
           det_total_bytes,
           (double)det_total_bytes / 1000000000.0,
           (double)det_total_bytes / (1024.0 * 1024.0 * 1024.0));

    if ((sector_count != EXPECTED_SECTOR_COUNT) ||
        (sector_size != EXPECTED_SECTOR_SIZE)) {
        printk("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printk("SAFETY STOP: Card capacity mismatch detected! Formatting ABORTED.\n");
        printk("  Expected: %u sectors x %u bytes (~%.2f GB)\n",
               EXPECTED_SECTOR_COUNT, EXPECTED_SECTOR_SIZE, (double)exp_total_bytes / 1e9);
        printk("  Detected: %u sectors x %u bytes (~%.2f GB)\n",
               sector_count, sector_size, (double)det_total_bytes / 1e9);
        printk("Protection rule: The formatter will NOT adopt any detected card capacity\n");
        printk("as trusted without verification of the physical label on the card.\n\n");
        printk("To format this card, confirm its physical label matches the detected size:\n");
        printk("  - 32 GB card (~31.9 GB, 62,333,952 sectors): set CONFIG_FORMAT_CARD_32GB=y\n");
        printk("  - 64 GB card (~63.9 GB, 124,735,488 sectors): set CONFIG_FORMAT_CARD_64GB=y\n");
        printk("  - Other capacity: set CONFIG_FORMAT_CARD_CUSTOM=y and configure sector count.\n");
        printk("Recompile the formatter with the correct profile after verifying your card.\n");
        printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
        halt_forever();
    }

    /*
     * Explicitly configure exFAT format parameters:
     * - FM_EXFAT: FatFs requires (fmt & FM_ANY) == FM_EXFAT to guarantee exFAT
     *   selection for volumes < 64M sectors (e.g. 32 GB cards).
     * - FM_SFD: Creates a super-floppy volume, replacing the entire disk without
     *   MBR partition table dependencies.
     * - n_fat = 1: Standard exFAT single FAT table.
     * - au_size = 0: FatFs automatically calculates optimal allocation unit
     *   (32 KiB for <32 GB volumes, 128 KiB for >=32 GB volumes).
     */
    MKFS_PARM fmt = {
        .fmt = FM_EXFAT | FM_SFD,
        .n_fat = 1,
        .align = 0,
        .n_root = 0,
        .au_size = 0,
    };

    printk("\nCapacity verified: %u sectors (~%.2f GB) matches expected profile.\n",
           sector_count, (double)det_total_bytes / 1e9);
    printk("Formatting SD card as exFAT now... LEAVE POWER CONNECTED.\n");
    ret = fs_mkfs(FS_FATFS, (uintptr_t)"SD:", &fmt, 0);
    if (ret != 0) {
        printk("FORMAT_FAILED: fs_mkfs returned error %d.\n", ret);
        halt_forever();
    }

    printk("\n=========================================================================\n");
    printk("FORMAT_SUCCESS: exFAT volume creation completed successfully!\n");
    printk("IMPORTANT: Do NOT reset the board with this formatter firmware installed.\n");
    printk("Flash the normal recorder firmware now.\n");
    printk("=========================================================================\n\n");
    halt_forever();
    return 0;
}
