/* SPDX-License-Identifier: Apache-2.0 */
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)

#include "sd_benchmark.h"
#include "recorder.h"
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <ff.h>
#include <stdio.h>
#include <string.h>

#define BENCH_BUF_SIZE SD_IO_CHUNK_BYTES
#define s_bench_buf (recorder_get_bounce_buf_a())

static void init_bench_buffer(void)
{
	uint8_t *buf = s_bench_buf;
	for (uint32_t i = 0; i < BENCH_BUF_SIZE; i++) {
		buf[i] = (uint8_t)(0xA5 ^ (i & 0xFF));
	}
}

int sd_benchmark_run_test(enum sd_bench_type type,
                          uint32_t block_bytes,
                          uint32_t total_bytes,
                          struct sd_bench_result *out)
{
	if (!out || block_bytes == 0 || total_bytes == 0 || block_bytes > total_bytes) {
		return -EINVAL;
	}

	init_bench_buffer();
	memset(out, 0, sizeof(*out));
	out->type = type;
	out->block_bytes = block_bytes;
	out->total_bytes = total_bytes;
	out->min_write_us = UINT32_MAX;

	const char *test_path = "/SD:/sdbench.tmp";
	struct fs_file_t f;
	fs_file_t_init(&f);

	uint32_t num_blocks = total_bytes / block_bytes;
	out->num_operations = num_blocks;

	int rc = 0;
	if (type == SD_BENCH_RAW_BLOCK) {
		/*
		 * Safely allocate a temporary file on the filesystem first,
		 * pre-allocate contiguous clusters, and determine starting disk sector.
		 * This guarantees raw block writes target valid allocated clusters
		 * without risking filesystem table or boot sector corruption.
		 */
		rc = fs_open(&f, test_path, FS_O_CREATE | FS_O_WRITE);
		if (rc != 0) {
			printk("[SDBENCH] Failed to create raw placeholder file: %d\n", rc);
			return rc;
		}

		FIL *fil = (FIL *)f.filep;
		FRESULT f_res = f_expand(fil, (FSIZE_t)total_bytes, 1);
		if (f_res != FR_OK) {
			f_res = f_expand(fil, (FSIZE_t)total_bytes, 0);
		}
		if (f_res != FR_OK) {
			fs_close(&f);
			fs_unlink(test_path);
			return -EIO;
		}

		DWORD start_clust = fil->obj.sclust;
		FATFS *fs = fil->obj.fs;
		uint32_t start_sector = fs->database + ((start_clust - 2U) * fs->csize);
		fs_close(&f);

		uint64_t t_start_cyc = k_cycle_get_64();
		uint64_t total_us = 0;
		uint32_t cur_sector = start_sector;

		for (uint32_t b = 0; b < num_blocks; b++) {
			uint32_t remaining = block_bytes;
			uint64_t t0 = k_cycle_get_64();

			while (remaining > 0) {
				uint32_t chunk = (remaining > BENCH_BUF_SIZE) ? BENCH_BUF_SIZE : remaining;
				uint32_t sector_count = chunk / 512U;
				int d_rc = disk_access_write("SD", s_bench_buf, cur_sector, sector_count);
				if (d_rc != 0) {
					printk("[SDBENCH] disk_access_write failed at sector %u (rc=%d)\n", cur_sector, d_rc);
					fs_unlink(test_path);
					return d_rc;
				}
				cur_sector += sector_count;
				remaining -= chunk;
			}

			uint64_t t1 = k_cycle_get_64();
			uint32_t op_us = (uint32_t)k_cyc_to_us_floor64(t1 - t0);
			total_us += op_us;
			if (op_us > out->max_write_us) out->max_write_us = op_us;
			if (op_us < out->min_write_us) out->min_write_us = op_us;
		}

		uint64_t t_end_cyc = k_cycle_get_64();
		uint64_t total_elapsed_us = k_cyc_to_us_floor64(t_end_cyc - t_start_cyc);
		out->elapsed_ms = (uint32_t)(total_elapsed_us / 1000ULL);
		out->avg_write_us = num_blocks ? (uint32_t)(total_us / num_blocks) : 0;
		if (total_elapsed_us > 0) {
			out->throughput_kb_s = (uint32_t)(((uint64_t)total_bytes * 1000000ULL) / (total_elapsed_us * 1024ULL));
		}
		fs_unlink(test_path);
	} else {
		/* Filesystem benchmark: dynamic allocation vs pre-allocated */
		rc = fs_open(&f, test_path, FS_O_CREATE | FS_O_WRITE);
		if (rc != 0) {
			printk("[SDBENCH] fs_open failed: %d\n", rc);
			return rc;
		}

		if (type == SD_BENCH_FS_PREALLOC) {
			FIL *fil = (FIL *)f.filep;
			FRESULT f_res = f_expand(fil, (FSIZE_t)total_bytes, 1);
			if (f_res != FR_OK) {
				f_res = f_expand(fil, (FSIZE_t)total_bytes, 0);
			}
			if (f_res != FR_OK) {
				fs_close(&f);
				fs_unlink(test_path);
				return -EIO;
			}
		}

		uint64_t t_start_cyc = k_cycle_get_64();
		uint64_t total_us = 0;

		for (uint32_t b = 0; b < num_blocks; b++) {
			uint32_t remaining = block_bytes;
			uint64_t t0 = k_cycle_get_64();

			while (remaining > 0) {
				uint32_t chunk = (remaining > BENCH_BUF_SIZE) ? BENCH_BUF_SIZE : remaining;
				ssize_t w = fs_write(&f, s_bench_buf, chunk);
				if (w != (ssize_t)chunk) {
					printk("[SDBENCH] fs_write failed: %d\n", (int)w);
					fs_close(&f);
					fs_unlink(test_path);
					return (w < 0) ? (int)w : -EIO;
				}
				remaining -= chunk;
			}

			uint64_t t1 = k_cycle_get_64();
			uint32_t op_us = (uint32_t)k_cyc_to_us_floor64(t1 - t0);
			total_us += op_us;
			if (op_us > out->max_write_us) out->max_write_us = op_us;
			if (op_us < out->min_write_us) out->min_write_us = op_us;
		}

		uint64_t t_end_cyc = k_cycle_get_64();
		uint64_t total_elapsed_us = k_cyc_to_us_floor64(t_end_cyc - t_start_cyc);
		out->elapsed_ms = (uint32_t)(total_elapsed_us / 1000ULL);
		out->avg_write_us = num_blocks ? (uint32_t)(total_us / num_blocks) : 0;
		if (total_elapsed_us > 0) {
			out->throughput_kb_s = (uint32_t)(((uint64_t)total_bytes * 1000000ULL) / (total_elapsed_us * 1024ULL));
		}
		fs_close(&f);
		fs_unlink(test_path);
	}

	out->passed_audio_rate = (out->throughput_kb_s >= 384U);
	out->passed_margin = (out->throughput_kb_s >= 768U);
	return 0;
}

int sd_benchmark_run_matrix(void)
{
	if (recorder_is_active()) {
		printk("\n[REJECTED] Cannot run SD benchmark while recording is active.\n\n");
		return -EBUSY;
	}

	printk("\n=========================================================================================================\n");
	printk("                        WILDSONIC SD STORAGE PERFORMANCE & THROUGHPUT BENCHMARK                           \n");
	printk("=========================================================================================================\n");
	printk("Target Audio Requirement: 384 KB/s (192 kHz, 16-bit mono PCM)\n");
	printk("Operating Margin Target : 768 KB/s (2.0x margin)\n");
#if defined(CONFIG_SOC_SERIES_STM32L4X)
	uint32_t clkcr = SDMMC1->CLKCR;
	uint32_t clk_div = clkcr & SDMMC_CLKCR_CLKDIV_Msk;
	uint32_t bus_w = ((clkcr & SDMMC_CLKCR_WIDBUS_Msk) == SDMMC_CLKCR_WIDBUS_0) ? 4 :
	                 ((clkcr & SDMMC_CLKCR_WIDBUS_Msk) == SDMMC_CLKCR_WIDBUS_1) ? 8 : 1;
	uint32_t clk_hz = 48000000U / (clk_div + 2U);
	printk("SDMMC Interface Clock   : %u MHz %u-bit bus (%.1f MB/s raw bus capacity, CLKDIV=%u)\n",
	       clk_hz / 1000000U, bus_w, (double)(clk_hz * bus_w) / 8000000.0, clk_div);
#else
	printk("SDMMC Interface Clock   : 8 MHz 4-bit bus (4.0 MB/s raw bus capacity)\n");
#endif
	printk("Filesystem Type         : exFAT (FatFs ELM)\n");
	printk("Configured I/O Chunk    : %u KB\n", (unsigned)(SD_IO_CHUNK_BYTES / 1024U));
	printk("---------------------------------------------------------------------------------------------------------\n");
	printk("%-18s | %-10s | %-10s | %-6s | %-12s | %-12s | %-12s | %-8s\n",
	       "Mode", "Block Size", "Total Data", "Ops", "Avg Latency", "Max Latency", "Throughput", "Margin");
	printk("-------------------+------------+------------+--------+--------------+--------------+--------------+----------\n");

	const uint32_t test_sizes[] = {
		8 * 1024,
		16 * 1024,
		32 * 1024,
		64 * 1024,
		128 * 1024,
		256 * 1024,
		512 * 1024
	};
	const uint32_t total_data = 1024 * 1024; /* 1024 KB sustained per test */

	const char *mode_names[] = {
		"FS (Dynamic)",
		"FS (Pre-alloc)",
		"Raw Block (DMA)"
	};

	for (int m = 0; m < 3; m++) {
		for (size_t s = 0; s < ARRAY_SIZE(test_sizes); s++) {
			uint32_t blk = test_sizes[s];
			struct sd_bench_result res;
			int rc = sd_benchmark_run_test((enum sd_bench_type)m, blk, total_data, &res);
			if (rc == 0) {
				const char *margin_str = res.passed_margin ? "PASS (>=2x)" :
				                         res.passed_audio_rate ? "PASS (1x)" : "FAIL";
				printk("%-18s | %4u KB    | %4u KB    | %4u   | %6u us     | %6u us     | %5u KB/s    | %s\n",
				       mode_names[m],
				       blk >> 10,
				       total_data >> 10,
				       res.num_operations,
				       res.avg_write_us,
				       res.max_write_us,
				       res.throughput_kb_s,
				       margin_str);
			} else {
				printk("%-18s | %4u KB    | FAILED (code %d)\n",
				       mode_names[m], blk >> 10, rc);
			}
		}
		printk("-------------------+------------+------------+--------+--------------+--------------+--------------+----------\n");
	}

	printk("=========================================================================================================\n\n");
	return 0;
}

#endif /* CONFIG_WILDSONIC_VERBOSE_LOGS */

