/* SPDX-License-Identifier: Apache-2.0 */
#if defined(CONFIG_WILDSONIC_VERBOSE_LOGS)

#include "sd_benchmark.h"
#include "recorder.h"
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/drivers/disk/sdmmc_stm32.h>
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

static uint32_t calc_crc32(const uint8_t *data, size_t len)
{
	uint32_t crc = 0xFFFFFFFFU;
	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc >> 1) ^ (0xEDB88320U & -(crc & 1));
		}
	}
	return ~crc;
}

int sd_benchmark_run_stage1_dma_tests(void)
{
	if (recorder_is_active()) {
		printk("\n[REJECTED] Cannot run SD tests while recording is active.\n\n");
		return -EBUSY;
	}

	printk("\n=========================================================================================================\n");
	printk("                        STAGE 1: STANDALONE SDMMC DMA VERIFICATION SUITE                                  \n");
	printk("=========================================================================================================\n");
	printk("Hardware : STM32L476RG (Cortex-M4F @ 72 MHz)\n");
	printk("SDMMC1   : 4-bit bus @ 8 MHz (CLKDIV=4)\n");
	printk("DMA2     : Channel 4, Request Slot 7 (32-bit transfer width, shared txrx)\n");
	printk("---------------------------------------------------------------------------------------------------------\n");

	struct sdmmc_dma_transfer_diag diag;
	memset(&diag, 0, sizeof(diag));
	stm32_sdmmc_get_dma_diag(&diag);

	printk("[DMA CONFIG] Initialized: %s | Controller: DMA2 | Channel: %u | Slot: %u\n",
	       diag.dma_init_success ? "YES (DMA enabled)" : "NO (polling/IT fallback)",
	       diag.dma_channel, diag.dma_slot);

	if (!diag.dma_init_success) {
		printk("[STAGE 1 ERROR] SDMMC DMA is not active in driver! Aborting tests.\n");
		return -ENODEV;
	}

	uint8_t *wbuf = recorder_get_bounce_buf_a();
	uint8_t *rbuf = recorder_get_bounce_buf_b();
	if (!wbuf || !rbuf) {
		printk("[STAGE 1 ERROR] Failed to obtain test bounce buffers.\n");
		return -ENOMEM;
	}

	int tests_passed = 0;
	int total_tests = 4;

	/* -------------------------------------------------------------------------
	 * TEST 1: 512-byte transfer (1 sector = 128 DMA items)
	 * ------------------------------------------------------------------------- */
	printk("\n--> [TEST 1/4] 512-Byte Single Sector DMA Write & Readback CRC Test...\n");
	const char *test1_path = "/SD:/dma_test_512b.tmp";
	for (uint32_t i = 0; i < 512; i++) {
		wbuf[i] = (uint8_t)(0x3C ^ (i * 7 + 13));
	}
	uint32_t crc_w512 = calc_crc32(wbuf, 512);

	struct fs_file_t file;
	fs_file_t_init(&file);
	int rc = fs_open(&file, test1_path, FS_O_CREATE | FS_O_WRITE);
	if (rc != 0) {
		printk("    [FAIL] fs_open write failed: %d\n", rc);
		return rc;
	}
	ssize_t written = fs_write(&file, wbuf, 512);
	fs_close(&file);
	if (written != 512) {
		printk("    [FAIL] fs_write expected 512 B, got %d\n", (int)written);
		fs_unlink(test1_path);
		return -EIO;
	}

	memset(rbuf, 0, 512);
	rc = fs_open(&file, test1_path, FS_O_READ);
	if (rc != 0) {
		printk("    [FAIL] fs_open read failed: %d\n", rc);
		fs_unlink(test1_path);
		return rc;
	}
	ssize_t read_bytes = fs_read(&file, rbuf, 512);
	fs_close(&file);
	fs_unlink(test1_path);
	if (read_bytes != 512) {
		printk("    [FAIL] fs_read expected 512 B, got %d\n", (int)read_bytes);
		return -EIO;
	}

	uint32_t crc_r512 = calc_crc32(rbuf, 512);
	stm32_sdmmc_get_dma_diag(&diag);

	if (crc_w512 == crc_r512 && memcmp(wbuf, rbuf, 512) == 0 && diag.last_initial_dma_items == 128) {
		printk("    [PASS] 512 B: CRC=0x%08X (match), DMA items=%u (expected 128), DMA cplt=%u, DATAEND=%u\n",
		       crc_r512, diag.last_initial_dma_items, diag.dma_cplt_count, diag.dataend_cplt_count);
		tests_passed++;
	} else {
		printk("    [FAIL] 512 B mismatch: CRC write=0x%08X read=0x%08X, DMA items=%u (expected 128)\n",
		       crc_w512, crc_r512, diag.last_initial_dma_items);
		return -EIO;
	}

	/* -------------------------------------------------------------------------
	 * TEST 2: 4 KB transfer (8 sectors = 1024 DMA items)
	 * ------------------------------------------------------------------------- */
	printk("\n--> [TEST 2/4] 4 KB (4096-Byte) DMA Write & Readback CRC Test...\n");
	const char *test2_path = "/SD:/dma_test_4k.tmp";
	for (uint32_t i = 0; i < 4096; i++) {
		wbuf[i] = (uint8_t)(0x5A ^ (i * 11 + 29));
	}
	uint32_t crc_w4k = calc_crc32(wbuf, 4096);

	rc = fs_open(&file, test2_path, FS_O_CREATE | FS_O_WRITE);
	if (rc != 0) {
		printk("    [FAIL] fs_open write failed: %d\n", rc);
		return rc;
	}
	written = fs_write(&file, wbuf, 4096);
	fs_close(&file);
	if (written != 4096) {
		printk("    [FAIL] fs_write expected 4096 B, got %d\n", (int)written);
		fs_unlink(test2_path);
		return -EIO;
	}

	memset(rbuf, 0, 4096);
	rc = fs_open(&file, test2_path, FS_O_READ);
	if (rc != 0) {
		printk("    [FAIL] fs_open read failed: %d\n", rc);
		fs_unlink(test2_path);
		return rc;
	}
	read_bytes = fs_read(&file, rbuf, 4096);
	fs_close(&file);
	fs_unlink(test2_path);
	if (read_bytes != 4096) {
		printk("    [FAIL] fs_read expected 4096 B, got %d\n", (int)read_bytes);
		return -EIO;
	}

	uint32_t crc_r4k = calc_crc32(rbuf, 4096);
	stm32_sdmmc_get_dma_diag(&diag);

	if (crc_w4k == crc_r4k && memcmp(wbuf, rbuf, 4096) == 0 && diag.last_initial_dma_items == 1024) {
		printk("    [PASS] 4 KB: CRC=0x%08X (match), DMA items=%u (expected 1024), DMA cplt=%u, DATAEND=%u\n",
		       crc_r4k, diag.last_initial_dma_items, diag.dma_cplt_count, diag.dataend_cplt_count);
		tests_passed++;
	} else {
		printk("    [FAIL] 4 KB mismatch: CRC write=0x%08X read=0x%08X, DMA items=%u (expected 1024)\n",
		       crc_w4k, crc_r4k, diag.last_initial_dma_items);
		return -EIO;
	}

	/* -------------------------------------------------------------------------
	 * TEST 3: 16 KB transfer (32 sectors = 4096 DMA items) - Production Chunk Size
	 * ------------------------------------------------------------------------- */
	printk("\n--> [TEST 3/4] 16 KB Production Chunk DMA Write & Readback CRC Test...\n");
	const char *test3_path = "/SD:/dma_test_16k.tmp";
	for (uint32_t i = 0; i < 16384; i++) {
		wbuf[i] = (uint8_t)(0xC3 ^ (i * 17 + 41));
	}
	uint32_t crc_w16k = calc_crc32(wbuf, 16384);

	rc = fs_open(&file, test3_path, FS_O_CREATE | FS_O_WRITE);
	if (rc != 0) {
		printk("    [FAIL] fs_open write failed: %d\n", rc);
		return rc;
	}
	written = fs_write(&file, wbuf, 16384);
	fs_close(&file);
	if (written != 16384) {
		printk("    [FAIL] fs_write expected 16384 B, got %d\n", (int)written);
		fs_unlink(test3_path);
		return -EIO;
	}

	memset(rbuf, 0, 16384);
	rc = fs_open(&file, test3_path, FS_O_READ);
	if (rc != 0) {
		printk("    [FAIL] fs_open read failed: %d\n", rc);
		fs_unlink(test3_path);
		return rc;
	}
	read_bytes = fs_read(&file, rbuf, 16384);
	fs_close(&file);
	fs_unlink(test3_path);
	if (read_bytes != 16384) {
		printk("    [FAIL] fs_read expected 16384 B, got %d\n", (int)read_bytes);
		return -EIO;
	}

	uint32_t crc_r16k = calc_crc32(rbuf, 16384);
	stm32_sdmmc_get_dma_diag(&diag);

	if (crc_w16k == crc_r16k && memcmp(wbuf, rbuf, 16384) == 0 && diag.last_initial_dma_items == 4096) {
		printk("    [PASS] 16 KB: CRC=0x%08X (match), DMA items=%u (expected 4096), DMA cplt=%u, DATAEND=%u\n",
		       crc_r16k, diag.last_initial_dma_items, diag.dma_cplt_count, diag.dataend_cplt_count);
		tests_passed++;
	} else {
		printk("    [FAIL] 16 KB mismatch: CRC write=0x%08X read=0x%08X, DMA items=%u (expected 4096)\n",
		       crc_w16k, crc_r16k, diag.last_initial_dma_items);
		return -EIO;
	}

	/* -------------------------------------------------------------------------
	 * TEST 4: Repeated Sequential DMA Writes (256 KB = 16 x 16 KB) with Full Readback CRC
	 * ------------------------------------------------------------------------- */
	printk("\n--> [TEST 4/4] Repeated Sequential 256 KB (16 x 16 KB) Write & CRC Readback...\n");
	const char *test4_path = "/SD:/dma_seq_256k.tmp";
	const uint32_t num_chunks = 16;
	const uint32_t chunk_size = 16384;
	const uint32_t total_seq_bytes = num_chunks * chunk_size;
	uint32_t expected_crcs[16];

	rc = fs_open(&file, test4_path, FS_O_CREATE | FS_O_WRITE);
	if (rc != 0) {
		printk("    [FAIL] fs_open write failed: %d\n", rc);
		return rc;
	}

	uint64_t t_w_start = k_cycle_get_64();
	for (uint32_t c = 0; c < num_chunks; c++) {
		for (uint32_t i = 0; i < chunk_size; i++) {
			wbuf[i] = (uint8_t)((c * 37 + 19) ^ (i & 0xFF));
		}
		expected_crcs[c] = calc_crc32(wbuf, chunk_size);

		written = fs_write(&file, wbuf, chunk_size);
		if (written != (ssize_t)chunk_size) {
			printk("    [FAIL] Chunk %u fs_write returned %d\n", c, (int)written);
			fs_close(&file);
			fs_unlink(test4_path);
			return -EIO;
		}
	}
	uint64_t t_w_end = k_cycle_get_64();
	fs_close(&file);

	uint64_t w_elapsed_us = k_cyc_to_us_floor64(t_w_end - t_w_start);
	uint32_t w_thpt_kb_s = w_elapsed_us ? (uint32_t)(((uint64_t)total_seq_bytes * 1000000ULL) / (w_elapsed_us * 1024ULL)) : 0;

	/* Readback and verify all 16 chunks */
	rc = fs_open(&file, test4_path, FS_O_READ);
	if (rc != 0) {
		printk("    [FAIL] fs_open read failed: %d\n", rc);
		fs_unlink(test4_path);
		return rc;
	}

	int crc_mismatches = 0;
	uint64_t t_r_start = k_cycle_get_64();
	for (uint32_t c = 0; c < num_chunks; c++) {
		read_bytes = fs_read(&file, rbuf, chunk_size);
		if (read_bytes != (ssize_t)chunk_size) {
			printk("    [FAIL] Chunk %u fs_read returned %d\n", c, (int)read_bytes);
			crc_mismatches++;
			break;
		}

		uint32_t r_crc = calc_crc32(rbuf, chunk_size);
		if (r_crc != expected_crcs[c]) {
			printk("    [FAIL] Chunk %u CRC mismatch: expected 0x%08X, got 0x%08X\n",
			       c, expected_crcs[c], r_crc);
			crc_mismatches++;
		}
	}
	uint64_t t_r_end = k_cycle_get_64();
	fs_close(&file);
	fs_unlink(test4_path);

	uint64_t r_elapsed_us = k_cyc_to_us_floor64(t_r_end - t_r_start);
	uint32_t r_thpt_kb_s = r_elapsed_us ? (uint32_t)(((uint64_t)total_seq_bytes * 1000000ULL) / (r_elapsed_us * 1024ULL)) : 0;

	if (crc_mismatches == 0) {
		printk("    [PASS] 256 KB Sequential: All 16 chunks CRC32 verified!\n");
		printk("           Write Throughput: %u KB/s (%u ms) | Read Throughput: %u KB/s (%u ms)\n",
		       w_thpt_kb_s, (unsigned)(w_elapsed_us / 1000ULL),
		       r_thpt_kb_s, (unsigned)(r_elapsed_us / 1000ULL));
		tests_passed++;
	} else {
		printk("    [FAIL] 256 KB Sequential failed: %d chunk CRC mismatches!\n", crc_mismatches);
		return -EIO;
	}

	printk("---------------------------------------------------------------------------------------------------------\n");
	printk("STAGE 1 SUMMARY: %d of %d tests passed.\n", tests_passed, total_tests);
	if (tests_passed == total_tests) {
		printk("VERDICT: >>> ALL STANDALONE SDMMC DMA VERIFICATION TESTS PASSED SUCCESSFULLY! <<<\n");
	} else {
		printk("VERDICT: >>> STAGE 1 VERIFICATION FAILED <<<\n");
	}
	printk("=========================================================================================================\n\n");
	return (tests_passed == total_tests) ? 0 : -EIO;
}

#endif /* CONFIG_WILDSONIC_VERBOSE_LOGS */

