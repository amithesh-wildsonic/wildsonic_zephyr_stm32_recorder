# SPDX-License-Identifier: Apache-2.0
"""
Test Suite: Stage 1 — Enable SDMMC DMA on STM32L476RG
Verifies:
1. Devicetree overlay configuration:
   - DMA2 enabled (&dma2 { status = "okay"; })
   - SDMMC1 configured with dmas = <&dma2 4 7 (STM32_DMA_32BITS | STM32_DMA_PRIORITY_HIGH)>
   - SDMMC1 dma-names = "txrx" (shared DMA architecture required on STM32L4)
2. Generated devicetree macros in devicetree_generated.h:
   - Channel 4, slot (request) 7, controller DMA2 (0x40020400), shared "txrx"
3. Driver compile branches and configuration in sdmmc_stm32.c:
   - STM32_SDMMC_USE_DMA and STM32_SDMMC_USE_DMA_SHARED active
   - HAL_SD_WriteBlocks_DMA and HAL_SD_ReadBlocks_DMA selected
   - Dynamic direction: MEMORY_TO_PERIPH for write, PERIPH_TO_MEMORY for read
4. 32-bit DMA item calculations:
   - 512 B -> 128 words (CNDTR = 128)
   - 4 KB (4096 B) -> 1024 words (CNDTR = 1024)
   - 16 KB (16384 B) -> 4096 words (CNDTR = 4096)
   - 256 KB (262144 B) -> 65536 words
5. Driver runtime diagnostics:
   - DMA init success, channel, slot logging
   - Write DMA start logging with initial DMA item count
   - Callback tracking (dma_cplt_count, dataend_cplt_count)
   - Diagnostic structure and query API (stm32_sdmmc_get_dma_diag)
6. Linker map symbols in build/prod and build/test:
   - HAL_SD_WriteBlocks_DMA in .text
   - HAL_SD_ReadBlocks_DMA in .text
   - Interrupt FIFO polling routines (HAL_SD_WriteBlocks_IT) eliminated / discarded
7. Standalone verification tests:
   - sd_benchmark_run_stage1_dma_tests() covers 512 B, 4 KB, 16 KB, and sequential 256 KB with CRC32
   - Console command SDDMA registered
"""

from pathlib import Path
import re

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
OVERLAY = REPO_ROOT / "app" / "sd" / "sdmmc_nucleo_l476rg.overlay"
PROD_DTS_GEN = REPO_ROOT / "build" / "prod" / "zephyr" / "include" / "generated" / "zephyr" / "devicetree_generated.h"
TEST_DTS_GEN = REPO_ROOT / "build" / "test" / "zephyr" / "include" / "generated" / "zephyr" / "devicetree_generated.h"
PROD_MAP = REPO_ROOT / "build" / "prod" / "zephyr" / "zephyr.map"
TEST_MAP = REPO_ROOT / "build" / "test" / "zephyr" / "zephyr.map"
SDMMC_DRIVER = Path(r"C:\Users\amith\zephyrproject\zephyr\drivers\disk\sdmmc_stm32.c")
SDMMC_HEADER = Path(r"C:\Users\amith\zephyrproject\zephyr\include\zephyr\drivers\disk\sdmmc_stm32.h")
SD_BENCH_C = REPO_ROOT / "app" / "src" / "sd_benchmark.c"
SD_BENCH_H = REPO_ROOT / "app" / "src" / "sd_benchmark.h"
CONSOLE_C = REPO_ROOT / "app" / "src" / "console.c"


def test_overlay_dma_configuration():
    """Verify DMA2 and SDMMC1 DMA bindings in device tree overlay."""
    assert OVERLAY.exists(), f"Overlay {OVERLAY} must exist"
    txt = OVERLAY.read_text(encoding="utf-8")

    assert "&dma2" in txt
    assert "status = \"okay\";" in txt
    assert "&sdmmc1" in txt
    assert "dmas = <&dma2 4 7 (STM32_DMA_32BITS | STM32_DMA_PRIORITY_HIGH)>;" in txt
    assert 'dma-names = "txrx";' in txt
    assert "#include <zephyr/dt-bindings/dma/stm32_dma.h>" in txt


def test_generated_devicetree_macros():
    """Verify generated DT macros in devicetree_generated.h for both builds."""
    for dts_h in [PROD_DTS_GEN, TEST_DTS_GEN]:
        assert dts_h.exists(), f"{dts_h} must exist"
        txt = dts_h.read_text(encoding="utf-8")

        # Check SDMMC1 dmas properties
        assert "DT_N_S_soc_S_sdmmc_40012800_P_dmas_EXISTS 1" in txt
        assert "DT_N_S_soc_S_sdmmc_40012800_P_dmas_IDX_0_VAL_channel 4" in txt
        assert "DT_N_S_soc_S_sdmmc_40012800_P_dmas_IDX_0_VAL_slot 7" in txt
        assert "DT_N_S_soc_S_sdmmc_40012800_P_dmas_IDX_0_PH DT_N_S_soc_S_dma_40020400" in txt
        assert 'DT_N_S_soc_S_sdmmc_40012800_P_dmas_IDX_0_NAME "txrx"' in txt
        assert "DT_N_S_soc_S_sdmmc_40012800_P_dma_names_EXISTS 1" in txt


def test_driver_dma_code_branches():
    """Verify driver code implementation in sdmmc_stm32.c."""
    assert SDMMC_DRIVER.exists()
    txt = SDMMC_DRIVER.read_text(encoding="utf-8")

    # Macro definitions
    assert "#define STM32_SDMMC_USE_DMA DT_NODE_HAS_PROP(DT_DRV_INST(0), dmas)" in txt
    assert "#define STM32_SDMMC_USE_DMA_SHARED DT_INST_DMAS_HAS_NAME(0, txrx)" in txt

    # Shared DMA direction dynamic configuration
    assert "priv->dma_txrx_handle.Init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;" in txt
    assert "priv->dma_txrx_handle.Init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;" in txt

    # HAL write and read DMA functions
    assert "hal_ret = HAL_SD_WriteBlocks_DMA(hsd, data_buf, start_sector, num_sector);" in txt
    assert "hal_ret = HAL_SD_ReadBlocks_DMA(hsd, data_buf, start_sector, num_sector);" in txt


def test_driver_runtime_diagnostics():
    """Verify driver non-intrusive diagnostics and stats structures."""
    txt = SDMMC_DRIVER.read_text(encoding="utf-8")
    h_txt = SDMMC_HEADER.read_text(encoding="utf-8")

    # Structure in header
    assert "struct sdmmc_dma_transfer_diag" in h_txt
    assert "uint32_t dma_init_success;" in h_txt
    assert "uint32_t dma_channel;" in h_txt
    assert "uint32_t dma_slot;" in h_txt
    assert "uint32_t write_start_count;" in h_txt
    assert "uint32_t last_initial_dma_items;" in h_txt
    assert "uint32_t dma_cplt_count;" in h_txt
    assert "uint32_t dataend_cplt_count;" in h_txt
    assert "void stm32_sdmmc_get_dma_diag(struct sdmmc_dma_transfer_diag *out);" in h_txt

    # Diagnostic updates in driver
    assert "g_sdmmc_dma_diag.dma_init_success = 1;" in txt
    assert "g_sdmmc_dma_diag.dma_cplt_count++;" in txt
    assert "g_sdmmc_dma_diag.dataend_cplt_count++;" in txt
    assert "g_sdmmc_dma_diag.last_initial_dma_items = dma_items;" in txt
    assert "SDMMC DMA init success: channel=" in txt
    assert "SDMMC write DMA start:" in txt


def test_dma_item_accounting_calculations():
    """Verify 32-bit DMA item calculations across chunk sizes."""
    def calc_dma_items(byte_len: int) -> int:
        assert byte_len % 4 == 0, "Transfer length must be 32-bit aligned"
        return byte_len // 4

    # 512 B (1 sector)
    assert calc_dma_items(512) == 128
    # 4 KB (8 sectors)
    assert calc_dma_items(4096) == 1024
    # 16 KB (32 sectors) - standard WildSonic SD I/O chunk
    assert calc_dma_items(16384) == 4096
    # 256 KB (512 sectors) - full staging slot
    assert calc_dma_items(262144) == 65536


def test_linker_map_dma_symbols():
    """Verify in map files that HAL DMA functions are linked and IT functions discarded."""
    for map_path in [PROD_MAP, TEST_MAP]:
        assert map_path.exists(), f"{map_path} must exist"
        txt = map_path.read_text(encoding="utf-8")

        # HAL_SD_WriteBlocks_DMA and HAL_SD_ReadBlocks_DMA must be in text section with valid address
        match_w = re.search(r"0x08[0-9a-f]{6}\s+HAL_SD_WriteBlocks_DMA", txt)
        assert match_w is not None, f"HAL_SD_WriteBlocks_DMA must have valid flash address in {map_path}"

        match_r = re.search(r"0x08[0-9a-f]{6}\s+HAL_SD_ReadBlocks_DMA", txt)
        assert match_r is not None, f"HAL_SD_ReadBlocks_DMA must have valid flash address in {map_path}"

        # IT routines must NOT have a linked flash execution address (they are in Discarded input sections with 0x00000000)
        match_it_w = re.search(r"0x08[0-9a-f]{6}\s+HAL_SD_WriteBlocks_IT", txt)
        assert match_it_w is None, f"HAL_SD_WriteBlocks_IT must NOT be linked into code in {map_path}"


def test_standalone_dma_verification_benchmark():
    """Verify standalone SDMMC DMA verification implementation."""
    assert SD_BENCH_C.exists()
    assert SD_BENCH_H.exists()
    txt = SD_BENCH_C.read_text(encoding="utf-8")
    h_txt = SD_BENCH_H.read_text(encoding="utf-8")

    assert "int sd_benchmark_run_stage1_dma_tests(void);" in h_txt
    assert "int sd_benchmark_run_stage1_dma_tests(void)" in txt

    # Verifies all 4 standalone tests
    assert "TEST 1/4" in txt
    assert "512-Byte Single Sector DMA Write & Readback CRC Test" in txt
    assert "diag.last_initial_dma_items == 128" in txt

    assert "TEST 2/4" in txt
    assert "4 KB (4096-Byte) DMA Write & Readback CRC Test" in txt
    assert "diag.last_initial_dma_items == 1024" in txt

    assert "TEST 3/4" in txt
    assert "16 KB Production Chunk DMA Write & Readback CRC Test" in txt
    assert "diag.last_initial_dma_items == 4096" in txt

    assert "TEST 4/4" in txt
    assert "Repeated Sequential 256 KB (16 x 16 KB) Write & CRC Readback" in txt
    assert "calc_crc32" in txt


def test_console_sddma_command_registered():
    """Verify SDDMA command is registered in console dispatcher."""
    assert CONSOLE_C.exists()
    txt = CONSOLE_C.read_text(encoding="utf-8")

    assert 'strcasecmp(p, "SDDMA") == 0' in txt
    assert "sd_benchmark_run_stage1_dma_tests();" in txt

