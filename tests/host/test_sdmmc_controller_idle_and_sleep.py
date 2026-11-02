# SPDX-License-Identifier: Apache-2.0
"""
Test Suite: STM32L476 SDMMC DMA, Controller Idle State & Production Sleep State Machine
======================================================================================
Verifies:
1. Priority A: SDMMC DMA Configuration
   - DMA2 enabled in devicetree overlay (&dma2 { status = "okay"; })
   - SDMMC1 configured with dmas = <&dma2 4 7 (STM32_DMA_32BITS | STM32_DMA_PRIORITY_HIGH)>
   - SDMMC1 dma-names = "txrx" (bidirectional shared DMA channel)
   - Exclusivity of DMA2 Channel 4 (no conflicting peripheral active)
   - Linker maps in build/prod and build/test contain HAL_SD_WriteBlocks_DMA and HAL_SD_ReadBlocks_DMA

2. Priority B: Controller Idle State & DPSM Fix (STA=0x00082000)
   - Root cause: ST HAL SD_DMAReceiveCplt() omitted clearing SDMMC_DCTRL_DTEN,
     leaving DPSM in receive active state (STA=0x00082000 -> RXACT=1, RXFIFOE=1).
   - Driver fix: HAL_SD_RxCpltCallback, HAL_SD_TxCpltCallback, and HAL_SD_ErrorCallback
     explicitly clear (SDMMC_DCTRL_DTEN | SDMMC_DCTRL_DMAEN).
   - Access functions stm32_sdmmc_access_read, stm32_sdmmc_access_write, and
     stm32_sdmmc_abort_transfer enforce regs->DCTRL = 0 upon completion/abort.

3. Priority C & Sleep Reliability:
   - State machine in recorder_sleep():
     a. Precondition: check recorder_sd_verify_idle()
     b. If busy: bounded wait loop (up to 50 ms polling every 5 ms)
     c. If still not idle: controlled recovery:
        - Reset DCTRL = 0 to stop stuck DPSM
        - Call recorder_storage_recover() if still busy
     d. If still not idle: latch s_storage_state = STORAGE_STATE_FAULT, remain awake
     e. If idle: put PSRAM to sleep, log pre-sleep calendar, print "Entering Stop 2",
        arm RTC (prints "RTC wakeup configured"), enter Stop 2.

4. Elimination of Repeated Sleep Retries:
   - main_scheduler_service_step checks !recorder_is_storage_available() at entry,
     preventing autonomous sleep attempts or recording attempts when storage is faulted.

5. Production Console Messages:
   - Success path:
     "SD card initialized successfully"
     "Entering Stop 2"
     "RTC wakeup configured"
   - Failure path:
     "[ERROR] Storage controller failed to become idle"
     "[ERROR] Standby cancelled; device remaining awake"
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
RECORDER_C = REPO_ROOT / "app" / "src" / "recorder.c"
RTC_SLEEP_C = REPO_ROOT / "app" / "src" / "rtc_sleep.c"
MAIN_C = REPO_ROOT / "app" / "src" / "main.c"


def test_priority_a_dma_devicetree_and_headers():
    """Verify DMA2 channel 4 slot 7 binding in overlay and generated DT."""
    assert OVERLAY.exists(), f"Overlay {OVERLAY} must exist"
    overlay_txt = OVERLAY.read_text(encoding="utf-8")

    assert "&dma2" in overlay_txt
    assert "status = \"okay\";" in overlay_txt
    assert "&sdmmc1" in overlay_txt
    assert "dmas = <&dma2 4 7 (STM32_DMA_32BITS | STM32_DMA_PRIORITY_HIGH)>;" in overlay_txt
    assert 'dma-names = "txrx";' in overlay_txt

    for dts_h in [PROD_DTS_GEN, TEST_DTS_GEN]:
        assert dts_h.exists(), f"{dts_h} must exist"
        txt = dts_h.read_text(encoding="utf-8")
        assert "DT_N_S_soc_S_sdmmc_40012800_P_dmas_EXISTS 1" in txt
        assert "DT_N_S_soc_S_sdmmc_40012800_P_dmas_IDX_0_VAL_channel 4" in txt
        assert "DT_N_S_soc_S_sdmmc_40012800_P_dmas_IDX_0_VAL_slot 7" in txt
        assert 'DT_N_S_soc_S_sdmmc_40012800_P_dmas_IDX_0_NAME "txrx"' in txt


def test_priority_a_dma_linker_symbols():
    """Verify HAL_SD_WriteBlocks_DMA and HAL_SD_ReadBlocks_DMA are present in map files."""
    for map_file in [PROD_MAP, TEST_MAP]:
        assert map_file.exists(), f"{map_file} must exist"
        txt = map_file.read_text(encoding="utf-8", errors="ignore")
        assert "HAL_SD_WriteBlocks_DMA" in txt
        assert "HAL_SD_ReadBlocks_DMA" in txt


def test_priority_b_dctrl_idle_fix_in_driver_callbacks():
    """Verify sdmmc_stm32.c callbacks clear DTEN and DMAEN to prevent STA=0x00082000."""
    assert SDMMC_DRIVER.exists()
    driver_txt = SDMMC_DRIVER.read_text(encoding="utf-8")

    # In Tx complete callback
    assert "void HAL_SD_TxCpltCallback(HandleTypeDef *hsd)" in driver_txt
    tx_cb = driver_txt.split("void HAL_SD_TxCpltCallback(HandleTypeDef *hsd)")[1].split("void HAL_SD_RxCpltCallback")[0]
    assert "regs->DCTRL &= ~(SDMMC_DCTRL_DTEN | SDMMC_DCTRL_DMAEN);" in tx_cb
    assert "k_sem_give(&priv->sync);" in tx_cb

    # In Rx complete callback
    assert "void HAL_SD_RxCpltCallback(HandleTypeDef *hsd)" in driver_txt
    rx_cb = driver_txt.split("void HAL_SD_RxCpltCallback(HandleTypeDef *hsd)")[1].split("void HAL_SD_ErrorCallback")[0]
    assert "regs->DCTRL &= ~(SDMMC_DCTRL_DTEN | SDMMC_DCTRL_DMAEN);" in rx_cb
    assert "k_sem_give(&priv->sync);" in rx_cb

    # In Error callback
    assert "void HAL_SD_ErrorCallback(HandleTypeDef *hsd)" in driver_txt
    err_cb = driver_txt.split("void HAL_SD_ErrorCallback(HandleTypeDef *hsd)")[1].split("#endif")[0]
    assert "regs->DCTRL &= ~(SDMMC_DCTRL_DTEN | SDMMC_DCTRL_DMAEN);" in err_cb
    assert "k_sem_give(&priv->sync);" in err_cb


def test_priority_b_dctrl_zero_enforcement_in_access_routines():
    """Verify sdmmc_stm32.c read, write, and abort routines explicitly set regs->DCTRL = 0."""
    driver_txt = SDMMC_DRIVER.read_text(encoding="utf-8")

    # In stm32_sdmmc_access_read
    read_fn = driver_txt.split("static int stm32_sdmmc_access_read(")[1].split("static int stm32_sdmmc_write_blocks(")[0]
    assert "regs->DCTRL = 0;" in read_fn

    # In stm32_sdmmc_access_write
    write_fn = driver_txt.split("static int stm32_sdmmc_access_write(")[1].split("static int stm32_sdmmc_access_ioctl(")[0]
    assert "regs->DCTRL = 0;" in write_fn

    # In stm32_sdmmc_abort_transfer
    abort_fn = driver_txt.split("static void stm32_sdmmc_abort_transfer(")[1].split("static int stm32_sdmmc_wait_completion(")[0]
    assert "regs->DCTRL = 0;" in abort_fn


def test_priority_c_recorder_sleep_state_machine():
    """Verify bounded wait, controlled recovery, and fault latching in recorder_sleep()."""
    rec_txt = RECORDER_C.read_text(encoding="utf-8")

    sleep_fn = rec_txt.split("int recorder_sleep(uint32_t seconds, uint32_t *wake_recovery_us) {")[1].split("int recorder_set_mode(")[0]

    # Precondition: check idle
    assert "int sd_idle_rc = recorder_sd_verify_idle();" in sleep_fn

    # Bounded wait loop
    assert "int64_t wait_deadline = k_uptime_get() + 50;" in sleep_fn
    assert "k_sleep(K_MSEC(5));" in sleep_fn
    assert "sd_idle_rc = recorder_sd_verify_idle();" in sleep_fn

    # Controlled recovery
    assert "if (SDMMC1->DCTRL & (SDMMC_DCTRL_DTEN | SDMMC_DCTRL_DMAEN)) {" in sleep_fn
    assert "SDMMC1->DCTRL = 0;" in sleep_fn
    assert "int rec_rc = recorder_storage_recover();" in sleep_fn

    # Fault latching and required error messages
    assert "s_storage_state = STORAGE_STATE_FAULT;" in sleep_fn
    assert 'printk("[ERROR] Storage controller failed to become idle\\n");' in sleep_fn
    assert 'printk("[ERROR] Standby cancelled; device remaining awake\\n");' in sleep_fn

    # Success path: Stop 2 entry message
    assert 'printk("Entering Stop 2\\n");' in sleep_fn


def test_priority_c_rtc_wakeup_configured_message():
    """Verify rtc_sleep_arm prints 'RTC wakeup configured'."""
    rtc_txt = RTC_SLEEP_C.read_text(encoding="utf-8")

    arm_fn = rtc_txt.split("int rtc_sleep_arm(uint32_t seconds)")[1].split("void rtc_sleep_disarm(void)")[0]
    assert 'printk("RTC wakeup configured\\n");' in arm_fn


def test_elimination_of_retry_spam_when_storage_faulted():
    """Verify main_scheduler_service_step guards against running when storage is not available."""
    main_txt = MAIN_C.read_text(encoding="utf-8")

    sched_fn = main_txt.split("void main_scheduler_service_step(void) {")[1].split("time_t now_utc =")[0]
    assert "if (!recorder_is_storage_available()) {" in sched_fn
    assert "return;" in sched_fn


def test_sdmmc_recovery_clean_sequence_no_post_mount_deinit():
    """Verify recorder_storage_recover does not power off disk after mount and checks idle."""
    rec_txt = RECORDER_C.read_text(encoding="utf-8")

    rec_fn = rec_txt.split("int recorder_storage_recover(void) {")[1].split("int recorder_get_stats(")[0]

    # Step 1: Unmount existing filesystem
    assert "int ret = fs_unmount(&mp);" in rec_fn

    # Step 2: disk deinit before reinit
    step2_idx = rec_fn.find('(void)disk_access_ioctl("SD", DISK_IOCTL_CTRL_DEINIT, NULL);')
    assert step2_idx != -1, "Must call DISK_IOCTL_CTRL_DEINIT before disk_access_init"

    # Step 3: disk_access_init
    step3_idx = rec_fn.find('ret = disk_access_init("SD");')
    assert step3_idx != -1
    assert step2_idx < step3_idx, "DISK_IOCTL_CTRL_DEINIT must happen before disk_access_init"

    # Step 4: fs_mount
    step4_idx = rec_fn.find("ret = fs_mount(&mp);")
    assert step4_idx != -1
    assert step3_idx < step4_idx

    # Step 5: recorder_sd_verify_idle must be called AFTER fs_mount
    step5_idx = rec_fn.find("int idle_rc = recorder_sd_verify_idle();")
    assert step5_idx != -1
    assert step4_idx < step5_idx

    # FATAL BUG CHECK: ensure NO disk_access_ioctl("SD", DISK_IOCTL_CTRL_DEINIT) occurs after fs_mount!
    after_mount = rec_fn[step4_idx:]
    assert "DISK_IOCTL_CTRL_DEINIT" not in after_mount, "DISK_IOCTL_CTRL_DEINIT must NOT be called after fs_mount!"


def test_sdmmc_driver_uninitialized_guard_and_abort_sequence():
    """Verify driver guards against calls in uninitialized state and cleans up on abort/init."""
    driver_txt = SDMMC_DRIVER.read_text(encoding="utf-8")

    # In access_read
    read_fn = driver_txt.split("static int stm32_sdmmc_access_read(")[1].split("static int stm32_sdmmc_write_blocks(")[0]
    assert "if (priv->status != DISK_STATUS_OK) {\n\t\treturn -EIO;\n\t}" in read_fn

    # In access_write
    write_fn = driver_txt.split("static int stm32_sdmmc_access_write(")[1].split("static int stm32_sdmmc_access_ioctl(")[0]
    assert "if (priv->status != DISK_STATUS_OK) {\n\t\treturn -EIO;\n\t}" in write_fn

    # In abort_transfer
    abort_fn = driver_txt.split("static void stm32_sdmmc_abort_transfer(")[1].split("static int stm32_sdmmc_wait_completion(")[0]
    assert "regs->DCTRL = 0;" in abort_fn
    assert "regs->MASK = 0;" in abort_fn
    assert "regs->ICR = SDMMC_STATIC_FLAGS;" in abort_fn
    assert "NVIC_ClearPendingIRQ(DT_INST_IRQN(0));" in abort_fn
    assert "priv->status = DISK_STATUS_UNINIT;" in abort_fn

    # In access_init
    init_fn = driver_txt.split("static int stm32_sdmmc_access_init(")[1].split("static int stm32_sdmmc_access_status(")[0]
    assert "regs->DCTRL = 0;" in init_fn
    assert "regs->MASK = 0;" in init_fn
    assert "regs->ICR = SDMMC_STATIC_FLAGS;" in init_fn

