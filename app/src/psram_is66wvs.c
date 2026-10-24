/* SPDX-License-Identifier: Apache-2.0 */
#include "psram_is66wvs.h"
#include "recorder.h"
#include <stdio.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_gpio.h>
#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_qspi.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(psram, LOG_LEVEL_INF);

static QSPI_HandleTypeDef hqspi;
static bool psram_initialized = false;
static bool psram_present = false;             /* JEDEC ID detected and valid */
static bool psram_integrity_verified = false;  /* Memory integrity passed */
static bool quad_mode_enabled = false;         /* Quad I/O 1-4-4 active */
static bool quad_test_active = false;          /* Probe Quad I/O before enabling it */
static bool quad_write_override = false;       /* Diagnostic override for write */
static bool quad_read_override = false;        /* Diagnostic override for read */
static uint32_t active_burst_size = PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI;
static uint32_t current_sample_shifting = QSPI_SAMPLE_SHIFTING_NONE;

#if defined(__ZEPHYR__)
static struct k_mutex psram_mutex;
#define PSRAM_LOCK()   k_mutex_lock(&psram_mutex, K_FOREVER)
#define PSRAM_UNLOCK() k_mutex_unlock(&psram_mutex)
#else
#define PSRAM_LOCK()   do {} while (0)
#define PSRAM_UNLOCK() do {} while (0)
#endif

/*
 * Opcodes for ISSI IS66WVS4M8
 */
#define CMD_READ           0x03
#define CMD_FAST_READ      0x0B
#define CMD_WRITE          0x02
#define CMD_QUAD_WRITE     0x38
#define CMD_QUAD_FAST_READ 0xEB
#define CMD_RESET_ENABLE   0x66
#define CMD_RESET          0x99
#define CMD_READ_ID        0x9F
#define CMD_ENTER_DPD      0xB9
#define PSRAM_QSPI_TARGET_HZ 18000000U

bool psram_is_quad_mode(void) { return quad_mode_enabled; }

uint32_t psram_get_burst_size(void) { return active_burst_size; }

uint32_t psram_get_sample_shifting(void) { return current_sample_shifting; }

int psram_set_sample_shifting(uint32_t sample_shifting) {
  if (sample_shifting != QSPI_SAMPLE_SHIFTING_NONE &&
      sample_shifting != QSPI_SAMPLE_SHIFTING_HALFCYCLE) {
    return -EINVAL;
  }
  current_sample_shifting = sample_shifting;
  if (psram_initialized) {
    PSRAM_LOCK();
    hqspi.Init.SampleShifting = current_sample_shifting;
    HAL_StatusTypeDef status = HAL_QSPI_Init(&hqspi);
    PSRAM_UNLOCK();
    if (status != HAL_OK) {
      LOG_ERR("Re-init with SampleShifting %u failed: %d", sample_shifting, status);
      return -EIO;
    }
    LOG_INF("QSPI SampleShifting updated to %s",
            (sample_shifting == QSPI_SAMPLE_SHIFTING_HALFCYCLE) ? "HALFCYCLE" : "NONE");
  }
  return 0;
}

uint32_t psram_get_clock_hz(void) {
  uint32_t divider = ((QUADSPI->CR & QUADSPI_CR_PRESCALER) >> QUADSPI_CR_PRESCALER_Pos) + 1U;
  uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();
  return divider ? (hclk_hz / divider) : 0U;
}

int psram_set_burst_size(uint32_t burst_size) {
  uint32_t mode_limit = quad_mode_enabled ? PSRAM_IS66WVS_SAFE_BURST_BYTES_QUAD
                                          : PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI;
  if (burst_size == 0 || burst_size > mode_limit) {
    return -EINVAL;
  }
  active_burst_size = burst_size;
  return 0;
}

bool psram_is_present(void) { return psram_present; }

bool psram_is_integrity_verified(void) { return psram_integrity_verified; }

static void psram_invalidate_state(void) {
  psram_initialized = false;
  psram_present = false;
  psram_integrity_verified = false;
  quad_mode_enabled = false;
  quad_test_active = false;
  quad_write_override = false;
  quad_read_override = false;
  active_burst_size = PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI;
}

static void psram_recover_controller(void) {
  HAL_StatusTypeDef abort_status = HAL_QSPI_Abort(&hqspi);
  if (abort_status != HAL_OK || __HAL_QSPI_GET_FLAG(&hqspi, QSPI_FLAG_BUSY) != RESET) {
    LOG_ERR("QSPI abort incomplete (status=%d SR=0x%08X); resetting controller",
            (int)abort_status, (unsigned int)QUADSPI->SR);
    LL_AHB3_GRP1_ForceReset(LL_AHB3_GRP1_PERIPH_QSPI);
    LL_AHB3_GRP1_ReleaseReset(LL_AHB3_GRP1_PERIPH_QSPI);
    hqspi.State = HAL_QSPI_STATE_RESET;
    hqspi.Lock = HAL_UNLOCKED;
    if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
      LOG_ERR("QSPI controller reinitialization failed");
    }
  }
  psram_invalidate_state();
}

int psram_init(void) {
#if defined(__ZEPHYR__)
  k_mutex_init(&psram_mutex);
#endif
  psram_invalidate_state();

  /* 1. Enable clocks for GPIOA, GPIOB, and QUADSPI peripheral */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA |
                           LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_QSPI);

  /* 2. Configure QUADSPI GPIO pins on NUCLEO-L476RG:
   *    PB10 -> QUADSPI_CLK (AF10)
   *    PB11 -> QUADSPI_BK1_NCS (AF10)
   *    PB1  -> QUADSPI_BK1_IO0 (AF10)
   *    PB0  -> QUADSPI_BK1_IO1 (AF10)
   *    PA7  -> QUADSPI_BK1_IO2 (AF10)
   *    PA6  -> QUADSPI_BK1_IO3 (AF10)
   */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_0, LL_GPIO_AF_10);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_1, LL_GPIO_AF_10);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_1, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_10, LL_GPIO_AF_10);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_11, LL_GPIO_AF_10);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_10);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_NO);

  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_10);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_NO);

  /* 3. Configure QUADSPI hardware controller */
  memset(&hqspi, 0, sizeof(hqspi));
  hqspi.Instance = QUADSPI;
  uint32_t qspi_hclk_hz = HAL_RCC_GetHCLKFreq();
  if (qspi_hclk_hz == 0U) {
    LOG_ERR("Cannot determine HCLK for QUADSPI configuration");
    return -EIO;
  }
  uint32_t qspi_divider = (qspi_hclk_hz + PSRAM_QSPI_TARGET_HZ - 1U) /
                          PSRAM_QSPI_TARGET_HZ;
  if (qspi_divider == 0U || qspi_divider > 256U) {
    LOG_ERR("Unsupported HCLK for safe QUADSPI rate: %u Hz", qspi_hclk_hz);
    return -ERANGE;
  }
  hqspi.Init.ClockPrescaler = qspi_divider - 1U;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = current_sample_shifting;
  hqspi.Init.FlashSize = 21; /* 2^(21+1) = 4,194,304 bytes = 4 MBytes */
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE; /* 100 ns >= 50 ns tCPH */
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;

  HAL_StatusTypeDef status = HAL_QSPI_Init(&hqspi);
  if (status != HAL_OK) {
    LOG_ERR("HAL_QSPI_Init failed: %d", status);
    return -EIO;
  }

  /* 4. Power-up delay: wait at least 150 us per datasheet */
  k_busy_wait(150);

  /* 5. Software Reset sequence: Reset Enable (0x66) then Reset (0x99) */
  QSPI_CommandTypeDef cmd = {0};
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.AddressMode = QSPI_ADDRESS_NONE;
  cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = QSPI_DATA_NONE;
  cmd.DummyCycles = 0;
  cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
  cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  cmd.Instruction = CMD_RESET_ENABLE;
  status = HAL_QSPI_Command(&hqspi, &cmd, 100);
  if (status != HAL_OK) {
    LOG_ERR("PSRAM reset-enable command failed: status=%d SR=0x%08X",
            (int)status, (unsigned int)QUADSPI->SR);
    psram_recover_controller();
    return -EIO;
  }
  k_busy_wait(5);

  cmd.Instruction = CMD_RESET;
  status = HAL_QSPI_Command(&hqspi, &cmd, 100);
  if (status != HAL_OK) {
    LOG_ERR("PSRAM reset command failed: status=%d SR=0x%08X",
            (int)status, (unsigned int)QUADSPI->SR);
    psram_recover_controller();
    return -EIO;
  }
  k_busy_wait(50); /* Reset recovery delay */

  psram_initialized = true;
  LOG_INF("IS66WVS QUADSPI initialized: HCLK=%u Hz divider=%u bus=%u Hz sshift=%u",
          HAL_RCC_GetHCLKFreq(), qspi_divider, psram_get_clock_hz(), current_sample_shifting);
  return 0;
}

int psram_read_id(struct psram_id *id) {
  if (!id) {
    return -EINVAL;
  }
  if (!psram_initialized) {
    int ret = psram_init();
    if (ret) {
      return ret;
    }
  }

  PSRAM_LOCK();

  uint8_t rx_buf[3] = {0};
  QSPI_CommandTypeDef cmd = {0};

  cmd.Instruction = CMD_READ_ID;
  cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd.AddressMode = QSPI_ADDRESS_1_LINE;
  cmd.AddressSize = QSPI_ADDRESS_24_BITS;
  cmd.Address = 0x000000; /* 24 dummy address bits */
  cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  cmd.DataMode = QSPI_DATA_1_LINE;
  cmd.NbData = sizeof(rx_buf);
  cmd.DummyCycles = 0;
  cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
  cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  HAL_StatusTypeDef status = HAL_QSPI_Command(&hqspi, &cmd, 100);
  if (status != HAL_OK) {
    LOG_ERR("QSPI Command 0x9F failed: %d (QSPI state: 0x%02X, err: 0x%08X)",
            (int)status, hqspi.State, hqspi.ErrorCode);
    psram_recover_controller();
    PSRAM_UNLOCK();
    return -EIO;
  }

  status = HAL_QSPI_Receive(&hqspi, rx_buf, 100);
  if (status != HAL_OK) {
    LOG_ERR("QSPI Receive ID failed: %d (QSPI state: 0x%02X, err: 0x%08X)",
            (int)status, hqspi.State, hqspi.ErrorCode);
    psram_recover_controller();
    PSRAM_UNLOCK();
    return -EIO;
  }

  PSRAM_UNLOCK();

  id->mfid = rx_buf[0];
  id->kgd = rx_buf[1];
  id->density = rx_buf[2];

  LOG_INF("PSRAM ID response: MFID=0x%02X, KGD=0x%02X, Density=0x%02X",
          id->mfid, id->kgd, id->density);

  if (id->mfid == PSRAM_ISSI_MFID && id->kgd == PSRAM_ISSI_KGD_PASS &&
      id->density == PSRAM_ISSI_DENSITY_32M) {
    psram_present = true;
    return 0;
  }

  psram_present = false;
  LOG_ERR("Unexpected PSRAM ID; expected 9D/5D/4D, got %02X/%02X/%02X",
          id->mfid, id->kgd, id->density);
  return -ENODEV;
}

int psram_write(uint32_t address, const uint8_t *data, uint32_t length) {
  if (!psram_initialized || !psram_present) {
    return -ENODEV;
  }
  if (!data || length == 0) {
    return 0;
  }
  if (address >= PSRAM_IS66WVS_CAPACITY_BYTES ||
      length > PSRAM_IS66WVS_CAPACITY_BYTES - address) {
    LOG_ERR("PSRAM write bounds violation: addr 0x%06X + len %u exceeds 4MB capacity",
            address, length);
    return -EINVAL;
  }

  PSRAM_LOCK();

  uint32_t offset = 0;
  int ret = 0;

  while (offset < length) {
    uint32_t current_addr = address + offset;
    uint32_t page_remaining = PSRAM_IS66WVS_PAGE_SIZE -
                              (current_addr & (PSRAM_IS66WVS_PAGE_SIZE - 1));

    bool is_quad = (quad_mode_enabled || quad_test_active || quad_write_override);
    uint32_t burst_limit = is_quad ? active_burst_size : PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI;
    uint32_t chunk = length - offset;
    if (chunk > burst_limit) {
      chunk = burst_limit;
    }
    if (chunk > page_remaining) {
      chunk = page_remaining;
    }

    QSPI_CommandTypeDef cmd = {0};
    if (is_quad) {
      cmd.Instruction = CMD_QUAD_WRITE;
      cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
      cmd.AddressMode = QSPI_ADDRESS_4_LINES;
      cmd.AddressSize = QSPI_ADDRESS_24_BITS;
      cmd.Address = current_addr;
      cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      cmd.DataMode = QSPI_DATA_4_LINES;
      cmd.NbData = chunk;
      cmd.DummyCycles = 0;
    } else {
      cmd.Instruction = CMD_WRITE;
      cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
      cmd.AddressMode = QSPI_ADDRESS_1_LINE;
      cmd.AddressSize = QSPI_ADDRESS_24_BITS;
      cmd.Address = current_addr;
      cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      cmd.DataMode = QSPI_DATA_1_LINE;
      cmd.NbData = chunk;
      cmd.DummyCycles = 0;
    }
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    /* Ensure controller is ready before command */
    if (hqspi.State != HAL_QSPI_STATE_READY) {
      if (HAL_QSPI_Abort(&hqspi) != HAL_OK ||
          __HAL_QSPI_GET_FLAG(&hqspi, QSPI_FLAG_BUSY) != RESET) {
        LOG_ERR("QSPI not idle before write: state=0x%02X SR=0x%08X",
                hqspi.State, (unsigned int)QUADSPI->SR);
        psram_recover_controller();
        ret = -EIO;
        break;
      }
    }
    __HAL_QSPI_CLEAR_FLAG(&hqspi, QSPI_FLAG_TE | QSPI_FLAG_TC);

    HAL_StatusTypeDef status = HAL_QSPI_Command(&hqspi, &cmd, 50);
    if (status == HAL_OK) {
      status = HAL_QSPI_Transmit(&hqspi, (uint8_t *)(data + offset), 50);
    }

    if (status != HAL_OK) {
      LOG_ERR("PSRAM physical write failed: addr=0x%06X, chunk=%u/%u, status=%d, qspi_state=0x%02X, err=0x%08X",
              current_addr, chunk, length, (int)status, hqspi.State, hqspi.ErrorCode);
      psram_recover_controller();
      ret = -EIO;
      break;
    }

    offset += chunk;
  }

  PSRAM_UNLOCK();
  return ret;
}

int psram_read(uint32_t address, uint8_t *data, uint32_t length) {
  if (!psram_initialized || !psram_present) {
    return -ENODEV;
  }
  if (!data || length == 0) {
    return 0;
  }
  if (address >= PSRAM_IS66WVS_CAPACITY_BYTES ||
      length > PSRAM_IS66WVS_CAPACITY_BYTES - address) {
    LOG_ERR("PSRAM read bounds violation: addr 0x%06X + len %u exceeds 4MB capacity",
            address, length);
    return -EINVAL;
  }

  PSRAM_LOCK();

  uint32_t offset = 0;
  int ret = 0;

  while (offset < length) {
    uint32_t current_addr = address + offset;
    uint32_t page_remaining = PSRAM_IS66WVS_PAGE_SIZE -
                              (current_addr & (PSRAM_IS66WVS_PAGE_SIZE - 1));

    bool is_quad = (quad_mode_enabled || quad_test_active || quad_read_override);
    uint32_t burst_limit = is_quad ? active_burst_size : PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI;
    uint32_t chunk = length - offset;
    if (chunk > burst_limit) {
      chunk = burst_limit;
    }
    if (chunk > page_remaining) {
      chunk = page_remaining;
    }

    QSPI_CommandTypeDef cmd = {0};
    if (is_quad) {
      cmd.Instruction = CMD_QUAD_FAST_READ;
      cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
      cmd.AddressMode = QSPI_ADDRESS_4_LINES;
      cmd.AddressSize = QSPI_ADDRESS_24_BITS;
      cmd.Address = current_addr;
      cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      cmd.DataMode = QSPI_DATA_4_LINES;
      cmd.NbData = chunk;
      cmd.DummyCycles = 6;
    } else {
      cmd.Instruction = CMD_READ;
      cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
      cmd.AddressMode = QSPI_ADDRESS_1_LINE;
      cmd.AddressSize = QSPI_ADDRESS_24_BITS;
      cmd.Address = current_addr;
      cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
      cmd.DataMode = QSPI_DATA_1_LINE;
      cmd.NbData = chunk;
      cmd.DummyCycles = 0;
    }
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    /* Ensure controller is ready before command */
    if (hqspi.State != HAL_QSPI_STATE_READY) {
      if (HAL_QSPI_Abort(&hqspi) != HAL_OK ||
          __HAL_QSPI_GET_FLAG(&hqspi, QSPI_FLAG_BUSY) != RESET) {
        LOG_ERR("QSPI not idle before read: state=0x%02X SR=0x%08X",
                hqspi.State, (unsigned int)QUADSPI->SR);
        psram_recover_controller();
        ret = -EIO;
        break;
      }
    }
    __HAL_QSPI_CLEAR_FLAG(&hqspi, QSPI_FLAG_TE | QSPI_FLAG_TC);

    HAL_StatusTypeDef status = HAL_QSPI_Command(&hqspi, &cmd, 50);
    if (status == HAL_OK) {
      status = HAL_QSPI_Receive(&hqspi, data + offset, 50);
    }

    if (status != HAL_OK) {
      LOG_ERR("PSRAM physical read failed: addr=0x%06X, chunk=%u/%u, status=%d, qspi_state=0x%02X, err=0x%08X",
              current_addr, chunk, length, (int)status, hqspi.State, hqspi.ErrorCode);
      psram_recover_controller();
      ret = -EIO;
      break;
    }

    offset += chunk;
  }

  PSRAM_UNLOCK();
  return ret;
}

/* ========================================================================= */
/* 10 DETERMINISTIC PATTERN GENERATORS                                        */
/* ========================================================================= */

const char *psram_get_pattern_name(enum psram_pattern_type type) {
  switch (type) {
    case PSRAM_PAT_ALL_ZEROS:     return "All Zeros (0x00)";
    case PSRAM_PAT_ALL_ONES:      return "All Ones (0xFF)";
    case PSRAM_PAT_ALT_55_AA:     return "Alternating 0x55/0xAA";
    case PSRAM_PAT_ALT_AA_55:     return "Inverse Alt 0xAA/0x55";
    case PSRAM_PAT_INCREMENTING:  return "Incrementing Ramp";
    case PSRAM_PAT_DECREMENTING:  return "Decrementing Ramp";
    case PSRAM_PAT_WALKING_ONES:  return "Walking Ones";
    case PSRAM_PAT_WALKING_ZEROS: return "Walking Zeros";
    case PSRAM_PAT_ALT_NIBBLES:   return "Alternating Nibbles (0xF0/0x0F)";
    case PSRAM_PAT_PSEUDO_RANDOM: return "Pseudo-Random (Seed 0x12345678)";
    default:                      return "Unknown";
  }
}

void psram_fill_pattern(enum psram_pattern_type type, uint8_t *buf, uint32_t len, uint32_t base_addr) {
  if (!buf || len == 0) return;

  switch (type) {
    case PSRAM_PAT_ALL_ZEROS:
      memset(buf, 0x00, len);
      break;

    case PSRAM_PAT_ALL_ONES:
      memset(buf, 0xFF, len);
      break;

    case PSRAM_PAT_ALT_55_AA:
      for (uint32_t i = 0; i < len; i++) {
        buf[i] = (i & 1U) ? 0xAAU : 0x55U;
      }
      break;

    case PSRAM_PAT_ALT_AA_55:
      for (uint32_t i = 0; i < len; i++) {
        buf[i] = (i & 1U) ? 0x55U : 0xAAU;
      }
      break;

    case PSRAM_PAT_INCREMENTING:
      for (uint32_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)((base_addr + i) & 0xFFU);
      }
      break;

    case PSRAM_PAT_DECREMENTING:
      for (uint32_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)(0xFFU - ((base_addr + i) & 0xFFU));
      }
      break;

    case PSRAM_PAT_WALKING_ONES:
      for (uint32_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)(1U << (i % 8U));
      }
      break;

    case PSRAM_PAT_WALKING_ZEROS:
      for (uint32_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)(~(1U << (i % 8U)));
      }
      break;

    case PSRAM_PAT_ALT_NIBBLES:
      for (uint32_t i = 0; i < len; i++) {
        buf[i] = (i & 1U) ? 0x0FU : 0xF0U;
      }
      break;

    case PSRAM_PAT_PSEUDO_RANDOM: {
      uint32_t state = 0x12345678U ^ base_addr;
      for (uint32_t i = 0; i < len; i++) {
        state = state * 1664525U + 1013904223U;
        buf[i] = (uint8_t)(state >> 24);
      }
      break;
    }

    default:
      memset(buf, 0x5A, len);
      break;
  }
}

/*
 * Low-level decoupled cross-mode transfer and verification probe.
 * Compares every byte across length; never halts after first error.
 */
static int psram_exec_probe(uint32_t addr, uint32_t len,
                            enum psram_pattern_type pat,
                            bool quad_w, bool quad_r,
                            uint8_t *wbuf, uint8_t *rbuf,
                            uint32_t *mismatches,
                            uint32_t *first_offset,
                            uint8_t *exp_byte, uint8_t *act_byte,
                            uint8_t *diff_mask,
                            int *wret_out, int *rret_out) {
  *mismatches = 0;
  *first_offset = 0;
  *exp_byte = 0;
  *act_byte = 0;
  *diff_mask = 0;

  psram_fill_pattern(pat, wbuf, len, addr);
  memset(rbuf, 0x5A, len);

  quad_write_override = quad_w;
  quad_read_override = quad_r;

  int wret = psram_write(addr, wbuf, len);
  int rret = (wret == 0) ? psram_read(addr, rbuf, len) : -EIO;

  quad_write_override = false;
  quad_read_override = false;

  if (wret_out) *wret_out = wret;
  if (rret_out) *rret_out = rret;

  if (wret != 0 || rret != 0) {
    return -EIO;
  }

  for (uint32_t i = 0; i < len; i++) {
    if (wbuf[i] != rbuf[i]) {
      if (*mismatches == 0) {
        *first_offset = i;
        *exp_byte = wbuf[i];
        *act_byte = rbuf[i];
        *diff_mask = (uint8_t)(wbuf[i] ^ rbuf[i]);
      }
      (*mismatches)++;
    }
  }

  return (*mismatches == 0) ? 0 : -EBADMSG;
}

/*
 * Self-test transfer wrapper for Phase 1-4 validation.
 */
static void record_mismatch_context(uint32_t base_addr, const uint8_t *expected,
                                    const uint8_t *observed, uint32_t len,
                                    struct psram_test_report *report) {
  for (uint32_t i = 0; i < len; i++) {
    if (expected[i] != observed[i]) {
      if (report->error_count == 0) {
        report->first_failing_addr = base_addr + i;
        report->expected_val = expected[i];
        report->observed_val = observed[i];
        report->failing_bitmask = (uint8_t)(expected[i] ^ observed[i]);

        uint32_t start_idx = (i >= 4) ? (i - 4) : 0;
        uint32_t end_idx = (i + 4 < len) ? (i + 4) : len;
        report->ctx_len = end_idx - start_idx;
        for (uint32_t k = 0; k < report->ctx_len; k++) {
          report->ctx_expected[k] = expected[start_idx + k];
          report->ctx_observed[k] = observed[start_idx + k];
        }
      }
      report->error_count++;
    }
  }
}

int psram_run_self_test(struct psram_test_report *report) {
  memset(report, 0, sizeof(*report));

#if defined(__ZEPHYR__)
  if (recorder_is_active()) {
    LOG_WRN("Cannot run PSRAM memory test while recording is active");
    snprintf(report->status_msg, sizeof(report->status_msg),
             "FAIL: Audio recording session active; PSRAM test rejected");
    return -EBUSY;
  }
#endif

  quad_mode_enabled = false;
  quad_test_active = false;
  quad_write_override = false;
  quad_read_override = false;
  active_burst_size = PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI;
  psram_integrity_verified = false;

  int ret = psram_init();
  if (ret) {
    snprintf(report->status_msg, sizeof(report->status_msg),
             "QSPI init failed (%d)", ret);
    return ret;
  }

  ret = psram_read_id(&report->id);
  if (ret) {
    report->detected = false;
    snprintf(report->status_msg, sizeof(report->status_msg),
             "PSRAM not detected: MFID=0x%02X KGD=0x%02X (expected 0x9D / 0x5D). Check wiring or voltage.",
             report->id.mfid, report->id.kgd);
    return -ENODEV;
  }

  report->detected = true;
  report->sample_shifting = current_sample_shifting;

  LOG_INF("PSRAM SPI probe config: QSPI=%u Hz 1-1-1 mode dummy=0 burst=%u sshift=%u",
          psram_get_clock_hz(), PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI, current_sample_shifting);
  LOG_INF("PSRAM JEDEC: ISSI MFID=0x%02X, KGD=0x%02X (PASS), Density=0x%02X (32Mb/4MB)",
          report->id.mfid, report->id.kgd, report->id.density);

  /* ----------------------------------------------------------------------- */
  /* PHASE 4: TEST A — Dedicated 1-1-1 SPI Mode Integrity Baseline           */
  /* Dedicated test region: 0x00300000..0x003FFFFF (outside live staging)   */
  /* ----------------------------------------------------------------------- */
  const uint32_t num_passes = 3;
  report->passes_tested = num_passes;

  uint8_t wbuf[64];
  uint8_t rbuf[64];

  for (uint32_t pass = 1; pass <= num_passes; pass++) {
    uint32_t errors_before_pass = report->error_count;

    for (int pat = 0; pat < PSRAM_PAT_COUNT; pat++) {
      report->patterns_tested++;
      uint32_t test_addr = PSRAM_DIAG_TEST_BASE_ADDR + (pat * 0x80U);

      uint32_t mismatches = 0, first_off = 0;
      uint8_t exp_b = 0, act_b = 0, diff_m = 0;
      int wret = 0, rret = 0;

      int pret = psram_exec_probe(test_addr, sizeof(wbuf), (enum psram_pattern_type)pat,
                                  false, false, wbuf, rbuf,
                                  &mismatches, &first_off, &exp_b, &act_b, &diff_m,
                                  &wret, &rret);
      if (pret == 0) {
        report->patterns_passed++;
      } else {
        record_mismatch_context(test_addr, wbuf, rbuf, sizeof(wbuf), report);
      }
    }

    /* Page boundary test at 0x003003FE (straddles 1024-byte boundary 0x00300400) */
    static const uint32_t boundary_addrs[] = {
        PSRAM_DIAG_TEST_BASE_ADDR,          /* 3 MB base */
        PSRAM_DIAG_TEST_BASE_ADDR + 0x3FE,  /* Boundary wrap */
        PSRAM_DIAG_TEST_BASE_ADDR + 0x800,  /* 2 KB offset */
        PSRAM_DIAG_TEST_MAX_ADDR - 15       /* Last 16 bytes of 4 MB */
    };

    for (int b = 0; b < ARRAY_SIZE(boundary_addrs); b++) {
      uint32_t b_addr = boundary_addrs[b];
      uint32_t mismatches = 0, first_off = 0;
      uint8_t exp_b = 0, act_b = 0, diff_m = 0;
      int wret = 0, rret = 0;

      int pret = psram_exec_probe(b_addr, 16, PSRAM_PAT_ALT_55_AA,
                                  false, false, wbuf, rbuf,
                                  &mismatches, &first_off, &exp_b, &act_b, &diff_m,
                                  &wret, &rret);
      if (pret == 0) {
        if (pass == 1) report->boundary_checks_passed++;
      } else {
        record_mismatch_context(b_addr, wbuf, rbuf, 16, report);
      }
    }

    if (report->error_count == errors_before_pass) {
      report->passes_passed++;
    }
  }

  report->mode_a_errors = report->error_count;

  /* ----------------------------------------------------------------------- */
  /* PHASE 4: CROSS-MODE ISOLATION (TEST B, C, D)                            */
  /* ----------------------------------------------------------------------- */
  if (report->error_count == 0) {
    report->spi_integrity_verified = true;
    report->integrity_verified = true;
    psram_integrity_verified = true;

    static const uint32_t cross_sizes[] = {16, 32, 64};
    static const uint32_t cross_addrs[] = {
        PSRAM_DIAG_TEST_BASE_ADDR + 0x800,
        PSRAM_DIAG_TEST_BASE_ADDR + 0x840,
        PSRAM_DIAG_TEST_BASE_ADDR + 0x880,
        PSRAM_DIAG_TEST_BASE_ADDR + 0x3FE
    };

    uint32_t mode_b_errs = 0;
    uint32_t mode_c_errs = 0;
    uint32_t mode_d_errs = 0;

    active_burst_size = PSRAM_IS66WVS_SAFE_BURST_BYTES_QUAD;
    LOG_INF("Quad verify config: QSPI=%u Hz 0x38/0xEB dummy=6 burst=%u sshift=%u",
            psram_get_clock_hz(), active_burst_size, current_sample_shifting);

    for (uint32_t qpass = 1; qpass <= 3; qpass++) {
      for (uint32_t p = 0; p < ARRAY_SIZE(cross_sizes); p++) {
        uint32_t len = cross_sizes[p];
        uint32_t addr = cross_addrs[p % ARRAY_SIZE(cross_addrs)];

        /* Test B: Quad Write (0x38) + SPI Read (0x03) */
        uint32_t m_b = 0, off_b = 0;
        uint8_t exp_b = 0, act_b = 0, diff_b = 0;
        int wret_b = 0, rret_b = 0;
        int ret_b = psram_exec_probe(addr, len, PSRAM_PAT_ALT_55_AA,
                                     true, false, wbuf, rbuf,
                                     &m_b, &off_b, &exp_b, &act_b, &diff_b,
                                     &wret_b, &rret_b);
        if (ret_b != 0) {
          mode_b_errs += (m_b > 0) ? m_b : 1;
          LOG_ERR("Test B (Quad-W/SPI-R) failed: addr=0x%06X len=%u pass=%u off=+%u exp=0x%02X act=0x%02X diff=0x%02X (wret=%d, rret=%d)",
                  addr, len, qpass, off_b, exp_b, act_b, diff_b, wret_b, rret_b);
        }

        /* Test C: SPI Write (0x02) + Quad Read (0xEB) */
        uint32_t m_c = 0, off_c = 0;
        uint8_t exp_c = 0, act_c = 0, diff_c = 0;
        int wret_c = 0, rret_c = 0;
        int ret_c = psram_exec_probe(addr, len, PSRAM_PAT_ALT_AA_55,
                                     false, true, wbuf, rbuf,
                                     &m_c, &off_c, &exp_c, &act_c, &diff_c,
                                     &wret_c, &rret_c);
        if (ret_c != 0) {
          mode_c_errs += (m_c > 0) ? m_c : 1;
          LOG_ERR("Test C (SPI-W/Quad-R) failed: addr=0x%06X len=%u pass=%u off=+%u exp=0x%02X act=0x%02X diff=0x%02X (wret=%d, rret=%d)",
                  addr, len, qpass, off_c, exp_c, act_c, diff_c, wret_c, rret_c);
        }

        /* Test D: Quad Write (0x38) + Quad Read (0xEB) */
        uint32_t m_d = 0, off_d = 0;
        uint8_t exp_d = 0, act_d = 0, diff_d = 0;
        int wret_d = 0, rret_d = 0;
        int ret_d = psram_exec_probe(addr, len, PSRAM_PAT_WALKING_ONES,
                                     true, true, wbuf, rbuf,
                                     &m_d, &off_d, &exp_d, &act_d, &diff_d,
                                     &wret_d, &rret_d);
        if (ret_d != 0) {
          mode_d_errs += (m_d > 0) ? m_d : 1;
          LOG_ERR("Test D (Quad-W/Quad-R) failed: addr=0x%06X len=%u pass=%u off=+%u exp=0x%02X act=0x%02X diff=0x%02X (wret=%d, rret=%d)",
                  addr, len, qpass, off_d, exp_d, act_d, diff_d, wret_d, rret_d);
        }
      }
    }

    report->mode_b_errors = mode_b_errs;
    report->mode_c_errors = mode_c_errs;
    report->mode_d_errors = mode_d_errs;

    /* Phase 5: Timing / Sample-Shifting Diagnostic Investigation */
    if (mode_c_errs > 0 && current_sample_shifting == QSPI_SAMPLE_SHIFTING_NONE) {
      /* Re-probe Test C with HALFCYCLE to check if timing shift resolves mismatch */
      psram_set_sample_shifting(QSPI_SAMPLE_SHIFTING_HALFCYCLE);
      uint32_t sshift_c_errs = 0;
      for (uint32_t p = 0; p < ARRAY_SIZE(cross_sizes); p++) {
        uint32_t len = cross_sizes[p];
        uint32_t addr = cross_addrs[p % ARRAY_SIZE(cross_addrs)];
        uint32_t m = 0, off = 0;
        uint8_t exp_b = 0, act_b = 0, diff_b = 0;
        int wret = 0, rret = 0;
        if (psram_exec_probe(addr, len, PSRAM_PAT_ALT_AA_55, false, true,
                             wbuf, rbuf, &m, &off, &exp_b, &act_b, &diff_b,
                             &wret, &rret) != 0) {
          sshift_c_errs += (m > 0) ? m : 1;
        }
      }
      report->sshift_c_errors = sshift_c_errs;
      LOG_INF("Sample shifting comparison for Test C: SSHIFT_NONE=%u errors, SSHIFT_HALFCYCLE=%u errors",
              mode_c_errs, sshift_c_errs);
      /* Restore baseline configured setting */
      psram_set_sample_shifting(QSPI_SAMPLE_SHIFTING_NONE);
    }

    /* Quad Mode Qualification */
    if (mode_b_errs == 0 && mode_c_errs == 0 && mode_d_errs == 0) {
      quad_test_active = false;
      quad_mode_enabled = true;
      report->quad_mode_verified = true;
      LOG_INF("PSRAM Quad I/O mode (1-4-4, CMD 0x38/0xEB) verified and ENABLED (burst %u bytes)",
              active_burst_size);
      snprintf(report->status_msg, sizeof(report->status_msg),
               "PASS: IS66WVS4M8 detected (4MB), %u passes verified, Quad I/O active (burst %u B)",
               report->passes_passed, active_burst_size);
      return 0;
    } else {
      quad_test_active = false;
      quad_mode_enabled = false;
      active_burst_size = PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI;
      report->quad_mode_verified = false;

      /* Fallback validation: confirm 1-line SPI remains sound */
      uint32_t m_fb = 0, off_fb = 0;
      uint8_t exp_fb = 0, act_fb = 0, diff_fb = 0;
      int wret_fb = 0, rret_fb = 0;
      if (psram_exec_probe(PSRAM_DIAG_TEST_BASE_ADDR + 0x800, 16, PSRAM_PAT_ALT_55_AA,
                           false, false, wbuf, rbuf,
                           &m_fb, &off_fb, &exp_fb, &act_fb, &diff_fb,
                           &wret_fb, &rret_fb) != 0) {
        psram_integrity_verified = false;
        report->spi_integrity_verified = false;
        report->integrity_verified = false;
        snprintf(report->status_msg, sizeof(report->status_msg),
                 "FAIL: Quad failed (B:%u C:%u D:%u) and SPI fallback failed",
                 mode_b_errs, mode_c_errs, mode_d_errs);
        return -EIO;
      }

      psram_integrity_verified = true;
      report->spi_integrity_verified = true;
      report->integrity_verified = true;
      LOG_WRN("PSRAM Quad I/O test failed (B:%u C:%u D:%u); falling back to 1-line SPI (burst %u bytes)",
              mode_b_errs, mode_c_errs, mode_d_errs, active_burst_size);
      snprintf(report->status_msg, sizeof(report->status_msg),
               "PASS: SPI fallback read-back verified; Quad I/O unavailable (Test B:%u C:%u D:%u errs)",
               mode_b_errs, mode_c_errs, mode_d_errs);
      return 0;
    }
  } else {
    /* SPI verification failed */
    quad_mode_enabled = false;
    active_burst_size = PSRAM_IS66WVS_SAFE_BURST_BYTES_SPI;
    psram_integrity_verified = false;
    report->spi_integrity_verified = false;
    report->quad_mode_verified = false;
    report->integrity_verified = false;

    if (report->status_msg[0] == '\0') {
      snprintf(report->status_msg, sizeof(report->status_msg),
               "FAIL: %u errors (pass %u/%u). First mismatch at 0x%06X: expected 0x%02X, got 0x%02X (diff 0x%02X)",
               report->error_count, report->passes_passed, report->passes_tested,
               report->first_failing_addr, report->expected_val,
               report->observed_val, report->failing_bitmask);
    }
    return -EIO;
  }
}

/* ========================================================================= */
/* PHASE 2 & 3: EXHAUSTIVE MULTI-PATTERN / MULTI-ADDRESS DIAGNOSTIC MATRIX   */
/* ========================================================================= */

int psram_run_diagnostic_matrix(bool full_matrix, struct psram_diag_report *report) {
  if (!report) return -EINVAL;
  memset(report, 0, sizeof(*report));

#if defined(__ZEPHYR__)
  if (recorder_is_active()) {
    printk("[ERROR] Cannot run PSRAM diagnostic while recording is active.\n");
    snprintf(report->status_msg, sizeof(report->status_msg), "Recording active");
    return -EBUSY;
  }
#endif

  if (!psram_present || !psram_initialized) {
    printk("[ERROR] PSRAM hardware not detected or initialized.\n");
    snprintf(report->status_msg, sizeof(report->status_msg), "PSRAM not present");
    return -ENODEV;
  }

  report->sample_shifting = current_sample_shifting;

  /* Borrow audio pool buffer for zero-heap, zero-stack dynamic buffer */
  void *borrowed_pool = NULL;
  uint32_t borrowed_pool_max = 0;
  int b_ret = recorder_borrow_contiguous_pool(&borrowed_pool, &borrowed_pool_max);

  uint8_t fallback_w[256];
  uint8_t fallback_r[256];
  uint8_t *wbuf = fallback_w;
  uint8_t *rbuf = fallback_r;
  uint32_t max_test_len = 256;

  if (b_ret == 0 && borrowed_pool && borrowed_pool_max >= 2050U) {
    wbuf = (uint8_t *)borrowed_pool;
    rbuf = (uint8_t *)borrowed_pool + 1025U;
    max_test_len = 1025;
  }

  static const uint32_t test_lengths[] = {
      1, 2, 4, 8, 15, 16, 17, 31, 32, 33, 64, 128, 256, 1023, 1024, 1025
  };

  static const uint32_t test_addrs[] = {
      PSRAM_DIAG_TEST_BASE_ADDR,          /* 0x00300000 - Aligned */
      PSRAM_DIAG_TEST_BASE_ADDR + 1,      /* 0x00300001 - Unaligned */
      PSRAM_DIAG_TEST_BASE_ADDR + 0x3FE,  /* 0x003003FE - Page boundary wrap */
      PSRAM_DIAG_TEST_BASE_ADDR + 0x800,  /* 0x00300800 - Page boundary */
      PSRAM_DIAG_TEST_BASE_ADDR + 0x840,  /* 0x00300840 - Log failure addr */
      PSRAM_DIAG_TEST_BASE_ADDR + 0x880   /* 0x00300880 - Log failure addr */
  };

  struct mode_test_cfg {
    const char *name;
    const char *w_name;
    const char *r_name;
    bool quad_w;
    bool quad_r;
  } modes[4] = {
      {"Test A", "SPI  (0x02)", "SPI  (0x03)", false, false},
      {"Test B", "Quad (0x38)", "SPI  (0x03)", true,  false},
      {"Test C", "SPI  (0x02)", "Quad (0xEB)", false, true},
      {"Test D", "Quad (0x38)", "Quad (0xEB)", true,  true}
  };

  printk("\n========================================================================================\n");
  printk("                      PSRAM MULTI-PATTERN / MULTI-ADDRESS DIAGNOSTIC\n");
  printk("========================================================================================\n");
  printk("  Target Memory     : ISSI IS66WVS4M8 (4 MB, QSPI 18 MHz)\n");
  printk("  Sample Shifting   : %u (%s)\n", current_sample_shifting,
         (current_sample_shifting == QSPI_SAMPLE_SHIFTING_HALFCYCLE) ? "HALFCYCLE" : "NONE");
  printk("  Test Base Address : 0x%08X (strictly isolated in high memory)\n", PSRAM_DIAG_TEST_BASE_ADDR);
  printk("  Address Span      : 0x%08X - 0x%08X (Live audio staging: 0x000000-0x0FFFFF protected)\n",
         PSRAM_DIAG_TEST_BASE_ADDR, PSRAM_DIAG_TEST_MAX_ADDR);
  printk("----------------------------------------------------------------------------------------\n");
  printk(" Mode   Pattern                   Addr       Len  Write Mode  Read Mode   Mismatches Result\n");
  printk("----------------------------------------------------------------------------------------\n");

  uint32_t a_errs = 0, b_errs = 0, c_errs = 0, d_errs = 0;

  uint32_t num_lens = full_matrix ? ARRAY_SIZE(test_lengths) : 6;
  uint32_t num_addrs = full_matrix ? ARRAY_SIZE(test_addrs) : 3;

  for (int m = 0; m < 4; m++) {
    for (int pat = 0; pat < PSRAM_PAT_COUNT; pat++) {
      for (uint32_t a = 0; a < num_addrs; a++) {
        uint32_t addr = test_addrs[a];
        for (uint32_t l = 0; l < num_lens; l++) {
          uint32_t len = test_lengths[l];
          if (len > max_test_len) continue;
          if (addr + len > PSRAM_IS66WVS_CAPACITY_BYTES) continue;

          report->total_probes++;
          report->total_bytes_checked += len;

          uint32_t mismatches = 0, first_off = 0;
          uint8_t exp_b = 0, act_b = 0, diff_m = 0;
          int wret = 0, rret = 0;

          int pret = psram_exec_probe(addr, len, (enum psram_pattern_type)pat,
                                      modes[m].quad_w, modes[m].quad_r,
                                      wbuf, rbuf,
                                      &mismatches, &first_off, &exp_b, &act_b, &diff_m,
                                      &wret, &rret);

          if (pret == 0) {
            report->passed_probes++;
          } else {
            report->failed_probes++;
            report->total_mismatches += (mismatches > 0) ? mismatches : 1;

            if (m == 0) a_errs += mismatches;
            else if (m == 1) b_errs += mismatches;
            else if (m == 2) c_errs += mismatches;
            else if (m == 3) d_errs += mismatches;

            /* Always print failing probes */
            printk(" %-6s %-25s 0x%08X %-4u %-11s %-11s %-10u FAIL (off=+%u exp=0x%02X act=0x%02X xor=0x%02X w=%d r=%d)\n",
                   modes[m].name, psram_get_pattern_name((enum psram_pattern_type)pat),
                   addr, len, modes[m].w_name, modes[m].r_name,
                   mismatches, first_off, exp_b, act_b, diff_m, wret, rret);
          }
        }
      }
    }
  }

  if (borrowed_pool) {
    recorder_return_contiguous_pool();
  }

  report->mode_a_ok = (a_errs == 0);
  report->mode_b_ok = (b_errs == 0);
  report->mode_c_ok = (c_errs == 0);
  report->mode_d_ok = (d_errs == 0);

  printk("----------------------------------------------------------------------------------------\n");
  printk("  Total Probes Run  : %u | Passed: %u | Failed: %u | Total Bytes: %u B\n",
         report->total_probes, report->passed_probes, report->failed_probes, report->total_bytes_checked);
  printk("  Total Mismatches  : %u\n", report->total_mismatches);
  printk("  Cross-Mode Diagnostic Isolation Summary:\n");
  printk("    - Test A (SPI-W  / SPI-R ) : %s (%u errors)\n", report->mode_a_ok ? "PASS" : "FAIL", a_errs);
  printk("    - Test B (Quad-W / SPI-R ) : %s (%u errors)\n", report->mode_b_ok ? "PASS" : "FAIL", b_errs);
  printk("    - Test C (SPI-W  / Quad-R) : %s (%u errors)\n", report->mode_c_ok ? "PASS" : "FAIL", c_errs);
  printk("    - Test D (Quad-W / Quad-R) : %s (%u errors)\n", report->mode_d_ok ? "PASS" : "FAIL", d_errs);
  printk("========================================================================================\n\n");

  snprintf(report->status_msg, sizeof(report->status_msg),
           "Diag: %u probes, %u passed, %u failed, %u mismatches (A:%u B:%u C:%u D:%u)",
           report->total_probes, report->passed_probes, report->failed_probes,
           report->total_mismatches, a_errs, b_errs, c_errs, d_errs);

  return (report->failed_probes == 0) ? 0 : -EBADMSG;
}

/* ========================================================================= */
/* PSRAM BENCHMARK & POWER MANAGEMENT                                        */
/* ========================================================================= */

int psram_run_benchmark(struct psram_bench_report *report) {
  if (!report)
    return -EINVAL;
  memset(report, 0, sizeof(*report));

  if (!psram_initialized || !psram_present || !psram_integrity_verified) {
    snprintf(report->status_msg, sizeof(report->status_msg),
             "PSRAM not initialized or integrity not verified");
    return -ENODEV;
  }

  report->quad_mode = quad_mode_enabled;
  report->burst_size = active_burst_size;
  report->audio_period_192k_us = 10666; /* 2048 samples / 192000 Hz = 10.666 ms */

  uint8_t pat[64];
  uint8_t rbuf[64];
  for (uint32_t i = 0; i < sizeof(pat); i++) {
    pat[i] = (uint8_t)((i * 37 + 101) & 0xFF);
  }

  const uint32_t test_addr = 0x00200000; /* 2 MB mark (isolated from staging) */
  const uint32_t total_bench_bytes = 4096;

  /* 1. Benchmark 4096-byte block write */
  uint64_t t0 = k_cycle_get_64();
  for (uint32_t off = 0; off < total_bench_bytes; off += sizeof(pat)) {
    int w_ret = psram_write(test_addr + off, pat, sizeof(pat));
    if (w_ret != 0) {
      snprintf(report->status_msg, sizeof(report->status_msg),
               "4096-byte write failed (code %d)", w_ret);
      return w_ret;
    }
  }
  uint64_t t1 = k_cycle_get_64();
  uint64_t w_cyc = (t1 >= t0) ? (t1 - t0) : 0;
  report->write_4k_us = (uint32_t)k_cyc_to_us_floor64(w_cyc);
  if (report->write_4k_us > 0) {
    report->write_throughput_kb_s =
        (uint32_t)(((uint64_t)total_bench_bytes * 1000000ULL) /
                   (report->write_4k_us * 1024ULL));
  }

  /* 2. Benchmark 4096-byte block read and verify integrity */
  bool data_ok = true;
  t0 = k_cycle_get_64();
  for (uint32_t off = 0; off < total_bench_bytes; off += sizeof(rbuf)) {
    int r_ret = psram_read(test_addr + off, rbuf, sizeof(rbuf));
    if (r_ret != 0) {
      snprintf(report->status_msg, sizeof(report->status_msg),
               "4096-byte read failed (code %d)", r_ret);
      return r_ret;
    }
    if (memcmp(pat, rbuf, sizeof(pat)) != 0) {
      data_ok = false;
    }
  }
  t1 = k_cycle_get_64();
  uint64_t r_cyc = (t1 >= t0) ? (t1 - t0) : 0;
  report->read_4k_us = (uint32_t)k_cyc_to_us_floor64(r_cyc);
  if (report->read_4k_us > 0) {
    report->read_throughput_kb_s =
        (uint32_t)(((uint64_t)total_bench_bytes * 1000000ULL) /
                   (report->read_4k_us * 1024ULL));
  }

  report->integrity_verified = data_ok;
  report->write_margin_us =
      (int32_t)report->audio_period_192k_us - (int32_t)report->write_4k_us;

  /* 3. Representative chunk sizes (64, 256, 1024, 4096) */
  static const uint32_t chunks[4] = {64, 256, 1024, 4096};
  bool chunks_ok = true;
  for (int c = 0; c < 4; c++) {
    report->chunk_sizes[c] = chunks[c];
    uint32_t chunk_len = chunks[c];

    t0 = k_cycle_get_64();
    for (uint32_t off = 0; off < chunk_len; off += sizeof(pat)) {
      uint32_t step =
          (chunk_len - off > sizeof(pat)) ? sizeof(pat) : (chunk_len - off);
      int w_ret = psram_write(test_addr + 0x1000 + off, pat, step);
      if (w_ret != 0) {
        chunks_ok = false;
        snprintf(report->status_msg, sizeof(report->status_msg),
                 "chunk %u-byte write failed at 0x%06X (%d)",
                 chunk_len, test_addr + 0x1000 + off, w_ret);
        break;
      }
    }
    t1 = k_cycle_get_64();
    report->chunk_write_us[c] =
        (uint32_t)k_cyc_to_us_floor64((t1 >= t0) ? (t1 - t0) : 0);

    t0 = k_cycle_get_64();
    for (uint32_t off = 0; chunks_ok && off < chunk_len; off += sizeof(rbuf)) {
      uint32_t step =
          (chunk_len - off > sizeof(rbuf)) ? sizeof(rbuf) : (chunk_len - off);
      int r_ret = psram_read(test_addr + 0x1000 + off, rbuf, step);
      if (r_ret != 0) {
        chunks_ok = false;
        snprintf(report->status_msg, sizeof(report->status_msg),
                 "chunk %u-byte read failed at 0x%06X (%d)",
                 chunk_len, test_addr + 0x1000 + off, r_ret);
        break;
      }
      for (uint32_t i = 0; i < step; i++) {
        if (rbuf[i] != pat[i % sizeof(pat)]) {
          chunks_ok = false;
          LOG_ERR("PSRAM chunk mismatch: mode=%s burst=%u len=%u addr=0x%06X expected=0x%02X actual=0x%02X",
                  quad_mode_enabled ? "SPI Quad 1-4-4" : "SPI 1-1-1",
                  active_burst_size, chunk_len,
                  test_addr + 0x1000 + off + i, pat[i % sizeof(pat)], rbuf[i]);
          snprintf(report->status_msg, sizeof(report->status_msg),
                   "chunk %u-byte integrity mismatch at 0x%06X",
                   chunk_len, test_addr + 0x1000 + off + i);
          break;
        }
      }
      if (!chunks_ok) break;
    }
    t1 = k_cycle_get_64();
    report->chunk_read_us[c] =
        (uint32_t)k_cyc_to_us_floor64((t1 >= t0) ? (t1 - t0) : 0);
  }

  report->integrity_verified = report->integrity_verified && chunks_ok;
  report->success = report->integrity_verified;
  if (report->success) {
    snprintf(report->status_msg, sizeof(report->status_msg),
             "PASS: 4096B Write=%u us (%u KB/s), Read=%u us (%u KB/s), 192k Margin=+%d us",
             report->write_4k_us, report->write_throughput_kb_s,
             report->read_4k_us, report->read_throughput_kb_s,
             report->write_margin_us);
  } else {
    snprintf(report->status_msg, sizeof(report->status_msg),
             "FAIL: Readback data mismatch after 4096-byte transfer");
  }

  return report->success ? 0 : -EIO;
}

int psram_sleep(void) {
  if (!psram_present || !psram_initialized) {
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);
    return 0;
  }

  PSRAM_LOCK();

  QSPI_CommandTypeDef sCommand;
  memset(&sCommand, 0, sizeof(sCommand));
  sCommand.Instruction = CMD_ENTER_DPD;
  sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = QSPI_DATA_NONE;
  sCommand.DummyCycles = 0;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  HAL_StatusTypeDef status =
      HAL_QSPI_Command(&hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
  if (status != HAL_OK) {
    LOG_WRN("PSRAM DPD command failed: %d", status);
  }

  PSRAM_UNLOCK();

  /* Wait tDPDIN (min 150 us transition time) */
  k_busy_wait(150);

  /* Ensure CE# (PB11) is configured as GPIO pull-up during sleep so it never floats */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  LOG_INF("PSRAM entered Deep Power Down (CMD 0xB9, CE# high)");
  return 0;
}

int psram_wake(void) {
  if (!psram_present || !psram_initialized) {
    return 0;
  }

  /* Reconfigure PB11 as QUADSPI AF10 with pull-up */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_11, LL_GPIO_AF_10);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  /* Drive CE# low momentarily to wake from DPD (tDPDX) */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);
  k_busy_wait(10); /* 10 us pulse low */
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);

  /* Wait tDPDOUT (min 150 us) for device to exit DPD */
  k_busy_wait(200);

  /* Verify PSRAM ID */
  struct psram_id id;
  int ret = psram_read_id(&id);
  if (ret == 0 && id.mfid == PSRAM_ISSI_MFID) {
    LOG_INF("PSRAM successfully woken from Deep Power Down (MFID 0x%02X)",
            id.mfid);
    return 0;
  } else {
    LOG_WRN("PSRAM wake ID check warning (code %d, MFID 0x%02X)", ret, id.mfid);
    return ret;
  }
}

