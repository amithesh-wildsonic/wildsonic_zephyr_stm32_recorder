/* SPDX-License-Identifier: Apache-2.0 */
#include "retained_reboot.h"
#include <string.h>

#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#include <stm32l4xx_ll_pwr.h>
#include <stm32l4xx_ll_rcc.h>
#include <stm32l4xx_ll_rtc.h>

#define REBOOT_BKP_MAGIC 0x5242544EU /* "RBTN" */

#define BKP_REG_MAGIC   LL_RTC_BKP_DR9
#define BKP_REG_REASON  LL_RTC_BKP_DR10
#define BKP_REG_ASSERT_ID   LL_RTC_BKP_DR11
#define BKP_REG_ASSERT_LINE LL_RTC_BKP_DR12
#define BKP_REG_FILE_0      LL_RTC_BKP_DR13
#define BKP_REG_FILE_1      LL_RTC_BKP_DR14

static inline void bkp_unlock(void)
{
	if (!LL_PWR_IsEnabledBkUpAccess()) {
		LL_PWR_EnableBkUpAccess();
	}
}

void retained_reboot_init(void)
{
	bkp_unlock();
}

void retained_reboot_set_reason(reboot_reason_t reason)
{
	bkp_unlock();
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_MAGIC, REBOOT_BKP_MAGIC);
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_REASON, (uint32_t)reason);
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_ASSERT_ID, 0);
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_ASSERT_LINE, 0);
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_FILE_0, 0);
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_FILE_1, 0);
}

void retained_reboot_set_assert(uint32_t id, const char *file, uint32_t line)
{
	bkp_unlock();
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_MAGIC, REBOOT_BKP_MAGIC);
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_REASON, (uint32_t)REBOOT_REASON_ASSERT);
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_ASSERT_ID, id);
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_ASSERT_LINE, line);

	uint32_t f0 = 0, f1 = 0;
	if (file) {
		const char *slash = strrchr(file, '/');
		const char *base = slash ? slash + 1 : file;
		size_t blen = strlen(base);
		memcpy(&f0, base, blen < 4 ? blen : 4);
		if (blen > 4) {
			memcpy(&f1, base + 4, (blen - 4) < 4 ? (blen - 4) : 4);
		}
	}
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_FILE_0, f0);
	LL_RTC_BAK_SetRegister(RTC, BKP_REG_FILE_1, f1);
}

reboot_reason_t retained_reboot_get(struct retained_reboot_info *out_info)
{
	bkp_unlock();
	uint32_t magic = LL_RTC_BAK_GetRegister(RTC, BKP_REG_MAGIC);
	reboot_reason_t res = REBOOT_REASON_UNKNOWN;

	if (out_info) {
		memset(out_info, 0, sizeof(*out_info));
	}

	if (magic == REBOOT_BKP_MAGIC) {
		res = (reboot_reason_t)LL_RTC_BAK_GetRegister(RTC, BKP_REG_REASON);
		if (out_info) {
			out_info->reason = res;
			out_info->assert_id = LL_RTC_BAK_GetRegister(RTC, BKP_REG_ASSERT_ID);
			out_info->assert_line = LL_RTC_BAK_GetRegister(RTC, BKP_REG_ASSERT_LINE);
			uint32_t f0 = LL_RTC_BAK_GetRegister(RTC, BKP_REG_FILE_0);
			uint32_t f1 = LL_RTC_BAK_GetRegister(RTC, BKP_REG_FILE_1);
			memcpy(out_info->assert_file, &f0, 4);
			memcpy(out_info->assert_file + 4, &f1, 4);
			out_info->assert_file[8] = '\0';
		}
		/* Clear magic after reading so it is single-shot */
		LL_RTC_BAK_SetRegister(RTC, BKP_REG_MAGIC, 0);
		return res;
	}

	/* Fall back to hardware CSR flags */
	uint32_t csr = RCC->CSR;
	if (csr & RCC_CSR_IWDGRSTF || csr & RCC_CSR_WWDGRSTF) {
		res = REBOOT_REASON_WDT;
	} else if (csr & RCC_CSR_SFTRSTF) {
		res = REBOOT_REASON_SOFTWARE_RESET;
	} else if (csr & RCC_CSR_PINRSTF) {
		res = REBOOT_REASON_PIN_RESET;
	} else if (csr & RCC_CSR_BORRSTF) {
		res = REBOOT_REASON_POWER_ON_RESET;
	} else {
		res = REBOOT_REASON_UNKNOWN;
	}

	if (out_info) {
		out_info->reason = res;
	}
	return res;
}

#else

static reboot_reason_t s_host_reason = REBOOT_REASON_UNKNOWN;
static struct retained_reboot_info s_host_info;

void retained_reboot_init(void) {}
void retained_reboot_set_reason(reboot_reason_t reason)
{
	s_host_reason = reason;
	s_host_info.reason = reason;
}

void retained_reboot_set_assert(uint32_t id, const char *file, uint32_t line)
{
	s_host_reason = REBOOT_REASON_ASSERT;
	s_host_info.reason = REBOOT_REASON_ASSERT;
	s_host_info.assert_id = id;
	s_host_info.assert_line = line;
	if (file) {
		strncpy(s_host_info.assert_file, file, sizeof(s_host_info.assert_file) - 1);
	}
}

reboot_reason_t retained_reboot_get(struct retained_reboot_info *out_info)
{
	if (out_info) {
		*out_info = s_host_info;
	}
	return s_host_reason;
}

#endif

const char *retained_reboot_reason_str(reboot_reason_t reason)
{
	switch (reason) {
	case REBOOT_REASON_USER_RBT:
		return "USER_RBT";
	case REBOOT_REASON_USER_ARBT:
		return "USER_ARBT";
	case REBOOT_REASON_ASSERT:
		return "ASSERT";
	case REBOOT_REASON_WDT:
		return "WDT";
	case REBOOT_REASON_SLEEP_FAIL:
		return "SLEEP_FAIL";
	case REBOOT_REASON_STORAGE_FAIL:
		return "STORAGE_FAIL";
	case REBOOT_REASON_PIN_RESET:
		return "PIN_RESET";
	case REBOOT_REASON_POWER_ON_RESET:
		return "POWER_ON_RESET";
	case REBOOT_REASON_SOFTWARE_RESET:
		return "SOFTWARE_RESET";
	case REBOOT_REASON_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}
