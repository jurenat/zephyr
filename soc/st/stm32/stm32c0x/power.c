/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <clock_control/clock_stm32_ll_common.h>
#include <soc.h>

#include <stm32c0xx_ll_bus.h>
#include <stm32c0xx_ll_cortex.h>
#include <stm32c0xx_ll_pwr.h>
#include <stm32c0xx.h>

#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/interrupt_controller/gic.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/init.h>

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

BUILD_ASSERT(DT_SAME_NODE(DT_CHOSEN(zephyr_cortex_m_idle_timer), DT_NODELABEL(rtc)),
		"STM32C0x series needs RTC as an additional IDLE timer for power management");

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		LL_LPM_DisableEventOnPend();
		LL_PWR_ClearFlag_WU();

		LL_PWR_SetPowerMode(LL_PWR_MODE_STOP0);
		LL_LPM_EnableDeepSleep();

		k_cpu_idle();

		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
	case PM_STATE_SUSPEND_TO_IDLE:
		LL_LPM_DisableSleepOnExit();
		LL_LPM_EnableSleep();

		/* Restore the clock setup. */
		stm32_clock_control_init(NULL);
		break;
	default:
		LOG_DBG("Unsupported power substate-id %u", state);
		break;
	}

	/*
	 * System is now in active mode. Reenable interrupts which were
	 * disabled when OS started idling code.
	 */
	irq_unlock(0);
}

void stm32_power_init(void)
{
	/* Enable Power clock. */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
}
