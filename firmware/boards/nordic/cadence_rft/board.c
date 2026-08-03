/*
 * Force UICR REGOUT0 to 3.3 V for the external peripherals (IMU, LED).
 *
 * A blank nRF52840 (erased UICR = 0xFF) drives its GPIOs at 1.8 V, which is
 * below the LSM6DSV's VDD_IO minimum and too low to sink the common-anode LED.
 * This runs once per chip; UICR survives reboots but NOT a mass erase, so
 * expect exactly one extra reboot after tools/flash.command unlocks APPROTECT.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_power.h>
#include <hal/nrf_nvmc.h>
#include <nrfx.h>

static int board_regout0_init(void)
{
	uint32_t regout0 = NRF_UICR->REGOUT0;

	if ((regout0 & UICR_REGOUT0_VOUT_Msk) ==
	    (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos)) {
		return 0;
	}

	uint32_t desired = (regout0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
			   (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);

	/* UICR is flash: bits can only go 1->0. If any bit needs 0->1, erase
	 * the whole UICR page first to reset every bit to 1. Skipping this is
	 * what produces the classic "blinks every 1.5 s forever" boot loop. */
	if ((regout0 & desired) != desired) {
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
		}
		NRF_NVMC->ERASEUICR = 1;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
		}
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;

		desired = ~((uint32_t)UICR_REGOUT0_VOUT_Msk) |
			  (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);
	}

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
	NRF_UICR->REGOUT0 = desired;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {
	}
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;

	sys_reboot(SYS_REBOOT_COLD);

	return 0;
}

SYS_INIT(board_regout0_init, PRE_KERNEL_1, 0);
