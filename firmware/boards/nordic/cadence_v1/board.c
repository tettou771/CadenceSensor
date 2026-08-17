/*
 * Board bring-up that has to happen before anything else, in BOTH images.
 *
 * This file is linked into MCUboot as well as the application, which on this
 * PCB is the whole point rather than an accident: the board switches its own
 * 3.3 V supply, and MCUboot runs for up to ~20 s swapping an image in. Nothing
 * in MCUboot would otherwise hold the supply on for that long.
 *
 * Two jobs, in this order:
 *
 *   1. hold the power latch, so the board survives its own reboot
 *   2. force UICR REGOUT0 to 3.3 V, so the IMU sees a legal VDD_IO
 *
 * The order is not negotiable. Step 2 ends in sys_reboot() on a blank chip, and
 * a reboot with the latch not held is a power-off.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/devicetree.h>
/* For GPIO_ACTIVE_LOW in the assertion below. The dt-bindings header rather
 * than drivers/gpio.h: this file drives the pin through the HAL and has no
 * business pulling the GPIO driver API into the bootloader. */
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_power.h>
#include <hal/nrf_nvmc.h>
#include <hal/nrf_gpio.h>
#include <nrfx.h>

#define ZUSER DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZUSER, soft_latch_gpios)

/* Absolute nRF pin number (32 * port + pin), taken from the devicetree rather
 * than written out, so this file cannot drift from cadence_v1.dts. */
#define SOFT_LATCH_PIN                                                        \
	NRF_GPIO_PIN_MAP(DT_PROP(DT_GPIO_CTLR(ZUSER, soft_latch_gpios), port), \
			 DT_GPIO_PIN(ZUSER, soft_latch_gpios))

BUILD_ASSERT((DT_GPIO_FLAGS(ZUSER, soft_latch_gpios) & GPIO_ACTIVE_LOW) == 0,
	     "soft-latch-gpios is ACTIVE_LOW but this asserts it by driving "
	     "the pin high - a board wired the other way would power itself "
	     "off here instead of on");

/* Hold the supply, unconditionally and as early as possible.
 *
 * Reset turns every GPIO back into an input, so from the instant a reset
 * begins the only thing holding the LDO's enable pin up is the charge on it:
 * roughly 10 pF against the LDO's own ~10 nA of enable-pin leakage, which is
 * single-digit milliseconds. That is enough to coast through the reset itself
 * and into this function, and nothing like enough to cross MCUboot's image
 * swap. So the latch has to be re-asserted here, in whichever image booted.
 *
 * Deliberately unconditional, with no attempt to decide whether being powered
 * on is JUSTIFIED. That decision lives in exactly one place -- main()'s
 * power_latch_init() -- and duplicating it in the bootloader is how the two
 * copies would come to disagree. A board woken by ESD therefore boots, gets as
 * far as the application, finds neither a button nor VBUS, and switches itself
 * off, having spent one boot's worth of charge (~0.4 uAh) doing so.
 *
 * Raw HAL rather than the GPIO driver: this must not depend on a device being
 * initialised, and MCUboot is entitled to configure fewer drivers than the
 * application does.
 */
static void board_power_latch_hold(void)
{
	nrf_gpio_cfg_output(SOFT_LATCH_PIN);
	nrf_gpio_pin_set(SOFT_LATCH_PIN);
}

#else
static void board_power_latch_hold(void) { }
#endif

/* Returns without doing anything on a chip that is already set, which is every
 * boot after the first. The one that does act ends in a reboot. */
static void board_regout0_init(void)
{
	uint32_t regout0 = NRF_UICR->REGOUT0;

	if ((regout0 & UICR_REGOUT0_VOUT_Msk) ==
	    (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos)) {
		return;
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
}

/* One entry point rather than two SYS_INITs, so the ordering between them is
 * written down instead of being a matter of link order and equal priorities.
 * The latch must be held before board_regout0_init() can reach its reboot. */
static int board_init(void)
{
	board_power_latch_hold();
	board_regout0_init();

	return 0;
}

SYS_INIT(board_init, PRE_KERNEL_1, 0);
