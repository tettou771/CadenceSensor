/*
 * Battery voltage over the nRF52840 SAADC — see battery.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "battery.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#define ZUSER DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZUSER, io_channels)

static const struct adc_dt_spec batt_adc = ADC_DT_SPEC_GET(ZUSER);

/* The divider between the pack and the ADC pin, as a multiplier back to the
 * real voltage. Taken from the devicetree so the board file stays the single
 * place the hardware is described. */
#define DIVIDER_NUM DT_PROP(ZUSER, battery_divider_num)
#define DIVIDER_DEN DT_PROP(ZUSER, battery_divider_den)

/* Averaged, because this is a high-impedance node. The divider is 1 Mohm over
 * 1 Mohm, i.e. 500 kohm of source impedance, which the SAADC can track at the
 * 40 us acquisition time the board file asks for but with no margin to spare.
 * Eight samples cost about 300 us and take the ADC's own noise out of a number
 * a rider will read as a percentage. */
#define SAMPLES 8

static bool ready;

int battery_init(void)
{
	if (!adc_is_ready_dt(&batt_adc)) {
		LOG_ERR("ADC not ready");
		return -ENODEV;
	}

	int err = adc_channel_setup_dt(&batt_adc);

	if (err) {
		LOG_ERR("adc_channel_setup: %d", err);
		return err;
	}

	ready = true;

	return 0;
}

bool battery_present(void)
{
	return ready;
}

/* Open-circuit voltage against state of charge for a single lithium-polymer
 * cell, descending. Interpolated linearly between neighbours.
 *
 * The shape is the point of the table. Nearly two thirds of the usable charge
 * sits in the 200 mV between 3.9 and 3.7 V, so the curve is deliberately dense
 * there and coarse at both ends, where a small voltage change means very
 * little (top) or the cell is about to cut out anyway (bottom). */
static const struct {
	uint16_t mv;
	uint8_t pct;
} curve[] = {
	{4200, 100}, {4100, 90}, {4020, 80}, {3960, 70}, {3900, 60},
	{3850, 50}, {3810, 40}, {3780, 30}, {3750, 20}, {3700, 10},
	{3600,   5}, {3300,  0},
};

static uint8_t mv_to_percent(uint16_t mv)
{
	if (mv >= curve[0].mv) {
		return 100;
	}

	for (size_t i = 1; i < ARRAY_SIZE(curve); i++) {
		if (mv >= curve[i].mv) {
			const uint16_t span = curve[i - 1].mv - curve[i].mv;
			const uint8_t rise = curve[i - 1].pct - curve[i].pct;

			return (uint8_t)(curve[i].pct +
					 ((uint32_t)(mv - curve[i].mv) * rise) /
						 span);
		}
	}

	return 0;
}

int battery_read(uint16_t *mv_out, uint8_t *pct_out)
{
	int16_t buf;
	struct adc_sequence seq = {
		.buffer = &buf,
		.buffer_size = sizeof(buf),
	};
	int32_t total = 0;

	if (!ready) {
		return -ENODEV;
	}

	(void)adc_sequence_init_dt(&batt_adc, &seq);

	for (int i = 0; i < SAMPLES; i++) {
		int err = adc_read_dt(&batt_adc, &seq);

		if (err) {
			LOG_WRN("adc_read: %d", err);
			return err;
		}

		int32_t val = buf;

		/* Converts in place, using the gain and reference the board
		 * file declared, so none of that arithmetic is repeated here
		 * and cannot disagree with the devicetree. */
		err = adc_raw_to_millivolts_dt(&batt_adc, &val);
		if (err) {
			LOG_WRN("adc_raw_to_millivolts: %d", err);
			return err;
		}

		total += val;
	}

	uint32_t pin_mv = (uint32_t)total / SAMPLES;
	uint32_t batt_mv = pin_mv * DIVIDER_NUM / DIVIDER_DEN;

	if (mv_out) {
		*mv_out = (uint16_t)batt_mv;
	}
	if (pct_out) {
		*pct_out = mv_to_percent((uint16_t)batt_mv);
	}

	return 0;
}

#else /* no io-channels: this board cannot see its own battery */

int battery_init(void)
{
	return -ENODEV;
}

bool battery_present(void)
{
	return false;
}

int battery_read(uint16_t *mv_out, uint8_t *pct_out)
{
	ARG_UNUSED(mv_out);
	ARG_UNUSED(pct_out);

	return -ENODEV;
}

#endif
