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

/* Terminal voltage against state of charge for a single lithium-polymer cell,
 * descending. Interpolated linearly between neighbours.
 *
 * MEASURED, not assumed. These points come from a full discharge of an EEMB
 * 150 mAh cell of the same class on another board in this workshop, logged over
 * 12.342 h until cutoff, and shared by the tdk-viewer session. The percentages
 * are that run's remaining time as a fraction of the whole, which is the same
 * thing as remaining charge because the load never changed.
 *
 * What it replaced was a table written from general knowledge of lithium
 * chemistry, and it was wrong in the worst possible place: it put 3700 mV at
 * 10 % where the real cell still had 60 %. The flattest part of the curve had
 * been given the steepest slope, so a pack a little over half full reported
 * itself nearly dead - and a drain being estimated from the reported figure
 * came out more than twice its real size.
 *
 * The other board discharged at roughly C/12 against this one's C/350, which
 * does not matter here: 12 mA through a cell of 200-400 mohm is 2-5 mV of IR
 * drop, so both curves are open-circuit voltage in all but name. Its ABSOLUTE
 * RATE is not transferable, only the shape.
 *
 * The 3600 mV point is the least certain of these - it is the flattest part of
 * the curve, where the source data's two derivations disagreed by 5 % and a
 * 20 mV measurement wobble moves the answer by several points. Everything else
 * agreed to within 1.5 %.
 */
static const struct {
	uint16_t mv;
	uint8_t pct;
} curve[] = {
	{4180, 100}, {4100, 98}, {4050, 94}, {4000, 90}, {3950, 84},
	{3900,  80}, {3850, 75}, {3800, 70}, {3750, 65}, {3700, 60},
	{3650,  54}, {3600, 45}, {3500, 19}, {3450, 12}, {3400,  7},
	{3300,   3}, {3000,  0},
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
