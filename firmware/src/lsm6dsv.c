/*
 * LSM6DSV minimal SPI driver.
 *
 * Register map follows the LSM6DSV / LSM6DSV16B datasheet (ST DS13773 and
 * friends). Note this family MOVED the interface-control bits out of CTRL3_C
 * into a dedicated IF_CFG register compared to the older LSM6DSO — if you
 * cross-check against LSM6DSO code, expect that difference.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "lsm6dsv.h"

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(lsm6dsv, LOG_LEVEL_INF);

/* --- register map --- */
#define REG_IF_CFG           0x03
#define REG_WHO_AM_I         0x0f
#define REG_CTRL1            0x10 /* OP_MODE_XL[6:4] | ODR_XL[3:0] */
#define REG_CTRL2            0x11 /* OP_MODE_G[6:4]  | ODR_G[3:0]  */
#define REG_CTRL3            0x12 /* BOOT | BDU | ... | IF_INC | SW_RESET */
#define REG_CTRL8            0x17 /* HP_LPF2_XL_BW[7:5] | FS_XL[1:0] */
#define REG_STATUS           0x1e
/* First of the six accelerometer output bytes. Deliberately NOT named
 * OUTX_L_A: on the LSM6DSV16B this address is OUTZ_L_A and the three axes run
 * backwards from here. See lsm6dsv_read_accel_raw(). */
#define REG_ACCEL_OUT        0x28
#define REG_WAKE_UP_SRC      0x45
#define REG_D6D_SRC          0x47
#define REG_FUNCTIONS_ENABLE 0x50
#define REG_INACTIVITY_DUR   0x54 /* threshold WEIGHT + inactivity XL rate */
#define REG_INACTIVITY_THS   0x55
#define REG_TAP_CFG0         0x56
#define REG_TAP_THS_6D       0x59
#define REG_WAKE_UP_THS      0x5b
#define REG_WAKE_UP_DUR      0x5c
#define REG_MD1_CFG          0x5e

#define IF_CFG_I2C_I3C_DISABLE BIT(0)

#define CTRL3_SW_RESET BIT(0)
#define CTRL3_IF_INC   BIT(2)
#define CTRL3_BDU      BIT(6)
#define CTRL3_BOOT     BIT(7)

/* CTRL1 ODR_XL codes */
#define ODR_XL_OFF   0x0
#define ODR_XL_15HZ  0x3
#define ODR_XL_30HZ  0x4
#define ODR_XL_60HZ  0x5
#define ODR_XL_120HZ 0x6

/* CTRL1 OP_MODE_XL codes */
#define OP_MODE_XL_HIGH_PERF 0x0
#define OP_MODE_XL_LP1       0x4 /* low power, 1x averaging */

/* CTRL8 FS_XL codes */
#define FS_XL_2G  0x0
#define FS_XL_4G  0x1
#define FS_XL_8G  0x2
#define FS_XL_16G 0x3

/* Full scale used by this driver, and the matching sensitivity. The whole
 * family scales as 0.061 mg/LSB at +/-2 g and doubles per range step, so
 * +/-8 g is 0.244 mg/LSB.
 *
 * +/-8 g and not +/-4 g because of what a crank arm actually does to a sensor
 * bolted to it. Centripetal acceleration is w^2 * r, and at 120 rpm with the
 * sensor 170 mm out from the spindle that is 2.7 g on its own, before the 1 g
 * of gravity the detector is actually looking for. 150 rpm is 4.2 g. A part
 * clipping at 4 g would flat-top exactly when the rider is working hardest.
 * The cost is resolution, and there is plenty to spare: 0.244 mg/LSB puts the
 * 1 g signal at 4096 counts. */
#define FS_XL_SEL     FS_XL_8G
#define FS_FULLSCALE_MG 8000
#define UG_PER_LSB    LSM6DSV_UG_PER_LSB /* micro-g per LSB at +/-8 g */

/* WAKE_UP_DUR. Bits 5-6 are WAKE_DUR; bit 4 is RESERVED on this family.
 *
 * The LSM6DSO generation had a WAKE_THS_W bit here that rescaled the wake-up
 * threshold. It does NOT exist on the LSM6DSV — the weight moved to
 * INACTIVITY_DUR (0x54) and changed meaning entirely (see below). Writing bit 4
 * here does nothing useful and was a real bug in this driver until the register
 * map was checked against ST's own lsm6dsv16b_reg.h. */
#define WAKE_UP_DUR_WAKE_DUR_POS 5

/* INACTIVITY_DUR (0x54) bits 4-6: the wake-up threshold weight.
 *
 * The other generational change, and the one most likely to bite: on the
 * LSM6DSV the threshold LSB is an ABSOLUTE value in mg and does not scale with
 * the accelerometer full scale at all. On the LSM6DSO family it was FS/2^6 or
 * FS/2^8, so the same register value meant different things at different
 * ranges. Values below are from ST's lsm6dsv16b_act_thresholds_set(). */
#define INACT_DUR_THS_W_POS 4
#define INACT_DUR_THS_W_MSK (0x7 << INACT_DUR_THS_W_POS)

/* micro-g per LSB for weights 0..5 */
static const uint32_t wake_ths_ug_per_lsb[] = {
	7813, 15625, 31250, 62500, 125000, 250000,
};

/* INACTIVITY_DUR bits 2-3: the accelerometer rate the part falls back to once
 * it decides nothing is moving. 0 is the slowest it offers. */
#define INACT_DUR_XL_ODR_POS 2
#define INACT_DUR_XL_ODR_MSK (0x3 << INACT_DUR_XL_ODR_POS)
#define INACT_XL_ODR_1HZ875  0
#define INACT_XL_ODR_15HZ    1
#define INACT_XL_ODR_30HZ    2
#define INACT_XL_ODR_60HZ    3

/* INACTIVITY_DUR bits 0-1: how many samples of motion are needed to leave the
 * inactive state. Despite the field name this is the SLEEP-TO-ACTIVE duration,
 * not the time taken to fall asleep — that one is SLEEP_DUR in WAKE_UP_DUR. */
#define INACT_DUR_SLEEP_TO_ACT_MSK 0x3

/* WAKE_UP_DUR bits 0-3: how long without motion before the part idles down.
 * 1 LSB = 512 / ODR_XL, so the step is 8.5 s at 60 Hz and the field tops out
 * a little over two minutes. */
#define WAKE_UP_DUR_SLEEP_DUR_MSK 0x0f
#define SLEEP_DUR_LSB_SAMPLES     512

/* TAP_THS_6D bits 5-6: how far gravity has to move to count as a new
 * orientation. */
#define TAP_THS_6D_SIXD_THS_POS 5
#define TAP_THS_6D_SIXD_THS_MSK (0x3 << TAP_THS_6D_SIXD_THS_POS)

/* TAP_CFG0 */
#define TAP_CFG0_LIR            BIT(0)
#define TAP_CFG0_SLOPE_FDS      BIT(4) /* high-pass filter feeds activity  */
#define TAP_CFG0_LOW_PASS_ON_6D BIT(6) /* low-pass filter feeds 6D         */

/* FUNCTIONS_ENABLE (0x50).
 *
 * INTERRUPTS_ENABLE is bit 7. It was bit 3 in this driver, which is
 * DIS_RST_LIR_ALL_INT — a completely different function — so the interrupt
 * engine was never switched on and INT1 never fired even while the detector
 * was happily counting revolutions. Verified against ST's register header. */
#define FUNC_EN_INTERRUPTS_ENABLE   BIT(7)
#define FUNC_EN_DIS_RST_LIR_ALL_INT BIT(3)

/* FUNCTIONS_ENABLE bits 0-1: what the part does to itself on inactivity.
 * 1 = drop the accelerometer to the inactivity rate, leave the gyro alone
 * (which costs nothing here — the gyro is powered down anyway). */
#define FUNC_EN_INACT_MSK            0x3
#define FUNC_EN_INACT_XL_LOW_POWER   0x1

/* MD1_CFG */
#define MD1_CFG_INT1_6D BIT(2)
#define MD1_CFG_INT1_WU BIT(5)

#define SPI_READ_BIT 0x80

/* Map the Hz in the header onto the CTRL1 code. Kept as a build-time check so
 * changing LSM6DSV_ODR_HZ to something the part cannot do fails the build
 * rather than silently landing on power-down. */
#if LSM6DSV_ODR_HZ == 15
#define ODR_XL_SEL ODR_XL_15HZ
#elif LSM6DSV_ODR_HZ == 30
#define ODR_XL_SEL ODR_XL_30HZ
#elif LSM6DSV_ODR_HZ == 60
#define ODR_XL_SEL ODR_XL_60HZ
#elif LSM6DSV_ODR_HZ == 120
#define ODR_XL_SEL ODR_XL_120HZ
#else
#error "LSM6DSV_ODR_HZ has no CTRL1 code here — add one from the datasheet"
#endif

static const struct spi_dt_spec imu = SPI_DT_SPEC_GET(
	DT_NODELABEL(lsm6dsv),
	SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER, 0);

static bool present;
static uint8_t whoami;

int lsm6dsv_read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t tx[1 + 8] = {0};
	uint8_t rx[1 + 8] = {0};

	if (len == 0 || len > 8) {
		return -EINVAL;
	}

	tx[0] = reg | SPI_READ_BIT;

	const struct spi_buf tx_buf = {.buf = tx, .len = 1 + len};
	const struct spi_buf rx_buf = {.buf = rx, .len = 1 + len};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

	int err = spi_transceive_dt(&imu, &tx_set, &rx_set);

	if (err) {
		return err;
	}

	memcpy(buf, &rx[1], len);

	return 0;
}

int lsm6dsv_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t tx[2] = {reg, val};

	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};

	return spi_write_dt(&imu, &tx_set);
}

static int modify_reg(uint8_t reg, uint8_t clear, uint8_t set)
{
	uint8_t val;
	int err = lsm6dsv_read_reg(reg, &val, 1);

	if (err) {
		return err;
	}

	val = (uint8_t)((val & ~clear) | set);

	return lsm6dsv_write_reg(reg, val);
}

bool lsm6dsv_present(void)
{
	return present;
}

int lsm6dsv_init(void)
{
	uint8_t id = 0;
	int err;

	present = false;

	if (!spi_is_ready_dt(&imu)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* SPI-only wiring: SDO doubles as MISO, so leaving the I2C/I3C
	 * interface enabled lets bus noise be interpreted as I3C traffic. */
	(void)lsm6dsv_write_reg(REG_IF_CFG, IF_CFG_I2C_I3C_DISABLE);

	err = lsm6dsv_write_reg(REG_CTRL3, CTRL3_SW_RESET);
	if (err) {
		return err;
	}
	k_msleep(20);

	err = lsm6dsv_read_reg(REG_WHO_AM_I, &id, 1);
	if (err) {
		return err;
	}

	/* Kept even when the value is not one we know, so the console can say
	 * WHICH way this failed rather than only that it did. */
	whoami = id;

	if (id != LSM6DSV_WHOAMI && id != LSM6DSV16B_WHOAMI &&
	    id != LSM6DSV320X_WHOAMI) {
		LOG_ERR("WHO_AM_I 0x%02x — no LSM6DSV here (expected 0x70, "
			"0x71 or 0x73)", id);
		return -ENODEV;
	}

	/* BDU so a 16-bit sample is never torn across an update, IF_INC so
	 * burst reads walk the register file. BDU matters more here than it
	 * did for the tag: the cadence detector reads all six output bytes
	 * asynchronously to the ODR, so without it a sample straddling an
	 * update would put a garbage angle into the unwrapper. */
	err = lsm6dsv_write_reg(REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC);
	if (err) {
		return err;
	}

	err = modify_reg(REG_CTRL8, 0x03, FS_XL_SEL);
	if (err) {
		return err;
	}

	/* Accelerometer: low-power mode 1 at LSM6DSV_ODR_HZ. Gyro stays off —
	 * CTRL2 untouched after the reset leaves ODR_G = power-down. */
	err = lsm6dsv_write_reg(REG_CTRL1,
				(OP_MODE_XL_LP1 << 4) | ODR_XL_SEL);
	if (err) {
		return err;
	}

	present = true;
	LOG_INF("LSM6DSV ready (WHO_AM_I 0x%02x, +/-8 g, %d Hz LP)", id,
		LSM6DSV_ODR_HZ);

	return 0;
}

int lsm6dsv_read_accel_raw(int16_t out[3])
{
	uint8_t buf[6];
	int err;

	if (!present) {
		return -ENODEV;
	}

	err = lsm6dsv_read_reg(REG_ACCEL_OUT, buf, sizeof(buf));
	if (err) {
		return err;
	}

	/* The LSM6DSV16B lays its accelerometer outputs out BACKWARDS: 0x28 is
	 * OUTZ_L_A on that part and OUTX_L_A on every other one in the family
	 * (0x70 plain/16X, 0x73 320X/80X — checked against ST's headers). The
	 * same six-byte burst therefore arrives Z,Y,X on a 16B, so put it back
	 * here rather than leave callers to work out which part they have.
	 *
	 * Nothing in the cadence path ever noticed, which is exactly why this
	 * went unseen through a real ride: the detector finds the spindle axis
	 * by looking for the one carrying the least AC energy, so ANY
	 * permutation of the three axes gives the same answer. Something that
	 * cares which way is up — a body or racket tracker on this same board —
	 * would not be so forgiving. */
	const bool reversed = (whoami == LSM6DSV16B_WHOAMI);

	for (int i = 0; i < 3; i++) {
		int w = reversed ? 2 - i : i;

		out[i] = (int16_t)((uint16_t)buf[w * 2] |
				   ((uint16_t)buf[w * 2 + 1] << 8));
	}

	return 0;
}

uint8_t lsm6dsv_whoami(void)
{
	return whoami;
}

int lsm6dsv_read_accel_mg(int16_t out[3])
{
	int16_t raw[3];
	int err = lsm6dsv_read_accel_raw(raw);

	if (err) {
		return err;
	}

	for (int i = 0; i < 3; i++) {
		out[i] = (int16_t)(((int32_t)raw[i] * UG_PER_LSB) / 1000);
	}

	return 0;
}

/* Pick the finest threshold weight that can still express `mg` in the 6-bit
 * field, the way ST's own driver does, and return the register code.
 * *weight_out gets the INACTIVITY_DUR weight selector. */
static uint8_t threshold_code(uint16_t mg, uint8_t *weight_out)
{
	uint8_t weight = ARRAY_SIZE(wake_ths_ug_per_lsb) - 1;
	uint32_t code = 0x3f;

	for (uint8_t w = 0; w < ARRAY_SIZE(wake_ths_ug_per_lsb); w++) {
		uint32_t c = ((uint32_t)mg * 1000U) / wake_ths_ug_per_lsb[w];

		if (c <= 0x3f) {
			weight = w;
			code = c;
			break;
		}
	}

	if (code < 1) {
		code = 1;
	}
	*weight_out = weight;

	return (uint8_t)code;
}

int lsm6dsv_arm_orientation_wake(enum lsm6dsv_6d_deg deg, uint16_t inact_ths_mg,
				 uint16_t quiet_s)
{
	uint8_t weight;
	uint8_t inact_ths;
	uint8_t sleep_dur;
	int err;

	if (!present) {
		return -ENODEV;
	}

	inact_ths = threshold_code(inact_ths_mg, &weight);

	/* 1 LSB = 512/ODR, so the code is quiet_s in units of 512 samples. */
	uint32_t dur = ((uint32_t)quiet_s * LSM6DSV_ODR_HZ) / SLEEP_DUR_LSB_SAMPLES;

	if (dur > WAKE_UP_DUR_SLEEP_DUR_MSK) {
		dur = WAKE_UP_DUR_SLEEP_DUR_MSK;
	}
	sleep_dur = (uint8_t)dur;

	/* 6D wants the LOW-passed signal: it is measuring where gravity points,
	 * so the DC component IS the signal. The activity detector immediately
	 * below wants the opposite — high-passed, so that gravity does not read
	 * as permanent motion and the part never decides it is inactive. The two
	 * filter choices are independent bits precisely because the two
	 * functions need opposite things. LIR left clear: a pulsed interrupt
	 * needs no acknowledgement to re-arm. */
	err = modify_reg(REG_TAP_CFG0, TAP_CFG0_LIR,
			 TAP_CFG0_SLOPE_FDS | TAP_CFG0_LOW_PASS_ON_6D);
	if (err) {
		return err;
	}

	err = modify_reg(REG_TAP_THS_6D, TAP_THS_6D_SIXD_THS_MSK,
			 (uint8_t)((uint8_t)deg << TAP_THS_6D_SIXD_THS_POS));
	if (err) {
		return err;
	}

	/* Weight, the rate to fall back to, and how promptly to come out of it. */
	err = modify_reg(REG_INACTIVITY_DUR,
			 INACT_DUR_THS_W_MSK | INACT_DUR_XL_ODR_MSK |
				 INACT_DUR_SLEEP_TO_ACT_MSK,
			 (uint8_t)((weight << INACT_DUR_THS_W_POS) |
				   (INACT_XL_ODR_1HZ875 << INACT_DUR_XL_ODR_POS) |
				   0 /* leave inactivity on the first sample */));
	if (err) {
		return err;
	}

	err = lsm6dsv_write_reg(REG_INACTIVITY_THS, inact_ths);
	if (err) {
		return err;
	}

	err = modify_reg(REG_WAKE_UP_DUR, WAKE_UP_DUR_SLEEP_DUR_MSK, sleep_dur);
	if (err) {
		return err;
	}

	/* Master enable for the interrupt engine, plus "drop the accelerometer
	 * rate when inactive". */
	err = modify_reg(REG_FUNCTIONS_ENABLE,
			 FUNC_EN_DIS_RST_LIR_ALL_INT | FUNC_EN_INACT_MSK,
			 FUNC_EN_INTERRUPTS_ENABLE | FUNC_EN_INACT_XL_LOW_POWER);
	if (err) {
		return err;
	}

	/* Route 6D — and ONLY 6D — to INT1. Wake-up is explicitly cleared: it
	 * fires on vibration, which is the thing this design is trying not to
	 * be woken by. */
	err = modify_reg(REG_MD1_CFG, MD1_CFG_INT1_WU, MD1_CFG_INT1_6D);
	if (err) {
		return err;
	}

	/* Read back what actually landed.
	 *
	 * This matters more than it looks. The sampler no longer wakes itself
	 * periodically, so INT1 is the only thing that starts cadence counting:
	 * a mis-set bit here is not a small inefficiency, it is a sensor that
	 * never reports at all. Two such bits were found in this very routine
	 * (FUNCTIONS_ENABLE bit 3 instead of 7, and a WAKE_UP_DUR bit that does
	 * not exist on this part), and the symptom was an arming routine
	 * cheerfully returning success. Never again. */
	uint8_t fen = 0, md1 = 0, sixd = 0, idur = 0;

	(void)lsm6dsv_read_reg(REG_FUNCTIONS_ENABLE, &fen, 1);
	(void)lsm6dsv_read_reg(REG_MD1_CFG, &md1, 1);
	(void)lsm6dsv_read_reg(REG_TAP_THS_6D, &sixd, 1);
	(void)lsm6dsv_read_reg(REG_INACTIVITY_DUR, &idur, 1);

	bool ok = (fen & FUNC_EN_INTERRUPTS_ENABLE) &&
		  ((fen & FUNC_EN_INACT_MSK) == FUNC_EN_INACT_XL_LOW_POWER) &&
		  (md1 & MD1_CFG_INT1_6D) && !(md1 & MD1_CFG_INT1_WU) &&
		  (((sixd & TAP_THS_6D_SIXD_THS_MSK) >> TAP_THS_6D_SIXD_THS_POS) ==
		   (uint8_t)deg);

	if (!ok) {
		LOG_ERR("6D arming did not take: FUNCTIONS_ENABLE=0x%02x "
			"MD1_CFG=0x%02x TAP_THS_6D=0x%02x INACTIVITY_DUR=0x%02x",
			fen, md1, sixd, idur);
		return -EIO;
	}

	static const uint8_t deg_name[] = {80, 70, 60, 50};

	LOG_INF("INT1 armed on %u deg orientation change; idles to 1.875 Hz "
		"after %u s below %u mg (code %u, weight %u)",
		deg_name[(uint8_t)deg & 3],
		(unsigned int)((uint32_t)sleep_dur * SLEEP_DUR_LSB_SAMPLES /
			       LSM6DSV_ODR_HZ),
		(unsigned int)(((uint32_t)inact_ths * wake_ths_ug_per_lsb[weight]) / 1000U),
		inact_ths, weight);

	return 0;
}

int lsm6dsv_6d_src(uint8_t *src)
{
	if (!present) {
		return -ENODEV;
	}

	return lsm6dsv_read_reg(REG_D6D_SRC, src, 1);
}

int lsm6dsv_wake_src(uint8_t *src)
{
	if (!present) {
		return -ENODEV;
	}

	return lsm6dsv_read_reg(REG_WAKE_UP_SRC, src, 1);
}
