/*
 * LSM6DSV 6-axis IMU — minimal SPI driver.
 *
 * Deliberately not the Zephyr sensor-subsystem driver: this application needs
 * a steady stream of raw accelerometer samples plus one wake-up interrupt, and
 * the in-tree driver pulls in the whole sensor API plus a trigger thread for
 * that. This is ~250 lines with no threads of its own.
 *
 * Lifted from the FindMyTag firmware, which drives the same part on the same
 * PCB; the only change is the accelerometer output data rate (see
 * LSM6DSV_ODR_HZ below).
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef LSM6DSV_H
#define LSM6DSV_H

#include <stdbool.h>
#include <stdint.h>

/* WHO_AM_I values seen on this family. The RideFormTracker PCB turned out to
 * carry an LSM6DSV16B (0x71) rather than the plain LSM6DSV (0x70), so accept
 * both — the accelerometer path is identical, only the sensor hub differs,
 * and nothing here uses the sensor hub. */
#define LSM6DSV_WHOAMI      0x70
#define LSM6DSV16B_WHOAMI   0x71

/* Accelerometer output data rate, in Hz.
 *
 * The cadence detector reads FASTER than this on purpose (see CADENCE_HZ in
 * cadence.h). Reading faster than the sensor updates costs a few duplicated
 * samples and nothing else; reading SLOWER would alias road vibration —
 * anything between ODR/2 and the read rate — straight down into the 0.5-3 Hz
 * band where the crank signal lives, which is the one band that must stay
 * clean. 60 Hz also puts the part's own anti-alias decimation at 30 Hz, about
 * 9x above the fastest crank anyone turns. */
#define LSM6DSV_ODR_HZ 60

/* Probe the part and put the accelerometer in low-power mode at
 * LSM6DSV_ODR_HZ, +/-8 g. Returns -ENODEV when the part does not answer.
 *
 * THE GYRO STAYS POWERED DOWN, and that is a deliberate trade rather than an
 * oversight — it is worth being clear about, because a gyro is the obvious
 * instrument for measuring a rotation and would make cadence.c far simpler.
 * With rate about the spindle axis you would just integrate and count turns:
 * no centripetal term to subtract, so no DC tracker, no period-relative
 * filters, and no "is this a rotation or just a shake" guard, since a linear
 * shake produces no rate at all. Every awkward part of the detector exists
 * because the signal is gravity seen from a rotating frame.
 *
 * What buys all that back is current. A gyro runs one to two orders of
 * magnitude hotter than an accelerometer in low-power mode, and this thing has
 * to live on a small battery bolted to a crank for months. The accelerometer
 * also has to be on regardless, since it is what provides the wake-on-motion
 * interrupt that stops the radio when the bike is parked — so the gyro would
 * be pure added cost on top.
 *
 * (Order-of-magnitude from experience, not from the LSM6DSV datasheet, which
 * is not in this repo. If the exact figures ever matter, get them from ST.) */
int lsm6dsv_init(void);

/* True once lsm6dsv_init() has succeeded. Everything else is a no-op
 * returning -ENODEV until then. */
bool lsm6dsv_present(void);

/* Latest acceleration, in milli-g per axis. */
int lsm6dsv_read_accel_mg(int16_t out[3]);

/* Raw 16-bit acceleration, for anything that wants to do its own scaling. */
int lsm6dsv_read_accel_raw(int16_t out[3]);

/* milli-g per raw LSB, scaled by 1000 (i.e. micro-g per LSB) at the full
 * scale this driver selects. */
#define LSM6DSV_UG_PER_LSB 244

/* How far the sensed direction of gravity has to move before INT1 fires. */
enum lsm6dsv_6d_deg {
	LSM6DSV_6D_80DEG = 0,
	LSM6DSV_6D_70DEG = 1,
	LSM6DSV_6D_60DEG = 2,
	LSM6DSV_6D_50DEG = 3,
};

/* Set the part up to wake the host ONLY when the sensor's orientation relative
 * to gravity actually changes, and to idle itself down in between.
 *
 * WHY ORIENTATION AND NOT MOTION. The obvious wake source is the wake-up
 * (activity) function, which fires on acceleration magnitude. It is useless
 * here, because the thing this sensor most needs to ignore — a bike on a train,
 * in a car, or on a roof rack — produces continuous acceleration while the
 * crank never turns. Orientation is the discriminator that actually separates
 * the two: vibration does not rotate gravity, and a turning crank sweeps it
 * through a full circle.
 *
 * It works better than it sounds, because a crank hangs. Push the bike, lift
 * it, tilt it, load it into a van: the crank swings to stay pointing down, so
 * the sensor's orientation relative to gravity does not change. Essentially the
 * only thing that changes it is the crank being turned, which is exactly when
 * this sensor has anything to say.
 *
 * The cost is that walking a bike will NOT wake the sensor, and neither will
 * picking it up — you have to turn the cranks. In practice that is what a rider
 * does anyway, and every commercial cadence sensor behaves the same way.
 *
 * Also enables the part's own inactivity handling: after quiet_s seconds with
 * no motion above inact_ths_mg it drops the accelerometer to 1.875 Hz by
 * itself, and restores the configured rate when motion resumes. That is a
 * hardware function — the host is not involved and cannot get it wrong.
 *
 * Returns -EIO if the configuration did not read back, which is worth acting
 * on: with no periodic wake-up in the sampler, an INT1 that never fires means
 * a sensor that never reports. */
int lsm6dsv_arm_orientation_wake(enum lsm6dsv_6d_deg deg, uint16_t inact_ths_mg,
				 uint16_t quiet_s);

/* Read D6D_SRC. Bit 6 (D6D_IA) is the orientation-change flag; bits 0-5 say
 * which face is up. For bring-up and for the console. */
int lsm6dsv_6d_src(uint8_t *src);

/* Read WAKE_UP_SRC. Bit 3 (WU_IA) is the wake-up event flag. Not used as an
 * interrupt source any more, but still readable for diagnostics. */
int lsm6dsv_wake_src(uint8_t *src);

/* Raw register access, for bring-up. */
int lsm6dsv_read_reg(uint8_t reg, uint8_t *buf, uint8_t len);
int lsm6dsv_write_reg(uint8_t reg, uint8_t val);

#endif /* LSM6DSV_H */
