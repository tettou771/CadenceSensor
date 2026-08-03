/*
 * Crank cadence detection from a single accelerometer.
 *
 * The sensor is bolted to a crank arm. As the crank turns, the sensor turns
 * with it, and GRAVITY sweeps a full circle through the sensor's own frame
 * once per revolution. That rotating 1 g vector is the signal; everything
 * else is what has to be removed to see it.
 *
 * What else is there, and why each one is separable:
 *
 *   centripetal (w^2 * r, up to several g at racing cadence)
 *       points at the spindle, which is a FIXED direction in the rotating
 *       sensor frame. It is therefore DC in that frame, and the DC tracker
 *       below removes it. This is the reason the whole approach works at
 *       cadences where centripetal acceleration dwarfs gravity.
 *
 *   tangential (dw/dt * r)
 *       real, a few hundred mg, and the only one here that cannot be removed.
 *       It comes from the pedal stroke's dead spots, so it sits at exactly
 *       TWICE the crank frequency, and it lands on the in-plane axis at right
 *       angles to the centripetal term. Attenuated by a low-pass whose corner
 *       tracks the crank frequency; what survives distorts the circle the
 *       angle is measured on, which the detector has to tolerate.
 *
 *   road vibration
 *       broadband and mostly well above the crank's 0.5-3 Hz. Removed by the
 *       same low-pass, and by reading the part faster than its own output rate
 *       so nothing folds down into the crank band (see LSM6DSV_ODR_HZ).
 *
 *   mounting orientation
 *       never assumed. The detector works out which two axes span the rotation
 *       plane by watching which two carry the AC energy, so the sensor can be
 *       glued on in any orientation.
 *
 * The output is deliberately shaped for the BLE Cycling Speed and Cadence
 * profile: a cumulative revolution count and the timestamp of the last
 * revolution, which is what a head unit differentiates to get RPM. The rpm
 * field here is only for the console and the LED.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CADENCE_H
#define CADENCE_H

#include <stdbool.h>
#include <stdint.h>

/* Sampler rate. Faster than LSM6DSV_ODR_HZ on purpose — see the note there.
 * Must divide the kernel tick rate evenly; cadence.c asserts that. */
#define CADENCE_HZ 64

struct cadence_state {
	/* The two CSC Measurement fields, and the only two that need to be
	 * consistent with each other. cadence_get() reads them as a pair. */
	uint16_t revs;             /* cumulative crank revolutions, wraps    */
	uint16_t last_event_1024;  /* time of that revolution, 1/1024 s      */

	uint16_t rpm_x10;          /* smoothed, for humans; 0 when stopped   */
	uint16_t amp_mg;           /* in-plane AC amplitude; ~1000 = healthy */
	bool     rotating;         /* a revolution within CADENCE_STOP_MS    */
	bool     idle;             /* sampler parked, waiting on INT1        */
	uint8_t  plane[2];         /* axes the detector picked (0=X,1=Y,2=Z) */
	uint32_t total_revs;       /* since boot, does not wrap at 16 bits   */
};

/* Start the sampler thread. Returns -ENODEV if the IMU is not there, in which
 * case nothing else in here does anything. */
int cadence_init(void);

/* Snapshot. Safe from any thread. */
void cadence_get(struct cadence_state *out);

/* Called from the LSM6DSV INT1 handler (ISR context) to pull the sampler out
 * of its idle state. Safe to call when already running. */
void cadence_notify_motion(void);

/* Reasons the sampler may be pinned awake. A bitmask and not a bool because
 * more than one holder exists at once and they must not clear each other's
 * hold: a head unit disconnecting while USB is plugged in would otherwise put
 * the sampler to sleep in the middle of a debugging session. */
#define CADENCE_HOLD_BLE   BIT(0)  /* a head unit is connected               */
#define CADENCE_HOLD_USB   BIT(1)  /* someone is at the console              */
#define CADENCE_HOLD_FAULT BIT(2)  /* INT1 is not trustworthy — never sleep  */

/* Pin the sampler awake for `reason`, or release that one reason. The sampler
 * parks only when every reason has been released.
 *
 * CADENCE_HOLD_FAULT is the important one. The sampler has no periodic
 * wake-up of its own — it sleeps until INT1 says otherwise — so if the
 * interrupt could not be armed, sleeping at all would mean never reporting
 * again. Holding it awake burns battery instead, which is the right way round:
 * a sensor that works and drains is diagnosable, one that is silent looks
 * identical to a flat cell. */
void cadence_hold_awake(uint32_t reason, bool hold);

/* Fired from the sampler thread on each counted revolution, so the notify can
 * go out on the revolution instead of on a poll. Keep it short. */
typedef void (*cadence_rev_cb_t)(void);
void cadence_set_rev_callback(cadence_rev_cb_t cb);

/* No revolution for this long and rpm_x10 reads 0. 2.5 s puts the floor at
 * 24 rpm, below which nobody is meaningfully pedalling. */
#define CADENCE_STOP_MS 2500

#endif /* CADENCE_H */
