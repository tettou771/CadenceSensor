/*
 * Crank cadence detection — see cadence.h for what is being measured and why
 * it is separable from everything else the crank does to a sensor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "cadence.h"
#include "lsm6dsv.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(cadence, LOG_LEVEL_INF);

#define TAU 6.283185307179586f

/* Sampling. The kernel tick has to divide evenly or the sampler drifts against
 * the timestamps it is stamping revolutions with, which is the one error this
 * whole design cannot absorb. */
#define SAMPLE_TICKS (CONFIG_SYS_CLOCK_TICKS_PER_SEC / CADENCE_HZ)
BUILD_ASSERT(CONFIG_SYS_CLOCK_TICKS_PER_SEC % CADENCE_HZ == 0,
	     "CADENCE_HZ must divide CONFIG_SYS_CLOCK_TICKS_PER_SEC evenly");

/* 1/1024 s per sample — exact at 64 Hz, which is why 64 was chosen. */
#define SAMPLE_1024 (1024 / CADENCE_HZ)
BUILD_ASSERT(1024 % CADENCE_HZ == 0, "CADENCE_HZ must divide 1024 evenly");

#define DT_S (1.0f / (float)CADENCE_HZ)

/* One-pole low-pass on each axis. Like the DC tracker below, its corner is set
 * from the measured crank period rather than pinned in Hz, and for the same
 * reason: the disturbance it has to suppress is not at a fixed frequency
 * either.
 *
 * The disturbance that matters is the pedal stroke's own unevenness. A crank
 * slows through the dead spots and surges through the two power phases, which
 * puts a tangential term of a few hundred mg on the in-plane axis at exactly
 * TWICE the crank frequency. That term distorts the circle the angle is
 * measured on, and a distorted circle makes the angle stall and reverse within
 * a revolution instead of advancing steadily — which costs counts.
 *
 * A filter at 1.6x the crank frequency passes the fundamental at 0.85 and the
 * 2x term at 0.62, improving the ratio that matters by about a quarter. It
 * costs 32 degrees of phase on the fundamental, and that is free: it is
 * constant, and the unwrapper only looks at differences.
 *
 * This value is a genuine trade, not an optimum. Widening it helps a violent
 * sprint (the filter tracks the transient better) and hurts a low-cadence
 * grind (more of the 2x term gets through); narrowing it does the reverse.
 * 1.6 is chosen towards the grind, because sustained low-cadence grinding is
 * an ordinary way to ride a bicycle and a two-second 60->140 rpm sprint is
 * not. The measured cost of that choice is in test/test_cadence.c.
 *
 * The clamps keep it sane before a period is known and stop it from tracking a
 * nonsense cadence into uselessness. */
#ifndef CADENCE_LP_FC_REVS
#define CADENCE_LP_FC_REVS 1.6f
#endif
#define LP_FC_MIN_HZ 1.2f
#define LP_FC_MAX_HZ 8.0f

/* alpha = 1 - exp(-2*pi*fc*dt) */
static inline float lp_alpha_for(float fc_hz)
{
	if (fc_hz < LP_FC_MIN_HZ) {
		fc_hz = LP_FC_MIN_HZ;
	} else if (fc_hz > LP_FC_MAX_HZ) {
		fc_hz = LP_FC_MAX_HZ;
	}

	return 1.0f - expf(-TAU * fc_hz * DT_S);
}

/* DC tracker time constant. This is what removes centripetal acceleration and
 * the mounting offset, and it is the most consequential number in the file:
 * it is squeezed hard from both ends.
 *
 * Too FAST and it eats the signal — the tracker's corner has to stay below the
 * crank tone, which is only 0.5 Hz at 30 rpm.
 *
 * Too SLOW and it cannot follow a sprint. Centripetal acceleration is w^2*r,
 * so it goes 0.9 g -> 2.7 g as a rider winds up from 70 to 130 rpm. While the
 * tracker lags, the residual pushes the gravity circle off centre, and once
 * the offset exceeds the 1 g radius the angle stops going all the way round
 * and revolutions are silently dropped.
 *
 * A FIXED time constant cannot satisfy both: measured against the scenarios in
 * test/test_cadence.c, 8 s loses 16 of 68 revolutions in a four-second sprint,
 * and shortening it far enough to fix that starts encroaching on 30 rpm.
 *
 * The way out is that the constraint is not really about seconds, it is about
 * CRANK REVOLUTIONS. What the tracker must not do is respond within one turn
 * of the crank; everything slower than that is wasted margin. So the time
 * constant is set from the measured revolution period instead:
 *
 *     tau = DC_TAU_REVS * period,  clamped to [DC_TAU_MIN_S, DC_TAU_MAX_S]
 *
 * which lands where it is needed at both ends — fast at high cadence, which is
 * exactly when centripetal acceleration is largest and changing fastest
 * (dC/dt = 2*w*r*dw/dt), and slow at low cadence, where it is small and lazy.
 *
 * At DC_TAU_REVS = 0.5 the corner sits at f/pi, costing 4.7% of amplitude and
 * 18 degrees of phase — and a CONSTANT phase offset costs no revolutions at
 * all, since the unwrapper only ever looks at differences.
 *
 * The clamps matter for the unlocked case: before the first revolution there
 * is no period to scale from, so the tracker starts at DC_TAU_MAX_S.
 * Overridable so the test can sweep them. */
#ifndef CADENCE_DC_TAU_REVS
#define CADENCE_DC_TAU_REVS 0.5f
#endif
#ifndef CADENCE_DC_TAU_MIN_S
#define CADENCE_DC_TAU_MIN_S 0.3f
#endif
#ifndef CADENCE_DC_TAU_MAX_S
#define CADENCE_DC_TAU_MAX_S 3.0f
#endif

#define DC_ALPHA_MIN (DT_S / CADENCE_DC_TAU_MAX_S)
#define DC_ALPHA_MAX (DT_S / CADENCE_DC_TAU_MIN_S)

/* AC energy tracker, 2 s, used both to pick the rotation plane and to gate
 * counting. Deliberately slower than the signal so the gate cannot chatter. */
#define VAR_ALPHA (DT_S / 2.0f)

/* Gate thresholds on the in-plane AC amplitude, in mg. A crank turning with
 * the sensor on it produces almost exactly 1000 mg here, by construction —
 * it is gravity, and gravity does not vary. Anything far off that is not a
 * crank: 350 mg rejects a bike being wheeled or rattling in a car boot,
 * 2000 mg rejects the sensor being shaken by hand or a mounting that has come
 * loose and is slapping around. */
#define AMP_MIN_MG 350.0f
#define AMP_MAX_MG 2000.0f

/* Re-pick which axes span the rotation plane only when there is clearly
 * rotation to learn it from; otherwise the choice is latched. A sensor in a
 * shed has three near-equal near-zero variances and would otherwise shuffle
 * its axis assignment on noise. */
#define AXIS_LATCH_MG 400.0f

/* Per-sample angle step sanity bound, in radians. A crank at 200 rpm advances
 * 0.33 rad per sample at 64 Hz, and stroke distortion can locally triple that,
 * so 0.8 clears any real rider with room to spare.
 *
 * What it is actually there to reject is nearly a whole half-turn at once, and
 * for that it works with AC_MIN_MG below — see the note there. */
#ifndef DTH_MAX
#define DTH_MAX 0.8f
#endif

/* Minimum instantaneous in-plane magnitude, in mg, for the angle to mean
 * anything.
 *
 * This is the guard against a disturbance that is NOT a rotation being read as
 * one. Consider a bike strapped to a roof rack, oscillating along one line at
 * a couple of Hz. In the rotation plane that traces a line THROUGH THE ORIGIN,
 * and the measured angle does not wander — it flips by almost exactly pi every
 * half cycle, always the same way. Two flips is 2*pi, which is a revolution
 * that never happened. Left unguarded this really does happen: it produced a
 * phantom revolution in the "stopped, being shaken" scenario.
 *
 * DTH_MAX alone does not catch it, because with any noise at all the vector
 * misses the origin and the angle swings round over several samples, each step
 * individually small enough to pass.
 *
 * The discriminator is the magnitude, not the angle. A crank carries a
 * rotating 1 g of gravity and its in-plane magnitude never collapses; a line
 * oscillation must pass through zero to get to the other side. So when the
 * magnitude drops into the region where the angle is meaningless, the update
 * is SKIPPED WITHOUT touching theta_prev — which bridges the crossing, so the
 * next valid sample sees one step of nearly pi, and DTH_MAX rejects that. The
 * two guards only work as a pair. */
#define AC_MIN_MG 300.0f

/* Park the sampler after this long with nothing turning and no wake-up
 * interrupt. The IMU keeps running either way; what this saves is 64
 * CPU wake-ups and SPI transactions per second. */
#define IDLE_AFTER_MS 30000

/* --- shared state ------------------------------------------------------- */

static struct k_spinlock lock;

/* Written by the sampler, read by anyone. revs/last_event_1024 must be read
 * together or a head unit can compute a nonsense RPM from a new count paired
 * with an old timestamp. */
static uint16_t s_revs;
static uint16_t s_last_event_1024;
static uint32_t s_total_revs;
static uint16_t s_rpm_x10;
static uint16_t s_amp_mg;
static bool     s_rotating;
static bool     s_idle;
static uint8_t  s_plane[2] = {0, 1};

K_SEM_DEFINE(motion_sem, 0, 1);
static atomic_t s_hold_awake;
static cadence_rev_cb_t s_rev_cb;

/* --- helpers ------------------------------------------------------------ */

/* Uptime in 1/1024 s, the unit the CSC profile speaks. Taken from the tick
 * counter rather than from milliseconds so no rounding creeps into the
 * interval a head unit differentiates. */
static uint32_t now_1024(void)
{
	return (uint32_t)((k_uptime_ticks() * 1024) /
			  CONFIG_SYS_CLOCK_TICKS_PER_SEC);
}

static float wrap_pi(float a)
{
	while (a > (TAU / 2.0f)) {
		a -= TAU;
	}
	while (a < -(TAU / 2.0f)) {
		a += TAU;
	}
	return a;
}

/* --- the detector ------------------------------------------------------- */

struct detector {
	float lp[3];       /* low-passed acceleration, mg                  */
	float dc[3];       /* slow mean: gravity offset + centripetal, mg  */
	float var[3];      /* AC energy per axis, mg^2                     */
	uint32_t n;        /* samples seen, for the running-mean warm-up   */

	uint8_t p0, p1;    /* the two axes spanning the rotation plane     */
	float theta_prev;
	bool  have_theta;
	float accum;       /* unwrapped angle since the last revolution    */

	uint32_t last_event_1024;
	bool     have_event;
	float    rpm_x10_lp;

	/* Both set from the measured revolution period; see the notes on
	 * CADENCE_LP_FC_REVS and CADENCE_DC_TAU_REVS. */
	float dc_alpha;
	float lp_alpha;
};

static void detector_reset(struct detector *d)
{
	memset(d, 0, sizeof(*d));
	d->p0 = 0;
	d->p1 = 1;
	/* Until a revolution has been timed there is no period to scale from,
	 * so start at the widest settings: the slowest DC tracker and the
	 * highest low-pass corner. Both only ever tighten from here, which is
	 * the safe direction — a filter that is too wide costs accuracy, one
	 * that is too narrow costs the signal itself. */
	d->dc_alpha = DC_ALPHA_MIN;
	d->lp_alpha = lp_alpha_for(LP_FC_MAX_HZ);
}

/* Choose the axis carrying the LEAST AC energy: that is the one pointing along
 * the crank spindle, because a rotation about the spindle never moves gravity
 * along it. The other two span the plane the angle is measured in. */
static void pick_plane(struct detector *d)
{
	uint8_t spindle = 0;

	for (uint8_t i = 1; i < 3; i++) {
		if (d->var[i] < d->var[spindle]) {
			spindle = i;
		}
	}

	uint8_t p[2];
	uint8_t k = 0;

	for (uint8_t i = 0; i < 3; i++) {
		if (i != spindle) {
			p[k++] = i;
		}
	}

	if (p[0] != d->p0 || p[1] != d->p1) {
		/* The angle is measured in the old plane; changing the plane
		 * makes theta_prev meaningless. Drop it rather than feed a
		 * step change into the unwrapper. */
		d->have_theta = false;
		d->p0 = p[0];
		d->p1 = p[1];
		LOG_INF("rotation plane: axes %u,%u (spindle %u)", p[0], p[1],
			spindle);
	}
}

static void emit_rev(struct detector *d, uint32_t event_1024)
{
	uint16_t ev = (uint16_t)event_1024;
	uint16_t rpm_x10 = 0;

	if (d->have_event) {
		uint16_t interval = ev - (uint16_t)d->last_event_1024;

		/* 614400 = 60 s * 1024 ticks * 10, i.e. rpm*10 for one
		 * revolution taking `interval` ticks of 1/1024 s. The bounds
		 * (256 rpm and 3 rpm) reject an interval that wrapped or that
		 * implies a cadence nobody produces.
		 *
		 * Note the feedback here: a MISSED revolution doubles the
		 * interval, which makes the filters below slower than they
		 * should be, which makes the next one easier to miss too. The
		 * clamps bound it and it recovers within a revolution or two —
		 * the violent-sprint numbers in test/test_cadence.c are this
		 * effect, and they are the reason the clamps are not wider. */
		if (interval >= 240 && interval <= 20480) {
			rpm_x10 = (uint16_t)(614400u / interval);

			/* Retune both filters to this crank period. Once per
			 * revolution, so the expf() is free. */
			float period_s = (float)interval / 1024.0f;
			float alpha = DT_S / (CADENCE_DC_TAU_REVS * period_s);

			if (alpha < DC_ALPHA_MIN) {
				alpha = DC_ALPHA_MIN;
			} else if (alpha > DC_ALPHA_MAX) {
				alpha = DC_ALPHA_MAX;
			}
			d->dc_alpha = alpha;
			d->lp_alpha = lp_alpha_for(CADENCE_LP_FC_REVS /
						   period_s);
		}
	}

	d->last_event_1024 = event_1024;
	d->have_event = true;

	if (rpm_x10 > 0) {
		if (d->rpm_x10_lp <= 0.0f) {
			d->rpm_x10_lp = (float)rpm_x10;
		} else {
			d->rpm_x10_lp += 0.5f * ((float)rpm_x10 - d->rpm_x10_lp);
		}
	}

	K_SPINLOCK(&lock) {
		s_revs++;
		s_total_revs++;
		s_last_event_1024 = ev;
		s_rotating = true;
		s_rpm_x10 = (uint16_t)d->rpm_x10_lp;
	}

	if (s_rev_cb) {
		s_rev_cb();
	}
}

/* One sample. Returns true if the crank looks like it is turning. */
static bool detector_feed(struct detector *d, const int16_t raw[3],
			  uint32_t t_1024)
{
	float ac[3];

	/* A plain running mean until the running mean becomes the slower of the
	 * two, then the EMA. Without it a sensor powered up mid-ride starts
	 * with dc = 0 and needs several time constants to converge, miscounting
	 * throughout; with it the first revolution is already usable. */
	if (d->n < 1000000) {
		d->n++;
	}

	float a_dc = 1.0f / (float)d->n;

	if (a_dc < d->dc_alpha) {
		a_dc = d->dc_alpha;
	}

	for (int i = 0; i < 3; i++) {
		float mg = (float)raw[i] * (float)LSM6DSV_UG_PER_LSB / 1000.0f;

		d->lp[i] += d->lp_alpha * (mg - d->lp[i]);
		d->dc[i] += a_dc * (d->lp[i] - d->dc[i]);
		ac[i] = d->lp[i] - d->dc[i];
		d->var[i] += VAR_ALPHA * (ac[i] * ac[i] - d->var[i]);
	}

	float total_rms = sqrtf(d->var[0] + d->var[1] + d->var[2]);

	if (total_rms > AXIS_LATCH_MG) {
		pick_plane(d);
	}

	float amp = sqrtf(d->var[d->p0] + d->var[d->p1]);

	K_SPINLOCK(&lock) {
		s_amp_mg = (uint16_t)amp;
		s_plane[0] = d->p0;
		s_plane[1] = d->p1;
	}

	if (amp < AMP_MIN_MG || amp > AMP_MAX_MG) {
		/* Not a crank. Let the angle re-acquire from scratch when it
		 * becomes one again — a stale theta_prev from before the gap
		 * would inject a bogus step into the accumulator. */
		d->have_theta = false;
		d->accum = 0.0f;
		return false;
	}

	/* Too close to the origin for the angle to mean anything — bridge it.
	 * See AC_MIN_MG. */
	if ((ac[d->p0] * ac[d->p0] + ac[d->p1] * ac[d->p1]) <
	    (AC_MIN_MG * AC_MIN_MG)) {
		return true;
	}

	float theta = atan2f(ac[d->p1], ac[d->p0]);

	if (!d->have_theta) {
		d->theta_prev = theta;
		d->have_theta = true;
		return true;
	}

	float dth = wrap_pi(theta - d->theta_prev);

	d->theta_prev = theta;

	if (fabsf(dth) > DTH_MAX) {
		return true; /* bad sample; hold the accumulator */
	}

	d->accum += dth;

	/* A revolution is 2*pi of accumulated angle, in either direction — a
	 * rider backpedalling is turning the crank, and a magnet-and-reed
	 * sensor would have counted it too. The leftover past the boundary
	 * says how far into this sample interval the crossing happened, which
	 * is worth recovering: the head unit's RPM comes from the difference
	 * of these timestamps, so quantising them to the 15.6 ms sample grid
	 * would show up as visible jitter in the displayed cadence. */
	while (d->accum >= TAU || d->accum <= -TAU) {
		float leftover;

		if (d->accum >= TAU) {
			d->accum -= TAU;
			leftover = d->accum;
		} else {
			d->accum += TAU;
			leftover = -d->accum;
		}

		float frac = 0.0f;

		if (fabsf(dth) > 1e-6f) {
			frac = leftover / fabsf(dth);
			if (frac < 0.0f) {
				frac = 0.0f;
			} else if (frac > 1.0f) {
				frac = 1.0f;
			}
		}

		emit_rev(d, t_1024 - (uint32_t)(frac * (float)SAMPLE_1024));
	}

	return true;
}

/* --- sampler thread ----------------------------------------------------- */

static void cadence_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	struct detector d;
	int64_t next = k_uptime_ticks();
	uint32_t quiet_samples = 0;

	detector_reset(&d);

	while (1) {
		int16_t raw[3];

		/* Absolute deadline, so a slow SPI read or a burst of logging
		 * costs one late sample instead of permanently shifting the
		 * sample grid the revolution timestamps sit on. */
		next += SAMPLE_TICKS;
		k_sleep(K_TIMEOUT_ABS_TICKS(next));

		if (lsm6dsv_read_accel_raw(raw) != 0) {
			continue;
		}

		bool turning = detector_feed(&d, raw, now_1024());

		/* rotating goes false on a timeout rather than the instant the
		 * gate opens, so a single dropout mid-pedal-stroke does not
		 * blink the LED and the console. */
		K_SPINLOCK(&lock) {
			uint16_t since = (uint16_t)now_1024() - s_last_event_1024;

			if (s_rotating && since > (CADENCE_STOP_MS * 1024) / 1000) {
				s_rotating = false;
				s_rpm_x10 = 0;
			}
		}

		if (turning) {
			quiet_samples = 0;
			continue;
		}

		/* Counted in samples, not milliseconds: 1000/64 truncates to 15
		 * and the timeout would run 4% long. */
		quiet_samples++;

		if (quiet_samples < (IDLE_AFTER_MS * CADENCE_HZ) / 1000 ||
		    atomic_get(&s_hold_awake)) {
			continue;
		}

		/* Park until INT1 says the sensor's orientation changed, and
		 * ONLY until then.
		 *
		 * There used to be a 60 s timeout here as insurance against a
		 * dead interrupt. It was removed deliberately: waking the
		 * sampler every minute to look at a stationary bicycle cost
		 * about a third of the parked duty cycle, and it insured
		 * against a fault that is better caught at boot than paid for
		 * forever. lsm6dsv_arm_orientation_wake() now reads its own
		 * configuration back, and main() pins this thread awake with
		 * CADENCE_HOLD_FAULT if that check fails — so a broken INT1
		 * produces a sensor that works and drains, not one that sleeps
		 * for good. Plugging in USB also pins it awake, which leaves a
		 * human a way in regardless. */
		K_SPINLOCK(&lock) {
			s_idle = true;
		}
		LOG_DBG("sampler idle");

		k_sem_take(&motion_sem, K_FOREVER);

		K_SPINLOCK(&lock) {
			s_idle = false;
		}

		/* The dc/var trackers have been looking at a stationary sensor
		 * that may since have been picked up and re-oriented, so their
		 * contents describe a world that no longer exists. */
		detector_reset(&d);
		quiet_samples = 0;
		next = k_uptime_ticks();
	}
}

K_THREAD_DEFINE(cadence_tid, 2048, cadence_thread, NULL, NULL, NULL, 5, 0,
		K_TICKS_FOREVER);

/* --- API ---------------------------------------------------------------- */

int cadence_init(void)
{
	if (!lsm6dsv_present()) {
		return -ENODEV;
	}

	k_thread_start(cadence_tid);

	return 0;
}

void cadence_get(struct cadence_state *out)
{
	K_SPINLOCK(&lock) {
		out->revs = s_revs;
		out->last_event_1024 = s_last_event_1024;
		out->total_revs = s_total_revs;
		out->rpm_x10 = s_rpm_x10;
		out->amp_mg = s_amp_mg;
		out->rotating = s_rotating;
		out->idle = s_idle;
		out->plane[0] = s_plane[0];
		out->plane[1] = s_plane[1];
	}
}

void cadence_notify_motion(void)
{
	k_sem_give(&motion_sem);
}

void cadence_hold_awake(uint32_t reason, bool hold)
{
	if (hold) {
		atomic_or(&s_hold_awake, (atomic_val_t)reason);
		/* Wake it now rather than at the next interrupt — with no
		 * timeout left, "at the next interrupt" could be never. */
		k_sem_give(&motion_sem);
	} else {
		atomic_and(&s_hold_awake, ~(atomic_val_t)reason);
	}
}

void cadence_set_rev_callback(cadence_rev_cb_t cb)
{
	s_rev_cb = cb;
}
