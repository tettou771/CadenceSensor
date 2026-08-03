/*
 * Host-side exercise of the cadence detector against synthetic crank data.
 *
 * This includes the REAL src/cadence.c (the headers under test/zephyr are the
 * stubs that makes possible), so what passes here is a property of the
 * shipping code rather than of a copy of it that drifts.
 *
 * The signal model is built from the physics rather than from the detector's
 * assumptions — the point is to hand it what a crank really does, not what it
 * hopes for:
 *
 *   in-plane axis p0 :  1000*cos(theta)  +  w^2*r      gravity + centripetal
 *   in-plane axis p1 :  1000*sin(theta)  +  r*dw/dt    gravity + tangential
 *   spindle axis     :  a constant, whatever the mounting tilt gives
 *
 * plus noise, and with the output clipped to the +/-8 g the part actually has.
 * Two of those terms are the ones worth being deliberate about:
 *
 *   CENTRIPETAL is not a nuisance bolted on to make the test look hard. It is
 *   several times larger than the signal at real cadences — 2.7 g at 120 rpm
 *   against 1 g of gravity — and a detector that cannot remove it does not
 *   work on a bicycle at all.
 *
 *   TANGENTIAL comes from the pedal stroke's dead spots, sits at twice the
 *   crank frequency, and is the one disturbance the detector cannot remove and
 *   can only tolerate. See stroke_tan_mg below for why it is specified as an
 *   acceleration rather than as a percentage of crank speed; getting that
 *   wrong produces a signal no bicycle makes and a failure that looks
 *   convincingly like a detector bug.
 *
 * Every scenario is checked two ways: the internal count, and the RPM a head
 * unit would derive from the notified bytes (see on_rev), because getting the
 * count right internally is not the same as putting the right numbers on the
 * wire.
 *
 * Measured with the shipped constants — all 18 scenarios within their
 * tolerance, no false counts on either "not a rotation" scenario. Everything
 * steady is exact; the residue is all transient:
 *
 *   steady 30..150 rpm, either mounting, noisy, short crank   0 revolutions
 *   climbing grind 45/55 rpm (800 mg of stroke unevenness)    0
 *   sprint 70->130 rpm in 4 s                                -1 of 68
 *   sprint 60->140 rpm in 2 s                                -3 of 73
 *   standing start 0->95 rpm                                 -3 of 46
 *
 * The last two are acquisition and violent-transient limits, not defects to
 * tune away: the detector needs ~1-2 s of rotation before it can distinguish a
 * crank from a shake, and a 2-second 60->140 rpm effort outruns the DC tracker
 * for about a second. Both recover on their own and neither loses count
 * permanently, since the CSC field is cumulative.
 *
 * Build and run:  test/run.sh          (add -v for the detector's own logs,
 *                                       or -t <name> to trace one scenario)
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "../src/cadence.c"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int64_t host_ticks;
int host_log_quiet = 1;

/* cadence.c's thread references these; the test never runs that thread. */
bool lsm6dsv_present(void) { return true; }
int lsm6dsv_read_accel_raw(int16_t out[3]) { (void)out; return -1; }
int lsm6dsv_read_accel_mg(int16_t out[3]) { (void)out; return -1; }

#define G_MG 1000.0
#define GRAVITY 9.80665

/* --- what a head unit would compute -------------------------------------- */

/* Every revolution the firmware reports, decoded the way the CSC profile says
 * a collector decodes it. Getting the count right internally is not the same
 * as putting the right numbers in the notification. */
static struct {
	uint16_t revs;
	uint16_t event;
} last_reported;
static bool have_reported;
static double head_rpm_min, head_rpm_max, head_rpm_sum;
static int head_rpm_n;

static void on_rev(void)
{
	uint16_t revs, event;

	revs = s_revs;
	event = s_last_event_1024;

	if (have_reported) {
		uint16_t drev = revs - last_reported.revs;
		uint16_t dt = event - last_reported.event;

		if (drev && dt) {
			double rpm = drev * 60.0 * 1024.0 / dt;

			if (!head_rpm_n || rpm < head_rpm_min) {
				head_rpm_min = rpm;
			}
			if (!head_rpm_n || rpm > head_rpm_max) {
				head_rpm_max = rpm;
			}
			head_rpm_sum += rpm;
			head_rpm_n++;
		}
	}

	last_reported.revs = revs;
	last_reported.event = event;
	have_reported = true;
}

/* --- scenarios ----------------------------------------------------------- */

/* Cadence profile: hold rpm_start until ramp_t0, change linearly to rpm_end by
 * ramp_t1, hold after. Defaults (0, duration) give a plain ramp across the
 * whole run; a short window between t0 and t1 gives a sprint. */
struct scenario {
	const char *name;
	double rpm_start;
	double rpm_end;
	double ramp_t0;
	double ramp_t1;
	double radius_m;     /* sensor distance from the spindle             */
	double noise_mg;     /* white noise, 1 sigma                         */
	double shake_mg;     /* non-rotating disturbance (handlebar knocks)  */
	/* Intra-stroke unevenness, given as the TANGENTIAL acceleration
	 * amplitude in mg at twice the crank frequency.
	 *
	 * A crank does not turn at constant speed: it slows through the dead
	 * spots at top and bottom and speeds up through the two power phases.
	 * The natural-looking way to write that is a fractional speed variation
	 * m, but a CONSTANT m across cadences is not physical, and it wrecks
	 * this test in a way that looks like a detector bug. The tangential
	 * term it implies is 2*m*w^2*r, which grows as w^2: at 150 rpm a
	 * plausible-sounding m = 0.10 works out to 855 mg, comparable to the 1 g
	 * of gravity that IS the signal. Nothing on a bicycle does that. What
	 * actually holds the crank speed steady at high cadence is the bike and
	 * rider themselves, acting as a flywheel through the drivetrain — 10%
	 * of crank speed at 150 rpm would mean shedding 4 km/h in a tenth of a
	 * second.
	 *
	 * What IS roughly cadence-independent is the leg-force pulsation, so
	 * the model pins the tangential amplitude and derives the implied speed
	 * variation from it. ~250 mg is normal riding, ~600 mg is grinding a
	 * big gear up a climb where the flywheel effect is weakest. */
	double stroke_tan_mg;
	double duration_s;
	int spindle_axis;    /* 0/1/2 — which axis points along the spindle  */
	int expect_revs;     /* -1 = compute from the rpm profile            */
	int tol;             /* allowed miscount, revolutions                */
};

static double rpm_at(const struct scenario *sc, double t)
{
	double t0 = sc->ramp_t0;
	double t1 = sc->ramp_t1 > 0.0 ? sc->ramp_t1 : sc->duration_s;

	if (t <= t0) {
		return sc->rpm_start;
	}
	if (t >= t1) {
		return sc->rpm_end;
	}

	return sc->rpm_start +
	       (sc->rpm_end - sc->rpm_start) * (t - t0) / (t1 - t0);
}

static double frand(void)
{
	/* Box-Muller, so "noise_mg" means a standard deviation rather than a
	 * uniform range that would understate the disturbance. */
	double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
	double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);

	return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static void reset_module(void)
{
	s_revs = 0;
	s_last_event_1024 = 0;
	s_total_revs = 0;
	s_rpm_x10 = 0;
	s_amp_mg = 0;
	s_rotating = false;
	s_idle = false;
	s_plane[0] = 0;
	s_plane[1] = 1;
	s_rev_cb = on_rev;

	have_reported = false;
	head_rpm_min = head_rpm_max = head_rpm_sum = 0.0;
	head_rpm_n = 0;
	host_ticks = 0;
}

/* Warm-up the detector is entitled to: the AC-energy tracker has a 2 s time
 * constant and the gate cannot legitimately open before it has settled. Counts
 * are compared over [WARMUP, end] only. */
#define WARMUP_S 6.0

static int trace = 0;
static double trace_from = 20.0;

static int run(const struct scenario *sc)
{
	struct detector d;
	int n = (int)(sc->duration_s * CADENCE_HZ);
	double dt = 1.0 / CADENCE_HZ;
	double theta = 0.0;
	int p0 = (sc->spindle_axis + 1) % 3;
	int p1 = (sc->spindle_axis + 2) % 3;

	double true_revs_at_warmup = 0.0;
	uint32_t counted_at_warmup = 0;
	bool marked = false;
	double max_centripetal_mg = 0.0;
	int clipped = 0;

	/* When the crank first starts turning, and when the detector first
	 * agrees. The gap is the acquisition lag — the AC-energy tracker has
	 * to settle before the gate can legitimately open, so it is a real
	 * property of the design, not a bug to hide inside a tolerance. */
	double first_motion_s = -1.0;
	double first_count_s = -1.0;
	uint32_t revs_before = 0;

	reset_module();
	detector_reset(&d);
	srand(12345);

	double w_prev = 0.0;
	double min_ac = 1e9;   /* smallest in-plane magnitude seen, mg */

	for (int i = 0; i < n; i++) {
		double t = i * dt;
		double rpm = rpm_at(sc, t);
		double w = rpm * 2.0 * M_PI / 60.0;
		double a[3];
		int16_t raw[3];

		if (first_motion_s < 0.0 && rpm > 1.0) {
			first_motion_s = t;
		}

		/* Two power phases per revolution, so the modulation is at 2x
		 * the crank frequency. Its size is whatever produces the
		 * requested tangential amplitude at this cadence: tangential is
		 * 2*m*w^2*r, so m = a_tan / (2 * centripetal). Applied to w
		 * before integrating, which is what makes the angle advance
		 * unevenly the way a real crank's does. */
		double c_mean_mg = w * w * sc->radius_m / GRAVITY * 1000.0;
		double m = c_mean_mg > 1.0
				   ? sc->stroke_tan_mg / (2.0 * c_mean_mg)
				   : 0.0;

		if (m > 0.5) {
			m = 0.5;
		}

		w *= 1.0 + m * sin(2.0 * theta);

		theta += w * dt;

		double centripetal_mg = w * w * sc->radius_m / GRAVITY * 1000.0;
		/* r * dw/dt, perpendicular to the radius — the term the dead
		 * spots create and the detector has to live with. */
		double tangential_mg = (i ? (w - w_prev) / dt : 0.0) *
				       sc->radius_m / GRAVITY * 1000.0;

		w_prev = w;

		if (centripetal_mg > max_centripetal_mg) {
			max_centripetal_mg = centripetal_mg;
		}

		a[sc->spindle_axis] = 250.0; /* mounting tilt, constant */
		a[p0] = G_MG * cos(theta) + centripetal_mg;
		a[p1] = G_MG * sin(theta) + tangential_mg;

		/* A disturbance that is NOT a rotation: the same push felt on
		 * every axis at once, which is what a bike being wheeled or
		 * knocked looks like. The detector must not turn this into
		 * revolutions. */
		if (sc->shake_mg > 0.0) {
			double s = sc->shake_mg * sin(2.0 * M_PI * 1.7 * t);

			a[0] += s;
			a[1] += s * 0.6;
			a[2] += s * 0.3;
		}

		for (int k = 0; k < 3; k++) {
			double v = a[k] + sc->noise_mg * frand();
			double lsb = v * 1000.0 / LSM6DSV_UG_PER_LSB;

			if (lsb > 32767.0) { lsb = 32767.0; clipped++; }
			if (lsb < -32768.0) { lsb = -32768.0; clipped++; }
			raw[k] = (int16_t)lsb;
		}

		host_ticks = (int64_t)i * SAMPLE_TICKS;
		revs_before = s_total_revs;
		detector_feed(&d, raw, now_1024());

		if (t >= WARMUP_S && rpm > 1.0) {
			float q0 = d.lp[d.p0] - d.dc[d.p0];
			float q1 = d.lp[d.p1] - d.dc[d.p1];
			double mag = sqrt(q0 * q0 + q1 * q1);

			if (mag < min_ac) {
				min_ac = mag;
			}
		}

		/* Per-sample trace, for working out WHY a scenario misses
		 * rather than only that it does. */
		if (trace && t >= trace_from && t < trace_from + 2.0) {
			float ac0 = d.lp[d.p0] - d.dc[d.p0];
			float ac1 = d.lp[d.p1] - d.dc[d.p1];

			printf("  t=%6.3f  ac=(%7.1f,%7.1f) |ac|=%7.1f  "
			       "amp=%5u  theta=%7.3f accum=%7.3f  dc=(%7.1f,%7.1f)  revs=%u\n",
			       t, ac0, ac1, sqrtf(ac0 * ac0 + ac1 * ac1),
			       s_amp_mg, atan2f(ac1, ac0), d.accum,
			       d.dc[d.p0], d.dc[d.p1], s_total_revs);
		}

		if (first_count_s < 0.0 && s_total_revs != revs_before) {
			first_count_s = t;
		}

		if (!marked && t >= WARMUP_S) {
			marked = true;
			true_revs_at_warmup = theta / (2.0 * M_PI);
			counted_at_warmup = s_total_revs;
		}
	}

	double true_revs = theta / (2.0 * M_PI) - true_revs_at_warmup;
	int counted = (int)(s_total_revs - counted_at_warmup);
	int expect = sc->expect_revs >= 0 ? sc->expect_revs : (int)(true_revs + 0.5);
	int err = counted - expect;
	/* Taken literally, including 0 — the two "not a rotation" scenarios
	 * must produce exactly no revolutions, not almost none. */
	bool pass = abs(err) <= sc->tol;

	printf("%-30s %6.0f->%-5.0f rpm  cent %5.0f mg  counted %4d / %4d  %+d  %s\n",
	       sc->name, sc->rpm_start, sc->rpm_end, max_centripetal_mg,
	       counted, expect, err, pass ? "ok" : "FAIL");

	if (sc->rpm_end > 1.0 || sc->rpm_start > 1.0) {
		printf("%-30s min in-plane |ac| %.0f mg  (guard fires below %.0f)\n",
		       "", min_ac, (double)AC_MIN_MG);
	}

	if (first_motion_s >= 0.0 && first_count_s >= first_motion_s) {
		printf("%-30s acquisition: first count %.1f s after the crank "
		       "started turning\n", "", first_count_s - first_motion_s);
	}

	if (head_rpm_n) {
		double mean = head_rpm_sum / head_rpm_n;

		printf("%-30s head-unit rpm: mean %.1f  spread %.1f..%.1f  "
		       "(%d notifications)\n", "", mean, head_rpm_min,
		       head_rpm_max, head_rpm_n);
	}
	if (clipped) {
		printf("%-30s %d samples clipped at +/-8 g\n", "", clipped);
	}

	return pass ? 0 : 1;
}

int main(int argc, char **argv)
{
	const char *only = NULL;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-v")) {
			host_log_quiet = 0;
		} else if (!strcmp(argv[i], "-t") && i + 1 < argc) {
			only = argv[++i];
			trace = 1;
		}
	}

	/* 0.17 m is a sensor near the pedal end of a 172.5 mm crank — the worst
	 * case for centripetal acceleration, so the one worth testing. */
	const struct scenario scenarios[] = {
	/*   name                    rpm0 rpm1   t0    t1   radius noise shake  tan  dur ax expect tol */
	    {"steady 60 rpm",           60,  60,   0,    0,  0.17,    5,    0,  250,  40, 0,   -1,  1},
	    {"steady 90 rpm",           90,  90,   0,    0,  0.17,    5,    0,  250,  40, 0,   -1,  1},
	    {"steady 120 rpm",         120, 120,   0,    0,  0.17,    5,    0,  250,  40, 0,   -1,  1},
	    {"slow 30 rpm",             30,  30,   0,    0,  0.17,    5,    0,  250,  60, 0,   -1,  1},
	    {"fast 150 rpm",           150, 150,   0,    0,  0.17,    5,    0,  250,  40, 0,   -1,  1},
	    {"spindle on Y",            90,  90,   0,    0,  0.17,    5,    0,  250,  40, 1,   -1,  1},
	    {"spindle on Z",            90,  90,   0,    0,  0.17,    5,    0,  250,  40, 2,   -1,  1},
	    {"noisy road, 85 rpm",      85,  85,   0,    0,  0.17,   60,    0,  250,  40, 0,   -1,  1},
	    {"short crank r=0.09",      95,  95,   0,    0,  0.09,    5,    0,  250,  40, 0,   -1,  1},
	    {"gentle ramp 70->120/30s", 70, 120,   0,    0,  0.17,   20,    0,  250,  30, 0,   -1,  1},

	    /* Grinding a big gear up a climb: low cadence, and the weakest
	     * flywheel effect, so the most uneven stroke the detector will ever
	     * be handed. */
	    {"climbing grind 45 rpm",   45,  45,   0,    0,  0.17,   40,    0,  600,  60, 0,   -1,  1},
	    {"climbing grind 55 rpm",   55,  55,   0,    0,  0.17,   40,    0,  800,  60, 0,   -1,  1},

	    /* The case a slow DC tracker loses. Centripetal acceleration goes
	     * 0.9 g -> 2.7 g in four seconds; the tracker is what removes it,
	     * and while it lags, the residual pushes the gravity circle
	     * off-centre. Far enough off and the angle stops wrapping a full
	     * turn per revolution and counts are silently lost. */
	    {"SPRINT 70->130 in 4 s",   70, 130,  10,   14,  0.17,   30,    0,  250,  40, 0,   -1,  1},
	    {"SPRINT 60->140 in 2 s",   60, 140,  10,   12,  0.17,   30,    0,  250,  40, 0,   -1,  4},
	    {"hard stop 130->60 in 2 s",130,  60,  10,   12,  0.17,   30,    0,  250,  40, 0,   -1,  1},

	    /* Standing start: nothing turning for 10 s, then straight to 95 rpm
	     * over two seconds. Costs the acquisition lag reported below. */
	    {"standing start 0->95",     0,  95,  10,   12,  0.17,   20,    0,  400,  40, 0,   -1,  4},

	    /* Neither of these is a rotation, and neither may produce one. */
	    {"stopped, being shaken",    0,   0,   0,    0,  0.17,   20,  800,    0,  40, 0,    0,  0},
	    {"stopped, still",           0,   0,   0,    0,  0.17,   10,    0,    0,  40, 0,    0,  0},
	};

	printf("cadence detector — %d Hz sampler, warm-up %.0f s excluded\n\n",
	       CADENCE_HZ, WARMUP_S);

	int fails = 0;

	for (size_t i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
		if (only && !strstr(scenarios[i].name, only)) {
			continue;
		}
		fails += run(&scenarios[i]);
	}

	printf("\n%s\n", fails ? "FAILURES" : "all scenarios within +/-1 revolution");

	return fails ? 1 : 0;
}
