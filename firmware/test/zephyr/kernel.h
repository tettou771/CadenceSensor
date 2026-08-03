/*
 * Just enough of the Zephyr kernel API for cadence.c to compile and run on a
 * host, so the detector can be exercised against synthetic crank data without
 * a board.
 *
 * The point of stubbing rather than reimplementing is that test_cadence.c
 * includes the REAL cadence.c. Anything that passes here is a property of the
 * firmware's own arithmetic, not of a copy of it that drifts.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef HOSTTEST_ZEPHYR_KERNEL_H
#define HOSTTEST_ZEPHYR_KERNEL_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 32768

#define ARG_UNUSED(x) ((void)(x))
#define BIT(n) (1UL << (n))
#define BUILD_ASSERT(cond, msg) _Static_assert(cond, msg)

/* The test drives time explicitly: it advances this once per simulated sample
 * so revolution timestamps are exactly as deterministic as the sampler's
 * absolute-deadline scheduling makes them on real hardware. */
extern int64_t host_ticks;

static inline int64_t k_uptime_ticks(void) { return host_ticks; }
static inline int64_t k_uptime_get(void)
{
	return host_ticks * 1000 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
}
static inline uint32_t k_uptime_get_32(void) { return (uint32_t)k_uptime_get(); }

typedef int64_t k_timeout_t;
#define K_TIMEOUT_ABS_TICKS(t) (t)
#define K_SECONDS(s) ((int64_t)(s) * CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define K_TICKS_FOREVER (-1)
#define K_FOREVER (-1)
#define K_NO_WAIT 0

static inline void k_sleep(k_timeout_t t) { ARG_UNUSED(t); }
static inline void k_msleep(int ms) { ARG_UNUSED(ms); }

/* Single-threaded host run: the lock only has to compile. */
struct k_spinlock { int unused; };
#define K_SPINLOCK(l) for (int _kspin_once = 0; _kspin_once < 1; _kspin_once++)

struct k_sem { int count; };
#define K_SEM_DEFINE(name, initial, limit) static struct k_sem name = {initial}
static inline int k_sem_take(struct k_sem *s, k_timeout_t t)
{
	ARG_UNUSED(t);
	if (s->count > 0) { s->count--; return 0; }
	return -11;
}
static inline void k_sem_give(struct k_sem *s) { s->count = 1; }

typedef long atomic_t;
typedef long atomic_val_t;
static inline long atomic_get(const atomic_t *t) { return *t; }
static inline void atomic_set(atomic_t *t, long v) { *t = v; }
static inline void atomic_inc(atomic_t *t) { (*t)++; }
static inline void atomic_or(atomic_t *t, long v) { *t |= v; }
static inline void atomic_and(atomic_t *t, long v) { *t &= v; }

/* The sampler thread is never started by the test — test_cadence.c calls
 * detector_feed() directly so it controls the sample clock. */
#define K_THREAD_DEFINE(name, stack, fn, a, b, c, prio, opt, delay) \
	static void *name = (void *)0
static inline void k_thread_start(void *tid) { ARG_UNUSED(tid); }

#endif /* HOSTTEST_ZEPHYR_KERNEL_H */
