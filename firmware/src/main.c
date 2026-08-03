/*
 * A bicycle cadence sensor.
 *
 * Bolt it to a crank arm. It watches gravity rotate through the accelerometer
 * once per pedal stroke (src/cadence.c explains why that works even when
 * centripetal acceleration is several times larger than gravity) and reports
 * the count over the standard BLE Cycling Speed and Cadence profile, so a
 * head unit or phone app picks it up as an ordinary cadence sensor.
 *
 * The whole power budget is "do not transmit when the bike is not moving":
 *   parked        -> sampler parked on the IMU's wake-up interrupt, no radio
 *   moving        -> sampling at 64 Hz, advertising
 *   connected     -> as above, plus a measurement per revolution and at 1 Hz
 *
 * Diagnostics are on the USB-CDC console, and only when USB is actually
 * plugged in — on battery the port is never enumerated, because enumerating
 * it costs more current than the radio does. Commands (single key + Enter):
 *   h = help   s = status   c = live cadence   a = accel   r = reboot
 *
 * Firmware updates go over the SECOND USB-CDC endpoint via mcumgr SMP
 * (tools/flash_usb.command) with the application running. SWD is needed only
 * for the very first flash, which is what installs MCUboot.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/bluetooth/services/bas.h>
#include <hal/nrf_power.h>

#include "ble_csc.h"
#include "cadence.h"
#include "lsm6dsv.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* How far gravity has to swing before INT1 fires. The most sensitive setting
 * the part offers, deliberately: a turning crank sweeps gravity through a full
 * circle, so there is nothing to gain by being coarse, and being sensitive is
 * what keeps a slow, gentle first pedal stroke from being missed. */
#define WAKE_ORIENTATION_DEG LSM6DSV_6D_50DEG

/* What counts as "still" for the part's own inactivity timer, and for how long
 * before it drops the accelerometer to 1.875 Hz. 60 s is long enough that a
 * traffic light does not cycle it, short enough that a bike left in a shed is
 * down to microamps within the minute. */
#define INACT_THRESH_MG 120
#define INACT_QUIET_S   60

/* How long after the last sign of motion the sensor keeps advertising. A rider
 * stopped at a traffic light is still riding; dropping the advertisement (and
 * with it the head unit's ability to reconnect) after ten seconds of standing
 * still would be worse than useless. */
#define ADV_HOLD_MS (2 * 60 * 1000)

/* Main loop period. Everything here is slow; this only needs to be fast
 * enough that typing into the console feels responsive. */
#define TICK_MS 100

static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(DT_ALIAS(led_red), gpios);
static const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(DT_ALIAS(led_green), gpios);
static const struct gpio_dt_spec led_b = GPIO_DT_SPEC_GET(DT_ALIAS(led_blue), gpios);

static const struct gpio_dt_spec imu_int1 =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), imu_int1_gpios);

static const struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static struct gpio_callback imu_int1_cb;

/* Written from the INT1 ISR, read from the main loop. */
static atomic_t last_motion_ms;
static atomic_t motion_events;

static bool usb_present;
static bool int1_ok;
static bool usb_held;
static bool stream_cadence;
static uint32_t seen_revs;

/* --- LEDs ------------------------------------------------------------- */

static void leds_init(void)
{
	gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_b, GPIO_OUTPUT_INACTIVE);
}

/* Blink only while USB is attached. On battery the LED would be the single
 * largest current draw in the whole design — a 2 mA blink once a minute
 * outweighs the radio. */
static void blink(const struct gpio_dt_spec *led, int times, int on_ms)
{
	if (!usb_present) {
		return;
	}

	for (int i = 0; i < times; i++) {
		gpio_pin_set_dt(led, 1);
		k_msleep(on_ms);
		gpio_pin_set_dt(led, 0);
		if (i + 1 < times) {
			k_msleep(on_ms);
		}
	}
}

/* --- motion ----------------------------------------------------------- */

static void imu_int1_handler(const struct device *dev, struct gpio_callback *cb,
			     uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	atomic_set(&last_motion_ms, (atomic_val_t)k_uptime_get_32());
	atomic_inc(&motion_events);

	cadence_notify_motion();
}

static int motion_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&imu_int1)) {
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&imu_int1, GPIO_INPUT);
	if (err) {
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&imu_int1, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		return err;
	}

	gpio_init_callback(&imu_int1_cb, imu_int1_handler, BIT(imu_int1.pin));

	return gpio_add_callback(imu_int1.port, &imu_int1_cb);
}

static bool recently_moved(void)
{
	uint32_t last = (uint32_t)atomic_get(&last_motion_ms);

	if (!lsm6dsv_present()) {
		/* No IMU means no wake-up interrupt either. Advertise anyway
		 * rather than going permanently silent — a sensor that can be
		 * connected to and reports zero is diagnosable; one that never
		 * appears looks like a flat battery. */
		return true;
	}

	return (k_uptime_get_32() - last) < ADV_HOLD_MS;
}

/* --- USB console ------------------------------------------------------ */

static bool vbus_present(void)
{
	return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
}

/* Bring USB up. Split from console_init() so the main loop can call it when a
 * cable appears later, without the boot-time wait for a terminal. */
static bool usb_start(void)
{
	if (usb_present) {
		return true;
	}
	if (usb_enable(NULL)) {
		LOG_ERR("usb_enable failed");
		return false;
	}
	usb_present = true;

	return true;
}

static void console_init(void)
{
	uint32_t dtr = 0;

	if (!vbus_present()) {
		return;
	}

	if (!usb_start()) {
		return;
	}

	/* Give a terminal a few seconds to attach, but never block forever:
	 * a USB charger asserts VBUS and will never raise DTR. */
	for (int i = 0; i < 50 && !dtr; i++) {
		uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
		k_msleep(100);
	}
}

static void print_status(void)
{
	struct cadence_state st;
	int16_t mg[3] = {0, 0, 0};

	cadence_get(&st);
	(void)lsm6dsv_read_accel_mg(mg);

	printk("\n--- cadence sensor ---\n");
	printk("name      : %s\n", ble_csc_name());
	printk("BLE addr  : %s\n", ble_csc_addr_str());
	printk("link      : %s\n",
	       ble_csc_is_connected() ? "connected" :
	       ble_csc_is_advertising() ? "advertising" : "quiet");
	/* Printed so "the app says 50, the head unit says 100" is immediately
	 * distinguishable from "the app never set it". */
	printk("battery   : %u %% (placeholder — this PCB cannot measure it)\n",
	       bt_bas_get_battery_level());
	printk("IMU       : %s\n", lsm6dsv_present() ? "ok" : "ABSENT");
	printk("INT1      : %s\n",
	       !lsm6dsv_present() ? "n/a" :
	       int1_ok ? "armed on orientation change (6D)" :
	       "FAILED TO ARM — held awake, battery life will be poor");
	printk("sampler   : %s\n", st.idle ? "idle (waiting on INT1)" : "running");
	printk("cadence   : %u.%u rpm (%s)\n", st.rpm_x10 / 10, st.rpm_x10 % 10,
	       st.rotating ? "turning" : "stopped");
	printk("revs      : %u (CSC field %u, event %u)\n", st.total_revs,
	       st.revs, st.last_event_1024);
	printk("plane     : axes %u,%u   amp %u mg (1000 = ideal)\n",
	       st.plane[0], st.plane[1], st.amp_mg);
	printk("accel mg  : %d %d %d\n", mg[0], mg[1], mg[2]);
	printk("motion evt: %u\n", (uint32_t)atomic_get(&motion_events));
	printk("uptime    : %u s\n", (uint32_t)(k_uptime_get() / 1000));
}

static void print_help(void)
{
	printk("\nh = help   s = status   c = live cadence   a = accel   r = reboot\n");
}

static void console_poll(void)
{
	unsigned char c;

	if (!usb_present) {
		return;
	}

	while (uart_poll_in(console_dev, &c) == 0) {
		switch (c) {
		case 'h':
			print_help();
			break;
		case 's':
			print_status();
			break;
		case 'c':
			stream_cadence = !stream_cadence;
			printk("live cadence %s\n", stream_cadence ? "on" : "off");
			break;
		case 'a': {
			int16_t mg[3];

			if (lsm6dsv_read_accel_mg(mg) == 0) {
				printk("accel mg: %d %d %d\n", mg[0], mg[1], mg[2]);
			} else {
				printk("accel: no IMU\n");
			}
			break;
		}
		case 'r':
			printk("rebooting...\n");
			k_msleep(50);
			sys_reboot(SYS_REBOOT_COLD);
			break;
		case '\r':
		case '\n':
			break;
		default:
			printk("? (h for help)\n");
			break;
		}
	}
}

/* --- revolution callback ---------------------------------------------- */

/* Runs on the sampler thread. Notifying from here rather than from the 1 Hz
 * tick is what makes the head unit's RPM smooth: it gets each revolution's
 * own timestamp instead of whichever one happened to be latest at tick time. */
static void on_revolution(void)
{
	ble_csc_notify();
}

/* --- main ------------------------------------------------------------- */

int main(void)
{
	uint32_t tick = 0;
	bool led_on = false;

	leds_init();
	console_init();

	printk("\ncadence sensor starting\n");

	/* Arm INT1, and treat any failure as "this sensor must not be allowed to
	 * sleep". The sampler has no timer of its own to fall back on, so an
	 * interrupt that cannot be armed is the difference between a sensor that
	 * drains faster than it should and one that is silent forever. */
	if (lsm6dsv_init() == 0) {
		int err = lsm6dsv_arm_orientation_wake(WAKE_ORIENTATION_DEG,
						       INACT_THRESH_MG,
						       INACT_QUIET_S);

		if (err || motion_init()) {
			LOG_ERR("INT1 could not be armed (%d) — staying awake; "
				"battery life will be poor but cadence works",
				err);
			cadence_hold_awake(CADENCE_HOLD_FAULT, true);
			int1_ok = false;
		} else {
			int1_ok = true;
		}
	} else {
		LOG_ERR("no IMU — cadence will read zero");
	}

	/* Count boot as motion.
	 *
	 * Without this a board that is powered on and left alone never
	 * advertises at all: recently_moved() is driven entirely by the IMU's
	 * wake-up interrupt, which has not fired yet, so the sensor sits
	 * silent and invisible. That is exactly backwards — the moment a rider
	 * most needs to FIND this thing is the moment they have just put a
	 * battery in it and are trying to add it to a head unit. So the first
	 * ADV_HOLD_MS after power-up are treated as "recently moved", and it
	 * goes quiet after that if nothing has actually moved.
	 *
	 * (FindMyTag booted into its fast-advertising state for the same
	 * reason: a device that has just been switched on is, almost by
	 * definition, in someone's hands.) */
	atomic_set(&last_motion_ms, (atomic_val_t)k_uptime_get_32());

	cadence_set_rev_callback(on_revolution);

	if (cadence_init()) {
		LOG_WRN("cadence detector not started (no IMU)");
	}

	if (ble_csc_init()) {
		/* Without the radio this is not a sensor. Blink red and stop
		 * rather than pretending to work. */
		LOG_ERR("BLE failed to start");
		while (1) {
			blink(&led_r, 1, 100);
			k_msleep(900);
		}
	}

	printk("advertising as %s (%s)\n", ble_csc_name(), ble_csc_addr_str());
	blink(&led_g, 2, 80);
	print_help();

	while (1) {
		k_msleep(TICK_MS);

		/* USB appearing is the manual way in. With the sampler's
		 * periodic wake-up gone, this and INT1 are the only two things
		 * that can start it — so a human with a cable can always reach
		 * a sensor whose interrupt is misbehaving. */
		bool vbus = vbus_present();

		if (vbus && !usb_present) {
			usb_start();
			print_help();
		}

		/* Track the hold against VBUS ITSELF, not against the
		 * enumeration transition. A board that boots with the cable
		 * already plugged in has usb_present set by console_init()
		 * before this loop ever runs, so an edge-triggered version
		 * never fires and the sampler parks with a console attached —
		 * which is exactly the case you are in whenever you are trying
		 * to watch it work. */
		if (vbus != usb_held) {
			cadence_hold_awake(CADENCE_HOLD_USB, vbus);
			usb_held = vbus;
		}

		console_poll();

		/* One LED pulse per revolution, cleared on the next tick.
		 * Doing it here and not in the revolution callback keeps the
		 * sampler thread free of anything that sleeps. */
		struct cadence_state st;

		cadence_get(&st);

		if (st.total_revs != seen_revs) {
			seen_revs = st.total_revs;
			if (usb_present) {
				gpio_pin_set_dt(&led_b, 1);
				led_on = true;
			}
		} else if (led_on) {
			gpio_pin_set_dt(&led_b, 0);
			led_on = false;
		}

		if (++tick < (1000 / TICK_MS)) {
			continue;
		}
		tick = 0;

		/* Once per second from here down. */

		/* A connected head unit needs a measurement even when nothing
		 * turned: an unchanged revolution count with a fresh timestamp
		 * is how the CSC profile says "cadence is zero". */
		ble_csc_notify();

		if (!ble_csc_is_connected()) {
			ble_csc_set_advertising(st.rotating || recently_moved());
		}

		if (stream_cadence) {
			printk("rpm %u.%u  revs %u  amp %4u mg  plane %u,%u  %s%s\n",
			       st.rpm_x10 / 10, st.rpm_x10 % 10, st.total_revs,
			       st.amp_mg, st.plane[0], st.plane[1],
			       st.rotating ? "turning" : "stopped",
			       st.idle ? " (idle)" : "");
		}
	}

	return 0;
}
