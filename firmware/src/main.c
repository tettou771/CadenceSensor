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
 *   h = help   s = status   c = live cadence   a = accel
 *   r = reboot   q = power off      both confirmed with '!' — see console_poll()
 *                                   for why a single key is unsafe here
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

#include "battery.h"
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

/* How often to sample the pack. A battery cannot move faster than this, the
 * Battery Service is advisory, and each read is eight SAADC conversions. */
#define BATTERY_PERIOD_S 60

/* Consecutive empty readings before the sensor switches itself off.
 *
 * Five, at one a minute, means five minutes of agreement. That is not caution
 * for its own sake: a reading is taken whenever it falls due, including during
 * a radio transmission, and a momentary droop on a tired pack must not be able
 * to end a ride. Nothing is lost by waiting - a cell that has genuinely reached
 * the bottom of the curve is not going to climb back out in five minutes. */
#define BATTERY_FLAT_READINGS 5

static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(DT_ALIAS(led_red), gpios);
static const struct gpio_dt_spec led_b = GPIO_DT_SPEC_GET(DT_ALIAS(led_blue), gpios);

/* Green is optional. The RideFormTracker bring-up board carries an RGB LED;
 * the v1 cadence PCB has two discrete LEDs and no green at all. Where there is
 * none the "started up cleanly" blink borrows blue, never red — a success
 * indicator that shares a colour with the failure indicator is worse than no
 * indicator. */
#if DT_NODE_EXISTS(DT_ALIAS(led_green))
static const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(DT_ALIAS(led_green), gpios);
#define LED_BOOT_OK (&led_g)
#else
#define LED_BOOT_OK (&led_b)
#endif

static const struct gpio_dt_spec imu_int1 =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), imu_int1_gpios);

/* The board's own power switch, present only on PCBs that gate their 3.3 V
 * rail. GET_OR leaves these zeroed on boards that do not, and gpio_is_ready_dt()
 * then reports them absent, so one image still builds and runs on both. */
static const struct gpio_dt_spec soft_latch =
	GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), soft_latch_gpios, {0});
static const struct gpio_dt_spec sw_read =
	GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), sw_read_gpios, {0});

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
static uint16_t batt_mv;
static uint8_t batt_flat;

/* --- LEDs ------------------------------------------------------------- */

static void leds_init(void)
{
	gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led_b, GPIO_OUTPUT_INACTIVE);
#if DT_NODE_EXISTS(DT_ALIAS(led_green))
	gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE);
#endif
}

/* Blink, on battery as well as on USB.
 *
 * It used to be suppressed on battery, on the grounds that an LED is the
 * largest current draw in the design. That is true of anything PERIODIC, which
 * is why the per-revolution pulse further down is still gated on USB, and it is
 * not true of the two things that call this: a one-shot at boot, and a
 * fatal-error loop on a sensor that is not going to work anyway.
 *
 * Boot is the case that decided it. The v1 board is switched on by a button and
 * has no other output, so without a flash here a rider pressing that button
 * gets no confirmation the press took — indistinguishable from a flat battery
 * or a broken latch. Two 80 ms flashes cost about 0.09 uAh, once per power-on,
 * against a daily budget of 468 uAh. */
static void blink(const struct gpio_dt_spec *led, int times, int on_ms)
{
	for (int i = 0; i < times; i++) {
		gpio_pin_set_dt(led, 1);
		k_msleep(on_ms);
		gpio_pin_set_dt(led, 0);
		if (i + 1 < times) {
			k_msleep(on_ms);
		}
	}
}

/* --- power latch ------------------------------------------------------- */

static bool vbus_present(void)
{
	return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
}

/* Decide whether this board has any business being powered on, and act on it.
 *
 * The v1 board gates its 3.3 V rail with an LDO whose enable pin is pulled up
 * by exactly two things: the momentary button SW2 through D2, and this GPIO
 * through R12. Board init (boards/nordic/cadence_v1/board.c) has already
 * asserted the latch by the time main() runs -- it has to, because MCUboot can
 * spend twenty seconds swapping an image and nothing else would hold the supply
 * for that long. What is left for here is the JUDGEMENT:
 *
 *   button down    a human is switching the sensor on
 *   VBUS present   a cable is attached -- and, just as importantly, this is how
 *                  the board survives its own reboot. sys_reboot() from the
 *                  console and the reset mcumgr issues after a firmware upload
 *                  both land back here with nobody touching the button.
 *
 * Neither, and the latch is actively released: a board brought up by ESD or by
 * a leakage path across the enable node switches itself off rather than quietly
 * flattening the battery in a drawer. (The alternative, a pulldown resistor on
 * the enable node, would cost 0.33 uA forever even at 10 M -- about 2 % of the
 * whole budget -- which is why this is done in software.)
 *
 * READ BEFORE REARRANGING: decide first, drive once. The tempting shape is to
 * drive the pin low, then look for a reason to raise it again, so that being on
 * is visibly a decision. On this hardware that is destructive. Releasing the
 * latch does not merely stop holding the enable node up, it discharges it
 * THROUGH R12: 100 kohm against ~10 pF is a one-microsecond time constant, so
 * the rail is gone long before the next instruction. That is five orders of
 * magnitude faster than the nanoamp leakage that drains the same node when the
 * pin is left floating, and it is exactly what makes the intentional power-off
 * crisp -- but it means a low pulse "just while we decide" is a power-off.
 */
static bool latched;

static void power_latch_init(void)
{
	if (!gpio_is_ready_dt(&soft_latch)) {
		return; /* a board whose rail is always on */
	}

	bool pressed = false;

	/* No pull. SW_READ sits on the 100k/330k divider off the button, which
	 * already defines both levels; an internal pulldown (~13 kohm) would
	 * swamp it and the button would never read as pressed. */
	if (gpio_is_ready_dt(&sw_read) &&
	    gpio_pin_configure_dt(&sw_read, GPIO_INPUT) == 0) {
		pressed = gpio_pin_get_dt(&sw_read) == 1;
	}

	latched = pressed || vbus_present();

	gpio_pin_configure_dt(&soft_latch, latched ? GPIO_OUTPUT_ACTIVE
						   : GPIO_OUTPUT_INACTIVE);

	LOG_INF("power latch %s (button %s, VBUS %s)",
		latched ? "held" : "RELEASED - powering off",
		pressed ? "down" : "up", vbus_present() ? "yes" : "no");
}

/* Switch the board off, for real.
 *
 * Releasing the latch does not merely stop holding the LDO's enable node up, it
 * discharges it through R12: 100 kohm against ~10 pF, so the rail collapses in
 * microseconds and this function does not return.
 *
 * Except when it does. The button and the SWD probe can both keep the board
 * alive independently of the latch, and on a board with no latch at all there
 * is nothing to release. All three cases are real (the last one is the
 * RideFormTracker bring-up board), so say what happened rather than spinning in
 * a loop waiting for a power-down that is not coming.
 */
static void power_off(const char *why)
{
	printk("\npowering off: %s\n", why);
	LOG_WRN("powering off: %s", why);

	/* The only warning a rider gets — there is no display, and by
	 * definition the thing about to stop reporting is the battery. */
	blink(&led_r, 3, 120);

	if (!gpio_is_ready_dt(&soft_latch)) {
		LOG_ERR("no power latch on this board - cannot switch off");
		return;
	}

	gpio_pin_set_dt(&soft_latch, 0);
	latched = false;

	k_msleep(500);

	printk("still running - something else is holding the enable node "
	       "(button, or an SWD probe supplying 3V3)\n");
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

/* Tear USB back down when the cable goes away.
 *
 * Leaving it up was a single omission with three separate consequences, none
 * of which look related from the outside:
 *
 *   the pack drains. An enabled USB device keeps the peripheral and the high
 *   frequency crystal alive whether or not a host is there, which is hundreds
 *   of microamps against a budget whose entire standby figure is eight. A
 *   sensor that had been plugged in once measured flat within days.
 *
 *   the console never comes back. usb_present latching true means the "cable
 *   appeared" branch in the main loop can never fire again, so a re-inserted
 *   cable depends entirely on the peripheral choosing to re-enumerate itself.
 *   Sometimes it does. When it does not there is no way in at all, short of a
 *   debug probe, because the board latches its own power and cannot be made to
 *   reboot without the very console that is missing.
 *
 *   the blue LED blinks on battery. The per-revolution pulse is gated on
 *   usb_present precisely so it costs nothing on a ride, and a stuck flag
 *   quietly removes that gate. This is the symptom that gave the whole thing
 *   away: an LED flashing where it had been designed not to.
 */
static void usb_stop(void)
{
	if (!usb_present) {
		return;
	}

	usb_disable();
	usb_present = false;
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
	/* Both numbers, because "the head unit has it and the phone cannot"
	 * and "the phone has simply not connected yet" look identical from the
	 * phone end and are told apart only here. */
	printk("link      : %u/%u connected%s\n", ble_csc_conn_count(),
	       CONFIG_BT_MAX_CONN,
	       ble_csc_is_advertising() ? ", advertising" : "");
	/* Printed so "the app says 50, the head unit says 100" is immediately
	 * distinguishable from "the app never set it". The millivolts go out
	 * alongside because the percentage comes off a curve, and when it looks
	 * wrong the first question is always which half is wrong.
	 *
	 * ASCII only. This goes to a terminal whose encoding is not ours to
	 * choose, and a stray em dash came back as a mojibake byte. */
	if (battery_present()) {
		printk("battery   : %u %% (%u mV%s)\n",
		       bt_bas_get_battery_level(), batt_mv,
		       vbus_present() ? ", on charge - reads high" : "");
	} else {
		printk("battery   : %u %% (placeholder, this board cannot "
		       "measure it)\n", bt_bas_get_battery_level());
	}
	/* The WHO_AM_I goes out either way. On a failure it is the whole
	 * diagnosis in one byte: 0x00 or 0xff means the SPI link is not
	 * carrying data, anything else means a real part answered and this
	 * driver does not know it yet. */
	printk("IMU       : %s (WHO_AM_I 0x%02x)\n",
	       lsm6dsv_present() ? "ok" : "ABSENT", lsm6dsv_whoami());
	printk("INT1      : %s\n",
	       !lsm6dsv_present() ? "n/a" :
	       int1_ok ? "armed on orientation change (6D)" :
	       "FAILED TO ARM — held awake, battery life will be poor");
	printk("sampler   : %s\n", st.idle ? "idle (waiting on INT1)" : "running");
	printk("cadence   : %u.%u rpm (%s)\n", st.rpm_x10 / 10, st.rpm_x10 % 10,
	       st.rotating ? "turning" : "stopped");
	printk("revs      : %u (CSC field %u, event %u)\n", st.total_revs,
	       st.revs, st.last_event_1024);
	/* amp is the RADIUS OF THE CIRCLE gravity traces in the detected plane,
	 * not the strength of gravity, so it collapses to nothing when the
	 * crank is still. Saying so here rather than printing a bare 0 that
	 * reads like a fault. */
	printk("plane     : axes %u,%u   amp %u mg (%s)\n",
	       st.plane[0], st.plane[1], st.amp_mg,
	       st.rotating ? "1000 = ideal" : "only meaningful while turning");
	printk("accel mg  : %d %d %d\n", mg[0], mg[1], mg[2]);
	printk("motion evt: %u\n", (uint32_t)atomic_get(&motion_events));
	if (gpio_is_ready_dt(&soft_latch)) {
		/* On a board that switches its own rail, "why am I still on"
		 * is a real question with a real answer. */
		printk("power     : latch %s, button %s\n",
		       latched ? "held" : "NOT HELD",
		       !gpio_is_ready_dt(&sw_read) ? "?" :
		       gpio_pin_get_dt(&sw_read) == 1 ? "down" : "up");
	}
	printk("uptime    : %u s\n", (uint32_t)(k_uptime_get() / 1000));
}

static void print_help(void)
{
	printk("\nh = help   s = status   c = live cadence   a = accel\n"
	       "r = reboot   q = power off   (both need '!' to confirm)\n");
}

/* Destructive console commands take two keys, and the second one is '!'.
 *
 * This console shares its USB cable with the firmware update path, and
 * tools/flash_usb.py finds the SMP endpoint by writing SMP frames to BOTH CDC
 * ports and seeing which answers. SMP over serial is base64, so frame data
 * arrives in this parser as keystrokes on a completely routine basis - and
 * every letter and digit is in the base64 alphabet. A single-key 'r' or 'q' is
 * therefore not a command, it is a trap: an update can reboot or switch off the
 * board it is halfway through flashing.
 *
 * Not hypothetical. A single-key power-off broke a USB update the first time it
 * ran, in exactly this way: the confirm step wrote a frame containing 'q' to the
 * console, the board switched itself off mid-confirm, and MCUboot correctly
 * reverted the un-confirmed image on the next power-up.
 *
 * '!' is the confirmation key precisely because it is NOT in the base64
 * alphabet [A-Za-z0-9+/=], so no amount of frame data can produce the pair.
 * Anything else cancels, as does letting it go stale. */
#define CONFIRM_KEY  '!'
#define CONFIRM_MS   5000

static char pending;
static uint32_t pending_at;

static void arm(char what, const char *desc)
{
	pending = what;
	pending_at = k_uptime_get_32();
	printk("%s: press '%c' within %d s to confirm\n", desc, CONFIRM_KEY,
	       CONFIRM_MS / 1000);
}

static void console_poll(void)
{
	unsigned char c;

	if (!usb_present) {
		return;
	}

	if (pending && (k_uptime_get_32() - pending_at) > CONFIRM_MS) {
		printk("(expired)\n");
		pending = 0;
	}

	while (uart_poll_in(console_dev, &c) == 0) {
		if (pending) {
			char what = pending;

			pending = 0;

			if (c != CONFIRM_KEY) {
				printk("(cancelled)\n");
				continue;
			}
			if (what == 'r') {
				printk("rebooting...\n");
				k_msleep(50);
				sys_reboot(SYS_REBOOT_COLD);
			} else {
				power_off("requested from the console");
			}
			continue;
		}

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
			arm('r', "reboot");
			break;
		case 'q':
			/* The only practical way to exercise the low-battery
			 * cutoff without first flattening a cell: it runs the
			 * same power_off() the empty-pack path does. Press the
			 * button to bring the board back. */
			arm('q', "power off");
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
	uint32_t secs = 0;
	bool led_on = false;

	/* FIRST, before anything that can block or take time: on the v1 board
	 * this is what keeps the 3.3 V rail up once the button is released. */
	power_latch_init();

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

	/* Not fatal either way. A board that cannot read its battery still
	 * reports cadence, and ble_csc_init() has already put the deliberately
	 * odd 50 % into the Battery Service — which then simply stays there,
	 * saying "nobody measured this" rather than pretending to be full. */
	if (battery_init() == 0) {
		LOG_INF("battery sense ready");
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
	blink(LED_BOOT_OK, 2, 80);
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
		} else if (!vbus && usb_present) {
			usb_stop();
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

		/* Battery, once a minute and once immediately at startup, so a
		 * head unit connecting early does not sit on the placeholder
		 * for a whole minute. */
		if (battery_present() && (secs % BATTERY_PERIOD_S) == 0) {
			uint16_t mv;
			uint8_t pct;

			if (battery_read(&mv, &pct) == 0) {
				batt_mv = mv;
				ble_csc_set_battery(pct);

				/* Empty is defined by the PERCENTAGE, never by
				 * a millivolt threshold written out here.
				 *
				 * The curve in src/battery.c is the only place
				 * this firmware describes what a lithium cell
				 * does, and a cutoff voltage duplicated at this
				 * end would agree with it exactly until the
				 * first time somebody tuned that table - after
				 * which the sensor would either keep running
				 * below its own zero, or switch off while still
				 * reporting charge left. Zero percent means
				 * zero percent, by construction.
				 *
				 * Not while a cable is attached, though. The
				 * pack is being charged and the reading is
				 * elevated anyway, and a sensor that switches
				 * itself off mid-diagnosis is a sensor nobody
				 * can diagnose - which matters more than it
				 * sounds, because a charger fault presents
				 * exactly as a pack that will not come up. */
				if (pct == 0 && !vbus_present()) {
					if (++batt_flat >= BATTERY_FLAT_READINGS) {
						power_off("battery empty");
					}
				} else {
					batt_flat = 0;
				}
			}
		}
		secs++;

		/* Deliberately NOT gated on being disconnected: with a spare
		 * connection slot the sensor has to keep advertising while
		 * already connected, or the second collector never finds it.
		 * ble_csc_set_advertising() is what knows when the slots are
		 * full; this only says whether the bike is worth finding. */
		ble_csc_set_advertising(st.rotating || recently_moved());

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
