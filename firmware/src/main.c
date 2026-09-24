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
#include <zephyr/linker/devicetree_regions.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/poweroff.h>
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

/* How long USB stays up with nothing attached to either endpoint.
 *
 * A cable does not mean a host. The common case by far is a charger, or a PC
 * that nobody has opened a terminal on, and USB costs about 1.3 mA - which on
 * this board comes out of the PACK, not out of the cable: the LDO's input is
 * the charger's output, and once charging terminates the charger stops
 * sourcing, so the sensor quietly eats its own battery while apparently
 * plugged in. Left overnight that is most of a tenth of the cell, and the next
 * ride starts at 90 %.
 *
 * Generous on purpose. Five minutes of USB is 0.1 mAh, 0.07 % of the pack, and
 * a firmware update needs somewhere to fit. Unplugging and replugging always
 * brings it back.
 */
#define USB_IDLE_MS (5 * 60 * 1000)

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

/* How long everything has to stay quiet before the sensor stops being a running
 * computer and becomes a switch waiting to be flipped.
 *
 * System OFF takes the parked draw from 30 uA to about 8, because it stops
 * paying for the parts of "idle" that are not free: the core, the clocks and
 * the Bluetooth stack all go away, and what is left is the accelerometer still
 * watching for motion, the battery divider, the protection IC and a sleeping
 * core. Measured, not projected. Roughly doubles the life of a charge.
 *
 * Ten minutes rather than the two that advertising uses, and the margin costs
 * almost nothing: 30 uA for ten minutes is 5 uAh, so even waking once an hour
 * all year spends under a tenth of the pack. What it buys is the certainty that
 * a rider stopped at a long light, or fixing a puncture, is never switched off
 * underneath them - the sensor is asleep only when nothing has touched it for
 * longer than any pause in a ride.
 */
#define SYSTEM_OFF_MS (10 * 60 * 1000)

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
static const struct device *smp_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_uart_mcumgr));

static struct gpio_callback imu_int1_cb;

/* Written from the INT1 ISR, read from the main loop. */
static atomic_t last_motion_ms;
static atomic_t motion_events;

static bool usb_present;
static uint32_t reset_reason;
static bool usb_idled;
static uint32_t usb_idle_since;
static bool int1_ok;
static bool usb_held;
static bool ble_held;

/* --- lifetime accounting ---------------------------------------------- *
 *
 * Every wake from System OFF is a reset, so uptime and the duty counters
 * describe only the current stretch — which is useless for the one question
 * the field keeps asking: a pack went flat in eight days, WHERE did it go?
 * Without an ammeter in series the answer has to come from the sensor itself,
 * and the only thing it needs to report is how it spent its time.
 *
 * Surviving a wake takes more than keeping the startup code away from it.
 * System OFF powers RAM down section by section, and sys_poweroff() turns
 * retention off for ALL of RAM on its way out — so this lives in the 4 KB
 * region the board's devicetree declares as zephyr,retained-ram, which is the
 * one thing poweroff switches back on before it stops the core. An earlier
 * version set the retention bits by hand just before sleeping; they were
 * cleared again two statements later inside sys_poweroff(), and the counters
 * came back empty with nothing to say why.
 *
 * Placed by section rather than with __noinit for the same reason: __noinit
 * only promises the startup code will not zero it, which says nothing about
 * where it lands or whether that RAM is still powered.
 *
 * What they do not survive is the rail actually collapsing: a flat battery, a
 * disconnected cell, the latch released. That is the right place to start over
 * — the question these answer is "where did THIS charge go" — and the magic
 * word is what notices.
 */
#define LIFETIME_MAGIC 0x43414431UL /* "CAD1" */

static __attribute__((section(LINKER_DT_NODE_REGION_NAME(
	DT_PARENT(DT_NODELABEL(retained_mem)))))) struct {
	uint32_t magic;
	uint32_t wakes;   /* System OFF -> awake transitions          */
	uint32_t awake_s; /* seconds NOT in System OFF                */
	uint32_t conn_s;  /* seconds with at least one collector up   */
} lifetime;

static void lifetime_init(void)
{
	if (lifetime.magic != LIFETIME_MAGIC) {
		memset(&lifetime, 0, sizeof(lifetime));
		lifetime.magic = LIFETIME_MAGIC;
	}

	lifetime.wakes++;
}

static bool stream_cadence;
static uint32_t seen_revs;
static uint16_t batt_mv;
static uint8_t batt_flat;

/* Seconds since boot, and how many of them the sampler was awake for and the
 * radio was advertising for.
 *
 * The instrument this investigation was missing. A week of "the pack went from
 * full to a tenth" says only that something is wrong; it cannot say WHAT,
 * because getting from a voltage to a current runs through a discharge curve
 * and an assumed capacity, and being wrong about either changes the answer by
 * more than the thing being looked for. These two numbers are measured
 * directly and settle it: a sensor that parked properly and stayed quiet has
 * single-digit percentages here, and one that never slept has a hundred. */
static uint32_t duty_secs;
static uint32_t duty_awake;
static uint32_t duty_adv;

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

/* Why this boot happened. Read once and cleared, because the register latches
 * every reason since the last clear and would otherwise still be reporting the
 * power-on that happened days ago. */
static void reset_reason_capture(void)
{
	reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = 0xffffffffUL;
}

/* Woken out of System OFF by the DETECT signal - which here can only mean the
 * IMU's interrupt line, since nothing else is wired to wake us. */
static bool woke_from_system_off(void)
{
	return (reset_reason & POWER_RESETREAS_OFF_Msk) != 0;
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

	/* Three reasons, and the third is the one that makes System OFF work at
	 * all: waking from it IS a reset, so this function runs again with no
	 * finger on the button and no cable attached. Without this the sensor
	 * would wake, get as far as here, decide it had no business being on,
	 * and switch itself off again - which is exactly what the first attempt
	 * did before RESETREAS was consulted. */
	latched = pressed || vbus_present() || woke_from_system_off();

	gpio_pin_configure_dt(&soft_latch, latched ? GPIO_OUTPUT_ACTIVE
						   : GPIO_OUTPUT_INACTIVE);

	LOG_INF("power latch %s (button %s, VBUS %s, resetreas 0x%08x%s)",
		latched ? "held" : "RELEASED - powering off",
		pressed ? "down" : "up", vbus_present() ? "yes" : "no",
		(unsigned int)reset_reason,
		woke_from_system_off() ? " = woke from System OFF" : "");
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

/* Drop to System OFF once nothing has happened for SYSTEM_OFF_MS.
 *
 * This is the deepest state the part has: the core, the clocks and the whole
 * Bluetooth stack stop existing, and waking from it is a reset rather than a
 * return. What keeps running is the accelerometer, still watching for the
 * orientation change that will wake us, and the GPIO block - which matters
 * twice over, because GPIO is also what holds the 3.3 V rail up. System OFF
 * does not power GPIO down and PIN_CNF is retained, so the latch survives.
 *
 * INT1 has to be re-armed as a LEVEL interrupt on the way out. An edge
 * interrupt runs through GPIOTE, and GPIOTE is one of the peripherals System
 * OFF switches off; the only thing that can wake the chip is the pin's own
 * SENSE/DETECT circuit, which a level interrupt is what configures. Get this
 * wrong and the sensor sleeps forever.
 *
 * The guards are about never sleeping somewhere we cannot be woken from. If
 * INT1 was not armed, or there is no IMU at all, then nothing is watching the
 * pin and System OFF would be permanent - that case is already held awake by
 * CADENCE_HOLD_FAULT, and checked again here because the cost of being wrong
 * is a sensor that never comes back.
 */
static void maybe_system_off(const struct cadence_state *st)
{
	if (!lsm6dsv_present() || !int1_ok || !gpio_is_ready_dt(&imu_int1)) {
		return; /* nothing could wake us again */
	}

	if (vbus_present()) {
		return; /* on the cable: not our battery being spent */
	}

	if ((k_uptime_get_32() - (uint32_t)atomic_get(&last_motion_ms)) <
	    SYSTEM_OFF_MS) {
		return;
	}

	/* The crank has been still for ten minutes, so the ride is over and
	 * this sensor has nothing left to say. A collector still holding the
	 * link open does NOT get to veto that.
	 *
	 * It used to. `ble_csc_conn_count() > 0` was in the guard above, on the
	 * reasoning that a live link means someone is watching — but a central
	 * is under no obligation to ever let go, and an Apple Watch left in
	 * radio range does not. The sensor then stayed awake indefinitely,
	 * which is both the connection's own current and, through
	 * CADENCE_HOLD_BLE, a sampler pinned at 60 Hz instead of 1.875 Hz.
	 *
	 * So hang up and come back next tick. Disconnection is asynchronous:
	 * the callbacks land within a connection interval or two, the main loop
	 * then releases CADENCE_HOLD_BLE, the sampler parks on its next sample,
	 * and the tick after that finds every condition met. About a second in
	 * all, and no new state to get wrong. */
	if (ble_csc_conn_count() > 0) {
		printk("\nquiet for %d min - hanging up on %u collector(s)\n",
		       SYSTEM_OFF_MS / 60000, ble_csc_conn_count());
		ble_csc_disconnect_all();
		return;
	}

	if (!st->idle) {
		return;
	}

	printk("\nnothing for %d min - System OFF. INT1 wakes it.\n",
	       SYSTEM_OFF_MS / 60000);
	k_msleep(50); /* let the console drain */

	ble_csc_set_advertising(false);
	(void)gpio_pin_interrupt_configure_dt(&imu_int1, GPIO_INT_LEVEL_ACTIVE);

	sys_poweroff();
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
	usb_idle_since = k_uptime_get_32();

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
/* Is anybody actually there?
 *
 * DTR goes high when a host opens the port, which is exactly the distinction
 * that matters: a charger never raises it, and neither does a PC until someone
 * runs a terminal or a flashing tool.
 *
 * BOTH endpoints, and that is not symmetry for its own sake. tools/flash_usb.py
 * talks to the SMP endpoint and never opens the console, so watching only the
 * console would tear USB down in the middle of a firmware update.
 */
static bool host_attached(void)
{
	uint32_t dtr = 0;

	if (uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr) {
		return true;
	}

	dtr = 0;
	if (uart_line_ctrl_get(smp_dev, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr) {
		return true;
	}

	return false;
}

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
	/* The age matters more than the count. recently_moved() holds
	 * advertising up for ADV_HOLD_MS after the last interrupt, so this is
	 * what says whether the radio has any business being on. */
	printk("motion evt: %u (last %u s ago)\n",
	       (uint32_t)atomic_get(&motion_events),
	       (k_uptime_get_32() -
		(uint32_t)atomic_get(&last_motion_ms)) / 1000);
	printk("duty      : sampler %u %% / advertising %u %%  (over %u s)\n",
	       duty_secs ? duty_awake * 100 / duty_secs : 0,
	       duty_secs ? duty_adv * 100 / duty_secs : 0, duty_secs);
	if (gpio_is_ready_dt(&soft_latch)) {
		/* On a board that switches its own rail, "why am I still on"
		 * is a real question with a real answer. */
		printk("power     : latch %s, button %s\n",
		       latched ? "held" : "NOT HELD",
		       !gpio_is_ready_dt(&sw_read) ? "?" :
		       gpio_pin_get_dt(&sw_read) == 1 ? "down" : "up");
	}
	/* Which of the three USB states this is, because "no console" has
	 * three different causes and they need different reactions: no cable,
	 * a cable nobody is talking through, or a cable whose console timed
	 * out and needs a replug. */
	printk("usb       : %s\n",
	       !vbus_present()  ? "no cable" :
	       !usb_present     ? "cable, USB off (idle) - replug to wake it" :
	       host_attached()  ? "up, host attached" : "up, no host yet");
	/* Why the last boot happened, which after System OFF is the difference
	 * between "somebody moved the bike" and "something went wrong". */
	printk("boot      : resetreas 0x%08x%s\n", (unsigned int)reset_reason,
	       woke_from_system_off() ? " (woke from System OFF)" : "");
	printk("uptime    : %u s\n", (uint32_t)(k_uptime_get() / 1000));
	/* Since the pack was last connected, across every System OFF cycle.
	 * "awake" is what costs: System OFF is a rounding error next to it, so
	 * awake_s against wall-clock time since the last charge is the whole
	 * battery story without an ammeter. */
	printk("lifetime  : %u wakes, awake %u s, connected %u s\n",
	       lifetime.wakes, lifetime.awake_s, lifetime.conn_s);
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
	reset_reason_capture();
	power_latch_init();
	lifetime_init();

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

		uint32_t now = k_uptime_get_32();

		if (!vbus) {
			/* Cable gone. Tear USB down and re-arm, so the next
			 * insertion always gets a console even if the last one
			 * timed out. */
			if (usb_present) {
				usb_stop();
			}
			usb_idled = false;
		} else if (!usb_present) {
			/* usb_idled is what stops this from immediately undoing
			 * the timeout below and spinning. */
			if (!usb_idled) {
				usb_start();
				print_help();
			}
		} else if (host_attached()) {
			usb_idle_since = now;
		} else if ((now - usb_idle_since) > USB_IDLE_MS) {
			usb_stop();
			usb_idled = true;
			LOG_INF("USB idle - shutting it down to save the pack");
		}

		/* Hold the sampler awake while USB is UP, tracked as a level
		 * rather than an edge. A board that boots with the cable
		 * already plugged in has usb_present set by console_init()
		 * before this loop ever runs, so an edge-triggered version
		 * never fires and the sampler parks with a console attached —
		 * which is exactly the case you are in whenever you are trying
		 * to watch it work.
		 *
		 * Keyed on usb_present, not on VBUS: once USB has timed out
		 * there is nobody watching, and keeping the sampler running for
		 * an audience that went home is the whole thing this is
		 * supposed to stop. */
		if (usb_present != usb_held) {
			cadence_hold_awake(CADENCE_HOLD_USB, usb_present);
			usb_held = usb_present;
		}

		/* The same idea for a connected collector, and driven from here
		 * for the same reason: one owner per hold, tracked as a level.
		 *
		 * ble_csc.c used to take this hold on connect and drop it on
		 * disconnect, which made the 60 Hz sampler last exactly as long
		 * as the link did — and a link can outlast the ride by days. A
		 * head unit does need measurements after the rider stops, but it
		 * needs them from ble_csc_notify() on the 1 Hz tick, which runs
		 * whatever the sampler is doing; what the sampler is for is
		 * noticing the crank turn again, and INT1 does that from the
		 * idle state within about a seventh of a revolution.
		 *
		 * So hold it only while the bike is still interesting, which is
		 * the same window advertising uses. */
		const bool ble_hold = ble_csc_conn_count() > 0 && recently_moved();

		if (ble_hold != ble_held) {
			cadence_hold_awake(CADENCE_HOLD_BLE, ble_hold);
			ble_held = ble_hold;
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

		lifetime.awake_s++;
		if (ble_csc_conn_count() > 0) {
			lifetime.conn_s++;
		}

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

		/* Counted AFTER the call above, so the sample describes the
		 * state this second is actually spent in. */
		duty_secs++;
		if (!st.idle) {
			duty_awake++;
		}
		if (ble_csc_is_advertising()) {
			duty_adv++;
		}

		maybe_system_off(&st);

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
