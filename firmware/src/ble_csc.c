/*
 * BLE Cycling Speed and Cadence peripheral — see ble_csc.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ble_csc.h"
#include "cadence.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <nrfx.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_csc, LOG_LEVEL_INF);

/* What this sensor supports, declared ONCE.
 *
 * The CSC profile puts speed and cadence in one service and uses the same two
 * bit positions in two different places: the Feature characteristic (what the
 * sensor CAN do) and the Measurement flags (what THIS packet contains). Those
 * are easy to write separately and easy to get out of step, and claiming wheel
 * data you do not send is not a cosmetic mislabel — the collector reads the
 * four bytes after the flags as a wheel revolution count, so every crank field
 * lands at the wrong offset and the cadence it displays is garbage.
 *
 * So both are derived from this one definition, along with the PDU length and
 * the field offsets. A crank-mounted sensor cannot see the wheel; adding
 * CSC_WHEEL_BIT here would move the crank fields and change the length
 * automatically, and the assertions below would hold. */
#define CSC_WHEEL_BIT 0x01
#define CSC_CRANK_BIT 0x02

#define CSC_SUPPORTED (CSC_CRANK_BIT)

#define CSC_HAS_WHEEL (!!(CSC_SUPPORTED & CSC_WHEEL_BIT))
#define CSC_HAS_CRANK (!!(CSC_SUPPORTED & CSC_CRANK_BIT))

/* flags(1) [+ wheel revs(4) + wheel event(2)] [+ crank revs(2) + crank event(2)] */
#define CSC_WHEEL_OFFSET 1
#define CSC_CRANK_OFFSET (1 + (CSC_HAS_WHEEL ? 6 : 0))
#define CSC_MEAS_LEN     (CSC_CRANK_OFFSET + (CSC_HAS_CRANK ? 4 : 0))

BUILD_ASSERT(CSC_SUPPORTED != 0,
	     "a CSC sensor that reports neither wheel nor crank data is not a sensor");

/* The guard that matters. Setting CSC_WHEEL_BIT above would widen the PDU and
 * push the crank fields to offset 7 — correctly — but nothing in this firmware
 * writes bytes 1..6, so the sensor would transmit six bytes of stack garbage
 * as a wheel revolution count, and any collector would then read the crank
 * fields as valid data derived from it. That is the exact failure this whole
 * single-definition arrangement exists to prevent, so it is asserted rather
 * than left to whoever edits CSC_SUPPORTED next: flag wheel data only once
 * something actually fills pdu[CSC_WHEEL_OFFSET]. */
/* ASCII only in the message: GCC renders a UTF-8 dash as octal escapes in the
 * diagnostic, and an assertion nobody can read is an assertion that does not
 * do its job. */
BUILD_ASSERT(!CSC_HAS_WHEEL,
	     "CSC_WHEEL_BIT is set but nothing fills the wheel fields - "
	     "a crank-mounted sensor cannot see the wheel");
BUILD_ASSERT(CSC_MEAS_LEN == 5,
	     "crank-only CSC Measurement must be flags(1) + revs(2) + event(2)");

/* Sensor Location enumeration, 5 = "Left Crank". Nothing in the firmware
 * depends on which crank it is — the value is advisory, and some head units
 * display it. Change it here if the sensor ends up on the drive side. */
#define SENSOR_LOCATION_LEFT_CRANK 5

/* Placeholder battery level. See the note in ble_csc_init() for why it is 50
 * and not 100 — it is a debug marker, not an estimate, and it goes away the
 * moment a board can actually measure its battery. */
#define BATTERY_DEBUG_PERCENT 50

/* Sized from the Kconfig limit rather than a round number, so lengthening
 * CONFIG_BT_DEVICE_NAME cannot silently truncate the advertised name. */
static char s_name[CONFIG_BT_DEVICE_NAME_MAX + 1];
static char s_addr[BT_ADDR_LE_STR_LEN];
/* Links currently up, 0..CONFIG_BT_MAX_CONN. A count rather than a flag
 * because this sensor accepts more than one collector at a time — see the note
 * on CONFIG_BT_MAX_CONN in prj.conf for why that is not optional on BLE.
 * Written only from the connection callbacks, which all run on the same
 * thread, and read elsewhere as a single byte. */
static uint8_t s_conn_count;
static bool s_advertising;
static bool s_subscribed;

/* --- GATT --------------------------------------------------------------- */

static void meas_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	/* Zephyr passes the AGGREGATE of every connection's CCC config, not the
	 * one that just changed, so this stays correct with two collectors
	 * attached: it means "at least one of them wants notifications". */
	s_subscribed = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("CSC notifications %s", s_subscribed ? "on" : "off");
}

static ssize_t read_feature(struct bt_conn *conn,
			    const struct bt_gatt_attr *attr, void *buf,
			    uint16_t len, uint16_t offset)
{
	ARG_UNUSED(attr);

	uint16_t feature = sys_cpu_to_le16(CSC_SUPPORTED);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &feature,
				 sizeof(feature));
}

static ssize_t read_location(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr, void *buf,
			     uint16_t len, uint16_t offset)
{
	static const uint8_t location = SENSOR_LOCATION_LEFT_CRANK;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &location,
				 sizeof(location));
}

BT_GATT_SERVICE_DEFINE(csc_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_CSC),
	BT_GATT_CHARACTERISTIC(BT_UUID_CSC_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(meas_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_CSC_FEATURE, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_feature, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_LOCATION, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_location, NULL, NULL),
);

/* attrs[1] is the CSC Measurement value attribute. */
#define CSC_MEAS_ATTR (&csc_svc.attrs[1])

void ble_csc_notify(void)
{
	struct cadence_state st;
	uint8_t pdu[CSC_MEAS_LEN];

	if (s_conn_count == 0 || !s_subscribed) {
		return;
	}

	cadence_get(&st);

	/* The flags byte says what follows, so it is the same value the Feature
	 * characteristic advertises — this sensor sends every field it claims,
	 * every time. */
	pdu[0] = CSC_SUPPORTED;
	sys_put_le16(st.revs, &pdu[CSC_CRANK_OFFSET]);
	sys_put_le16(st.last_event_1024, &pdu[CSC_CRANK_OFFSET + 2]);

	/* NULL fans the notification out to every connection that has enabled
	 * them, so a head unit and a phone each get the same measurement
	 * without this having to track how many collectors are attached. */
	int err = bt_gatt_notify(NULL, CSC_MEAS_ATTR, pdu, sizeof(pdu));

	if (err && err != -ENOTCONN) {
		LOG_WRN("notify: %d", err);
	}
}

/* --- advertising -------------------------------------------------------- */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	/* Head units scan for the service UUID, not for a name, so this is the
	 * element that actually gets the sensor discovered. */
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_CSC_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
	/* Appearance, so a scanner can say "cadence sensor" BEFORE connecting.
	 * The same value also lives in the GAP service, but that one can only
	 * be read after a connection, which is too late to help someone
	 * choosing from a list. Derived from the Kconfig rather than written
	 * out, so the two cannot disagree. Costs 4 of the 22 spare bytes. */
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		      (CONFIG_BT_DEVICE_APPEARANCE & 0xff),
		      ((CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff)),
};

static struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, s_name, 0),
};

void ble_csc_set_advertising(bool on)
{
	int err;

	/* Asking to be findable with every connection slot already taken is not
	 * an error, it is just nothing to do — the controller would reject the
	 * start for want of a connection object and main() calls this once a
	 * second, so letting it through would fill the log at 1 Hz. Folding it
	 * into `on` here rather than gating the call site keeps the caller
	 * expressing intent ("the bike is moving, be findable") and leaves this
	 * file the only thing that has to know how many links are free. */
	if (on && s_conn_count >= CONFIG_BT_MAX_CONN) {
		on = false;
	}

	if (on == s_advertising) {
		return;
	}

	if (on) {
		sd[0].data_len = (uint8_t)strlen(s_name);

		/* 100-150 ms. The profile wants a sensor to be found quickly
		 * when a rider powers up next to their head unit; this is the
		 * standard "fast connect" window, and advertising only runs
		 * while the bike is actually moving anyway. */
		err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad),
				      sd, ARRAY_SIZE(sd));
		if (err) {
			LOG_ERR("adv start: %d", err);
			return;
		}
	} else {
		err = bt_le_adv_stop();
		if (err) {
			LOG_ERR("adv stop: %d", err);
			return;
		}
	}

	s_advertising = on;
}

/* --- connection --------------------------------------------------------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
	ARG_UNUSED(conn);

	/* The controller has stopped advertising either way — on success
	 * because it is now connected, on failure because the attempt consumed
	 * the advertiser. Clearing the flag unconditionally is what lets the
	 * once-a-second call in main() start it again; leaving it set on the
	 * failure path would make the sensor go quiet until the next reboot.
	 *
	 * With a connection slot still free, that same restart is also the only
	 * thing that lets a SECOND collector find the sensor: a peripheral stops
	 * advertising the instant it is connected, so without this the first
	 * device to connect would keep the sensor to itself. */
	s_advertising = false;

	if (err) {
		LOG_WRN("connect failed: 0x%02x", err);
		return;
	}

	s_conn_count++;

	/* A connected head unit expects measurements to keep arriving even
	 * when the rider stops pedalling — that is how it displays 0 rather
	 * than freezing on the last number. */
	cadence_hold_awake(CADENCE_HOLD_BLE, true);

	LOG_INF("connected (%u/%u)", s_conn_count, CONFIG_BT_MAX_CONN);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);

	if (s_conn_count > 0) {
		s_conn_count--;
	}

	/* Only the LAST collector leaving releases the hold. Dropping it
	 * whenever either link goes would park the sampler while the other one
	 * is still subscribed and waiting for its measurement every second.
	 *
	 * s_subscribed is normally maintained by meas_ccc_changed(), which
	 * Zephyr calls as it clears the departing connection's CCC config;
	 * clearing it here too is unconditionally right once nothing is
	 * connected, and does not depend on the order of the two callbacks. */
	if (s_conn_count == 0) {
		s_subscribed = false;
		cadence_hold_awake(CADENCE_HOLD_BLE, false);
	}

	LOG_INF("disconnected (0x%02x, %u/%u left)", reason, s_conn_count,
		CONFIG_BT_MAX_CONN);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* --- battery ------------------------------------------------------------ */

void ble_csc_set_battery(uint8_t percent)
{
	if (percent > 100) {
		percent = 100;
	}

	bt_bas_set_battery_level(percent);
}

/* --- init --------------------------------------------------------------- */

int ble_csc_init(void)
{
	/* Pin the identity to the factory device id BEFORE bt_enable().
	 *
	 * Zephyr otherwise generates a fresh random static address on every
	 * boot. For a cadence sensor that is not cosmetic: a head unit pairs
	 * the sensor once and then remembers it BY ADDRESS, so a sensor that
	 * changes address on every battery change appears as a new device the
	 * rider has to go and re-add. On macOS the same change also rotates
	 * the CoreBluetooth peripheral UUID. Deriving the address from FICR
	 * costs nothing to provision, survives a reflash, and needs no
	 * settings partition (a preset address is not persisted). */
	bt_addr_le_t addr = { .type = BT_ADDR_LE_RANDOM };

	sys_put_le32(NRF_FICR->DEVICEID[0], &addr.a.val[0]);
	sys_put_le16((uint16_t)NRF_FICR->DEVICEID[1], &addr.a.val[4]);
	addr.a.val[5] |= 0xC0; /* random static requires the top two bits set */

	int err = bt_id_create(&addr, NULL);

	if (err < 0) {
		LOG_WRN("bt_id_create: %d (falling back to a generated address)",
			err);
	}

	/* The advertised name. CONFIG_BT_DEVICE_NAME_PREFIX is the human part;
	 * the suffix comes from the factory device id so that two of these
	 * built from the same source are still tellable apart in a scan list.
	 */
	snprintf(s_name, sizeof(s_name), "%s-%04X", CONFIG_BT_DEVICE_NAME,
		 (unsigned int)(NRF_FICR->DEVICEID[0] & 0xffff));

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable: %d", err);
		return err;
	}

	/* Push the same string into the GAP Device Name characteristic.
	 *
	 * Without this there are two different names: the one advertised in the
	 * scan response, and CONFIG_BT_DEVICE_NAME sitting in 0x2A00. Hosts
	 * differ over which they show and when — several display the advertised
	 * name while scanning and then replace it with the GAP name once
	 * connected, so the sensor a rider picked from a list is not the one
	 * their head unit ends up labelling. One name, set in one place. */
	err = bt_set_name(s_name);
	if (err) {
		LOG_WRN("bt_set_name: %d (GAP name stays '%s')", err,
			CONFIG_BT_DEVICE_NAME);
	}

	bt_addr_le_to_str(&addr, s_addr, sizeof(s_addr));

	/* Seed the Battery Service with a DELIBERATELY ODD CONSTANT, which
	 * main() then overwrites within a second on any board that can actually
	 * measure itself (see src/battery.c).
	 *
	 * 50 rather than 100 for a specific reason: 100 % is what a host shows
	 * when it has no battery information at all, so it is indistinguishable
	 * from the Battery Service never being written. 50 % cannot be confused
	 * with a default — a head unit showing half a battery means the
	 * plumbing works and only the measurement is missing.
	 *
	 * Which is exactly what it still means on the bring-up board, where
	 * BAT_CHECK landed on P0.09 and the nRF52840's analog inputs are
	 * P0.02-P0.05 and P0.28-P0.31 only. There the 50 % simply stays. */
	ble_csc_set_battery(BATTERY_DEBUG_PERCENT);

	LOG_INF("BLE up as %s (%s)", s_name, s_addr);

	return 0;
}

bool ble_csc_is_connected(void)
{
	return s_conn_count > 0;
}

uint8_t ble_csc_conn_count(void)
{
	return s_conn_count;
}

bool ble_csc_is_advertising(void)
{
	return s_advertising;
}

const char *ble_csc_name(void)
{
	return s_name;
}

const char *ble_csc_addr_str(void)
{
	return s_addr;
}
