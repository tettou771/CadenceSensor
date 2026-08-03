/*
 * BLE Cycling Speed and Cadence (CSC) peripheral.
 *
 * Standard profile, so this shows up as an ordinary cadence sensor to a Garmin
 * or Wahoo head unit, to Zwift/TrainerRoad, and to iOS/Android cycling apps —
 * nothing has to be taught about it.
 *
 * CRANK DATA ONLY, and asserted as such in three places: the Feature
 * characteristic, the Measurement flags, and the GAP appearance (1155 "Cadence
 * Sensor", not 1157 "Speed and Cadence Sensor"). All three come from the single
 * CSC_SUPPORTED definition in ble_csc.c precisely so they cannot drift apart —
 * a sensor that flags wheel data it never sends makes collectors parse the
 * crank fields at the wrong offset, which shows up as nonsense cadence rather
 * than as an obviously missing feature.
 *
 * The one thing that stays ambiguous is outside the firmware's control: the
 * advertised service UUID 0x1816 is *named* "Cycling Speed and Cadence" and
 * there is no cadence-only alternative, so a scanner may label this a speed
 * and cadence sensor until it connects and reads the Feature characteristic.
 *
 *   0x1816 Cycling Speed and Cadence
 *     0x2A5B CSC Measurement    notify   flags(1) revs(2) event_time(2)
 *     0x2A5C CSC Feature        read     0x0002 = crank revolution data
 *     0x2A5D Sensor Location    read     5 = left crank
 *   0x180F Battery Service
 *   0x180A Device Information
 *
 * SC Control Point (0x2A55) is deliberately absent. The profile makes it
 * mandatory only for sensors that support wheel revolutions or multiple sensor
 * locations, and this supports neither, so the only thing implementing it would
 * add is a writable attribute.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef BLE_CSC_H
#define BLE_CSC_H

#include <stdbool.h>
#include <stdint.h>

/* Bring up the stack, pin the identity address, register the services.
 * Does not start advertising — ble_csc_set_advertising() does. */
int ble_csc_init(void);

/* Advertising follows motion, not power: a bike parked in a garage for a week
 * has no reason to broadcast, and a cadence sensor's whole battery budget is
 * advertising. Safe to call repeatedly with the same value. */
void ble_csc_set_advertising(bool on);

/* Read the current cadence state and push a CSC Measurement notification.
 * No-op when nothing is subscribed. Called on each revolution and once a
 * second regardless, because a head unit needs a fresh measurement with an
 * UNCHANGED revolution count to conclude that cadence has fallen to zero. */
void ble_csc_notify(void);

bool ble_csc_is_connected(void);
bool ble_csc_is_advertising(void);

/* Battery Service level, 0-100 %. See the note in ble_csc.c about why this
 * board always reports full. */
void ble_csc_set_battery(uint8_t percent);

/* "CAD-1A2B" — the advertised name, derived from the factory device id. */
const char *ble_csc_name(void);

/* The identity address, for the console. */
const char *ble_csc_addr_str(void);

#endif /* BLE_CSC_H */
