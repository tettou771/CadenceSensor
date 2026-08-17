/*
 * Battery voltage, for the BLE Battery Service.
 *
 * Optional: a board declares it by giving its zephyr,user node an io-channels
 * property pointing at the ADC input the divider lands on. Boards that route
 * the divider somewhere the SAADC cannot see it (the RideFormTracker bring-up
 * board put it on P0.09, which has no analog function at all) simply leave the
 * property out, and battery_present() then reports false.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdbool.h>
#include <stdint.h>

/* Set up the ADC channel. Safe to call on a board without one. */
int battery_init(void);

/* True once battery_init() has found a usable channel. */
bool battery_present(void);

/* Sample the pack.
 *
 * Returns the terminal voltage in millivolts and a state-of-charge percentage.
 * Both are out-parameters and either may be NULL.
 *
 * The percentage is derived from a lithium-polymer discharge curve, not from a
 * straight line between two voltages: a single cell spends most of its life
 * between 3.7 and 3.9 V and then falls off a cliff, so a linear map would
 * report about 40 % for most of the ride and then collapse.
 *
 * Treat it as advisory. It is an open-circuit approximation read under whatever
 * load the sensor happens to be applying, and it reads HIGH whenever USB is
 * attached: the charger sits on the same node, so during constant-current
 * charge the number tracks the cell's rising terminal voltage rather than its
 * resting one, and once charging finishes it is simply pinned at the
 * regulation voltage regardless of what the cell would settle back to.
 */
int battery_read(uint16_t *mv, uint8_t *percent);

#endif /* BATTERY_H */
