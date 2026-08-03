#!/usr/bin/env python3
"""Battery life model for the cadence sensor — mainly to answer "is a gyro affordable?".

WHERE THE NUMBERS COME FROM, because it changes how much to trust each one:

  SOLID    battery capacity, ride hours, LDO quiescent current. Given, or from
           the TPS7A02 datasheet (~25 nA, which is why it does not appear below
           at all: it is a thousandth of everything else).

  DECENT   nRF52840 figures. System ON idle with RTC and RAM retention, and CPU
           run current with the DC/DC converter, are well documented and this
           design uses the DC/DC (see reg1 in the board dts).

  ESTIMATE the LSM6DSV accelerometer and gyro currents. The LSM6DSV datasheet is
           NOT in this repo, so these are order-of-magnitude figures for this
           class of part rather than looked-up values. They are the numbers the
           gyro conclusion depends on, so treat the RATIO as the finding and the
           absolute days as indicative. Override them on the command line once
           the real figures are to hand.

Usage:
  power_budget.py [--capacity-mah 12] [--ride-h 1] [--accel-ua 25]
                  [--gyro-ua 450] [--parked-accel-ua 25]
"""
import argparse


def model(capacity_mah, ride_h, accel_ua, gyro_ua, parked_accel_ua):
    standby_h = 24.0 - ride_h

    # --- nRF52840 ---
    NRF_IDLE_UA = 3.0        # System ON, RTC running, RAM retained
    NRF_CPU_MA = 3.3         # 64 MHz with DC/DC
    NRF_BLE_UA = 75.0        # connected peripheral, ~50 ms interval, tiny payload

    # The 64 Hz sampler: one SPI burst read plus the detector's float work.
    # 150 us of CPU per sample is a deliberately pessimistic guess.
    SAMPLER_HZ = 64
    CPU_US_PER_SAMPLE = 150
    sampler_ua = NRF_CPU_MA * 1000.0 * (SAMPLER_HZ * CPU_US_PER_SAMPLE / 1e6)

    riding_base = NRF_BLE_UA + sampler_ua + accel_ua

    # Standby as the firmware behaves TODAY: the sampler thread parks, but the
    # accelerometer is left running at its full 60 Hz because only the thread
    # sleeps, not the sensor.
    standby_now = NRF_IDLE_UA + accel_ua
    # ...and as it could behave, with the part dropped to a low ODR when parked.
    standby_opt = NRF_IDLE_UA + parked_accel_ua

    def days(ride_ua, idle_ua):
        per_day_mah = (ride_ua * ride_h + idle_ua * standby_h) / 1000.0
        return capacity_mah / per_day_mah, per_day_mah

    # A gyro could be read more slowly than the accelerometer has to be — you
    # integrate rate instead of unwrapping an angle — so the sampler CPU cost
    # falls. It buys much less than it looks like it should, for two reasons
    # worth being explicit about:
    #
    #   the gyro's own current barely follows its ODR. A MEMS gyro burns most
    #   of its power keeping the proof mass RESONATING, and that drive loop
    #   runs continuously whatever the output rate. An accelerometer has no
    #   driven mass, which is exactly why ITS low-power modes work so well.
    #
    #   the accelerometer does not go away. It is what provides wake-on-motion,
    #   so a gyro is always ADDED to the 25 uA, never substituted for it.
    GYRO_LOW_ODR_SCALE = 0.8      # generous: 20% off for a much lower ODR
    sampler_slow_ua = sampler_ua * (16.0 / SAMPLER_HZ)

    rows = [
        # Everything below assumes the parked ODR drop, so the comparison is
        # like-for-like; the first row shows what it is worth on its own.
        ("accel only, no parked drop", riding_base, standby_now),
        ("accel only + parked drop", riding_base, standby_opt),
        ("+ gyro, riding only", riding_base + gyro_ua, standby_opt),
        ("+ gyro, riding only, 16 Hz",
         NRF_BLE_UA + sampler_slow_ua + accel_ua + gyro_ua * GYRO_LOW_ODR_SCALE,
         standby_opt),
        ("+ gyro, never sleeps", riding_base + gyro_ua, standby_opt + gyro_ua),
        # What a train journey looks like: motion keeps waking it, nothing ever
        # rotates, so it never gets to park.
        ("accel only, awake 24/7 (train)", riding_base, riding_base),
    ]

    print(f"battery {capacity_mah} mAh, riding {ride_h} h/day, "
          f"standby {standby_h} h/day")
    print(f"accel {accel_ua} uA, gyro {gyro_ua} uA, "
          f"parked accel {parked_accel_ua} uA")
    print(f"sampler CPU cost: {sampler_ua:.0f} uA "
          f"({SAMPLER_HZ} Hz x {CPU_US_PER_SAMPLE} us @ {NRF_CPU_MA} mA)\n")

    hdr = f"{'':32s} {'riding':>9s} {'standby':>9s} {'mAh/day':>9s} {'days':>7s} {'standby share':>14s}"
    print(hdr)
    print("-" * len(hdr))
    for name, ride_ua, idle_ua in rows:
        d, per_day = days(ride_ua, idle_ua)
        share = (idle_ua * standby_h) / (ride_ua * ride_h + idle_ua * standby_h)
        print(f"{name:32s} {ride_ua:8.0f}u {idle_ua:8.0f}u "
              f"{per_day:9.3f} {d:7.1f} {share:13.0%}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--capacity-mah", type=float, default=12)
    p.add_argument("--ride-h", type=float, default=1)
    p.add_argument("--accel-ua", type=float, default=25)
    p.add_argument("--gyro-ua", type=float, default=450)
    p.add_argument("--parked-accel-ua", type=float, default=25)
    a = p.parse_args()
    model(a.capacity_mah, a.ride_h, a.accel_ua, a.gyro_ua, a.parked_accel_ua)
