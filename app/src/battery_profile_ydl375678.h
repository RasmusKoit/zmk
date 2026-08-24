/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Voltage → state-of-charge mapping for the YDL375678 cell, derived from
 * the nPM1300 battery profiler characterization at 22°C
 * (zmk-keyboard-pipar/nrf-bat-profile/YDL375678/profileSettings.json,
 * fitted OCV curve in Tparam_3). Regenerate with
 * pipar/tests/gen_ocv_table.py if the cell is re-profiled.
 *
 * Self-contained (stdint only) so host-side unit tests can include the
 * exact code the firmware runs. Do not add Zephyr dependencies.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

// battery_ocv_table[i] is the cell voltage in mV at i% state of charge.
// Entry 0 is clamped to the profiler's 3.0V empty cutoff (the fit
// extrapolates non-physically below 1% SOC).
static const uint16_t battery_ocv_table[] = {
    2990, 3049, 3543, 3597, 3598, 3618, 3626, 3635, 3643, 3650, 3654, 3660, 3664, 3670, 3678, 3687,
    3695, 3700, 3705, 3713, 3717, 3723, 3729, 3733, 3737, 3740, 3742, 3744, 3748, 3751, 3753, 3757,
    3761, 3762, 3765, 3766, 3769, 3771, 3772, 3773, 3774, 3778, 3779, 3781, 3782, 3785, 3786, 3787,
    3789, 3791, 3795, 3797, 3799, 3803, 3804, 3807, 3811, 3814, 3816, 3819, 3823, 3826, 3830, 3834,
    3837, 3841, 3845, 3848, 3854, 3860, 3863, 3868, 3874, 3878, 3884, 3892, 3900, 3911, 3922, 3934,
    3944, 3953, 3960, 3965, 3972, 3978, 3985, 3991, 3997, 4004, 4011, 4018, 4025, 4032, 4040, 4049,
    4058, 4067, 4076, 4085, 4093};

#define BATTERY_OCV_TABLE_SIZE (sizeof(battery_ocv_table) / sizeof(battery_ocv_table[0]))

static inline uint8_t battery_profile_mv_to_pct(int16_t bat_mv) {
    if (bat_mv <= battery_ocv_table[0]) {
        return 0;
    }
    if (bat_mv >= battery_ocv_table[BATTERY_OCV_TABLE_SIZE - 1]) {
        return 100;
    }

    // Each index step is one percent; interpolate within the bracketing
    // voltage interval.
    for (size_t i = 0; i < BATTERY_OCV_TABLE_SIZE - 1; i++) {
        if (bat_mv >= battery_ocv_table[i] && bat_mv <= battery_ocv_table[i + 1]) {
            uint16_t span = battery_ocv_table[i + 1] - battery_ocv_table[i];
            uint16_t off = bat_mv - battery_ocv_table[i];
            return (uint8_t)(i + (off + span / 2) / span);
        }
    }

    return 50; // Unreachable with a monotonic table; safe fallback
}
