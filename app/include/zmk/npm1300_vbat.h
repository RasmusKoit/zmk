/* SPDX-License-Identifier: MIT */

#pragma once

/**
 * Read battery voltage directly from nPM1300 via I2C.
 *
 * @param out_mv  Pointer to store the voltage in millivolts.
 * @return 0 on success, negative errno on failure.
 */
int zmk_npm1300_read_vbat_mv(int *out_mv);
