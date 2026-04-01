/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/services/bas.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/battery.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/activity.h>
#include <zmk/workqueue.h>

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_NPM1300_DIRECT)
#include <zmk/npm1300_vbat.h>
#endif

static uint8_t last_state_of_charge = 0;

uint8_t zmk_battery_state_of_charge(void) { return last_state_of_charge; }

#if DT_HAS_CHOSEN(zmk_battery)
static const struct device *const battery = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));
#else
#warning                                                                                           \
    "Using a node labeled BATTERY for the battery sensor is deprecated. Set a zmk,battery chosen node instead. (Ignore this if you don't have a battery sensor.)"
static const struct device *battery;
#endif

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_LITHIUM_VOLTAGE) ||                         \
    IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_NPM1300_DIRECT)

#if IS_ENABLED(CONFIG_ZMK_BATTERY_PROFILE_YDL375678)
// Battery profile for YDL375678 - OCV table from characterization
// Voltage points from 2990mV to 4200mV in 10mV steps
static const uint16_t battery_ocv_table[] = {
    2990, 3000, 3010, 3020, 3030, 3040, 3050, 3060, 3070, 3080, 3090, 3100,
    3110, 3120, 3130, 3140, 3150, 3160, 3170, 3180, 3190, 3200,
    3210, 3220, 3230, 3240, 3250, 3260, 3270, 3280, 3290, 3300,
    3310, 3320, 3330, 3340, 3350, 3360, 3370, 3380, 3390, 3400,
    3410, 3420, 3430, 3440, 3450, 3460, 3470, 3480, 3490, 3500,
    3510, 3520, 3530, 3540, 3550, 3560, 3570, 3580, 3590, 3600,
    3610, 3620, 3630, 3640, 3650, 3660, 3670, 3680, 3690, 3700,
    3710, 3720, 3730, 3740, 3750, 3760, 3770, 3780, 3790, 3800,
    3810, 3820, 3830, 3840, 3850, 3860, 3870, 3880, 3890, 3900,
    3910, 3920, 3930, 3940, 3950, 3960, 3970, 3980, 3990, 4000,
    4010, 4020, 4030, 4040, 4050, 4060, 4070, 4080, 4090, 4100,
    4110, 4120, 4130, 4140, 4150, 4160, 4170, 4180, 4190, 4200
};

#define BATTERY_OCV_TABLE_SIZE (sizeof(battery_ocv_table) / sizeof(battery_ocv_table[0]))

static uint8_t battery_profile_mv_to_pct(int16_t bat_mv) {
    // Clamp to table range
    if (bat_mv <= battery_ocv_table[0]) {
        return 0;
    }
    if (bat_mv >= battery_ocv_table[BATTERY_OCV_TABLE_SIZE - 1]) {
        return 100;
    }

    // Find the voltage range and interpolate
    for (size_t i = 0; i < BATTERY_OCV_TABLE_SIZE - 1; i++) {
        if (bat_mv >= battery_ocv_table[i] && bat_mv <= battery_ocv_table[i + 1]) {
            // Linear interpolation between two points
            // Each index step represents (100 / (table_size - 1)) percent
            float soc_per_step = 100.0f / (BATTERY_OCV_TABLE_SIZE - 1);
            float soc_low = i * soc_per_step;
            float soc_high = (i + 1) * soc_per_step;

            // Interpolate based on voltage position between the two points
            float voltage_fraction = (float)(bat_mv - battery_ocv_table[i]) /
                                   (float)(battery_ocv_table[i + 1] - battery_ocv_table[i]);
            float soc = soc_low + (soc_high - soc_low) * voltage_fraction;

            return (uint8_t)(soc + 0.5f); // Round to nearest integer
        }
    }

    return 50; // Fallback
}

static uint8_t lithium_ion_mv_to_pct(int16_t bat_mv) {
    return battery_profile_mv_to_pct(bat_mv);
}

#else
// Generic lithium-ion battery profile
static uint8_t lithium_ion_mv_to_pct(int16_t bat_mv) {
    // Simple linear approximation of a battery based off adafruit's discharge graph:
    // https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages

    if (bat_mv >= 4200) {
        return 100;
    } else if (bat_mv <= 3450) {
        return 0;
    }

    return bat_mv * 2 / 15 - 459;
}
#endif // IS_ENABLED(CONFIG_ZMK_BATTERY_PROFILE_YDL375678)

#endif // IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_LITHIUM_VOLTAGE) || IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_NPM1300_DIRECT)

static int zmk_battery_update(const struct device *battery) {
    struct sensor_value state_of_charge;
    int rc;

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_STATE_OF_CHARGE)

    rc = sensor_sample_fetch_chan(battery, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE);
    if (rc != 0) {
        LOG_DBG("Failed to fetch battery values: %d", rc);
        return rc;
    }

    rc = sensor_channel_get(battery, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &state_of_charge);

    if (rc != 0) {
        LOG_DBG("Failed to get battery state of charge: %d", rc);
        return rc;
    }
#elif IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_LITHIUM_VOLTAGE)
    rc = sensor_sample_fetch_chan(battery, SENSOR_CHAN_GAUGE_VOLTAGE);
    if (rc != 0) {
        LOG_DBG("Failed to fetch battery values: %d", rc);
        return rc;
    }

    struct sensor_value voltage;
    rc = sensor_channel_get(battery, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage);

    if (rc != 0) {
        LOG_DBG("Failed to get battery voltage: %d", rc);
        return rc;
    }

    uint16_t mv = voltage.val1 * 1000 + (voltage.val2 / 1000);
    state_of_charge.val1 = lithium_ion_mv_to_pct(mv);

    LOG_DBG("State of change %d from %d mv", state_of_charge.val1, mv);
#elif IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_NPM1300_DIRECT)
    ARG_UNUSED(battery);

    int mv;
    rc = zmk_npm1300_read_vbat_mv(&mv);
    if (rc != 0) {
        LOG_DBG("Failed to read VBAT from nPM1300: %d", rc);
        return rc;
    }

    state_of_charge.val1 = lithium_ion_mv_to_pct(mv);
    state_of_charge.val2 = 0;

    LOG_DBG("State of charge %d from %d mV", state_of_charge.val1, mv);
#else
#error "Not a supported reporting fetch mode"
#endif

    if (last_state_of_charge != state_of_charge.val1) {
        last_state_of_charge = state_of_charge.val1;

        rc = raise_zmk_battery_state_changed(
            (struct zmk_battery_state_changed){.state_of_charge = last_state_of_charge});

        if (rc != 0) {
            LOG_ERR("Failed to raise battery state changed event: %d", rc);
            return rc;
        }
    }

#if IS_ENABLED(CONFIG_BT_BAS)
    if (bt_bas_get_battery_level() != last_state_of_charge) {
        LOG_DBG("Setting BAS GATT battery level to %d.", last_state_of_charge);

        rc = bt_bas_set_battery_level(last_state_of_charge);

        if (rc != 0) {
            LOG_WRN("Failed to set BAS GATT battery level (err %d)", rc);
            return rc;
        }
    }
#endif

    return rc;
}

static void zmk_battery_work(struct k_work *work) {
    int rc = zmk_battery_update(battery);

    if (rc != 0) {
        LOG_DBG("Failed to update battery value: %d.", rc);
    }
}

K_WORK_DEFINE(battery_work, zmk_battery_work);

static void zmk_battery_timer(struct k_timer *timer) {
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &battery_work);
}

K_TIMER_DEFINE(battery_timer, zmk_battery_timer, NULL);

static void zmk_battery_start_reporting() {
#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_NPM1300_DIRECT)
    k_timer_start(&battery_timer, K_NO_WAIT, K_SECONDS(CONFIG_ZMK_BATTERY_REPORT_INTERVAL));
#else
    if (device_is_ready(battery)) {
        k_timer_start(&battery_timer, K_NO_WAIT, K_SECONDS(CONFIG_ZMK_BATTERY_REPORT_INTERVAL));
    }
#endif
}

static int zmk_battery_init(void) {
#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_NPM1300_DIRECT)
    zmk_battery_start_reporting();
    return 0;
#else
#if !DT_HAS_CHOSEN(zmk_battery)
    battery = device_get_binding("BATTERY");

    if (battery == NULL) {
        return -ENODEV;
    }

    LOG_WRN("Finding battery device labeled BATTERY is deprecated. Use zmk,battery chosen node.");
#endif

    if (!device_is_ready(battery)) {
        LOG_ERR("Battery device \"%s\" is not ready", battery->name);
        return -ENODEV;
    }

    zmk_battery_start_reporting();
    return 0;
#endif
}

static int battery_event_listener(const zmk_event_t *eh) {

    if (as_zmk_activity_state_changed(eh)) {
        switch (zmk_activity_get_state()) {
        case ZMK_ACTIVITY_ACTIVE:
            zmk_battery_start_reporting();
            return 0;
        case ZMK_ACTIVITY_IDLE:
        case ZMK_ACTIVITY_SLEEP:
            k_timer_stop(&battery_timer);
            return 0;
        default:
            break;
        }
    }
    return -ENOTSUP;
}

ZMK_LISTENER(battery, battery_event_listener);

ZMK_SUBSCRIPTION(battery, zmk_activity_state_changed);

SYS_INIT(zmk_battery_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
