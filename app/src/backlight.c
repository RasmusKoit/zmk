/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <stdlib.h>

#include <zmk/activity.h>
#include <zmk/backlight.h>
#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_BACKLIGHT_BREATHE_SYNC) && \
    IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/split/central.h>
#define BREATHE_SYNC_CENTRAL 1
#endif

// Only the central (or non-split) should decide when to start breathing.
// Peripherals only breathe when told to via sync commands.
#if IS_ENABLED(CONFIG_ZMK_SPLIT_PERIPHERAL_BACKLIGHT_BREATHE_SYNC) && \
    !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#define BREATHE_PERIPHERAL_ONLY 1
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_HAS_CHOSEN(zmk_backlight),
             "CONFIG_ZMK_BACKLIGHT is enabled but no zmk,backlight chosen node found");

static const struct device *const backlight_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_backlight));

#define CHILD_COUNT(...) +1
#define DT_NUM_CHILD(node_id) (DT_FOREACH_CHILD(node_id, CHILD_COUNT))

#define BACKLIGHT_NUM_LEDS (DT_NUM_CHILD(DT_CHOSEN(zmk_backlight)))

#define BRT_MAX 100

struct backlight_state {
    uint8_t brightness;
    bool on;
};

static struct backlight_state state = {.brightness = CONFIG_ZMK_BACKLIGHT_BRT_START,
                                       .on = IS_ENABLED(CONFIG_ZMK_BACKLIGHT_ON_START)};

static int zmk_backlight_update(void) {
    uint8_t brt = zmk_backlight_get_brt();
    LOG_DBG("Update backlight brightness: %d%%", brt);

    for (int i = 0; i < BACKLIGHT_NUM_LEDS; i++) {
        int rc = led_set_brightness(backlight_dev, i, brt);
        if (rc != 0) {
            LOG_ERR("Failed to update backlight LED %d: %d", i, rc);
            return rc;
        }
    }
    return 0;
}

#if IS_ENABLED(CONFIG_SETTINGS)
static int backlight_settings_load_cb(const char *name, size_t len, settings_read_cb read_cb,
                                      void *cb_arg) {
    const char *next;
    if (settings_name_steq(name, "state", &next) && !next) {
        if (len != sizeof(state)) {
            return -EINVAL;
        }

        int rc = read_cb(cb_arg, &state, sizeof(state));
        if (rc >= 0) {
            rc = zmk_backlight_update();
        }

        return MIN(rc, 0);
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(backlight, "backlight", NULL, backlight_settings_load_cb, NULL,
                               NULL);

static void backlight_save_work_handler(struct k_work *work) {
    settings_save_one("backlight/state", &state, sizeof(state));
}

static struct k_work_delayable backlight_save_work;
#endif

static int zmk_backlight_init(void) {
    if (!device_is_ready(backlight_dev)) {
        LOG_ERR("Backlight device \"%s\" is not ready", backlight_dev->name);
        return -ENODEV;
    }

#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_init_delayable(&backlight_save_work, backlight_save_work_handler);
#endif
#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_AUTO_OFF_USB)
    state.on = zmk_usb_is_powered();
#endif
    return zmk_backlight_update();
}

static int zmk_backlight_update_and_save(void) {
    int rc = zmk_backlight_update();
    if (rc != 0) {
        return rc;
    }

#if IS_ENABLED(CONFIG_SETTINGS)
    int ret = k_work_reschedule(&backlight_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
#else
    return 0;
#endif
}

int zmk_backlight_on(void) {
    state.brightness = MAX(state.brightness, CONFIG_ZMK_BACKLIGHT_BRT_STEP);
    state.on = true;
    return zmk_backlight_update_and_save();
}

int zmk_backlight_off(void) {
    state.on = false;
    return zmk_backlight_update_and_save();
}

int zmk_backlight_toggle(void) { return state.on ? zmk_backlight_off() : zmk_backlight_on(); }

bool zmk_backlight_is_on(void) { return state.on; }

int zmk_backlight_set_brt(uint8_t brightness) {
    state.brightness = MIN(brightness, BRT_MAX);
    state.on = (state.brightness > 0);
    return zmk_backlight_update_and_save();
}

uint8_t zmk_backlight_get_brt(void) { return state.on ? state.brightness : 0; }

uint8_t zmk_backlight_calc_brt(int direction) {
    int brt = state.brightness + (direction * CONFIG_ZMK_BACKLIGHT_BRT_STEP);
    return CLAMP(brt, 0, BRT_MAX);
}

uint8_t zmk_backlight_calc_brt_cycle(void) {
    if (state.brightness == BRT_MAX) {
        return 0;
    } else {
        return zmk_backlight_calc_brt(1);
    }
}

#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_BREATHE_IDLE)

static bool breathing_active = false;
static uint16_t breathe_step = 0;

#if CONFIG_ZMK_BACKLIGHT_BREATHE_TIMEOUT > 0
static void backlight_breathe_timeout_cb(struct k_work *work) {
    if (breathing_active) {
        zmk_backlight_breathe_stop();
    }
}
static K_WORK_DELAYABLE_DEFINE(backlight_breathe_timeout_work, backlight_breathe_timeout_cb);
#endif

static void backlight_set_raw_brightness(uint8_t brt) {
    for (int i = 0; i < BACKLIGHT_NUM_LEDS; i++) {
        led_set_brightness(backlight_dev, i, brt);
    }
}

static void backlight_breathe_tick(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(backlight_breathe_work, backlight_breathe_tick);

static void backlight_breathe_tick(struct k_work *work) {
    if (!breathing_active) {
        return;
    }

    uint8_t brt_pct = abs((int)breathe_step - 1200) / 12;
    uint8_t actual_brt = (uint8_t)((uint16_t)brt_pct * state.brightness / 100);

    backlight_set_raw_brightness(actual_brt);

    breathe_step += CONFIG_ZMK_BACKLIGHT_BREATHE_SPEED * 10;
    if (breathe_step > 2400) {
        breathe_step = 0;
    }

    k_work_schedule(&backlight_breathe_work, K_MSEC(50));
}

int zmk_backlight_breathe_start(void) {
    if (breathing_active) {
        return 0;
    }
    breathing_active = true;
    breathe_step = 0;
    k_work_schedule(&backlight_breathe_work, K_MSEC(50));

#if CONFIG_ZMK_BACKLIGHT_BREATHE_TIMEOUT > 0
    k_work_schedule(&backlight_breathe_timeout_work,
                    K_SECONDS(CONFIG_ZMK_BACKLIGHT_BREATHE_TIMEOUT));
#endif

#if BREATHE_SYNC_CENTRAL
    zmk_split_central_update_backlight_breathe(1);
#endif

    return 0;
}

int zmk_backlight_breathe_stop(void) {
    if (!breathing_active) {
        return 0;
    }
    breathing_active = false;
    k_work_cancel_delayable(&backlight_breathe_work);

#if CONFIG_ZMK_BACKLIGHT_BREATHE_TIMEOUT > 0
    k_work_cancel_delayable(&backlight_breathe_timeout_work);
#endif

#if BREATHE_SYNC_CENTRAL
    zmk_split_central_update_backlight_breathe(0);
#endif

    return zmk_backlight_update();
}

#else

int zmk_backlight_breathe_start(void) { return -ENOTSUP; }
int zmk_backlight_breathe_stop(void) { return -ENOTSUP; }

#endif // IS_ENABLED(CONFIG_ZMK_BACKLIGHT_BREATHE_IDLE)

#define BACKLIGHT_NEEDS_LISTENER                                                                   \
    (IS_ENABLED(CONFIG_ZMK_BACKLIGHT_AUTO_OFF_IDLE) ||                                             \
     IS_ENABLED(CONFIG_ZMK_BACKLIGHT_AUTO_OFF_USB) ||                                              \
     IS_ENABLED(CONFIG_ZMK_BACKLIGHT_BREATHE_IDLE))

#if BACKLIGHT_NEEDS_LISTENER

#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_AUTO_OFF_IDLE) || IS_ENABLED(CONFIG_ZMK_BACKLIGHT_AUTO_OFF_USB)
static int backlight_auto_state(bool *prev_state, bool new_state) {
    if (state.on == new_state) {
        return 0;
    }
    state.on = new_state && *prev_state;
    *prev_state = !new_state;
    return zmk_backlight_update();
}
#endif

static int backlight_event_listener(const zmk_event_t *eh) {

#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_AUTO_OFF_IDLE)
    const struct zmk_activity_state_changed *activity_ev;
    if ((activity_ev = as_zmk_activity_state_changed(eh)) != NULL) {
        enum zmk_activity_state activity = activity_ev->state;

#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_BREATHE_IDLE)
        if (breathing_active) {
            zmk_backlight_breathe_stop();
        }
#endif // IS_ENABLED(CONFIG_ZMK_BACKLIGHT_BREATHE_IDLE)

        static bool idle_prev_state = false;
        bool was_on = state.on;
        int rc = backlight_auto_state(&idle_prev_state, activity == ZMK_ACTIVITY_ACTIVE);

#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_BREATHE_IDLE)
        if (activity == ZMK_ACTIVITY_IDLE && was_on) {
#if !BREATHE_PERIPHERAL_ONLY
            bool usb_powered = false;
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
            usb_powered = zmk_usb_is_powered();
#endif
            if (usb_powered) {
                zmk_backlight_breathe_start();
            }
#endif // !BREATHE_PERIPHERAL_ONLY
        }
#endif // IS_ENABLED(CONFIG_ZMK_BACKLIGHT_BREATHE_IDLE)

        return rc;
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_AUTO_OFF_USB) || IS_ENABLED(CONFIG_ZMK_BACKLIGHT_BREATHE_IDLE)
    if (as_zmk_usb_conn_state_changed(eh)) {
#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_BREATHE_IDLE)
        bool usb_powered = false;
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
        usb_powered = zmk_usb_is_powered();
#endif
        if (breathing_active && !usb_powered) {
            zmk_backlight_breathe_stop();
        }
#if !BREATHE_PERIPHERAL_ONLY
        else if (!breathing_active && usb_powered &&
                   zmk_activity_get_state() == ZMK_ACTIVITY_IDLE) {
            zmk_backlight_breathe_start();
        }
#endif // !BREATHE_PERIPHERAL_ONLY
#endif
#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_AUTO_OFF_USB)
        static bool prev_state = false;
        return backlight_auto_state(&prev_state, zmk_usb_is_powered());
#else
        return 0;
#endif
    }
#endif

    return -ENOTSUP;
}

ZMK_LISTENER(backlight, backlight_event_listener);
#endif // BACKLIGHT_NEEDS_LISTENER

#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_AUTO_OFF_IDLE)
ZMK_SUBSCRIPTION(backlight, zmk_activity_state_changed);
#endif

#if IS_ENABLED(CONFIG_ZMK_BACKLIGHT_AUTO_OFF_USB) || IS_ENABLED(CONFIG_ZMK_BACKLIGHT_BREATHE_IDLE)
ZMK_SUBSCRIPTION(backlight, zmk_usb_conn_state_changed);
#endif

SYS_INIT(zmk_backlight_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
