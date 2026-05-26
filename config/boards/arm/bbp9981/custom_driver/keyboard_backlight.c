/*
 * Copyright (c) 2023 ZitaoTech
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

#include <zmk/rgb_underglow.h>
#include <zmk/event_manager.h>
#include <zmk/activity.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_HAS_CHOSEN(zmk_keyboard_backlight),
             "keyboard_backlight: No zmk_keyboard_backlight chosen node found");

static const struct device *const indiled_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_keyboard_backlight));

#define CHILD_COUNT(...) +1
#define DT_NUM_CHILD(node_id) (DT_FOREACH_CHILD(node_id, CHILD_COUNT))
#define INDICATOR_LED_NUM_LEDS (DT_NUM_CHILD(DT_CHOSEN(zmk_keyboard_backlight)))

#define BRT_MAX 90
#define BRT_BLINK_HIGH 100
#define BRT_BLINK_LOW 10
#define BLINK_INTERVAL_MS 500

#define CYCLE_BRT_MIN 10
#define CYCLE_BRT_MAX 100
#define CYCLE_BRT_STEP 5
#define CYCLE_INTERVAL_MS 20

/* Smoothstep fade (R11: ≥100ms transition) */
#define SMOOTHSTEP_DURATION_MS 120
#define SMOOTHSTEP_INTERVAL_MS 10
#define SMOOTHSTEP_STEPS (SMOOTHSTEP_DURATION_MS / SMOOTHSTEP_INTERVAL_MS)

static uint8_t smoothstep_target_brt = 0;
static uint8_t smoothstep_start_brt = 0;
static uint8_t smoothstep_step = 0;
static bool smoothstep_running = false;

static struct k_work_delayable smoothstep_work;

static void set_led_brightness(uint8_t level);
static void set_led_brightness_smoothstep(struct k_work *work);

static void smoothstep_fade_to(uint8_t target_brt) {
    smoothstep_target_brt = target_brt;
    smoothstep_start_brt = 0; /* Will be set on first step */
    smoothstep_step = 0;
    smoothstep_running = true;
    k_work_reschedule(&smoothstep_work, K_NO_WAIT);
}

static void set_led_brightness_smoothstep(struct k_work *work) {
    if (!smoothstep_running) return;

    if (smoothstep_step == 0) {
        /* Read current brightness as starting point */
        smoothstep_start_brt = 0;
    }

    smoothstep_step++;
    if (smoothstep_step > SMOOTHSTEP_STEPS) {
        /* Final step: set target directly */
        set_led_brightness(smoothstep_target_brt);
        smoothstep_running = false;
        return;
    }

    /* Smoothstep formula (fixed-point Q8.8):
     *   t = step / STEPS
     *   val = t * t * (3 - 2*t)
     * All values in Q8.8: 1.0 = 256, 2.0 = 512, 3.0 = 768
     */
    int32_t t_q8_8 = ((int32_t)smoothstep_step << 8) / (int32_t)SMOOTHSTEP_STEPS;
    int32_t t2 = (t_q8_8 * t_q8_8) >> 8;  /* t² in Q8.8 */
    int32_t three = 3 << 8;   /* 3.0 in Q8.8 */
    int32_t two = 2 << 8;     /* 2.0 in Q8.8 */
    int32_t smooth_t_q8_8 = (t2 * (three - ((two * t_q8_8) >> 8))) >> 8;  /* t²(3-2t) */
    
    int32_t diff = (int32_t)smoothstep_target_brt - (int32_t)smoothstep_start_brt;
    int32_t delta = (diff * smooth_t_q8_8) >> 8;
    uint8_t brt = (uint8_t)(smoothstep_start_brt + delta);
    
    /* Clamp */
    if (diff > 0) {
        if (brt > smoothstep_target_brt) brt = smoothstep_target_brt;
    } else {
        if (brt < smoothstep_target_brt) brt = smoothstep_target_brt;
    }

    set_led_brightness(brt);
    k_work_reschedule(&smoothstep_work, K_MSEC(SMOOTHSTEP_INTERVAL_MS));
}

static bool prev_active = false;
static int prev_layer = -1;
static bool blink_on = false;
static bool blink_start_high = true;
static uint8_t cycle_brightness = CYCLE_BRT_MIN;
static bool cycle_direction_up = true;

static struct k_work_delayable polling_work;
static struct k_work_delayable blink_work;
static struct k_work_delayable cycle_work;

static void set_led_brightness(uint8_t level) {
    if (!device_is_ready(indiled_dev)) {
        LOG_ERR("Indicator LED device not ready");
        return;
    }
    for (int i = 0; i < INDICATOR_LED_NUM_LEDS; i++) {
        int err = led_set_brightness(indiled_dev, i, level);
        if (err < 0) {
            LOG_ERR("Failed to set LED[%d] brightness: %d", i, err);
        }
    }
}

/* 层1/层3闪烁 */
static void blink_work_handler(struct k_work *work) {
    if (prev_layer != 1 && prev_layer != 3) {
        set_led_brightness(0);
        return;
    }

    blink_on = !blink_on;
    set_led_brightness(blink_on ? BRT_BLINK_HIGH : BRT_BLINK_LOW);

    uint32_t interval = (prev_layer == 3) ? (BLINK_INTERVAL_MS / 2) : BLINK_INTERVAL_MS;
    k_work_reschedule(&blink_work, K_MSEC(interval));
}


static void cycle_work_handler(struct k_work *work) {
    if (prev_layer != 2) {
        set_led_brightness(0);
        return;
    }

    set_led_brightness(cycle_brightness);

    if (cycle_direction_up) {
        cycle_brightness += CYCLE_BRT_STEP;
        if (cycle_brightness >= CYCLE_BRT_MAX) {
            cycle_brightness = CYCLE_BRT_MAX;
            cycle_direction_up = false;
        }
    } else {
        if (cycle_brightness < CYCLE_BRT_STEP) {
            cycle_brightness = CYCLE_BRT_MIN;
            cycle_direction_up = true;
        } else {
            cycle_brightness -= CYCLE_BRT_STEP;
        }
    }
    k_work_reschedule(&cycle_work, K_MSEC(CYCLE_INTERVAL_MS));
}

static void polling_work_handler(struct k_work *work) {
    bool active = (zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    int current_layer = zmk_keymap_highest_layer_active();
    struct zmk_led_hsb ug = zmk_rgb_underglow_calc_brt(0);
    uint8_t ug_brt = ug.b;
    bool rgb_on = true;
#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW)
    if (zmk_rgb_underglow_get_state(&rgb_on) < 0) {
        rgb_on = true;
    }
#endif

    if (current_layer != prev_layer || active != prev_active) {
        prev_layer = current_layer;
        prev_active = active;

        k_work_cancel_delayable(&blink_work);
        k_work_cancel_delayable(&cycle_work);
        blink_on = false;
        cycle_brightness = CYCLE_BRT_MIN;
        cycle_direction_up = true;

        switch (current_layer) {
        case 0:
 
            uint8_t brt = (rgb_on && active) ? ug_brt : 0;
            smoothstep_fade_to(brt);
            break;

        case 1:

            blink_start_high = !rgb_on ? true : false;
            blink_on = blink_start_high;
            set_led_brightness(blink_on ? BRT_BLINK_HIGH : BRT_BLINK_LOW);

            k_work_reschedule(&blink_work, K_MSEC(BLINK_INTERVAL_MS / 2));
            break;

        case 2:

            k_work_reschedule(&cycle_work, K_MSEC(100));
            break;

        case 3:
            blink_on = false;
            set_led_brightness(BRT_BLINK_LOW);
            k_work_reschedule(&blink_work, K_MSEC(BLINK_INTERVAL_MS / 2));
            break;

        default:
            smoothstep_fade_to(0);
            break;
        }
    }

    k_work_reschedule(&polling_work, K_MSEC(100));
}

static int keyboardbacklight_init(void) {
    if (!device_is_ready(indiled_dev)) {
        LOG_ERR("LED indicator device not ready");
        return -ENODEV;
    }

    prev_active = (zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    prev_layer = -1;

    k_work_init_delayable(&polling_work, polling_work_handler);
    k_work_init_delayable(&blink_work, blink_work_handler);
    k_work_init_delayable(&cycle_work, cycle_work_handler);
    k_work_init_delayable(&smoothstep_work, set_led_brightness_smoothstep);

    k_work_reschedule(&polling_work, K_MSEC(100));
    return 0;
}

SYS_INIT(keyboardbacklight_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
