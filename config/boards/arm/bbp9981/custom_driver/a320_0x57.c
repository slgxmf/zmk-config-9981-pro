/*
 * A320 optical sensor driver (polling via motion GPIO, Zephyr input subsystem)
 *
 * Complete rewrite v2 — 9981-Pro:
 *   - Decoupled from keyboard backlight
 *   - Nonlinear dynamic acceleration for mouse & scroll
 *   - Layer-aware routing
 *   - Custom behaviors are in a320_behaviors.c
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT avago_a320

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include <stdlib.h>
#include <math.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include "a320_0x57.h"

LOG_MODULE_REGISTER(a320, CONFIG_A320_LOG_LEVEL);

/* ==================================================================
 *  1.  STATE MANAGEMENT & DECOUPLING  (Section 2.1)
 *
 *     Removed all keyboard_backlight / HID indicator coupling.
 *     Replaced with two independent global gear variables.
 * ================================================================== */

static int current_mouse_gear  = 5;  /* range 1–10 */
static int current_scroll_gear = 5;  /* range 1–9  */

/* Exposed for a320_behaviors.c (header declares these) */
int *zmk_a320_mouse_gear_ptr(void)  { return &current_mouse_gear; }
int *zmk_a320_scroll_gear_ptr(void) { return &current_scroll_gear; }

/* === Configure Motion GPIO === */
#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 2
static const struct device *motion_gpio_dev;

/* ==== Touch status flag ==== */
static bool touched = false;

/* ---- Scroll-mode accumulation ---- */
static int16_t scroll_acc_x = 0;
static int16_t scroll_acc_y = 0;
static uint8_t scroll_samples = 0;
static uint32_t last_scroll_trigger_ms = 0;

/* Scroll inertia (Layer 4) */
static bool inertia_active = false;
static float inertia_vx = 0.0f;
static float inertia_vy = 0.0f;

/* Mouse inertia (Layer 11) */
static bool mouse_inertia_active = false;
static float mouse_inertia_vx = 0.0f;
static float mouse_inertia_vy = 0.0f;

/* ==================================================================
 *  2.  DATA & CONFIG STRUCTS
 * ================================================================== */
struct a320_dev_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
    uint16_t x_input_code;
    uint16_t y_input_code;
};

struct a320_data {
    const struct device *dev;
    struct k_work_delayable poll_work;
};

#ifndef CONFIG_A320_POLL_INTERVAL_MS
#define CONFIG_A320_POLL_INTERVAL_MS 10
#endif

static void a320_poll_work_handler(struct k_work *work);
static int  a320_read_motion(const struct device *dev, int16_t *dx, int16_t *dy);

/* ==================================================================
 *  3.  NONLINEAR DYNAMIC ACCELERATION  (Section 2.2)
 *      + STATE-AWARE ROUTING           (Section 2.3)
 * ================================================================== */

static inline int clamp_int(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* Scroll threshold: gear 1→9, gear 9→1  (higher gear = more responsive) */
static inline uint8_t scroll_gear_threshold(int gear)
{
    int t = 11 - clamp_int(gear, 1, 9);
    return (t * t * 3 + 5);
}

static void a320_poll_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct a320_data *data = CONTAINER_OF(dwork, struct a320_data, poll_work);
    const struct device *dev = data->dev;

    int pin_state = gpio_pin_get(motion_gpio_dev, MOTION_GPIO_PIN);

    /* ---- Query current active layer (Section 2.3) ---- */
    uint8_t active_layer = zmk_keymap_highest_layer_active();

    if (pin_state == 0) {
        int16_t raw_dx = 0, raw_dy = 0;

        if (a320_read_motion(dev, &raw_dx, &raw_dy) != 0) {
            k_work_reschedule(&data->poll_work, K_MSEC(CONFIG_A320_POLL_INTERVAL_MS));
            return;
        }

        if (active_layer == 4) {

            /* ==============================================
             *  SCROLL MODE  —  gear-based threshold
             * ============================================== */
            if (raw_dx == 0 && raw_dy == 0) {
                scroll_acc_x = 0;
                scroll_acc_y = 0;
                scroll_samples = 0;
            } else {
                scroll_acc_x += raw_dx;
                scroll_acc_y += raw_dy;
                scroll_samples++;

                uint8_t threshold = scroll_gear_threshold(current_scroll_gear);

                if (scroll_samples >= threshold) {
                    uint32_t now = k_uptime_get();
                    uint32_t dt = now - last_scroll_trigger_ms;
                    last_scroll_trigger_ms = now;

                    int speed = (dt < 30) ? 3 : (dt < 60) ? 2 : 1;
                    int step = speed * current_scroll_gear / 3;
                    if (step < 1) step = 1;

                    int16_t w_x = -(scroll_acc_x * step / 24);
                    int16_t w_y = -(scroll_acc_y * step / 24);

                    if (w_y > step)  w_y = step;
                    else if (w_y < -step) w_y = -step;
                    if (w_x > step)  w_x = step;
                    else if (w_x < -step) w_x = -step;

                    input_report_rel(dev, INPUT_REL_HWHEEL, -w_x, false, K_FOREVER);
                    input_report_rel(dev, INPUT_REL_WHEEL,   w_y, true,  K_FOREVER);

                    inertia_active = true;
                    inertia_vx = (float)(-w_x);
                    inertia_vy = (float)(w_y);

                    scroll_acc_x = 0;
                    scroll_acc_y = 0;
                    scroll_samples = 0;
                }
            }
            touched = true;

        } else if (active_layer == 8) {

            /* ==============================================
             *  REVERSE MOUSE  —  X/Y inverted
             * ============================================== */
            int16_t max_raw = MAX(abs(raw_dx), abs(raw_dy));

            float factor;
            if (max_raw <= 2) {
                factor = 1.0f;
            } else {
                float gear_factor = 0.72f + (current_mouse_gear * 0.048f);
                float base  = (0.1f + (current_mouse_gear * 0.3f)) * gear_factor;
                float accel = 1.0f + ((max_raw - 2) * 0.035f);
                if (accel > 1.75f) accel = 1.75f;
                factor = base * accel;
            }

            input_report_rel(dev, INPUT_REL_X, (int16_t)(-raw_dx * factor), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, (int16_t)( raw_dy * factor), true,  K_FOREVER);
            touched = true;

        } else if (active_layer == 7) {

            /* ==============================================
             *  REVERSE SCROLL  —  wheel inverted
             * ============================================== */
            if (raw_dx == 0 && raw_dy == 0) {
                scroll_acc_x = 0;
                scroll_acc_y = 0;
                scroll_samples = 0;
            } else {
                scroll_acc_x += raw_dx;
                scroll_acc_y += raw_dy;
                scroll_samples++;

                uint8_t threshold = scroll_gear_threshold(current_scroll_gear);

                if (scroll_samples >= threshold) {
                    int16_t w_x = scroll_acc_x / 24;
                    int16_t w_y = -(scroll_acc_y / 24);

                    if (w_y > 1)  w_y = 1;
                    else if (w_y < -1) w_y = -1;
                    if (w_x > 1)  w_x = 1;
                    else if (w_x < -1) w_x = -1;

                    input_report_rel(dev, INPUT_REL_HWHEEL, w_x, false, K_FOREVER);
                    input_report_rel(dev, INPUT_REL_WHEEL,  w_y, true,  K_FOREVER);

                    scroll_acc_x = 0;
                    scroll_acc_y = 0;
                    scroll_samples = 0;
                }
            }
            touched = true;

        } else if (active_layer == 11) {

            /* ==============================================
             *  MOUSE WITH INERTIA (Layer 11 - sliding)
             * ============================================== */
            int16_t max_raw = MAX(abs(raw_dx), abs(raw_dy));

            float factor;
            if (max_raw <= 2) {
                factor = 1.0f;
            } else {
                float gear_factor = 0.72f + (current_mouse_gear * 0.048f);
                float base  = (0.1f + (current_mouse_gear * 0.3f)) * gear_factor;
                float accel = 1.0f + ((max_raw - 2) * 0.035f);
                if (accel > 1.75f) accel = 1.75f;
                factor = base * accel;
            }

            mouse_inertia_vx = (float)(raw_dx * factor);
            mouse_inertia_vy = (float)(raw_dy * factor);

            input_report_rel(dev, INPUT_REL_X, (int16_t)(raw_dx * factor), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, (int16_t)(raw_dy * factor), true,  K_FOREVER);

            mouse_inertia_active = (fabsf(mouse_inertia_vx) > 3.0f || fabsf(mouse_inertia_vy) > 3.0f);
            touched = true;

        } else {

            /* ==============================================
             *  NORMAL MODE  —  nonlinear dynamic accel
             * ============================================== */
            int16_t max_raw = MAX(abs(raw_dx), abs(raw_dy));

            float factor;
            if (max_raw <= 2) {
                /* Low-speed precision: 1:1 pixel (Section 2.2) */
                factor = 1.0f;
            } else {
                /* Dynamic acceleration */
                float gear_factor = 0.72f + (current_mouse_gear * 0.048f);
                float base  = (0.1f + (current_mouse_gear * 0.3f)) * gear_factor;
                float accel = 1.0f + ((max_raw - 2) * 0.035f);
                if (accel > 1.75f) accel = 1.75f;
                factor = base * accel;
            }

            input_report_rel(dev, INPUT_REL_X, (int16_t)(raw_dx * factor), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, (int16_t)(raw_dy * factor), true,  K_FOREVER);
            touched = true;
        }
    } else {
        touched = false;

        /* Scroll inertia (Layer 4) */
        if (active_layer == 4 && inertia_active) {
            int16_t w_x = (int16_t)(inertia_vx + 0.5f);
            int16_t w_y = (int16_t)(inertia_vy + 0.5f);

            if (w_y > 1)  w_y = 1;
            else if (w_y < -1) w_y = -1;
            if (w_x > 1)  w_x = 1;
            else if (w_x < -1) w_x = -1;

            if (w_x != 0 || w_y != 0) {
                input_report_rel(dev, INPUT_REL_HWHEEL, -w_x, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_WHEEL,   w_y, true,  K_FOREVER);
            }

            inertia_vx *= 0.85f;
            inertia_vy *= 0.85f;

            if (fabsf(inertia_vx) < 0.3f && fabsf(inertia_vy) < 0.3f) {
                inertia_active = false;
                inertia_vx = 0.0f;
                inertia_vy = 0.0f;
            }
        }

        /* Mouse inertia (Layer 11) */
        if (active_layer == 11 && mouse_inertia_active) {
            if (fabsf(mouse_inertia_vx) > 3.0f || fabsf(mouse_inertia_vy) > 3.0f ||
                fabsf(mouse_inertia_vx) > 0.5f || fabsf(mouse_inertia_vy) > 0.5f) {

                int16_t out_x = (int16_t)(mouse_inertia_vx + 0.5f);
                int16_t out_y = (int16_t)(mouse_inertia_vy + 0.5f);

                input_report_rel(dev, INPUT_REL_X, out_x, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_Y, out_y, true,  K_FOREVER);

                mouse_inertia_vx *= 0.88f;
                mouse_inertia_vy *= 0.88f;

                if (fabsf(mouse_inertia_vx) < 0.5f && fabsf(mouse_inertia_vy) < 0.5f) {
                    mouse_inertia_active = false;
                    mouse_inertia_vx = 0.0f;
                    mouse_inertia_vy = 0.0f;
                }
            } else {
                mouse_inertia_active = false;
            }
        }
    }

    k_work_reschedule(&data->poll_work, K_MSEC(CONFIG_A320_POLL_INTERVAL_MS));
}

/* ==================================================================
 *  4.  I2C READ SEQUENCE
 * ================================================================== */
static int a320_read_motion(const struct device *dev, int16_t *dx, int16_t *dy)
{
    const struct a320_dev_config *cfg = dev->config;
    uint8_t buf[7] = {0};
    uint8_t reg = 0x0A;
    int ret;

    ret = i2c_write_dt(&cfg->i2c, &reg, 1);
    if (ret < 0) {
        LOG_ERR("i2c write 0x0A failed: %d", ret);
        return ret;
    }

    ret = i2c_burst_read_dt(&cfg->i2c, 0x0A, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("i2c burst read from 0x0A failed: %d", ret);
        return ret;
    }

    *dy = (int8_t)buf[1];
    *dx = (int8_t)buf[3];
    return 0;
}

bool tp_is_touched(void) { return touched; }

/* ==================================================================
 *  5.  DEVICE INIT & INSTANCE MACROS
 * ================================================================== */

static int a320_init(const struct device *dev)
{
    const struct a320_dev_config *cfg = dev->config;
    struct a320_data *data = dev->data;

    LOG_INF("A320 init: %s", dev->name);

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    motion_gpio_dev = DEVICE_DT_GET(MOTION_GPIO_NODE);
    if (!device_is_ready(motion_gpio_dev)) {
        LOG_ERR("Motion GPIO device not ready");
        return -ENODEV;
    }
    gpio_pin_configure(motion_gpio_dev, MOTION_GPIO_PIN, GPIO_INPUT | GPIO_PULL_UP);

    data->dev = dev;

    k_work_init_delayable(&data->poll_work, a320_poll_work_handler);
    k_work_schedule(&data->poll_work, K_MSEC(CONFIG_A320_POLL_INTERVAL_MS));

    return 0;
}

#define A320_INIT_PRIORITY CONFIG_INPUT_A320_INIT_PRIORITY

#define A320_DEFINE(inst)                                                                          \
    static struct a320_data a320_data_##inst;                                                      \
    static const struct a320_dev_config a320_config_##inst = {                                     \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .motion_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, motion_gpios, {0}),                          \
        .x_input_code = DT_PROP_OR(DT_DRV_INST(inst), x_input_code, INPUT_REL_X),                  \
        .y_input_code = DT_PROP_OR(DT_DRV_INST(inst), y_input_code, INPUT_REL_Y),                  \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320_data_##inst, &a320_config_##inst,           \
                          POST_KERNEL, A320_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
