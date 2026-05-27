/*
 * A320 optical sensor driver (polling via motion GPIO, Zephyr input subsystem)
 *
 * Complete rewrite v2 — 9981-Pro:
 *   - Decoupled from keyboard backlight
 *   - Nonlinear dynamic acceleration for mouse & scroll
 *   - Layer-aware routing
 *   - Custom behaviors are in a320_behaviors.c
 *
 * Scheme B (2026-05-27): Roll back mouse output to Build #37 "zero-filter"
 *   - raw_dx/dy → acceleration → input_report_rel (no filter pipeline)
 *   - Inertia uses unfiltered velocity (post-acceleration, pre-filter)
 *   - Scroll retains LUT-based speed factor, threshold-based accumulation
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
#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include "a320_0x57.h"

LOG_MODULE_REGISTER(a320, CONFIG_A320_LOG_LEVEL);

/* ==================================================================
 *  1. STATE MANAGEMENT & DECOUPLING
 * ================================================================== */

static int current_mouse_gear   = 2;  /* range 1–10 */
static int current_scroll_gear  = 2;  /* range 1–9  */
static int current_accel_profile = 3; /* range 1–5, default 3 (normal) */
static bool swap_buttons = false;     /* LCLK↔RCLK swap flag */

/* ==================================================================
 *  R13: Inertia state for exponential-decay glide
 * ================================================================== */

/*
 * Fast-decay LUT: each tick reduces velocity by 50%–94.5%.
 *   decay=128 → (256-128)/256 = 0.5     → τ ≈ 14ms
 *   decay=192 → (256-192)/256 = 0.25    → τ ≈ 7ms
 *   decay=224 → (256-224)/256 = 0.125   → τ ≈ 5ms
 *   decay=240 → (256-240)/256 = 0.0625  → τ ≈ 3.5ms
 */
static const uint8_t INERTIA_DECAY_LUT[4] = {
    128, /* τ≈14ms  (50% per tick) */
    192, /* τ≈7ms   (75% per tick) */
    224, /* τ≈5ms   (87.5% per tick) */
    240, /* τ≈3.5ms (93.75% per tick) */
};
#define INERTIA_DEFAULT_DECAY_IDX 0   /* τ≈14ms — quick but not instant */
#define INERTIA_THRESHOLD 16          /* Stop inertia below this velocity */

static int current_inertia_decay_idx = INERTIA_DEFAULT_DECAY_IDX;

static struct a320_inertia inertia_state = {
    .vx = 0,
    .vy = 0,
    .decay = INERTIA_DECAY_LUT[INERTIA_DEFAULT_DECAY_IDX],
    .active = false,
};

/* Exposed for a320_behaviors.c */
int  *zmk_a320_mouse_gear_ptr(void)           { return &current_mouse_gear; }
int  *zmk_a320_scroll_gear_ptr(void)          { return &current_scroll_gear; }
int  *zmk_a320_accel_profile_ptr(void)        { return &current_accel_profile; }
bool *zmk_a320_swap_buttons_ptr(void)         { return &swap_buttons; }
int  *zmk_a320_inertia_decay_idx_ptr(void)    { return &current_inertia_decay_idx; }
struct a320_inertia *zmk_a320_inertia_state_ptr(void) { return &inertia_state; }

/* === Configure Motion GPIO === */
#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 2
static const struct device *motion_gpio_dev;

/* ==== Touch status flag ==== */
static bool touched = false;
static bool first_touch = true;

/* ---- Scroll-mode accumulation ---- */
static int16_t scroll_acc_x = 0;
static int16_t scroll_acc_y = 0;
static uint8_t scroll_samples = 0;

/* ==================================================================
 *  1b. FILTER STATE (removed — zero-filter per Scheme B)
 * ================================================================== */

/* ==================================================================
 *  2. DATA & CONFIG STRUCTS
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
 *  3. MATH HELPERS & LUTs
 *     All arithmetic in Q8.8 fixed-point (no float).
 * ================================================================== */

static inline int clamp_int(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

#define Q8_8_TO_INT_ROUND(v)  (((v) + ((v) >= 0 ? 128 : -128)) >> 8)

/* === Build #37 base-speed table (Q8.8) ===
 *   base = 0.02 + gear² * 0.03  (Build #37 formula)
 *   gear 1=13, gear 2=36, ..., gear 10=773
 */
static const int16_t MOUSE_BASE_FACTOR_Q8_8[10] = {
    13, 36, 74, 128, 197, 282, 381, 497, 627, 773
};

/* Accel max-multiplier cap LUT (Q8.8) */
static const int16_t ACCEL_PROFILE_CAP_Q8_8[5] = {
    384, 512, 640, 896, 1280
};

/* Scroll speed factor LUT (Q10.6) */
static const uint8_t SCROLL_SPEED_FACTOR_Q10_6[9] = {
    38, 41, 44, 48, 52, 56, 60, 65, 70
};

static inline uint8_t scroll_gear_threshold(int gear)
{
    return (uint8_t)(12 - clamp_int(gear, 1, 9));
}

/* ==================================================================
 *  R13: Inertia helpers
 * ================================================================== */

static bool a320_inertia_tick(struct a320_inertia *inertia, bool is_scroll_mode,
                              const struct device *dev)
{
    int16_t vx = (int16_t)(((int32_t)inertia->vx * (256 - inertia->decay)) >> 8);
    int16_t vy = (int16_t)(((int32_t)inertia->vy * (256 - inertia->decay)) >> 8);

    if (abs(vx) < INERTIA_THRESHOLD && abs(vy) < INERTIA_THRESHOLD) {
        inertia->active = false;
        inertia->vx = 0;
        inertia->vy = 0;
        return false;
    }

    inertia->vx = vx;
    inertia->vy = vy;

    if (is_scroll_mode) {
        input_report_rel(dev, INPUT_REL_HWHEEL, -(vx >> 4), false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_WHEEL,  -(vy >> 4), true,  K_FOREVER);
    } else {
        input_report_rel(dev, INPUT_REL_X, vx >> 4, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, vy >> 4, true,  K_FOREVER);
    }

    return true;
}

static inline bool a320_inertia_layer_allowed(uint8_t layer)
{
    (void)layer;
    return true;
}

static void a320_inertia_update_on_touch(struct a320_inertia *inertia,
                                         int32_t velocity_x_q8_8,
                                         int32_t velocity_y_q8_8,
                                         bool allowed)
{
    if (!allowed) {
        inertia->active = false;
        inertia->vx = 0;
        inertia->vy = 0;
        return;
    }
    inertia->vx = (int16_t)velocity_x_q8_8;
    inertia->vy = (int16_t)velocity_y_q8_8;
    inertia->active = true;
}

/* ==================================================================
 *  MAIN POLL HANDLER
 * ================================================================== */

static void a320_poll_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct a320_data *data = CONTAINER_OF(dwork, struct a320_data, poll_work);
    const struct device *dev = data->dev;

    /* Sync decay LUT */
    inertia_state.decay = INERTIA_DECAY_LUT[current_inertia_decay_idx];

    int pin_state = gpio_pin_get(motion_gpio_dev, MOTION_GPIO_PIN);

    if (pin_state == 0) {
        /* ============================================================
         *  TOUCH ACTIVE
         * ============================================================ */
        int16_t raw_dx = 0, raw_dy = 0;

        if (a320_read_motion(dev, &raw_dx, &raw_dy) != 0) {
            k_work_reschedule(&data->poll_work, K_MSEC(CONFIG_A320_POLL_INTERVAL_MS));
            return;
        }

        /* ---- Query current active layer ---- */
        uint8_t active_layer = zmk_keymap_highest_layer_active();

        /* ---- L3-002: Reset scroll accumulators on layer transition ---- */
        {
            static uint8_t prev_active_layer = 0xFF;
            if (prev_active_layer != 0xFF && active_layer != prev_active_layer) {
                scroll_acc_x = 0;
                scroll_acc_y = 0;
                scroll_samples = 0;
                first_touch = true;
            }
            prev_active_layer = active_layer;
        }

        bool inertia_allowed = a320_inertia_layer_allowed(active_layer);

        if (active_layer == 4 || active_layer == 7) {

            /* ==============================================
             *  SCROLL MODE (4=normal, 7=rev)
             * ============================================== */
            bool rev = (active_layer == 7);

            /* Scroll uses un-filtered raw_dx/dy (no acceleration) */
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
                    int gear_idx = clamp_int(current_scroll_gear, 1, 9) - 1;
                    int32_t mul = (int32_t)SCROLL_SPEED_FACTOR_Q10_6[gear_idx];
                    int16_t w_x, w_y;

                    if (rev) {
                        w_x = (int16_t)((scroll_acc_x * mul) / (24 * 64));
                        w_y = (int16_t)((scroll_acc_y * mul) / (24 * 64));
                    } else {
                        w_x = (int16_t)(-(scroll_acc_x * mul) / (24 * 64));
                        w_y = (int16_t)(-(scroll_acc_y * mul) / (24 * 64));
                    }

                    if (w_y > 3)  w_y = 3;
                    else if (w_y < -3) w_y = -3;
                    if (w_x > 3)  w_x = 3;
                    else if (w_x < -3) w_x = -3;

                    input_report_rel(dev, INPUT_REL_HWHEEL, (rev ? w_x : -w_x), false, K_FOREVER);
                    input_report_rel(dev, INPUT_REL_WHEEL,  (rev ? w_y : w_y), true,  K_FOREVER);

                    /* Inertia: feed scroll velocity (Q8.8) */
                    a320_inertia_update_on_touch(&inertia_state,
                                                 (int32_t)scroll_acc_x * 256 / 24,
                                                 (int32_t)scroll_acc_y * 256 / 24,
                                                 inertia_allowed);

                    scroll_acc_x = 0;
                    scroll_acc_y = 0;
                    scroll_samples = 0;
                }
            }
            touched = true;

        } else if (active_layer == 6) {

            /* ==============================================
             *  L6 REV MOUSE  —  configurable axis mapping
             *  Same zero-filter pattern as normal mouse mode
             * ============================================== */
            int16_t max_raw = MAX(abs(raw_dx), abs(raw_dy));

            int32_t factor_q8_8;
            if (max_raw <= 2) {
                factor_q8_8 = 256; /* 1.0 */
            } else {
                int idx = clamp_int(current_mouse_gear, 1, 10) - 1;
                int16_t base_q8_8 = MOUSE_BASE_FACTOR_Q8_8[idx];
                int32_t accel_q8_8 = 256 + ((int32_t)(max_raw - 2) * 13);
                int accel_profile_idx = clamp_int(current_accel_profile, 1, 5) - 1;
                int16_t accel_cap = ACCEL_PROFILE_CAP_Q8_8[accel_profile_idx];
                if (accel_q8_8 > accel_cap) accel_q8_8 = accel_cap;
                factor_q8_8 = (base_q8_8 * accel_q8_8) >> 8;
            }

            int32_t vel_x, vel_y;  /* Q0 pixel output */
            int32_t inertia_vx, inertia_vy;  /* Q8.8 for inertia */

#if CONFIG_A320_L6_DIR_STANDARD
            vel_x      = ( raw_dx * factor_q8_8) >> 8;
            vel_y      = ( raw_dy * factor_q8_8) >> 8;
            inertia_vx = ( raw_dx * factor_q8_8);
            inertia_vy = ( raw_dy * factor_q8_8);
#elif CONFIG_A320_L6_DIR_SWAP_INV
            vel_x      = (-raw_dy * factor_q8_8) >> 8;
            vel_y      = (-raw_dx * factor_q8_8) >> 8;
            inertia_vx = (-raw_dy * factor_q8_8);
            inertia_vy = (-raw_dx * factor_q8_8);
#elif CONFIG_A320_L6_DIR_INV_X_ONLY
            vel_x      = (-raw_dx * factor_q8_8) >> 8;
            vel_y      = ( raw_dy * factor_q8_8) >> 8;
            inertia_vx = (-raw_dx * factor_q8_8);
            inertia_vy = ( raw_dy * factor_q8_8);
#elif CONFIG_A320_L6_DIR_INV_Y_ONLY
            vel_x      = ( raw_dx * factor_q8_8) >> 8;
            vel_y      = (-raw_dy * factor_q8_8) >> 8;
            inertia_vx = ( raw_dx * factor_q8_8);
            inertia_vy = (-raw_dy * factor_q8_8);
#endif

            input_report_rel(dev, INPUT_REL_X, Q8_8_TO_INT_ROUND(vel_x), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, Q8_8_TO_INT_ROUND(vel_y), true,  K_FOREVER);

            a320_inertia_update_on_touch(&inertia_state, inertia_vx, inertia_vy,
                                         inertia_allowed);
            touched = true;

        } else {

            /* ==============================================
             *  NORMAL MOUSE MODE  —  Build #37 style
             *  raw_dx/dy → acceleration → input_report_rel
             *  NO filter pipeline (zero-filter)
             * ============================================== */
            int16_t max_raw = MAX(abs(raw_dx), abs(raw_dy));

            int32_t factor_q8_8;
            if (max_raw <= 2) {
                factor_q8_8 = 256; /* 1.0 — low-speed precision */
            } else {
                int idx = clamp_int(current_mouse_gear, 1, 10) - 1;
                int16_t base_q8_8 = MOUSE_BASE_FACTOR_Q8_8[idx];
                int32_t accel_q8_8 = 256 + ((int32_t)(max_raw - 2) * 13);
                int accel_profile_idx = clamp_int(current_accel_profile, 1, 5) - 1;
                int16_t accel_cap = ACCEL_PROFILE_CAP_Q8_8[accel_profile_idx];
                if (accel_q8_8 > accel_cap) accel_q8_8 = accel_cap;
                factor_q8_8 = (base_q8_8 * accel_q8_8) >> 8;
            }

            int32_t vel_x = (raw_dx * factor_q8_8) >> 8;  /* Q0 pixel output */
            int32_t vel_y = (raw_dy * factor_q8_8) >> 8;

            /* Direct report — no filter, no LPF, no rate limiter */
            input_report_rel(dev, INPUT_REL_X, Q8_8_TO_INT_ROUND(vel_x), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, Q8_8_TO_INT_ROUND(vel_y), true,  K_FOREVER);

            /* Inertia: feed post-accel velocity in Q8.8 */
            a320_inertia_update_on_touch(&inertia_state, vel_x << 8, vel_y << 8,
                                         inertia_allowed);
            touched = true;
        }
    } else {
        /* ============================================================
         *  TOUCH INACTIVE  —  inertia decay only (triggers on release)
         * ============================================================ */
        if (touched) {
            /*
             * Release detected: inertia begins decay.
             * The inertia state was already updated during the last
             * touch tick with the pre-release velocity.
             */
            if (inertia_state.active) {
                uint8_t active_layer = zmk_keymap_highest_layer_active();
                bool is_scroll_mode = (active_layer == 4 || active_layer == 7);
                a320_inertia_tick(&inertia_state, is_scroll_mode, dev);
            }
        } else {
            /* Already released — continuous inertia decay */
            if (inertia_state.active) {
                uint8_t active_layer = zmk_keymap_highest_layer_active();
                bool is_scroll_mode = (active_layer == 4 || active_layer == 7);
                a320_inertia_tick(&inertia_state, is_scroll_mode, dev);
            }
        }

        /* Reset scroll accumulators on release */
        scroll_acc_x = 0;
        scroll_acc_y = 0;
        scroll_samples = 0;
        first_touch = true;
        touched = false;

    }

    k_work_reschedule(&data->poll_work, K_MSEC(CONFIG_A320_POLL_INTERVAL_MS));
}

/* ==================================================================
 *  4. I2C READ SEQUENCE
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
 *  5. DEVICE INIT & INSTANCE MACROS
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
