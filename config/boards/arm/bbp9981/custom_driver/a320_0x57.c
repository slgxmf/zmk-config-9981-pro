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

static int current_mouse_gear   = 2;  /* range 1–10 */
static int current_scroll_gear  = 2;  /* range 1–9  */
static int current_accel_profile = 3; /* range 1–5, default 3 (normal) */
static bool swap_buttons = false;      /* LCLK↔RCLK swap flag */

/* ==================================================================
 *  R13: Inertia state for exponential-decay glide
 * ================================================================== */

/*
 * Precomputed decay values (Q8.0) for poll_interval = 10 ms:
 *   These represent (256 - decay) = exp(-10ms/τ) in Q8.0.
 *   Larger decay = faster braking (smaller τ).
 *
 *   Fix 2: Quadrupled from original so inertia stops noticeably faster.
 *
 *   decay=32 → (256-32)/256 = 0.875 → τ = -10/ln(0.875) ≈ 75 ms  (quick)
 *   decay=48 → (256-48)/256 = 0.8125 → τ ≈ 48 ms  (aggressive)
 *   decay=64 → (256-64)/256 = 0.75 → τ ≈ 35 ms  (very quick)
 *   decay=96 → (256-96)/256 = 0.625 → τ ≈ 20 ms  (near-instant)
 */
static const uint8_t INERTIA_DECAY_LUT[4] = {
    32,  /* τ≈75ms (quick stop) */
    48,  /* τ≈48ms (aggressive — default) */
    64,  /* τ≈35ms (very quick) */
    96,  /* τ≈20ms (near-instant) */
};
#define INERTIA_DEFAULT_DECAY_IDX 1   /* τ≈48ms (aggressive) */
#define INERTIA_THRESHOLD 4           /* Stop inertia below this velocity */

static int current_inertia_decay_idx = INERTIA_DEFAULT_DECAY_IDX;

static struct a320_inertia inertia_state = {
    .vx = 0,
    .vy = 0,
    .decay = 8,  /* INERTIA_DECAY_LUT[INERTIA_DEFAULT_DECAY_IDX] */
    .active = false,
};

/* Exposed for a320_behaviors.c (header declares these) */
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

/* ---- Scroll-mode accumulation ---- */
static int16_t scroll_acc_x = 0;
static int16_t scroll_acc_y = 0;
static uint8_t scroll_samples = 0;

/* ==================================================================
 *  1b.  FILTER PIPELINE STATE  (R4)
 *       Baseline → LPF → Rate Limiter → Accel Comp
 *       All Kconfig-configurable, Q8.0 fixed-point.
 * ================================================================== */

/* --- LPF low-pass filter state --- */
static int16_t lpf_state_x = 0;
static int16_t lpf_state_y = 0;

/* --- Rate limiter previous values --- */
static int16_t rlim_prev_x = 0;
static int16_t rlim_prev_y = 0;

/* --- Accel compensation previous values --- */
static int16_t accel_prev_x = 0;
static int16_t accel_prev_y = 0;

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
 *
 *      All arithmetic in Q8.8 fixed-point (no float).
 * ================================================================== */

static inline int clamp_int(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/*
 * Q8.8 → Q0.0 signed rounding (Fix 1 — direction asymmetry)
 *
 * Standard (x + 128) >> 8 truncates negative values incorrectly
 * when x is negative (C arithmetic right-shift rounds toward -inf).
 * This macro adds ±128 before shift so that:
 *   v=256   → (256+128)>>8  = 384>>8  = 1    (was 1, same)
 *   v=138   → (138+128)>>8  = 266>>8  = 1    (was 0 — fixed!)
 *   v=1     → (1+128)>>8    = 129>>8  = 0    (still 0, correct for sub-pixel)
 *   v=-138  → (-138-128)>>8 = -266>>8 = -1   (was -1, same)
 *   v=-1    → (-1-128)>>8   = -129>>8 = -1   (was -1, same)
 */
#define Q8_8_TO_INT_ROUND(v)  (((v) + ((v) >= 0 ? 128 : -128)) >> 8)

/* ==================================================================
 *  3b.  FILTER PIPELINE  (R4)
 *       Low-Pass (1st-order IIR) → Rate Limiter
 *       → Acceleration Compensation (lead)
 *
 *       All arithmetic in Q8.0 fixed-point. All parameters Kconfig.
 *       Returns dx, dy as the filtered output (in-out params).
 * ================================================================== */

#ifndef CONFIG_A320_FILTER_LPF_ALPHA
#define CONFIG_A320_FILTER_LPF_ALPHA 128
#endif
#ifndef CONFIG_A320_FILTER_RATE_LIMIT
#define CONFIG_A320_FILTER_RATE_LIMIT 8
#endif
#ifndef CONFIG_A320_FILTER_ACCEL_GAIN
#define CONFIG_A320_FILTER_ACCEL_GAIN 32
#endif

static void a320_filter_pipeline(int16_t *dx, int16_t *dy)
{
    int16_t raw_dx = *dx;
    int16_t raw_dy = *dy;

    /* ----------------------------------------------------------
     *  Stage 1: Low-Pass Filter (1st-order IIR)
     *  Smooths jitter by blending with previous state.
     *  out = prev + (in - prev) * alpha / 256
     *  alpha=0 → no filtering, alpha=255 → full pass-through
     * ---------------------------------------------------------- */
    if (CONFIG_A320_FILTER_LPF_ALPHA < 255) {
        int16_t lpf_alpha = (int16_t)CONFIG_A320_FILTER_LPF_ALPHA;
        lpf_state_x += (int16_t)(((*dx - lpf_state_x) * lpf_alpha) >> 8);
        lpf_state_y += (int16_t)(((*dy - lpf_state_y) * lpf_alpha) >> 8);
        *dx = lpf_state_x;
        *dy = lpf_state_y;
    }

    /* ----------------------------------------------------------
     *  Stage 2: Rate Limiter (jitter spike suppression)
     *  Caps delta between successive readings.
     *  delta = clamp(current - prev, -limit, +limit)
     *  out = prev + delta
     * ---------------------------------------------------------- */
    if (CONFIG_A320_FILTER_RATE_LIMIT > 0) {
        int16_t rlim = (int16_t)CONFIG_A320_FILTER_RATE_LIMIT;
        int16_t delta_x = *dx - rlim_prev_x;
        int16_t delta_y = *dy - rlim_prev_y;

        if (delta_x > rlim)  delta_x = rlim;
        if (delta_x < -rlim) delta_x = -rlim;
        if (delta_y > rlim)  delta_y = rlim;
        if (delta_y < -rlim) delta_y = -rlim;

        rlim_prev_x += delta_x;
        rlim_prev_y += delta_y;
        *dx = rlim_prev_x;
        *dy = rlim_prev_y;
    } else {
        rlim_prev_x = *dx;
        rlim_prev_y = *dy;
    }

    /* ----------------------------------------------------------
     *  Stage 3: Acceleration Compensation (lead)
     *  When velocity changes, add a fraction of the delta to
     *  reduce perceived lag from earlier filtering.
     *  out += (out - prev_accel) * gain / 256
     * ---------------------------------------------------------- */
    if (CONFIG_A320_FILTER_ACCEL_GAIN > 0) {
        int16_t gain = (int16_t)CONFIG_A320_FILTER_ACCEL_GAIN;
        int16_t accel_dx = *dx - accel_prev_x;
        int16_t accel_dy = *dy - accel_prev_y;

        *dx += (int16_t)((accel_dx * gain) >> 8);
        *dy += (int16_t)((accel_dy * gain) >> 8);

        accel_prev_x = *dx;
        accel_prev_y = *dy;
    } else {
        accel_prev_x = *dx;
        accel_prev_y = *dy;
    }
}

/*
 * Geometric base-speed table (Q8.8).
 *   gear 1 → 0.5x  = 128
 *   gear 10 → 0.7x = 179
 *   ratio per step = (179/128)^(1/9) ≈ 1.0381
 */
static const int16_t MOUSE_BASE_FACTOR_Q8_8[10] = {
    128, 133, 138, 143, 149, 154, 160, 166, 172, 179
};

/*
 * Accel max-multiplier cap LUT (Q8.8 fixed-point).
 *   Level 1 (slow):   1.5x = 384
 *   Level 2 (normal-): 2.0x = 512
 *   Level 3 (normal):  2.5x = 640  (default)
 *   Level 4 (fast):    3.5x = 896
 *   Level 5 (aggressive): 5.0x = 1280
 */
static const int16_t ACCEL_PROFILE_CAP_Q8_8[5] = {
    384, 512, 640, 896, 1280
};

/*
 * Scroll speed factor LUT (Q10.6 fixed-point).
 *   gear 1 = 0.6x = 38,  gear 9 = 1.1x = 70
 *   geometric ratio per step = (70/38)^(1/8) ≈ 1.079
 */
/*
 * Scroll speed factor LUT (Q10.6 fixed-point).
 *   gear 1 = 0.59375x ≈ 0.6x    (38/64)
 *   gear 9 = 1.09375x ≈ 1.1x    (70/64)
 *   geometric (constant-ratio) series: r = (70/38)^(1/8) ≈ 1.079355
 *
 * Computed as: val[i] = round(38 * r^i),  i = 0..8
 */
static const uint8_t SCROLL_SPEED_FACTOR_Q10_6[9] = {
    38, 41, 44, 48, 52, 56, 60, 65, 70
};

/* Scroll threshold: gear 1→9, gear 9→1  (higher gear = more responsive) */
static inline uint8_t scroll_gear_threshold(int gear)
{
    return (uint8_t)(12 - clamp_int(gear, 1, 9));
}

/* ==================================================================
 *  R13: Inertia helpers
 * ================================================================== */

/*
 * @brief Apply one tick of exponential decay to the inertia state.
 *        v_next = (v * (256 - decay)) >> 8
 *        When velocity falls below INERTIA_THRESHOLD in both axes,
 *        inertia is deactivated.
 */
static bool a320_inertia_tick(struct a320_inertia *inertia, bool is_scroll_mode,
                              const struct device *dev)
{
    int16_t vx = (int16_t)(((int32_t)inertia->vx * (256 - inertia->decay)) >> 8);
    int16_t vy = (int16_t)(((int32_t)inertia->vy * (256 - inertia->decay)) >> 8);

    if (abs(vx) < INERTIA_THRESHOLD && abs(vy) < INERTIA_THRESHOLD) {
        inertia->active = false;
        inertia->vx = 0;
        inertia->vy = 0;
        return false;  /* inertia ended */
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

    return true;  /* inertia still active */
}

/*
 * @brief Check whether inertia glide is enabled on the given layer.
 *        Inertia is enabled on normal mouse layers, scroll temp (4),
 *        rev mouse (6), rev scroll (7), and scroll fixed (8).
 */
static inline bool a320_inertia_layer_allowed(uint8_t layer)
{
    /* All handled layers get inertia.
     * Layer 4 = L_SCROLL_TEMP (scroll),
     * Layer 6 = rev mouse,
     * Layer 7 = rev scroll,
     * Layer 8 = L_SCROLL_FIXED (scroll),
     * default (other) = normal mouse.
     * Layer 0 (base) doesn't reach here but won't hurt.
     */
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
 *  End R13 helpers
 * ================================================================== */

static void a320_poll_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct a320_data *data = CONTAINER_OF(dwork, struct a320_data, poll_work);
    const struct device *dev = data->dev;

    /* ---- R13: Sync decay LUT on first use after change ---- */
    inertia_state.decay = INERTIA_DECAY_LUT[current_inertia_decay_idx];

    int pin_state = gpio_pin_get(motion_gpio_dev, MOTION_GPIO_PIN);

    if (pin_state == 0) {
        /* ============================================================
         *  TOUCH ACTIVE: read motion, compute velocity, update inertia
         * ============================================================ */
        int16_t raw_dx = 0, raw_dy = 0;

        if (a320_read_motion(dev, &raw_dx, &raw_dy) != 0) {
            k_work_reschedule(&data->poll_work, K_MSEC(CONFIG_A320_POLL_INTERVAL_MS));
            return;
        }

        /* ---- Fix 3: Reset filter state on touch start ----
         *      Must happen BEFORE a320_filter_pipeline so filter
         *      starts clean on every new touch (not just on release).
         *      Release reset still exists in the inactive branch below. */
        if (!touched) {
            lpf_state_x = 0;
            lpf_state_y = 0;
            rlim_prev_x = 0;
            rlim_prev_y = 0;
            accel_prev_x = 0;
            accel_prev_y = 0;
        }

        /* ---- R4: Apply filter pipeline (LPF → Rate Limit → Accel Comp) ---- */
        a320_filter_pipeline(&raw_dx, &raw_dy);

        /* ---- Query current active layer (Section 2.3) ---- */
        uint8_t active_layer = zmk_keymap_highest_layer_active();

        /* ---- L3-002: Reset scroll accumulators on layer transition ---- */
        {
            static uint8_t prev_active_layer = 0xFF;
            if (prev_active_layer != 0xFF && active_layer != prev_active_layer) {
                scroll_acc_x = 0;
                scroll_acc_y = 0;
                scroll_samples = 0;
            }
            prev_active_layer = active_layer;
        }

        bool inertia_allowed = a320_inertia_layer_allowed(active_layer);

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
                    int gear_idx = clamp_int(current_scroll_gear, 1, 9) - 1;
                    int32_t w_x_acc = -(scroll_acc_x * (int32_t)SCROLL_SPEED_FACTOR_Q10_6[gear_idx]);
                    int32_t w_y_acc = -(scroll_acc_y * (int32_t)SCROLL_SPEED_FACTOR_Q10_6[gear_idx]);
                    int16_t w_x = (int16_t)(w_x_acc / (24 * 64));
                    int16_t w_y = (int16_t)(w_y_acc / (24 * 64));

                    if (w_y > 3)  w_y = 3;
                    else if (w_y < -3) w_y = -3;
                    if (w_x > 3)  w_x = 3;
                    else if (w_x < -3) w_x = -3;

                    input_report_rel(dev, INPUT_REL_HWHEEL, -w_x, false, K_FOREVER);
                    input_report_rel(dev, INPUT_REL_WHEEL,   w_y, true,  K_FOREVER);

                    /* R13: Update inertia velocity in scroll factor units (Q8.8) */
                    a320_inertia_update_on_touch(&inertia_state,
                                                 (int32_t)scroll_acc_x * 256 / 24,
                                                 (int32_t)scroll_acc_y * 256 / 24,
                                                 inertia_allowed);

                    scroll_acc_x = 0;
                    scroll_acc_y = 0;
                    scroll_samples = 0;
                } else {
                    /* Accumulating — update inertia with partial accumulation */
                    a320_inertia_update_on_touch(&inertia_state,
                                                 (int32_t)scroll_acc_x * 256 / threshold,
                                                 (int32_t)scroll_acc_y * 256 / threshold,
                                                 inertia_allowed);
                }
            }
            touched = true;

        } else if (active_layer == 6) {

            /* ==============================================
             *  L6 REV MOUSE  —  configurable axis mapping
             *
             *  Compile-time choice via Kconfig:
             *    A320_L6_DIR_STANDARD  —  dx→X, dy→Y
             *    A320_L6_DIR_SWAP_INV  —  dx→-Y, dy→-X  (default)
             *    A320_L6_DIR_INV_X_ONLY — dx→-X, dy→Y
             *    A320_L6_DIR_INV_Y_ONLY — dx→X, dy→-Y
             * ============================================== */
            int16_t max_raw = MAX(abs(raw_dx), abs(raw_dy));

            /* Fixed-point factor: Q8.8 */
            int32_t factor_q8_8;
            if (max_raw <= 2) {
                factor_q8_8 = 256; /* 1.0 in Q8.8 */
            } else {
                /* Base speed from geometric LUT, capped to [1,10] */
                int idx = clamp_int(current_mouse_gear, 1, 10) - 1;
                int16_t base_q8_8 = MOUSE_BASE_FACTOR_Q8_8[idx];

                /* Accel: 1.0 + (max_raw - 2) * 0.05  →  256 + (max_raw - 2) * 13 */
                int32_t accel_q8_8 = 256 + ((int32_t)(max_raw - 2) * 13);
                /* Apply configurable accel cap from profile LUT */
                int accel_profile_idx = clamp_int(current_accel_profile, 1, 5) - 1;
                int16_t accel_cap = ACCEL_PROFILE_CAP_Q8_8[accel_profile_idx];
                if (accel_q8_8 > accel_cap) accel_q8_8 = accel_cap;

                factor_q8_8 = (base_q8_8 * accel_q8_8) >> 8;
            }

            int32_t velocity_x_q8_8, velocity_y_q8_8;
#if CONFIG_A320_L6_DIR_STANDARD
            velocity_x_q8_8 = (raw_dx * factor_q8_8) >> 8;
            velocity_y_q8_8 = (raw_dy * factor_q8_8) >> 8;
            input_report_rel(dev, INPUT_REL_X, Q8_8_TO_INT_ROUND(velocity_x_q8_8), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, Q8_8_TO_INT_ROUND(velocity_y_q8_8), true,  K_FOREVER);
#elif CONFIG_A320_L6_DIR_SWAP_INV
            velocity_x_q8_8 = (-raw_dy * factor_q8_8) >> 8;
            velocity_y_q8_8 = (-raw_dx * factor_q8_8) >> 8;
            input_report_rel(dev, INPUT_REL_X, Q8_8_TO_INT_ROUND(velocity_x_q8_8), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, Q8_8_TO_INT_ROUND(velocity_y_q8_8), true,  K_FOREVER);
#elif CONFIG_A320_L6_DIR_INV_X_ONLY
            velocity_x_q8_8 = (-raw_dx * factor_q8_8) >> 8;
            velocity_y_q8_8 = (raw_dy * factor_q8_8) >> 8;
            input_report_rel(dev, INPUT_REL_X, Q8_8_TO_INT_ROUND(velocity_x_q8_8), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, Q8_8_TO_INT_ROUND(velocity_y_q8_8), true,  K_FOREVER);
#elif CONFIG_A320_L6_DIR_INV_Y_ONLY
            velocity_x_q8_8 = (raw_dx * factor_q8_8) >> 8;
            velocity_y_q8_8 = (-raw_dy * factor_q8_8) >> 8;
            input_report_rel(dev, INPUT_REL_X, Q8_8_TO_INT_ROUND(velocity_x_q8_8), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, Q8_8_TO_INT_ROUND(velocity_y_q8_8), true,  K_FOREVER);
#endif
            /* R13: Update inertia with scaled velocity */
            a320_inertia_update_on_touch(&inertia_state, velocity_x_q8_8, velocity_y_q8_8,
                                         inertia_allowed);
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
                    int gear_idx = clamp_int(current_scroll_gear, 1, 9) - 1;
                    int32_t w_x_acc = scroll_acc_x * (int32_t)SCROLL_SPEED_FACTOR_Q10_6[gear_idx];
                    int32_t w_y_acc = scroll_acc_y * (int32_t)SCROLL_SPEED_FACTOR_Q10_6[gear_idx];
                    int16_t w_x = (int16_t)(w_x_acc / (24 * 64));
                    int16_t w_y = (int16_t)(w_y_acc / (24 * 64)); /* not negated → reversed direction */

                    if (w_y > 3)  w_y = 3;
                    else if (w_y < -3) w_y = -3;
                    if (w_x > 3)  w_x = 3;
                    else if (w_x < -3) w_x = -3;

                    input_report_rel(dev, INPUT_REL_HWHEEL, w_x, false, K_FOREVER);
                    input_report_rel(dev, INPUT_REL_WHEEL,  w_y, true,  K_FOREVER);

                    /* R13: Update inertia velocity in scroll factor units (Q8.8) */
                    a320_inertia_update_on_touch(&inertia_state,
                                                 (int32_t)scroll_acc_x * 256 / 24,
                                                 (int32_t)scroll_acc_y * 256 / 24,
                                                 inertia_allowed);

                    scroll_acc_x = 0;
                    scroll_acc_y = 0;
                    scroll_samples = 0;
                } else {
                    /* Accumulating — update inertia with partial accumulation */
                    a320_inertia_update_on_touch(&inertia_state,
                                                 (int32_t)scroll_acc_x * 256 / (int32_t)threshold,
                                                 (int32_t)scroll_acc_y * 256 / (int32_t)threshold,
                                                 inertia_allowed);
                }
            }
            touched = true;

        } else {

            /* ==============================================
             *  NORMAL MODE  —  nonlinear dynamic accel
             * ============================================== */
            int16_t max_raw = MAX(abs(raw_dx), abs(raw_dy));

            /* Fixed-point factor: Q8.8 */
            int32_t factor_q8_8;
            if (max_raw <= 2) {
                /* Low-speed precision: 1:1 pixel (Section 2.2) */
                factor_q8_8 = 256; /* 1.0 in Q8.8 */
            } else {
                /* Base speed from geometric LUT, capped to [1,10] */
                int idx = clamp_int(current_mouse_gear, 1, 10) - 1;
                int16_t base_q8_8 = MOUSE_BASE_FACTOR_Q8_8[idx];

                /* Accel: 1.0 + (max_raw - 2) * 0.05  →  256 + (max_raw - 2) * 13 */
                int32_t accel_q8_8 = 256 + ((int32_t)(max_raw - 2) * 13);
                /* Apply configurable accel cap from profile LUT */
                int accel_profile_idx = clamp_int(current_accel_profile, 1, 5) - 1;
                int16_t accel_cap = ACCEL_PROFILE_CAP_Q8_8[accel_profile_idx];
                if (accel_q8_8 > accel_cap) accel_q8_8 = accel_cap;

                factor_q8_8 = (base_q8_8 * accel_q8_8) >> 8;
            }

            int32_t velocity_x_q8_8 = (raw_dx * factor_q8_8) >> 8;
            int32_t velocity_y_q8_8 = (raw_dy * factor_q8_8) >> 8;

            /* Fix 1: Use signed rounding to fix positive-value truncation */
            input_report_rel(dev, INPUT_REL_X, Q8_8_TO_INT_ROUND(velocity_x_q8_8), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, Q8_8_TO_INT_ROUND(velocity_y_q8_8), true,  K_FOREVER);

            /* R13: Update inertia with scaled velocity */
            a320_inertia_update_on_touch(&inertia_state, velocity_x_q8_8, velocity_y_q8_8,
                                         inertia_allowed);
            touched = true;
        }
    } else {
        /* ============================================================
         *  TOUCH INACTIVE  —  apply inertia decay if active
         * ============================================================ */
        if (inertia_state.active) {
            uint8_t active_layer = zmk_keymap_highest_layer_active();
            bool is_scroll_mode = (active_layer == 4 || active_layer == 7);
            a320_inertia_tick(&inertia_state, is_scroll_mode, dev);
        }
        touched = false;

        /* R4-fix: Reset filter state on touch release to prevent drift */
        lpf_state_x = 0;
        lpf_state_y = 0;
        rlim_prev_x = 0;
        rlim_prev_y = 0;
        accel_prev_x = 0;
        accel_prev_y = 0;
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
