/*
 * 9981-Pro: Custom ZMK behaviors for sensor gear control
 *
 *   &mgear  —  set current_mouse_gear  (range 1–10)
 *   &sgear  —  set current_scroll_gear (range 1–9)
 *   &msbtn  —  toggle LCLK/RCLK swap
 *   &mkpswap —  mouse key press with optional LCLK↔RCLK swap
 *
 * Uses BEHAVIOR_DT_INST_DEFINE so Zephyr's behavior system registers
 * the behaviors at &mgear / &sgear / &msbtn / &mkpswap.
 * .target is assigned at init time (not static) because the function
 * call result is not a compile-time constant in C99.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_gear

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <drivers/behavior.h>
#include <zmk/hid.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <dt-bindings/zmk/pointing.h>
#include "a320_0x57.h"

LOG_MODULE_REGISTER(a320_gear, CONFIG_A320_LOG_LEVEL);

/* ==================================================================
 *  Per-instance data: each DT node (mgear / sgear) gets one
 * ================================================================== */
struct sensor_gear_data {
    int *target;  /* pointer to current_mouse_gear or current_scroll_gear */
};

struct sensor_gear_cfg {
    int min_val;
    int max_val;
};

static int sensor_gear_binding_pressed(const struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event)
{
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    if (!dev) {
        LOG_ERR("sensor_gear: failed to resolve '%s'", log_strdup(binding->behavior_dev));
        return ZMK_BEHAVIOR_OPAQUE;
    }
    const struct sensor_gear_cfg *cfg = dev->config;
    struct sensor_gear_data *data = dev->data;
    if (!cfg || !data) {
        LOG_ERR("sensor_gear: null config/data for '%s'", log_strdup(binding->behavior_dev));
        return ZMK_BEHAVIOR_OPAQUE;
    }

    int gear = (int)binding->param1;
    if (gear < cfg->min_val) gear = cfg->min_val;
    if (gear > cfg->max_val) gear = cfg->max_val;
    *data->target = gear;

    LOG_INF("sensor_gear[%s] set to %d", dev->name, gear);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api sensor_gear_api = {
    .binding_pressed = sensor_gear_binding_pressed,
};

/* ==================================================================
 *  Init: set the target pointer at boot (function call not constexpr)
 * ================================================================== */
static int sensor_gear_init_0(const struct device *dev)
{
    struct sensor_gear_data *data = dev->data;
    data->target = zmk_a320_mouse_gear_ptr();
    return 0;
}

static int sensor_gear_init_1(const struct device *dev)
{
    struct sensor_gear_data *data = dev->data;
    data->target = zmk_a320_scroll_gear_ptr();
    return 0;
}

static int sensor_gear_init_2(const struct device *dev)
{
    struct sensor_gear_data *data = dev->data;
    data->target = zmk_a320_accel_profile_ptr();
    return 0;
}

/* ==================================================================
 *  Static device instances
 * ================================================================== */
#define SENSOR_GEAR_INST(n, _init_fn, _min, _max)                                              \
    static struct sensor_gear_data sensor_gear_data_##n;                                        \
    static const struct sensor_gear_cfg sensor_gear_cfg_##n = {                                 \
        .min_val = _min,                                                                       \
        .max_val = _max,                                                                       \
    };                                                                                         \
    BEHAVIOR_DT_INST_DEFINE(n, _init_fn, NULL, &sensor_gear_data_##n, &sensor_gear_cfg_##n,       \
                             POST_KERNEL, CONFIG_INPUT_A320_INIT_PRIORITY,                    \
                             &sensor_gear_api);

SENSOR_GEAR_INST(0, sensor_gear_init_0, 1, 10)
SENSOR_GEAR_INST(1, sensor_gear_init_1, 1, 9)
SENSOR_GEAR_INST(2, sensor_gear_init_2, 1, 5)

/* ==================================================================
 *  SWAP-BUTTONS TOGGLE BEHAVIOR (&msbtn)
 *
 *  Toggles the swap_buttons flag.
 *  When ON: &mkpswap LCLK sends right-click, &mkpswap RCLK sends left-click.
 *  Compat: zmk,behavior-swap-buttons
 * ================================================================== */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_swap_buttons

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int swap_buttons_binding_pressed(const struct zmk_behavior_binding *binding,
                                        struct zmk_behavior_binding_event event)
{
    bool *flag = zmk_a320_swap_buttons_ptr();
    *flag = !(*flag);
    LOG_INF("swap_buttons toggled to %s", *flag ? "ON" : "OFF");
    return ZMK_BEHAVIOR_OPAQUE;
}

static int swap_buttons_binding_released(const struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event)
{
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api swap_buttons_api = {
    .binding_pressed = swap_buttons_binding_pressed,
    .binding_released = swap_buttons_binding_released,
};

#define SWAP_BTN_INST(n)                                                                           \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_INPUT_A320_INIT_PRIORITY, &swap_buttons_api);

DT_INST_FOREACH_STATUS_OKAY(SWAP_BTN_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

/* ==================================================================
 *  MOUSE KEY PRESS WITH SWAP (&mkpswap)
 *
 *  Drop-in replacement for &mkp that checks the swap_buttons flag.
 *  When swap_buttons is true, MB1 (BIT(0)) is reported as INPUT_BTN_1
 *  and MB2 (BIT(1)) is reported as INPUT_BTN_0.
 *  Compat: zmk,behavior-mouse-key-press-swap
 * ================================================================== */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_mouse_key_press_swap

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static void mkpswap_process_key_state(const struct device *dev, int32_t val, bool pressed)
{
    bool *swap = zmk_a320_swap_buttons_ptr();
    int32_t swapped_val = val;

    if (*swap) {
        /* Swap MB1 (BIT(0)) ↔ MB2 (BIT(1)) */
        bool has_mb1 = (val & MB1) != 0;
        bool has_mb2 = (val & MB2) != 0;
        swapped_val = val & ~(MB1 | MB2); /* clear both */
        if (has_mb1) swapped_val |= MB2;
        if (has_mb2) swapped_val |= MB1;
    }

    for (int i = 0; i < ZMK_HID_MOUSE_NUM_BUTTONS; i++) {
        if (swapped_val & BIT(i)) {
            WRITE_BIT(swapped_val, i, 0);
            input_report_key(dev, INPUT_BTN_0 + i, pressed ? 1 : 0, swapped_val == 0, K_FOREVER);
        }
    }
}

static int mkpswap_binding_pressed(struct zmk_behavior_binding *binding,
                                   struct zmk_behavior_binding_event event)
{
    LOG_DBG("mkpswap pressed: param1=0x%02X", binding->param1);
    mkpswap_process_key_state(zmk_behavior_get_binding(binding->behavior_dev),
                              binding->param1, true);
    return 0;
}

static int mkpswap_binding_released(struct zmk_behavior_binding *binding,
                                    struct zmk_behavior_binding_event event)
{
    LOG_DBG("mkpswap released: param1=0x%02X", binding->param1);
    mkpswap_process_key_state(zmk_behavior_get_binding(binding->behavior_dev),
                              binding->param1, false);
    return 0;
}

static const struct behavior_driver_api mkpswap_api = {
    .binding_pressed = mkpswap_binding_pressed,
    .binding_released = mkpswap_binding_released,
};

#define MKPSWAP_INST(n)                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,                                \
                            CONFIG_INPUT_A320_INIT_PRIORITY, &mkpswap_api);

DT_INST_FOREACH_STATUS_OKAY(MKPSWAP_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

/* ==================================================================
 *  BT CLEAR BEHAVIOR (&bt_clr)
 *
 *  When pressed:
 *    1. Clears all BLE bonds via zmk_ble_clear_all_bonds()
 *    2. Flashes keyboard backlight 5 times (100 ms ON / 100 ms OFF)
 *    3. Restores original backlight brightness
 *
 *  Compat: zmk,behavior-bt-clr
 * ================================================================== */

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT zmk_behavior_bt_clr

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#include <zmk/ble.h>
#include <zmk/backlight.h>

struct bt_clr_blink_data {
    struct k_work_delayable blink_work;
    uint8_t blink_count;
    uint8_t phase;          /* 0 = OFF phase, 1 = ON phase */
    uint8_t original_brt;
    bool in_progress;
};

static struct bt_clr_blink_data bt_clr_state;

static void bt_clr_blink_handler(struct k_work *work);

static int bt_clr_binding_pressed(const struct zmk_behavior_binding *binding,
                                   struct zmk_behavior_binding_event event)
{
    if (bt_clr_state.in_progress) {
        LOG_WRN("bt_clr: blink already in progress, ignoring");
        return ZMK_BEHAVIOR_OPAQUE;
    }

    /* Step 1: Clear all BLE bonds (synchronous) */
    LOG_INF("bt_clr: clearing all BLE bonds");
    zmk_ble_clear_all_bonds();

    /* Step 2: Start backlight blink sequence */
    bt_clr_state.original_brt = zmk_backlight_get_brt();
    bt_clr_state.blink_count = 0;
    bt_clr_state.phase = 1;
    bt_clr_state.in_progress = true;

    zmk_backlight_set_brt(100);
    k_work_reschedule(&bt_clr_state.blink_work, K_MSEC(100));

    LOG_INF("bt_clr: blink sequence started (5 cycles)");
    return ZMK_BEHAVIOR_OPAQUE;
}

static void bt_clr_blink_handler(struct k_work *work)
{
    if (bt_clr_state.phase == 1) {
        /* ON → OFF */
        zmk_backlight_set_brt(0);
        bt_clr_state.phase = 0;
        k_work_reschedule(&bt_clr_state.blink_work, K_MSEC(100));
    } else {
        /* OFF → ON (unless this was the 5th off) */
        bt_clr_state.blink_count++;
        if (bt_clr_state.blink_count >= 5) {
            /* Done: restore original brightness */
            zmk_backlight_set_brt(bt_clr_state.original_brt);
            bt_clr_state.in_progress = false;
            LOG_INF("bt_clr: blink sequence complete, restored brt=%d",
                    bt_clr_state.original_brt);
            return;
        }
        zmk_backlight_set_brt(100);
        bt_clr_state.phase = 1;
        k_work_reschedule(&bt_clr_state.blink_work, K_MSEC(100));
    }
}

static int bt_clr_init(const struct device *dev)
{
    k_work_init_delayable(&bt_clr_state.blink_work, bt_clr_blink_handler);
    bt_clr_state.in_progress = false;
    return 0;
}

static const struct behavior_driver_api bt_clr_api = {
    .binding_pressed = bt_clr_binding_pressed,
};

BEHAVIOR_DT_INST_DEFINE(0, bt_clr_init, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &bt_clr_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
