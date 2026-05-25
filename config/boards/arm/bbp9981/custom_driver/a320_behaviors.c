/*
 * 9981-Pro: Custom ZMK behaviors for sensor gear control
 *
 *   &mgear  —  set current_mouse_gear  (range 1–10)
 *   &sgear  —  set current_scroll_gear (range 1–9)
 *
 * Registered as a separate driver with its own DT_DRV_COMPAT
 * so DEVICE_DT_INST_DEFINE works cleanly.
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
#include "a320_0x57.h"

LOG_MODULE_REGISTER(a320_gear, CONFIG_A320_LOG_LEVEL);

/* ==================================================================
 *  Per-instance data: each DT node (mgear / sgear) gets one
 * ================================================================== */
struct sensor_gear_cfg {
    int *target;  /* pointer to current_mouse_gear or current_scroll_gear */
    int min_val;
    int max_val;
};

static int sensor_gear_binding_pressed(const struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event)
{
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct sensor_gear_cfg *cfg = dev->config;

    int gear = (int)binding->param1;
    if (gear < cfg->min_val) gear = cfg->min_val;
    if (gear > cfg->max_val) gear = cfg->max_val;
    *cfg->target = gear;

    LOG_INF("sensor_gear[%s] set to %d", dev->name, gear);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api sensor_gear_api = {
    .binding_pressed = sensor_gear_binding_pressed,
};

/* ==================================================================
 *  Instance definitions
 *
 *  Each DT node with compatible = "zmk,behavior-sensor-gear"
 *  gets a device instance.  In the .keymap we define two nodes:
 *    - &mgear → targets current_mouse_gear  (1..10)
 *    - &sgear → targets current_scroll_gear (1..9)
 *
 *  We distinguish them by the DT node label's "compatible" — in
 *  practice both have the same compatible, so we use the DT node
 *  unit-address or ordinal.  Since we cannot distinguish by compat,
 *  we map both to the same device type and let the keymap handle
 *  which parameter binds to which action.
 *
 *  Alternative:  iterate and assign via instance number.
 *    Inst 0 = mgear  (mouse)
 *    Inst 1 = sgear  (scroll)
 * ================================================================== */

#define SENSOR_GEAR_INST(n, _target, _min, _max)                                              \
    static const struct sensor_gear_cfg sensor_gear_cfg_##n = {                                \
        .target  = _target,                                                                    \
        .min_val = _min,                                                                       \
        .max_val = _max,                                                                       \
    };                                                                                         \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &sensor_gear_cfg_##n,                           \
                          POST_KERNEL, CONFIG_INPUT_A320_INIT_PRIORITY + 1,                    \
                          &sensor_gear_api);

/*
 * NOTE: DT_INST_FOREACH_STATUS_OKAY would iterate all instances.
 * However, both mgear and sgear share the same compatible, so we
 * cannot rely on instance order being portable.
 *
 * Instead, we define each instance manually, matching the DT node
 * ordinal 0 → mgear, 1 → sgear as declared in the keymap.
 */
SENSOR_GEAR_INST(0, zmk_a320_mouse_gear_ptr(),  1, 10)
SENSOR_GEAR_INST(1, zmk_a320_scroll_gear_ptr(), 1, 9)
