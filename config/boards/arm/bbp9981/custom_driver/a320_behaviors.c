/*
 * 9981-Pro: Custom ZMK behaviors for sensor gear control
 *
 *   &mgear  —  set current_mouse_gear  (range 1–10)
 *   &sgear  —  set current_scroll_gear (range 1–9)
 *
 * Uses BEHAVIOR_DT_INST_DEFINE so Zephyr's behavior system registers &mgear / &sgear.
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
