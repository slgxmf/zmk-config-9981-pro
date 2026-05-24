<<<<<<< HEAD
/*
 * Copyright (c) 2023 ZitaoTech
 *
 * SPDX-License-Identifier: MIT
 *
 * Rewrite v2 — Exposes gear pointer API for ZMK behavior handlers.
 */

#ifndef A320_0x57_H
#define A320_0x57_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief Check if the touchpad is currently being touched.
 * @return true if touched
 */
bool tp_is_touched(void);

/**
 * @brief Get pointer to current_mouse_gear (range 1–10).
 *        Used by the sensor-gear behavior handler.
 */
int *zmk_a320_mouse_gear_ptr(void);

/**
 * @brief Get pointer to current_scroll_gear (range 1–9).
 *        Used by the sensor-gear behavior handler.
 */
int *zmk_a320_scroll_gear_ptr(void);

#ifdef __cplusplus
}
#endif

#endif /* A320_0x57_H */
=======
/*
 * Copyright (c) 2023 ZitaoTech
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef A320_0x57_H
#define A320_0x57_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief check if the touchpad is touched
 *
 * @return true if touched
 */
bool tp_is_touched(void);

#ifdef __cplusplus
}
#endif

#endif // A320__0x57_H
>>>>>>> upstream/main
