/*
 * Copyright (c) 2023 ZitaoTech
 *
 * SPDX-License-Identifier: MIT
 *
 * Rewrite v2 — Exposes gear pointer API for ZMK behavior handlers.
 *
 * R13: Added inertia decay state and configuration accessors.
 */

#ifndef A320_0x57_H
#define A320_0x57_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Inertia state for exponential-decay glide after touch release.
 */
struct a320_inertia {
    int16_t vx;     /* Current velocity X (Q8.8 sensor-factor units) */
    int16_t vy;     /* Current velocity Y (Q8.8 sensor-factor units) */
    uint8_t decay;  /* Decay factor per tick (Q8.0, 0-255) */
    bool active;    /* Inertia currently running */
};

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

/**
 * @brief Get pointer to swap_buttons flag.
 *        When true, LCLK and RCLK are swapped.
 *        Used by the swap-buttons behavior handler.
 */
bool *zmk_a320_swap_buttons_ptr(void);

/**
 * @brief Get pointer to current_accel_profile (range 1–5).
 *        Used by the agear sensor-gear behavior handler.
 */
int *zmk_a320_accel_profile_ptr(void);

/**
 * @brief Get pointer to inertia decay index (0-3).
 *        Used by the inertia-decay behavior handler.
 *        Maps to INERTIA_DECAY_LUT[]: 0=τ0.6s, 1=τ0.45s, 2=τ0.3s, 3=τ0.2s.
 */
int *zmk_a320_inertia_decay_idx_ptr(void);

/**
 * @brief Get pointer to the global inertia state.
 *        Used by the inertia-decay behavior handler to read/write state.
 */
struct a320_inertia *zmk_a320_inertia_state_ptr(void);

#ifdef __cplusplus
}
#endif

#endif /* A320_0x57_H */
