/*
 * autoclear.c — Auto-clear BLE bonds on firmware version change
 *
 * Uses a delayed work item (2s after boot) instead of SYS_INIT
 * to ensure ZMK settings subsystem is fully initialized before
 * reading/writing bond version data.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/settings/settings.h>
#include <zmk/ble.h>

#define CURRENT_BOND_VERSION 1

static int stored_bond_version = -1;

static int autoclear_read_cb(const char *key, size_t len,
                              settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    if (settings_name_steq(key, "bond_version", &next) && !*next) {
        if (len != sizeof(stored_bond_version)) return -EINVAL;
        ssize_t ret = read_cb(cb_arg, &stored_bond_version, len);
        return (ret >= 0) ? 0 : (int)ret;
    }
    return -ENOENT;
}

static void autoclear_work_handler(struct k_work *work)
{
    int ret = settings_load_subtree_direct("autoclear",
                                           autoclear_read_cb, NULL);

    if (ret || stored_bond_version != CURRENT_BOND_VERSION) {
        zmk_ble_clear_all_bonds();
        int version = CURRENT_BOND_VERSION;
        settings_save_one("autoclear/bond_version",
                          &version, sizeof(version));
    }
}

static K_WORK_DELAYABLE_DEFINE(autoclear_work, autoclear_work_handler);

static int autoclear_init(void)
{
    k_work_schedule(&autoclear_work, K_SECONDS(2));
    return 0;
}

SYS_INIT(autoclear_init, APPLICATION, 90);
