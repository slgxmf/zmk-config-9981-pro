/*
 * autoclear.c — Auto-clear BLE bonds on firmware version change
 *
 * On boot, reads stored bond_version from ZMK settings. If the
 * version doesn't match CURRENT_BOND_VERSION (hardcoded below),
 * all BLE pairings are cleared and the new version is persisted.
 *
 * This ensures a fresh firmware flash always starts with clean
 * bond data, preventing dead-pairing issues.
 */

#include <zephyr/init.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>

#define CURRENT_BOND_VERSION 1

static int stored_bond_version = -1;

static int autoclear_set(const char *key, size_t len,
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

static int autoclear_init(void)
{
    int ret = settings_load_subtree_direct("autoclear", autoclear_set, NULL);

    /* If read failed or version doesn't match, clear bonds */
    if (ret || stored_bond_version != CURRENT_BOND_VERSION) {
        bt_unpair(BT_ID_DEFAULT, NULL);
        int version = CURRENT_BOND_VERSION;
        settings_save_one("autoclear/bond_version",
                          &version, sizeof(version));
    }

    return 0;
}

SYS_INIT(autoclear_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
