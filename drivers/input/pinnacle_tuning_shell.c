#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "input_pinnacle.h"

#define PINNACLE_DEV_ENTRY(node_id) DEVICE_DT_GET(node_id),

static const struct device *const pinnacle_devs[] = {
    DT_FOREACH_STATUS_OKAY(cirque_pinnacle_toucan, PINNACLE_DEV_ENTRY)
};

static const struct device *pinnacle_default_dev(void) {
    for (size_t i = 0; i < ARRAY_SIZE(pinnacle_devs); i++) {
        if (device_is_ready(pinnacle_devs[i])) {
            return pinnacle_devs[i];
        }
    }

    return NULL;
}

static int parse_i32(const char *text, int32_t *value) {
    char *end;
    long parsed = strtol(text, &end, 0);

    if (text == end || *end != '\0' || parsed < INT32_MIN || parsed > INT32_MAX) {
        return -EINVAL;
    }

    *value = (int32_t)parsed;
    return 0;
}

static void print_runtime_error(const struct shell *sh, const char *op, int ret) {
    switch (ret) {
    case -ENOENT:
        shell_print(sh, "TP ERR %s unknown_param", op);
        break;
    case -ERANGE:
        shell_print(sh, "TP ERR %s out_of_range", op);
        break;
    case -ENODEV:
        shell_print(sh, "TP ERR %s no_device", op);
        break;
    case -EINVAL:
        shell_print(sh, "TP ERR %s invalid_arg", op);
        break;
    default:
        shell_print(sh, "TP ERR %s %d", op, ret);
        break;
    }
}

static int print_runtime_value(const struct shell *sh, const struct device *dev,
                               const char *name) {
    int32_t value;
    int ret = pinnacle_runtime_get(dev, name, &value);

    if (ret < 0) {
        print_runtime_error(sh, "get", ret);
        return ret;
    }

    shell_print(sh, "TP %s=%d", name, value);
    return 0;
}

static int cmd_trackpad_get(const struct shell *sh, size_t argc, char **argv) {
    const struct device *dev = pinnacle_default_dev();

    if (!dev) {
        print_runtime_error(sh, "get", -ENODEV);
        return 0;
    }

    if (argc == 2) {
        int ret = print_runtime_value(sh, dev, argv[1]);
        if (ret < 0) {
            return 0;
        }
        shell_print(sh, "TP OK");
        return 0;
    }

    for (size_t i = 0; i < pinnacle_runtime_param_count(); i++) {
        const char *name = pinnacle_runtime_param_name(i);
        if (name) {
            print_runtime_value(sh, dev, name);
        }
    }
    shell_print(sh, "TP OK");
    return 0;
}

static int cmd_trackpad_list(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    for (size_t i = 0; i < pinnacle_runtime_param_count(); i++) {
        const char *name = pinnacle_runtime_param_name(i);
        if (name) {
            shell_print(sh, "TP %s", name);
        }
    }
    shell_print(sh, "TP OK");
    return 0;
}

static int cmd_trackpad_set(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);

    const struct device *dev = pinnacle_default_dev();
    int32_t value;

    if (!dev) {
        print_runtime_error(sh, "set", -ENODEV);
        return 0;
    }

    int ret = parse_i32(argv[2], &value);
    if (ret < 0) {
        print_runtime_error(sh, "set", ret);
        return 0;
    }

    ret = pinnacle_runtime_set(dev, argv[1], value);
    if (ret < 0) {
        print_runtime_error(sh, "set", ret);
        return 0;
    }

    print_runtime_value(sh, dev, argv[1]);
    shell_print(sh, "TP OK");
    return 0;
}

static int cmd_trackpad_reset(const struct shell *sh, size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    const struct device *dev = pinnacle_default_dev();

    if (!dev) {
        print_runtime_error(sh, "reset", -ENODEV);
        return 0;
    }

    int ret = pinnacle_runtime_reset(dev);
    if (ret < 0) {
        print_runtime_error(sh, "reset", ret);
        return 0;
    }

    for (size_t i = 0; i < pinnacle_runtime_param_count(); i++) {
        const char *name = pinnacle_runtime_param_name(i);
        if (name) {
            print_runtime_value(sh, dev, name);
        }
    }
    shell_print(sh, "TP OK");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    trackpad_subcmds,
    SHELL_CMD_ARG(get, NULL, "Get one or all runtime trackpad tunables",
                  cmd_trackpad_get, 1, 1),
    SHELL_CMD_ARG(list, NULL, "List runtime trackpad tunables",
                  cmd_trackpad_list, 1, 0),
    SHELL_CMD_ARG(set, NULL, "Set a runtime trackpad tunable",
                  cmd_trackpad_set, 3, 0),
    SHELL_CMD_ARG(reset, NULL, "Reset runtime trackpad tunables",
                  cmd_trackpad_reset, 1, 0),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(trackpad, &trackpad_subcmds, "Toucan trackpad runtime tuning", NULL);
