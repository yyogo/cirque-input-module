#define DT_DRV_COMPAT cirque_pinnacle

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>

#include "input_pinnacle.h"

LOG_MODULE_REGISTER(pinnacle, CONFIG_INPUT_LOG_LEVEL);

static int pinnacle_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                             const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    return config->seq_read(dev, addr, buf, len);
}
static int pinnacle_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    return config->write(dev, addr, val);
}

// Now that we are counting ZIDLEs it would be very bad to miss one.  But in my testing I see that happen (rarely - once every
// couple of days of usage).  The fact that the current irq system is edge triggered probably isn't great for this reason.
// But for now just have the touch controller emit NUM_ZIDLE_PAD extra idles
#define NUM_ZIDLE  3
#define NUM_ZIDLE_PAD 2

// Fast-tap thresholds (when tap_fast DT prop is set).
#define TAP_FAST_MAX_MS         250  // lift-by deadline for a touch to count as a tap
#define TAP_FAST_BUFFER_MS      120  // motion held back for at most this long; absorbs tap noise without making slow drags feel laggy
#define TAP_FAST_MAX_DRAG        25  // motion budget within the tap window before it's reclassified as a drag
#define TAP_FAST_DRAG_WINDOW_MS 150  // window after a tap during which a new touch becomes a held-click drag

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

static int pinnacle_i2c_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    return i2c_burst_read_dt(&config->bus.i2c, PINNACLE_READ | addr, buf, len);
}

static int pinnacle_i2c_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    return i2c_reg_write_byte_dt(&config->bus.i2c, PINNACLE_WRITE | addr, val);
}

#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int pinnacle_spi_seq_read(const struct device *dev, const uint8_t addr, uint8_t *buf,
                                 const uint8_t len) {
    const struct pinnacle_config *config = dev->config;
    uint8_t tx_buffer[len + 3], rx_dummy[3];
    tx_buffer[0] = PINNACLE_READ | addr;
    memset(&tx_buffer[1], PINNACLE_AUTOINC, len + 2);

    const struct spi_buf tx_buf[2] = {
        {
            .buf = tx_buffer,
            .len = len + 3,
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = 1,
    };
    struct spi_buf rx_buf[2] = {
        {
            .buf = rx_dummy,
            .len = 3,
        },
        {
            .buf = buf,
            .len = len,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = 2,
    };
    int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    return ret;
}

static int pinnacle_spi_write(const struct device *dev, const uint8_t addr, const uint8_t val) {
    const struct pinnacle_config *config = dev->config;
    uint8_t tx_buffer[2] = {PINNACLE_WRITE | addr, val};
    uint8_t rx_buffer[2];

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = 2,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = 2,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    const int ret = spi_transceive_dt(&config->bus.spi, &tx, &rx);

    if (ret < 0) {
        LOG_ERR("spi ret: %d", ret);
    }

    if (rx_buffer[1] != PINNACLE_FILLER) {
        LOG_ERR("bad ret val %d - %d", rx_buffer[0], rx_buffer[1]);
        return -EIO;
    }

    k_usleep(50);

    return ret;
}
#endif // DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int set_int(const struct device *dev, const bool en) {
    const struct pinnacle_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->dr,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }

    return ret;
}

static int pinnacle_clear_status(const struct device *dev) {
    int ret = pinnacle_write(dev, PINNACLE_STATUS1, 0);
    if (ret < 0) {
        LOG_ERR("Failed to clear STATUS1 register: %d", ret);
    }

    return ret;
}

static int pinnacle_era_read(const struct device *dev, const uint16_t addr, uint8_t *val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_READ);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_VALUE, val, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read ERA value (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static int pinnacle_era_write(const struct device *dev, const uint16_t addr, uint8_t val) {
    int ret;

    set_int(dev, false);

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_VALUE, val);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA value (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_HIGH_BYTE, (uint8_t)(addr >> 8));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA high byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_LOW_BYTE, (uint8_t)(addr & 0x00FF));
    if (ret < 0) {
        LOG_ERR("Failed to write ERA low byte (%d)", ret);
        return -EIO;
    }

    ret = pinnacle_write(dev, PINNACLE_REG_ERA_CONTROL, PINNACLE_ERA_CONTROL_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to write ERA control (%d)", ret);
        return -EIO;
    }

    uint8_t control_val;
    do {

        ret = pinnacle_seq_read(dev, PINNACLE_REG_ERA_CONTROL, &control_val, 1);
        if (ret < 0) {
            LOG_ERR("Failed to read ERA control (%d)", ret);
            return -EIO;
        }

    } while (control_val != 0x00);

    ret = pinnacle_clear_status(dev);

    set_int(dev, true);

    return ret;
}

static void pinnacle_apply_smoothing(struct pinnacle_data *data, uint8_t strength,
                                     int8_t *dx, int8_t *dy) {
    if (strength < 2) {
        return;
    }

    int abs_dx = *dx < 0 ? -*dx : *dx;
    int abs_dy = *dy < 0 ? -*dy : *dy;
    int abs_total = abs_dx + abs_dy;

    int gain_q8;
    const int fast_thresh = 4;
    if (abs_total >= fast_thresh) {
        gain_q8 = 256;
    } else {
        int min_gain = 256 / strength;
        gain_q8 = min_gain + ((256 - min_gain) * abs_total) / fast_thresh;
    }

    data->smooth_accum_x_q8 += (int32_t)*dx * gain_q8;
    data->smooth_accum_y_q8 += (int32_t)*dy * gain_q8;

    int32_t out_dx = data->smooth_accum_x_q8 / 256;
    int32_t out_dy = data->smooth_accum_y_q8 / 256;
    data->smooth_accum_x_q8 -= out_dx * 256;
    data->smooth_accum_y_q8 -= out_dy * 256;

    if (out_dx > INT8_MAX) out_dx = INT8_MAX;
    if (out_dx < INT8_MIN) out_dx = INT8_MIN;
    if (out_dy > INT8_MAX) out_dy = INT8_MAX;
    if (out_dy < INT8_MIN) out_dy = INT8_MIN;

    *dx = (int8_t)out_dx;
    *dy = (int8_t)out_dy;
}

static void pinnacle_send_rel(const struct device *dev, int8_t dx, int8_t dy) {
    const struct pinnacle_config *config = dev->config;
    struct pinnacle_data *data = dev->data;

    pinnacle_clear_status(dev);
    set_int(dev, true);

    bool must_send = false;

    uint8_t btn = data->last_btn;
    if (!config->no_taps && (btn || data->btn_cache)) {
        for (int i = 0; i < 3; i++) {
            uint8_t btn_val = btn & BIT(i);
            if (btn_val != (data->btn_cache & BIT(i))) {
                input_report_key(dev, INPUT_BTN_0 + i, btn_val ? 1 : 0, false, K_FOREVER);
                must_send = true;
            }
        }
    }

    data->btn_cache = btn;
    bool is_touching = (data->last_z > 0);
    bool touch_changed = false;
    if(is_touching) {
        must_send = true;

        if(data->num_z_idle > 0) { // we just recently had z idles
            touch_changed = true;
            data->num_z_idle = 0;
            dx = 0;
            dy = 0; // starting a new press, must reset deltas
            data->smooth_accum_x_q8 = 0;
            data->smooth_accum_y_q8 = 0;
        }
    } else {
        data->num_z_idle++;
        if(data->num_z_idle == NUM_ZIDLE) {
            touch_changed = true;
        }
        dx = 0;
        dy = 0;
    }

    LOG_DBG("Rel move: touch_changed=%d z=%d dx=%d dy=%d", touch_changed, data->last_z, dx, dy);

    if(touch_changed)
    {
        // Finalize the input event only if we have something to report
        input_report_key(dev, INPUT_BTN_TOUCH, is_touching ? 1 : 0, false, K_FOREVER);
        must_send = true;
    }

    pinnacle_apply_smoothing(data, config->smoothing_strength, &dx, &dy);

    // Fast-tap state machine: hold motion back during the could-be-tap
    // window so a tap with slight finger drift doesn't drag the cursor;
    // synthesize a click on a quick lift; if a new touch arrives soon
    // after that click, treat it as a held-button drag.
    if (config->tap_fast) {
        int64_t now = k_uptime_get();

        if (touch_changed && is_touching) {
            data->tap_touch_started_ms = now;
            data->tap_buf_dx = 0;
            data->tap_buf_dy = 0;
            data->tap_motion_total = 0;
            data->tap_buffering = true;
            data->tap_eligible = true;

            // Drag-and-hold: a fresh touch within the post-tap window means
            // the user wants the previous click to "stick" — emit BTN_PRIM
            // press now, hold until lift, no buffering on this touch.
            if (data->tap_completed_ms != 0 &&
                now - data->tap_completed_ms < TAP_FAST_DRAG_WINDOW_MS) {
                input_report_key(dev, INPUT_BTN_0, 1, false, K_FOREVER);
                data->tap_drag_held = true;
                data->tap_buffering = false;
                data->tap_eligible = false;
            }
            data->tap_completed_ms = 0;
        }

        if (is_touching && (data->tap_buffering || data->tap_eligible)) {
            int abs_dx = dx < 0 ? -dx : dx;
            int abs_dy = dy < 0 ? -dy : dy;
            data->tap_motion_total += abs_dx + abs_dy;
            data->tap_buf_dx += dx;
            data->tap_buf_dy += dy;
            int64_t duration = now - data->tap_touch_started_ms;

            if (data->tap_motion_total > TAP_FAST_MAX_DRAG ||
                duration > TAP_FAST_MAX_MS) {
                data->tap_eligible = false;
            }

            if (data->tap_buffering) {
                // Flush only when motion is real or the user is clearly
                // dragging — a still finger past BUFFER_MS keeps buffering
                // so a slow tap (held still then lifted) still clicks.
                bool motion_flush = data->tap_motion_total > TAP_FAST_MAX_DRAG;
                bool time_flush = data->tap_motion_total > 0 &&
                                  duration > TAP_FAST_BUFFER_MS;
                if (motion_flush || time_flush) {
                    int32_t fx = data->tap_buf_dx;
                    int32_t fy = data->tap_buf_dy;
                    if (fx > INT8_MAX) fx = INT8_MAX;
                    if (fx < INT8_MIN) fx = INT8_MIN;
                    if (fy > INT8_MAX) fy = INT8_MAX;
                    if (fy < INT8_MIN) fy = INT8_MIN;
                    dx = (int8_t)fx;
                    dy = (int8_t)fy;
                    data->tap_buffering = false;
                } else {
                    dx = 0;
                    dy = 0;
                }
            }
        }

        if (touch_changed && !is_touching) {
            if (data->tap_drag_held) {
                input_report_key(dev, INPUT_BTN_0, 0, false, K_FOREVER);
                data->tap_drag_held = false;
            } else if (data->tap_eligible) {
                input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
                input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
                data->tap_completed_ms = now;
            }
            data->tap_buffering = false;
            data->tap_eligible = false;
        }
    }

    if(must_send) {
        input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
    }
}

static void pinnacle_send_abs(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    struct pinnacle_data *data = dev->data;
    int16_t x = data->last_x;
    int16_t y = data->last_y;
    int8_t z = data->last_z;

    LOG_DBG("Clearing status bit");
    pinnacle_clear_status(dev);
    set_int(dev, true);

    uint8_t btn = data->last_btn;
    if (!config->no_taps && (btn || data->btn_cache)) {
        for (int i = 0; i < 3; i++) {
            uint8_t btn_val = btn & BIT(i);
            if (btn_val != (data->btn_cache & BIT(i))) {
                input_report_key(dev, INPUT_BTN_0 + i, btn_val ? 1 : 0, false, K_FOREVER);
            }
        }
    }

    data->btn_cache = btn;

    if (z > 0) {
        if (x < config->absolute_mode_clamp_min_x) {
            x = config->absolute_mode_clamp_min_x;
        } else if (x > config->absolute_mode_clamp_max_x) {
            x = config->absolute_mode_clamp_max_x;
        }
        if (y < config->absolute_mode_clamp_min_y) {
            y = config->absolute_mode_clamp_min_y;
        } else if (y > config->absolute_mode_clamp_max_y) {
            y = config->absolute_mode_clamp_max_y;
        }

        // scale to be in the configured interval
        x = ((x - config->absolute_mode_clamp_min_x) * config->absolute_mode_scale_to_width) / (config->absolute_mode_clamp_max_x - config->absolute_mode_clamp_min_x);
        y = ((y - config->absolute_mode_clamp_min_y) * config->absolute_mode_scale_to_height) / (config->absolute_mode_clamp_max_y - config->absolute_mode_clamp_min_y);

        input_report_abs(dev, INPUT_ABS_X, x, false, K_FOREVER);
        input_report_abs(dev, INPUT_ABS_Y, y, true, K_FOREVER);
    }

    return;
}

static int pinnacle_read_abs(const struct device *dev) {
    uint8_t packet[6];
    int ret;
    ret = pinnacle_seq_read(dev, PINNACLE_STATUS1, packet, 1);
    if (ret < 0) {
        LOG_ERR("read status: %d", ret);
        return ret;
    }
    if (!(packet[0] & PINNACLE_STATUS1_SW_DR)) {
        return -1;
    }
    ret = pinnacle_seq_read(dev, PINNACLE_2_2_PACKET0, packet, 6);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return ret;
    }
    struct pinnacle_data *data = dev->data;
    // TODO: Enable SW3-SW5 as well
    data->last_btn = packet[0] &
                  (PINNACLE_PACKET0_BTN_PRIM | PINNACLE_PACKET0_BTN_SEC | PINNACLE_PACKET0_BTN_AUX);
    uint8_t x_low = packet[2];
    uint8_t y_low = packet[3];
    uint8_t xy_high = packet[4];
    data->last_x = ((xy_high & 0x0F) << 8) | x_low;
    data->last_y = ((xy_high & 0xF0) << 4) | y_low;
    data->last_z = (uint8_t)(packet[5] & 0x1F);

    LOG_DBG("button: %d, x: %d y: %d z: %d", data->last_btn, data->last_x, data->last_y, data->last_z);
    return 0;
}

static void pinnacle_report_data_abs(const struct device *dev) {
    int ret = pinnacle_read_abs(dev);
    if (ret == 0) {
        pinnacle_send_abs(dev);
    }
}

static void pinnacle_report_data_abs_rel(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    int16_t old_x = data->last_x;
    int16_t old_y = data->last_y;

    int ret = pinnacle_read_abs(dev);

    if (ret == 0) {
        int16_t dx = data->last_x - old_x;
        int16_t dy = data->last_y - old_y;
        const struct pinnacle_config *config = dev->config;

        dx /= config->abs_rel_divisor;
        dy /= config->abs_rel_divisor;

        pinnacle_send_rel(dev, (int8_t) dx, (int8_t) dy);
    }
}

static void pinnacle_report_data_rel(const struct device *dev) {
    uint8_t packet[3];
    int ret;
    ret = pinnacle_seq_read(dev, PINNACLE_STATUS1, packet, 1);
    if (ret < 0) {
        LOG_ERR("read status: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 1, "Pinnacle Status1");

    // Ignore 0xFF packets that indicate communcation failure, or if SW_DR isn't asserted
    if (packet[0] == 0xFF || !(packet[0] & PINNACLE_STATUS1_SW_DR)) {
        return;
    }
    ret = pinnacle_seq_read(dev, PINNACLE_2_2_PACKET0, packet, 3);
    if (ret < 0) {
        LOG_ERR("read packet: %d", ret);
        return;
    }

    LOG_HEXDUMP_DBG(packet, 3, "Pinnacle Packets");

    struct pinnacle_data *data = dev->data;
    data->last_btn = packet[0] &
                  (PINNACLE_PACKET0_BTN_PRIM | PINNACLE_PACKET0_BTN_SEC | PINNACLE_PACKET0_BTN_AUX);

    int8_t dx = (int8_t)packet[1];
    int8_t dy = (int8_t)packet[2];

    if (packet[0] & PINNACLE_PACKET0_X_SIGN) {
        WRITE_BIT(dx, 7, 1);
    }
    if (packet[0] & PINNACLE_PACKET0_Y_SIGN) {
        WRITE_BIT(dy, 7, 1);
    }

    // always claim touch changed
    data->last_z = 1;
    pinnacle_send_rel(dev, (int8_t) dx, (int8_t) dy);
}

static void pinnacle_work_cb(struct k_work *work) {
    struct pinnacle_data *data = CONTAINER_OF(work, struct pinnacle_data, work);
    const struct device *dev = data->dev;
    const struct pinnacle_config *config = dev->config;

    if (config->absolute_mode) {
        pinnacle_report_data_abs(dev);
    } else if (config->abs_rel_divisor) {
        pinnacle_report_data_abs_rel(dev);
    } else {
        pinnacle_report_data_rel(dev);
    }
}

static void pinnacle_gpio_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct pinnacle_data *data = CONTAINER_OF(cb, struct pinnacle_data, gpio_cb);

    LOG_DBG("HW DR asserted");
    set_int(data->dev, false); // mask the int until we've handled it (now level triggered)
    k_work_submit(&data->work);
}

static int pinnacle_adc_sensitivity_reg_value(enum pinnacle_sensitivity sensitivity) {
    switch (sensitivity) {
    case PINNACLE_SENSITIVITY_1X:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    case PINNACLE_SENSITIVITY_2X:
        return PINNACLE_TRACKING_ADC_CONFIG_2X;
    case PINNACLE_SENSITIVITY_3X:
        return PINNACLE_TRACKING_ADC_CONFIG_3X;
    case PINNACLE_SENSITIVITY_4X:
        return PINNACLE_TRACKING_ADC_CONFIG_4X;
    default:
        return PINNACLE_TRACKING_ADC_CONFIG_1X;
    }
}

static int pinnacle_tune_edge_sensitivity(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;
    int ret;

    uint8_t x_val;
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_X_AXIS_WIDE_Z_MIN, &x_val);
    if (ret < 0) {
        LOG_WRN("Failed to read X val");
        return ret;
    }

    LOG_WRN("X val: %d", x_val);

    uint8_t y_val;
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_Y_AXIS_WIDE_Z_MIN, &y_val);
    if (ret < 0) {
        LOG_WRN("Failed to read Y val");
        return ret;
    }

    LOG_WRN("Y val: %d", y_val);

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_X_AXIS_WIDE_Z_MIN, config->x_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set X-Axis Min-Z %d", ret);
        return ret;
    }
    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_Y_AXIS_WIDE_Z_MIN, config->y_axis_z_min);
    if (ret < 0) {
        LOG_ERR("Failed to set Y-Axis Min-Z %d", ret);
        return ret;
    }
    return 0;
}

static int pinnacle_set_adc_tracking_sensitivity(const struct device *dev) {
    const struct pinnacle_config *config = dev->config;

    uint8_t val;
    int ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    val &= 0x3F;
    val |= pinnacle_adc_sensitivity_reg_value(config->sensitivity);

    ret = pinnacle_era_write(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, val);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
    }
    ret = pinnacle_era_read(dev, PINNACLE_ERA_REG_TRACKING_ADC_CONFIG, &val);
    if (ret < 0) {
        LOG_ERR("Failed to get ADC sensitivity %d", ret);
    }

    return ret;
}

static int pinnacle_force_recalibrate(const struct device *dev) {
    uint8_t val;
    int ret = pinnacle_seq_read(dev, PINNACLE_CAL_CFG, &val, 1);
    if (ret < 0) {
        LOG_ERR("Failed to get cal config %d", ret);
    }

    val |= 0x01;
    ret = pinnacle_write(dev, PINNACLE_CAL_CFG, val);
    if (ret < 0) {
        LOG_ERR("Failed to force calibration %d", ret);
    }

    do {
        pinnacle_seq_read(dev, PINNACLE_CAL_CFG, &val, 1);
    } while (val & 0x01);

    return ret;
}

int pinnacle_set_sleep(const struct device *dev, bool enabled) {
    uint8_t sys_cfg;
    int ret = pinnacle_seq_read(dev, PINNACLE_SYS_CFG, &sys_cfg, 1);
    if (ret < 0) {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & PINNACLE_SYS_CFG_EN_SLEEP) != 0) == enabled) {
        return 0;
    }

    LOG_DBG("Setting sleep: %s", (enabled ? "on" : "off"));
    WRITE_BIT(sys_cfg, PINNACLE_SYS_CFG_EN_SLEEP_BIT, enabled ? 1 : 0);

    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, sys_cfg);
    if (ret < 0) {
        LOG_ERR("can't write sleep config %d", ret);
        return ret;
    }

    return ret;
}


int pinnacle_set_shutdown(const struct device *dev, bool enabled) {
    uint8_t sys_cfg;
    int ret = pinnacle_seq_read(dev, PINNACLE_SYS_CFG, &sys_cfg, 1);
    if (ret < 0) {
        LOG_ERR("can't read sys config %d", ret);
        return ret;
    }

    if (((sys_cfg & PINNACLE_SYS_CFG_SHUTDOWN) != 0) == enabled) {
        LOG_WRN("Shutdown already set to %s", (enabled ? "on" : "off"));
    }

    LOG_DBG("Setting shutdown: %s", (enabled ? "on" : "off"));

    if(enabled) {
        // interrupt pin might have bogus asserts in shtdown so disable ints here
        // FIXME if this is in things are fucked later
        set_int(dev, false);
        }

    WRITE_BIT(sys_cfg, PINNACLE_SYS_CFG_SHUTDOWN_BIT, enabled ? 1 : 0);

    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, sys_cfg);
    if (ret < 0) {
        LOG_ERR("can't write shutdown config %d", ret);
        return ret;
    }

    if (!enabled) {
        // pinnacle_clear_status(dev); // clear any spurious ints on wake
        ret = set_int(dev, true);
    }
    else {
        ret = pinnacle_seq_read(dev, PINNACLE_SYS_CFG, &sys_cfg, 1);
        if (ret < 0) {
            LOG_ERR("can't read sys config %d", ret);
            return ret;
        }
        LOG_DBG("Shutdown readback: %s", (sys_cfg & PINNACLE_SYS_CFG_SHUTDOWN) ? "on" : "off");
    }

    return ret;
}

static int pinnacle_init(const struct device *dev) {
    struct pinnacle_data *data = dev->data;
    const struct pinnacle_config *config = dev->config;
    int ret;

    uint8_t fw_id[2];
    ret = pinnacle_seq_read(dev, PINNACLE_FW_ID, fw_id, 2);
    if (ret < 0) {
        LOG_ERR("Failed to get the FW ID %d", ret);
    }

    LOG_DBG("Found device with FW ID: 0x%02x, Version: 0x%02x", fw_id[0], fw_id[1]);

    k_msleep(10);
    ret = pinnacle_write(dev, PINNACLE_STATUS1, 0); // Clear CC
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    k_usleep(50);
    ret = pinnacle_write(dev, PINNACLE_SYS_CFG, PINNACLE_SYS_CFG_RESET);
    if (ret < 0) {
        LOG_ERR("can't reset %d", ret);
        return ret;
    }
    k_msleep(20);
    ret = pinnacle_write(dev, PINNACLE_Z_IDLE, NUM_ZIDLE + NUM_ZIDLE_PAD);
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    ret = pinnacle_set_adc_tracking_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set ADC sensitivity %d", ret);
        return ret;
    }

    ret = pinnacle_tune_edge_sensitivity(dev);
    if (ret < 0) {
        LOG_ERR("Failed to tune edge sensitivity %d", ret);
        return ret;
    }
    ret = pinnacle_force_recalibrate(dev);
    if (ret < 0) {
        LOG_ERR("Failed to force recalibration %d", ret);
        return ret;
    }

    if (config->sleep_en) {
        ret = pinnacle_set_sleep(dev, true);
        if (ret < 0) {
            return ret;
        }
    }

    uint8_t packet[1];
    ret = pinnacle_seq_read(dev, PINNACLE_SLEEP_INTERVAL, packet, 1);
 
    if (ret >= 0) {
        LOG_DBG("Default sleep interval %d", packet[0]);
    }

    ret = pinnacle_write(dev, PINNACLE_SLEEP_INTERVAL, 255);
    if (ret <= 0) {
        LOG_DBG("Failed to update sleep interaval %d", ret);
    }

    uint8_t feed_cfg2 = PINNACLE_FEED_CFG2_EN_IM | PINNACLE_FEED_CFG2_EN_BTN_SCRL;
    if (config->no_taps || config->tap_fast) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_TAP;
    }

    if (config->no_secondary_tap || config->tap_fast) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_DIS_SEC;
    }

    if (config->rotate_90) {
        feed_cfg2 |= PINNACLE_FEED_CFG2_ROTATE_90;
    }
    ret = pinnacle_write(dev, PINNACLE_FEED_CFG2, feed_cfg2);
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }
    uint8_t feed_cfg1 = PINNACLE_FEED_CFG1_EN_FEED;
    if (config->absolute_mode || config->abs_rel_divisor) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_ABS_MODE;
        LOG_ERR("Using absolute mode");
    } else {
        LOG_ERR("Using relative mode");
    }
    if (config->x_invert) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_INV_X;
    }

    if (config->y_invert) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_INV_Y;
    }
    if (config->disable_filter) {
        feed_cfg1 |= PINNACLE_FEED_CFG1_DIS_FILT;
    }
    if (feed_cfg1) {
        ret = pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);
    }
    if (ret < 0) {
        LOG_ERR("can't write %d", ret);
        return ret;
    }

    data->dev = dev;

    pinnacle_clear_status(dev);

    gpio_pin_configure_dt(&config->dr, GPIO_INPUT);
    gpio_init_callback(&data->gpio_cb, pinnacle_gpio_cb, BIT(config->dr.pin));
    ret = gpio_add_callback(config->dr.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Failed to set DR callback: %d", ret);
        return -EIO;
    }

    k_work_init(&data->work, pinnacle_work_cb);

    pinnacle_write(dev, PINNACLE_FEED_CFG1, feed_cfg1);

    set_int(dev, true);

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int pinnacle_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        pinnacle_set_shutdown(dev, true);
        return 0;
    case PM_DEVICE_ACTION_RESUME:
        pinnacle_set_shutdown(dev, false);
        return 0;
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define PINNACLE_INST(n)                                                                           \
    static struct pinnacle_data pinnacle_data_##n;                                                 \
    static const struct pinnacle_config pinnacle_config_##n = {                                    \
        COND_CODE_1(DT_INST_ON_BUS(n, i2c),                                                        \
                    (.bus = {.i2c = I2C_DT_SPEC_INST_GET(n)}, .seq_read = pinnacle_i2c_seq_read,   \
                     .write = pinnacle_i2c_write),                                                 \
                    (.bus = {.spi = SPI_DT_SPEC_INST_GET(n,                                        \
                                                         SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |    \
                                                             SPI_TRANSFER_MSB | SPI_MODE_CPHA,     \
                                                         0)},                                      \
                     .seq_read = pinnacle_spi_seq_read, .write = pinnacle_spi_write)),             \
        .rotate_90 = DT_INST_PROP(n, rotate_90),                                                   \
        .x_invert = DT_INST_PROP(n, x_invert),                                                     \
        .y_invert = DT_INST_PROP(n, y_invert),                                                     \
        .sleep_en = DT_INST_PROP(n, sleep),                                                        \
        .no_taps = DT_INST_PROP(n, no_taps),                                                       \
        .no_secondary_tap = DT_INST_PROP(n, no_secondary_tap),                                     \
        .disable_filter = DT_INST_PROP(n, disable_filter),                                         \
        .smoothing_strength = DT_INST_PROP(n, smoothing_strength),                                 \
        .tap_fast = DT_INST_PROP(n, tap_fast),                                                     \
        .absolute_mode = DT_INST_PROP(n, absolute_mode),                                           \
        .abs_rel_divisor = DT_INST_PROP(n, abs_rel_divisor),                                       \
        .absolute_mode_scale_to_width = DT_INST_PROP(n, absolute_mode_scale_to_width),             \
        .absolute_mode_scale_to_height = DT_INST_PROP(n, absolute_mode_scale_to_height),           \
        .absolute_mode_clamp_min_x = DT_INST_PROP(n, absolute_mode_clamp_min_x),                   \
        .absolute_mode_clamp_max_x = DT_INST_PROP(n, absolute_mode_clamp_max_x),                   \
        .absolute_mode_clamp_min_y = DT_INST_PROP(n, absolute_mode_clamp_min_y),                   \
        .absolute_mode_clamp_max_y = DT_INST_PROP(n, absolute_mode_clamp_max_y),                   \
        .x_axis_z_min = DT_INST_PROP_OR(n, x_axis_z_min, 5),                                       \
        .y_axis_z_min = DT_INST_PROP_OR(n, y_axis_z_min, 4),                                       \
        .sensitivity = DT_INST_ENUM_IDX_OR(n, sensitivity, PINNACLE_SENSITIVITY_1X),               \
        .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(n), dr_gpios, {}),                                   \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, pinnacle_pm_action);                                               \
    DEVICE_DT_INST_DEFINE(n, pinnacle_init, PM_DEVICE_DT_INST_GET(n), &pinnacle_data_##n,          \
                          &pinnacle_config_##n, POST_KERNEL, CONFIG_INPUT_PINNACLE_INIT_PRIORITY,  \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(PINNACLE_INST)
