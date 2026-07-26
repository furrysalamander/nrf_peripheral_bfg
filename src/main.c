/*
 * nrf_peripheral_bfg
 * main.c
 * Bluetooth fuel gauge seeed board workshop main file
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <math.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mfd/npm2100.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm2100_vbat.h>
#include <zephyr/dt-bindings/regulator/npm2100.h>
#include <zephyr/sys/util.h>

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm2100_vbat.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <nrf_fuel_gauge.h>

static int64_t ref_time;
static bool fuel_gauge_initialized;

static const struct battery_model_primary battery_model = {
#include <battery_models/primary_cell/LR44.inc>
};
#define AVERAGE_CURRENT (CONFIG_ACTIVE_CURRENT_UA * 1e-6f)

static const struct i2c_dt_spec pmic_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(npm2100_pmic));
static const struct device *pmic_regulators = DEVICE_DT_GET(DT_NODELABEL(npm2100_regulators));
static const struct device *vbat = DEVICE_DT_GET(DT_NODELABEL(npm2100_vbat));

#define PMIC_THREAD_STACK_SIZE 1024
#define PMIC_THREAD_PRIORITY 5
#define PMIC_SLEEP_INTERVAL_MS 1000


LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct gpio_dt_spec bt_status_led = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);

static int shphld_state(bool *state)
{
    int ret;
    uint8_t reg;

    ret = i2c_reg_read_byte_dt(&pmic_i2c, 0x16, &reg);
    if (ret)
    {
        LOG_ERR("Could not read STATUS register (%d)", ret);
        return ret;
    }
    *state = reg & 1U;

    return 0;
}

static int read_sensors(const struct device *vbat, float *voltage, float *temp)
{
    struct sensor_value value;
    int ret;

    ret = sensor_sample_fetch(vbat);
    if (ret < 0)
    {
        return ret;
    }

    sensor_channel_get(vbat, SENSOR_CHAN_GAUGE_VOLTAGE, &value);
    *voltage = (float)value.val1 + ((float)value.val2 / 1000000);

    sensor_channel_get(vbat, SENSOR_CHAN_DIE_TEMP, &value);
    *temp = (float)value.val1 + ((float)value.val2 / 1000000);

    return 0;
}

int fuel_gauge_init(const struct device *vbat, char *bat_name, size_t n)
{
    int err;
    bool state;

    if (!device_is_ready(pmic_regulators))
    {
        LOG_ERR("PMIC device is not ready (init_res: %d)", pmic_regulators->state->init_res);
        return -ENODEV;
    }
    LOG_INF("PMIC device ok");

    // check if we are in shphld.
    err = shphld_state(&state);
    if (err)
    {
        return err;
    }

    struct nrf_fuel_gauge_init_parameters parameters = {
        .model_primary = &battery_model,
        .i0 = AVERAGE_CURRENT,
        .opt_params = NULL,
    };
    struct nrf_fuel_gauge_config_parameters rt_params;
    nrf_fuel_gauge_config_params_default_get(&rt_params);
    rt_params.discard_positive_deltaz = true;
    int ret;

    ret = read_sensors(vbat, &parameters.v0, &parameters.t0);
    if (ret < 0)
    {
        return ret;
    }

    ret = nrf_fuel_gauge_init(&parameters, NULL);
    if (ret < 0)
    {
        return ret;
    }

    ret = nrf_fuel_gauge_config_params_adjust(&rt_params);
    if (ret < 0)
    {
        return ret;
    }

    ref_time = k_uptime_get();
    strncpy(bat_name, battery_model.name, n);
    err = 0;

    return err;
}

int fuel_gauge_update(const struct device *vbat, uint8_t *soc)
{
    float voltage;
    float temp;
    float delta;
    int ret;

    ret = read_sensors(vbat, &voltage, &temp);
    if (ret < 0)
    {
        return ret;
    }

    delta = (float)k_uptime_delta(&ref_time) / 1000.f;

    float soc_float;
    ret = nrf_fuel_gauge_process(voltage, AVERAGE_CURRENT, temp, delta, &soc_float, NULL);
    if (ret < 0)
    {
        return ret;
    }
    *soc = (uint8_t)soc_float;

    LOG_INF("PMIC Thread sending: V: %.3f, T: %.2f, SoC: %d", (double)voltage, (double)temp, *soc);

    return 0;
}

int pmic_fg_thread(void)
{

    fuel_gauge_initialized = false;
    uint8_t soc;

    for (;;)
    {
        if (!fuel_gauge_initialized)
        {
            int err;
            char bat_name[16];
            err = fuel_gauge_init(vbat, bat_name, 16);
            if (err < 0)
            {
                LOG_INF("Could not initialise fuel gauge.");
                return 0;
            }
            LOG_INF("Fuel gauge initialised for %s battery.", bat_name);

            fuel_gauge_initialized = true;
        }
        fuel_gauge_update(vbat, &soc);
        k_msleep(PMIC_SLEEP_INTERVAL_MS);
    }
}

int pmic_reg_thread(void)
{

    for (;;)
    {
        k_msleep(2000);
    }
}

int main(void)
{
    int err;

    err = gpio_pin_configure_dt(&bt_status_led, GPIO_OUTPUT_INACTIVE);
    if (err)
    {
        LOG_WRN("Configuring BT status LED failed (err %d)", err);
        return -1;
    }

    for (;;)
    {
        gpio_pin_set_dt(&bt_status_led, 1);
        k_msleep(500);
        LOG_INF("I am alive");
        gpio_pin_set_dt(&bt_status_led, 0);
        k_msleep(2000);
    }
    return 0;
}

K_THREAD_DEFINE(pmic_reg_thread_id, PMIC_THREAD_STACK_SIZE, pmic_reg_thread, NULL, NULL, NULL, PMIC_THREAD_PRIORITY, 0,
                1000);
K_THREAD_DEFINE(pmic_fg_thread_id, PMIC_THREAD_STACK_SIZE, pmic_fg_thread, NULL, NULL, NULL, PMIC_THREAD_PRIORITY, 0,
                0);
