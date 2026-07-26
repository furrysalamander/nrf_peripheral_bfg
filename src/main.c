/*
 * nrf_peripheral_bfg
 * main.c
 * Bluetooth fuel gauge seeed board workshop main file
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <math.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mfd/npm2100.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm2100_vbat.h>
#include <zephyr/dt-bindings/regulator/npm2100.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <nrf_fuel_gauge.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm2100_vbat.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/types.h>

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

struct pmic_report_msg
{
    double batt_voltage;
    double temp;
    uint8_t batt_soc;
};

K_MSGQ_DEFINE(pmic_msgq, sizeof(struct pmic_report_msg), 8, 4);
K_MSGQ_DEFINE(ble_cfg_pmic_msgq, sizeof(int32_t), 8, 4);

#define BLE_NOTIFY_INTERVAL K_MSEC(1000)
#define BLE_THREAD_STACK_SIZE 1024
#define BLE_THREAD_PRIORITY 5

#define MAXLEN 19

#define PMIC_HUB_SERVICE_UUID BT_UUID_128_ENCODE(0x2100F600, 0x8445, 0x5fca, 0xb332, 0xc13064b9dea2)
#define PMIC_RD_ALL_CHARACTERISTIC_UUID BT_UUID_128_ENCODE(0x21002EAD, 0xA770, 0x57A7, 0xb333, 0xc13064b9dea2)
#define SHPHLD_WR_MV_CHARACTERISTIC_UUID BT_UUID_128_ENCODE(0x57EED111, 0x217E, 0x4faf, 0x956b, 0xafb01c17d0be)
#define BT_UUID_PMIC_HUB BT_UUID_DECLARE_128(PMIC_HUB_SERVICE_UUID)
#define BT_UUID_PMIC_HUB_RD_ALL BT_UUID_DECLARE_128(PMIC_RD_ALL_CHARACTERISTIC_UUID)
#define BT_UUID_PMIC_HUB_SHPHLD_WR BT_UUID_DECLARE_128(SHPHLD_WR_MV_CHARACTERISTIC_UUID)
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME // from prj.conf
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

struct bt_conn *m_connection_handle = NULL;

static const struct bt_le_adv_param *adv_param =
    BT_LE_ADV_PARAM((BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use
                                                                          identity address */
                    800,   /* Min Advertising Interval 500ms (800*0.625ms) 16383 max*/
                    801,   /* Max Advertising Interval 500.625ms (801*0.625ms) 16384 max*/
                    NULL); /* Set to NULL for undirected advertising */

static struct k_work adv_work;
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, PMIC_HUB_SERVICE_UUID),
};

/*This function is called whenever the Client Characteristic Control Descriptor
(CCCD) has been changed by the GATT client, for each of the characteristics*/
static void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    switch (value)
    {
    case BT_GATT_CCC_NOTIFY:
        break;
    case 0:
        break;
    default:
        LOG_ERR("Error, CCCD has been set to an invalid value");
    }
}

static ssize_t on_receive_shphld_wr(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                    uint16_t len, uint16_t offset, uint8_t flags)
{
    static bool request;

    LOG_INF("Received lsldo wr data, handle %d, conn %p, len %d, data: 0x", attr->handle, conn, len);
    LOG_INF("REQUESTED SHIP MODE: %d", request);
    bt_conn_disconnect(m_connection_handle, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    k_msgq_put(&ble_cfg_pmic_msgq, &request, K_NO_WAIT);

    return len;
}

BT_GATT_SERVICE_DEFINE(
    pmic_hub, BT_GATT_PRIMARY_SERVICE(BT_UUID_PMIC_HUB),
    BT_GATT_CHARACTERISTIC(BT_UUID_PMIC_HUB_RD_ALL, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_PMIC_HUB_SHPHLD_WR, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, on_receive_shphld_wr, NULL), );

static void adv_work_handler(struct k_work *work)
{
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err)
    {
        LOG_INF("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising successfully started");
}

static void advertising_start(void)
{
    k_work_submit(&adv_work);
}

static void recycled_cb(void)
{
    LOG_INF("Connection object available from previous conn. Disconnect complete.");
    advertising_start();
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err)
    {
        LOG_WRN("Connection failed (err %u)", err);
        return;
    }
    m_connection_handle = bt_conn_ref(conn);

    LOG_INF("Connected");

    struct bt_conn_info info;
    err = bt_conn_get_info(m_connection_handle, &info);
    if (err)
    {
        LOG_ERR("bt_conn_get_info() returned %d", err);
        return;
    }

    gpio_pin_set_dt(&bt_status_led, 1);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);
    bt_conn_unref(m_connection_handle);
    m_connection_handle = NULL;
    gpio_pin_set_dt(&bt_status_led, 0);
}

struct bt_conn_cb connection_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled_cb,
};

static void ble_report_batt_volt(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    const struct bt_gatt_attr *attr = &pmic_hub.attrs[2];
    struct bt_gatt_notify_params params = {
        .uuid = BT_UUID_PMIC_HUB_RD_ALL, .attr = attr, .data = data, .len = len, .func = NULL};

    if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY))
    {
        if (bt_gatt_notify_cb(conn, &params))
        {
            LOG_ERR("Error, unable to send notification");
        }
    }
    else
    {
        LOG_WRN("Warning, notification not enabled for pmic stat characteristic");
    }
}

int bt_init(void)
{
    int err;

    // Setting up Bluetooth
    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return -1;
    }
    LOG_INF("Bluetooth initialized");
    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        settings_load();
    }
    bt_conn_cb_register(&connection_callbacks);
    k_work_init(&adv_work, adv_work_handler);
    advertising_start();

    return 0;
}

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

    // check if we are in shphld. if it's high, we are "awake", so engage fuel gauge and typical operation.
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
    struct pmic_report_msg pmic_ble_report;
    pmic_ble_report.batt_voltage = voltage;
    pmic_ble_report.temp = temp;
    pmic_ble_report.batt_soc = *soc;
    k_msgq_put(&pmic_msgq, &pmic_ble_report, K_FOREVER);

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
    int request;

    for (;;)
    {
        k_msgq_get(&ble_cfg_pmic_msgq, &request, K_FOREVER); // suspend till msg avail
        regulator_parent_ship_mode(pmic_regulators);
        k_msleep(2000);
    }
}

int main(void)
{
    for (;;)
    {
        k_msleep(500);
        LOG_INF("I am alive");
        k_msleep(2000);
    }
    return 0;
}

void ble_write_thread(void)
{
    int err;
    LOG_INF("ble write thread: entered");

    if (bt_init() != 0)
    {
        LOG_ERR("unable to initialize BLE!");
    }

    err = gpio_pin_configure_dt(&bt_status_led, GPIO_OUTPUT_INACTIVE);
    if (err)
    {
        LOG_WRN("Configuring BT status LED failed (err %d)", err);
    }

    struct pmic_report_msg pmic_msg;
    for (;;)
    {
        // Wait indefinitely for msg's from other modules
        k_msgq_get(&pmic_msgq, &pmic_msg, K_FOREVER);
        LOG_INF("BLE thread rx from PMIC: V: %.2f T: %.2f SoC: %d ", pmic_msg.batt_voltage, pmic_msg.temp,
                pmic_msg.batt_soc);

        if (m_connection_handle) // if ble connection present
        {
            bt_bas_set_battery_level(pmic_msg.batt_soc); // report batt soc via standard service

            static uint8_t ble_batt_volt[MAXLEN]; // report batt volt string via custom service
            int len = snprintf(ble_batt_volt, MAXLEN, "BATT: %.2f V", pmic_msg.batt_voltage);
            if (!(len >= 0 && len < MAXLEN))
            {
                LOG_ERR("ble pmic report too large. (%d)", len);
            }
            else
            {
                ble_report_batt_volt(m_connection_handle, ble_batt_volt, len);
            }
        }
        else
        {
            LOG_INF("BLE Thread does not detect an active BLE connection");

            // toggle the LED while advertising
            gpio_pin_toggle_dt(&bt_status_led);
        }

        k_sleep(BLE_NOTIFY_INTERVAL);
    }
}

K_THREAD_DEFINE(pmic_reg_thread_id, PMIC_THREAD_STACK_SIZE, pmic_reg_thread, NULL, NULL, NULL, PMIC_THREAD_PRIORITY, 0,
                1000);
K_THREAD_DEFINE(pmic_fg_thread_id, PMIC_THREAD_STACK_SIZE, pmic_fg_thread, NULL, NULL, NULL, PMIC_THREAD_PRIORITY, 0,
                0);
K_THREAD_DEFINE(ble_write_thread_id, BLE_THREAD_STACK_SIZE, ble_write_thread, NULL, NULL, NULL, BLE_THREAD_PRIORITY, 0,
                0);
