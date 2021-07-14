/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
 
 
//------------------------------------------------------------------
 /*
 * Project IoT 2021 with IIT - Erika Marrazzo
 */
//------------------------------------------------------------------

#include <zephyr.h>
#include <zephyr/types.h>

#include <soc.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>

#include <bluetooth/services/nus.h>
#include <dk_buttons_and_leds.h>

#include <settings/settings.h>

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/util.h>

#include "gpio_map.h"

#include <logging/log.h>

// uncomment this line to enable leds for debug
// #define ENABLE_LEDS

#ifdef USE_TP0
const struct device *dev_tp0;
#endif

#ifdef USE_PWRCTRL0
const struct device *dev_pwrctrl0;
#endif

#ifdef ENABLE_LEDS

#ifdef USE_LED0
struct device *dev_led0;
#endif

#ifdef USE_LED1
struct device *dev_led1;
#endif

#ifdef USE_LED2
struct device *dev_led2;
#endif

#endif // ENABLE_LEDS

extern struct device *adc_init(void);
extern int adc_sample(const struct device *adc_dev);
extern s16_t *adc_get_millivolts(void);
const struct device *adc_dev;

// structure that store adc info and sequence of samples
struct payload{
    uint16_t header[4];
    s16_t sample_buff[ELEM_BUFF_SAMPLES];
}payload;

// var to take time of sample
s64_t time_stamp;
s64_t milliseconds_spent;  

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

static K_SEM_DEFINE(ble_init_ok, 0, 1);
static K_SEM_DEFINE(ble_send_ok, 0, 1);

static K_SEM_DEFINE(adc_init_ok, 0, 1);
static K_SEM_DEFINE(adc_read_ok, 0, 1);
static K_SEM_DEFINE(adc_data_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", log_strdup(addr));

	current_conn = bt_conn_ref(conn);

	dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", log_strdup(addr), reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		dk_set_led_off(CON_STATUS_LED);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", log_strdup(addr),
			level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", log_strdup(addr),
			level, err);
	}
}
#endif

static struct bt_conn_cb conn_callbacks = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", log_strdup(addr), passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", log_strdup(addr), passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}


static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", log_strdup(addr));
}


static void pairing_confirm(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	bt_conn_auth_pairing_confirm(conn);

	LOG_INF("Pairing confirmed: %s", log_strdup(addr));
}


static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", log_strdup(addr),
		bonded);
}


static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", log_strdup(addr),
		reason);
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
	.pairing_confirm = pairing_confirm,
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	//int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));
        
        if (!strncmp(data,"Start",5))
        {
        	k_sem_give(&ble_send_ok);
        }
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}

#ifdef ENABLE_LEDS

void blink_init(void)
{
#ifdef USE_LED0
    dev_led0 = device_get_binding(LED0_LAB);
    if (dev_led0)
        if (gpio_pin_configure(dev_led0, LED0_PIN, GPIO_OUTPUT_INACTIVE | LED0_FLG))
          dev_led0 = NULL;
#endif

#ifdef USE_LED1
    dev_led1 = device_get_binding(LED1_LAB);
    if (dev_led1)
        if (gpio_pin_configure(dev_led1, LED1_PIN, GPIO_OUTPUT_INACTIVE | LED1_FLG))
          dev_led1 = NULL;
#endif

#ifdef USE_LED2
    dev_led2 = device_get_binding(LED2_LAB);
    if (dev_led2)
        if (gpio_pin_configure(dev_led2, LED2_PIN, GPIO_OUTPUT_INACTIVE | LED2_FLG))
          dev_led2 = NULL;
#endif
}

void blink_ble(int n)
{
#ifdef USE_LED0
    if (dev_led0)
    {
        for (int cnt=0;cnt<2*n;cnt++)
        {
            gpio_pin_set(dev_led0, LED0_PIN, !(cnt&1));
            if (cnt<2*n-1)
                k_msleep(500);
        }
    }
#endif
}

void blink_act(int n)
{
#ifdef USE_LED1
    if (dev_led1)
    {
        for (int cnt=0;cnt<2*n;cnt++)
        {
            gpio_pin_set(dev_led1, LED1_PIN, !(cnt&1));
            if (cnt<2*n-1)
                k_msleep(500);
        }
    }
#endif
}

void blink_err(int n)
{
#ifdef USE_LED2
    if (dev_led2)
    {
        for (int cnt=0;cnt<2*n;cnt++)
        {
            gpio_pin_set(dev_led2, LED2_PIN, !(cnt&1));
            if (cnt<2*n-1)
                k_msleep(500);
        }
    }
#endif
}

#else // ENABLE_LEDS

#define blink_init()
#define blink_ble(n)
#define blink_act(n)
#define blink_err(n)

#endif // ENABLE LEDD

static void configure_gpio(void)
{
    int err;

    err = dk_buttons_init(button_changed);
    if (err) {
        LOG_ERR("Cannot init buttons (err: %d)", err);
    }

    err = dk_leds_init();
    if (err) {
        LOG_ERR("Cannot init LEDs (err: %d)", err);
    }
}

// disable power to SoC by lowering power ctrl pin
void powerctrl_off(void)
{

}


void main(void)
{
#ifdef USE_TP0
    dev_tp0 = device_get_binding(TP0_LAB);
    if (dev_tp0)
        if (gpio_pin_configure(dev_tp0, TP0_PIN, GPIO_OUTPUT_INACTIVE | TP0_FLG) < 0) 
            dev_tp0 = NULL;
#endif

#ifdef USE_PWRCTRL0
    dev_pwrctrl0 = device_get_binding(PWRCTRL0_LAB);
    if (dev_pwrctrl0)
        if (gpio_pin_configure(dev_pwrctrl0, PWRCTRL0_PIN, GPIO_OUTPUT_INACTIVE | PWRCTRL0_FLG) < 0) 
            dev_pwrctrl0 = NULL;
#endif

    blink_init();
    blink_act(1);

    int blink_status = 0;
    int err = 0;

    configure_gpio();

    // initialize adc device
    adc_dev = adc_init();
    if (!adc_dev)
    {
        LOG_ERR("Fail to initialize adc\n");
        powerctrl_off();
        blink_err(1);
        return;
    }

#ifdef USE_TP0
    // testpoint high
    if (dev_tp0)
        gpio_pin_set(dev_tp0, TP0_PIN, 1);
#endif

#ifdef USE_PWRCTRL0
    // enable control power
    if (dev_pwrctrl0)
        gpio_pin_set(dev_pwrctrl0, PWRCTRL0_PIN, 1);
#endif

 
    time_stamp = k_uptime_get();
    milliseconds_spent = k_uptime_delta(&time_stamp);
  
    k_sem_give(&adc_init_ok);

    payload.header[0] = NUM_CHANNEL_ADC;     // num adc channels input
    payload.header[1] = 40;    // interval acquisition time adc in microsends
    payload.header[2] = SAMPLES_IN_BUFFER;     // num of samples
    payload.header[3] = milliseconds_spent;		// reference time

#ifdef USE_PWRCTRL0
    // disable contrl power
    if (dev_pwrctrl0)
        gpio_pin_set(dev_pwrctrl0, PWRCTRL0_PIN, 0);
#endif

#ifdef USE_TP0
    // testpoint low
    if (dev_tp0)
        gpio_pin_set(dev_tp0, TP0_PIN, 0);
#endif

    bt_conn_cb_register(&conn_callbacks);

    if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
	bt_conn_auth_cb_register(&conn_auth_callbacks);
    }

    err = bt_enable(NULL);
    if (err) {
        error();
    }

    LOG_INF("Bluetooth initialized");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    err = bt_nus_init(&nus_cb);
    if (err) {
        LOG_ERR("Failed to initialize UART service (err: %d)", err);
            return;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
    }

    printk("Starting Nordic UART service example\n");

    k_sem_give(&ble_init_ok);

    k_sem_give(&adc_read_ok);

    for (;;) {
	dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
	k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
    }
}

void ble_write_thread(void)
{
    /* Don't go any further until BLE is initialized */
    k_sem_take(&ble_init_ok, K_FOREVER);

    for (;;) {
        /* Wait until semaphore triggered */
        k_sem_take(&ble_send_ok, K_FOREVER);

        /* Wait until data ready */
        k_sem_take(&adc_data_ok, K_FOREVER);
        
        // ----- send info on adc and data
        if (bt_nus_send(NULL, (void *)&payload, sizeof(payload)))
                LOG_WRN("Failed to send data over BLE connection");

        LOG_INF("%d %d", maxsz, sizeof(payload));

        /* Reset semaphore */
        k_sem_reset(&adc_data_ok);

        /* ready for new sampling */
        k_sem_give(&adc_read_ok);


        k_sleep(K_MSEC(1));
    }
}

void ble_adc_read(void)
{
    /* Don't go any further until ADC is initialized */
    k_sem_take(&adc_init_ok, K_FOREVER);
    
    for (;;) { 
        
          /* Wait until semaphore triggered */
          k_sem_take(&adc_read_ok, K_FOREVER);
        
          /*Acquisition samples from adc */
          int count = adc_sample(adc_dev);
          if (count<0)
          {
                LOG_ERR("Fail to read adc\n");
                powerctrl_off();
                blink_err(2);
                return;
          }

          /* Reset semaphore */
         k_sem_reset(&adc_read_ok);

         /* data ready */
         k_sem_give(&adc_data_ok);

         k_sleep(K_MSEC(1));
     } 
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(ble_adc_read_id, STACKSIZE, ble_adc_read, NULL, NULL,
		NULL, PRIORITY, 0, 0);
