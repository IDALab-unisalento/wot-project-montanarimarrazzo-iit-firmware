/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */

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

// #define min(x,y)  (((x)>=(y))?(x):(y))

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
extern uint8_t *get_data_adc(void);

const struct device *adc_dev;
u16_t app_max_sample_count;

#define SAMPLES_IN_BUFFER 20    // numbers of sample to read
#define NUM_CHANNEL_ADC 2

// --- buffer contained result of read adc
// first element contain time reference in millisecond for the read
// second and third element contain result of two input channel 
#define ELEM_BUFF_SAMPLES (NUM_CHANNEL_ADC+1)*SAMPLES_IN_BUFFER
//uint16_t buff_samples[ELEM_BUFF_SAMPLES];
int count_s = 0;

// ----- prepare buffer to send data on ble
uint8_t header[3];
uint16_t buffer_data[ELEM_BUFF_SAMPLES];

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

//#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
//#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
//#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);
static K_SEM_DEFINE(ble_send_ok, 0, 1);

static K_SEM_DEFINE(adc_init_ok, 0, 1);
static K_SEM_DEFINE(adc_read_ok, 0, 1);
static K_SEM_DEFINE(adc_data_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

//static const struct device *uart;
//static struct k_delayed_work uart_work;

//struct uart_data_t {
//	void *fifo_reserved;
//	uint8_t data[UART_BUF_SIZE];
//	uint16_t len;
//};

// dummy data buffer
//#define DUMMY_ITEMS    8
//#define DUMMY_RECORDS  1  // 1024
//#define DUMMY_COUNT    (DUMMY_ITEMS*DUMMY_RECORDS)

//uint16_t g_dummy[DUMMY_COUNT];

//static K_FIFO_DEFINE(fifo_uart_tx_data);
//static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

//static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
//{
//	ARG_UNUSED(dev);

//	static uint8_t *current_buf;
//	static size_t aborted_len;
//	static bool buf_release;
//	struct uart_data_t *buf;
//	static uint8_t *aborted_buf;

//	switch (evt->type) {
//	case UART_TX_DONE:
//		if ((evt->data.tx.len == 0) ||
//		    (!evt->data.tx.buf)) {
//			return;
//		}

//		if (aborted_buf) {
//			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
//					   data);
//			aborted_buf = NULL;
//			aborted_len = 0;
//		} else {
//			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
//					   data);
//		}

//		k_free(buf);

//		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
//		if (!buf) {
//			return;
//		}

//		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
//			LOG_WRN("Failed to send data over UART");
//		}

//		break;

//	case UART_RX_RDY:
//		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
//		buf->len += evt->data.rx.len;
//		buf_release = false;

//		if (buf->len == UART_BUF_SIZE) {
//			k_fifo_put(&fifo_uart_rx_data, buf);
//		} else if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
//			  (evt->data.rx.buf[buf->len - 1] == '\r')) {
//			k_fifo_put(&fifo_uart_rx_data, buf);
//			current_buf = evt->data.rx.buf;
//			buf_release = true;
//			uart_rx_disable(uart);
//		}

//		break;

//	case UART_RX_DISABLED:
//		buf = k_malloc(sizeof(*buf));
//		if (buf) {
//			buf->len = 0;
//		} else {
//			LOG_WRN("Not able to allocate UART receive buffer");
//			k_delayed_work_submit(&uart_work,
//					      UART_WAIT_FOR_BUF_DELAY);
//			return;
//		}

//		uart_rx_enable(uart, buf->data, sizeof(buf->data),
//			       UART_WAIT_FOR_RX);

//		break;

//	case UART_RX_BUF_REQUEST:
//		buf = k_malloc(sizeof(*buf));
//		if (buf) {
//			buf->len = 0;
//			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
//		} else {
//			LOG_WRN("Not able to allocate UART receive buffer");
//		}

//		break;

//	case UART_RX_BUF_RELEASED:
//		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
//				   data);
//		if (buf_release && (current_buf != evt->data.rx_buf.buf)) {
//			k_free(buf);
//			buf_release = false;
//			current_buf = NULL;
//		}

//		break;

//	case UART_TX_ABORTED:
//			if (!aborted_buf) {
//				aborted_buf = (uint8_t *)evt->data.tx.buf;
//			}

//			aborted_len += evt->data.tx.len;
//			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
//					   data);

//			uart_tx(uart, &buf->data[aborted_len],
//				buf->len - aborted_len, SYS_FOREVER_MS);

//		break;

//	default:
//		break;
//	}
//}

//static void uart_work_handler(struct k_work *item)
//{
//	struct uart_data_t *buf;

//	buf = k_malloc(sizeof(*buf));
//	if (buf) {
//		buf->len = 0;
//	} else {
//		LOG_WRN("Not able to allocate UART receive buffer");
//		k_delayed_work_submit(&uart_work, UART_WAIT_FOR_BUF_DELAY);
//		return;
//	}

//	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
//}

//static int uart_init(void)
//{
//	int err;
//	struct uart_data_t *rx;

//	uart = device_get_binding(DT_LABEL(DT_NODELABEL(uart0)));
//	if (!uart) {
//		return -ENXIO;
//	}

//	rx = k_malloc(sizeof(*rx));
//	if (rx) {
//		rx->len = 0;
//	} else {
//		return -ENOMEM;
//	}

//	k_delayed_work_init(&uart_work, uart_work_handler);

//	err = uart_callback_set(uart, uart_cb, NULL);
//	if (err) {
//		return err;
//	}

//	return uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
//}

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

//	LOG_INF("Received %d bytes of data from: %s", len, log_strdup(addr));

	//for (uint16_t pos = 0; pos != len;) {
	//	struct uart_data_t *tx = k_malloc(sizeof(*tx));

	//	if (!tx) {
	//		LOG_WRN("Not able to allocate UART send data buffer");
	//		return;
	//	}

	//	/* Keep the last byte of TX buffer for potential LF char. */
	//	size_t tx_data_size = sizeof(tx->data) - 1;

	//	if ((len - pos) > tx_data_size) {
	//		tx->len = tx_data_size;
	//	} else {
	//		tx->len = (len - pos);
	//	}

	//	memcpy(tx->data, &data[pos], tx->len);

	//	pos += tx->len;

	//	/* Append the LF character when the CR character triggered
	//	 * transmission from the peer.
	//	 */
	//	if ((pos == len) && (data[len - 1] == '\r')) {
	//		tx->data[tx->len] = '\n';
	//		tx->len++;
	//	}

	//	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	//	if (err) {
	//		k_fifo_put(&fifo_uart_tx_data, tx);
	//	}
	//}

        
        if (!strncmp(data,"Stop",4))
        {
                app_max_sample_count = 0;
        	k_sem_reset(&ble_send_ok);
        }
        else

        if (!strncmp(data,"Start",5))
        {
                app_max_sample_count = SAMPLES_IN_BUFFER;
        	k_sem_give(&ble_send_ok);
        }
}

//static void bt_sent_cb(struct bt_conn *conn)
//{
//	LOG_INF("... data sent");
//}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
//      .sent = bt_sent_cb,
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
    // powering off by lowering power control line
//  (*NRF_P0).OUTCLR = BIT(4);
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

/*
    while (1)
    {

       // sample all adc channels - return data count or 0 if error
        int count = adc_sample(adc_dev);
        if (count<0)
        {
            LOG_ERR("Fail to read adc\n");
            powerctrl_off();
            blink_err(2);
            return;
        }
    
        u16_t *pdata16 = adc_get_millivolts();

        // fill the dummy array
        for (uint16_t i=0; i<min(DUMMY_COUNT, ADC_BUFF_SIZE); i++)
            g_dummy[i] = pdata16[i];
       
       k_sleep(K_MSEC(100));

       uint8_t buf[64];
       snprintf(buf,64,"%d %d %d %d\n",g_dummy[0],g_dummy[1],g_dummy[2],g_dummy[3]);

       printk("ADC: %s\n",buf);
    }

    while(1);
*/
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

  /*
    // clear the dummy array
    for (uint16_t i=0; i<DUMMY_COUNT; i++)
        g_dummy[i]=0;;
  */
  
    k_sem_give(&adc_init_ok);

    header[0] = 2;     // num adc channels input
    header[1] = 40;    // interval acquisition time adc in microsends
    header[2] = SAMPLES_IN_BUFFER;     //num of samples
 
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

	/* Wait indefinitely for data to be sent over bluetooth */
	//struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
	//				     K_FOREVER);

	//if (bt_nus_send(NULL, buf->data, buf->len)) {
	//  LOG_WRN("Failed to send data over BLE connection");
	//}

	//k_free(buf);

        //snprintf(buf,64,"%d %d %d %d\n",g_dummy[0],g_dummy[1],g_dummy[2],g_dummy[3]);
        //size_t len = strlen(buf);
        
        /*
         if (bt_nus_send(NULL, buffer_data, ARRAY_SIZE(buffer_data)))
                  LOG_WRN("Failed to send data over BLE connection");
        */

        // ----- send info on adc and data
        if (bt_nus_send(NULL, header, 3))
                LOG_WRN("Failed to send data over BLE connection");

        //   ---- send data on samples
         int8_t buf[64];
         for(int i=0; i<ELEM_BUFF_SAMPLES; i=i+3)
          {
              snprintf(buf,64,"%d %d %d\n",buffer_data[i],buffer_data[i+1],buffer_data[i+2]);
              size_t len = strlen(buf);
              if (bt_nus_send(NULL, buf, len))
                    { LOG_WRN("Failed to send data over BLE connection"); }
          }
        

        //LOG_INF("Data sent: %d %d %d %d",g_dummy[0],g_dummy[1],g_dummy[2],g_dummy[3]);

        //if (bt_nus_send(NULL, buf, len))
        //   LOG_WRN("Failed to send data over BLE connection");
        

        /* Reset semaphore */
        k_sem_reset(&adc_data_ok);

        /* ready for new sampling */
        k_sem_give(&adc_read_ok);

        /* Start new sampling*/
        //app_max_sample_count--;
        //if (app_max_sample_count > 0)
        //    k_sem_give(&adc_read_ok);
        
        k_sleep(K_MSEC(1));
    }
}

void ble_adc_read(void)
{
    /* Don't go any further until ADC is initialized */
    k_sem_take(&adc_init_ok, K_FOREVER);
    
    for (;;) { 
          /* Wait until semaphore triggered */
         // k_sem_take(&ble_send_ok, K_FOREVER);
    
          /* Wait until semaphore triggered */
          k_sem_take(&adc_read_ok, K_FOREVER);
            
          uint16_t *pdata16;
          time_stamp = k_uptime_get();

          k_sleep(K_MSEC(500));

          // sample all adc channels - return data count or 0 if error
          int count = adc_sample(adc_dev);
          if (count<0)
          {
                LOG_ERR("Fail to read adc\n");
                powerctrl_off();
                blink_err(2);
                return;
          }
          
          pdata16 = adc_get_millivolts();
          milliseconds_spent = k_uptime_delta(&time_stamp);

          if (count_s == 0) { buffer_data[count_s] = milliseconds_spent; } 
          else { buffer_data[count_s] = buffer_data[count_s-3] + milliseconds_spent; }
        
          buffer_data[count_s+1] = pdata16[0];
          buffer_data[count_s+2] = pdata16[1];

          count_s = count_s + 3;

          /*
          // fill the dummy array
          for (uint16_t i=0; i<min(DUMMY_COUNT, ADC_BUFF_SIZE); i++)
                   g_dummy[i] = pdata16[i];
          */
          
         app_max_sample_count = app_max_sample_count - 1;

         if(app_max_sample_count>0)
          {
                k_sem_give(&adc_read_ok);
          } else { 
                  k_sem_give(&ble_send_ok);
                  k_sem_give(&adc_data_ok);
                  k_sem_reset(&adc_read_ok);
                  count_s = 0;
          }

//        LOG_INF("Data sampled");

          /* Reset semaphore */
         //k_sem_reset(&adc_read_ok);

         /* data ready */
         //k_sem_give(&adc_data_ok);

         k_sleep(K_MSEC(1));
     } 
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

K_THREAD_DEFINE(ble_adc_read_id, STACKSIZE, ble_adc_read, NULL, NULL,
		NULL, PRIORITY, 0, 0);
