#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "app_scheduler.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "nrf_drv_gpiote.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <stdio.h>
#include "nrf_drv_rng.h"
#include "nrf_assert.h"

#define LED_1_PIN                       BSP_LED_0     // LED 1 on the nRF51-DK or nRF52-DK
#define LED_2_PIN                       BSP_LED_1     // LED 3 on the nRF51-DK or nRF52-DK
#define LED_3_PIN                       BSP_LED_2     // LED 3 on the nRF51-DK or nRF52-DK
#define BUTTON_1_PIN                    BSP_BUTTON_0  // Button 1 on the nRF51-DK or nRF52-DK
#define BUTTON_2_PIN                    BSP_BUTTON_1  // Button 2 on the nRF51-DK or nRF52-DK
#define BUTTON_3_PIN                    BSP_BUTTON_2  // Button 1 on the nRF51-DK or nRF52-DK
#define BUTTON_4_PIN                    BSP_BUTTON_3  // Button 2 on the nRF51-DK or nRF52-DK

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, 0)
#define SCHED_QUEUE_SIZE                10

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "Nordic_Blinky"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF 
#define ST7586_SPI_INSTANCE  0/**< SPI instance index. */
static const nrf_drv_spi_t st7586_spi = NRF_DRV_SPI_INSTANCE(ST7586_SPI_INSTANCE);  /**< SPI instance. */
static volatile bool st7586_spi_xfer_done = false;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define ST_COMMAND   0
#define ST_DATA      1

#define RATIO_SPI0_LCD_SCK          4
#define RATIO_SPI0_LCD_A0          28
#define RATIO_SPI0_LCD_MOSI          29
#define RATIO_SPI0_LCD_BSTB          30
#define RATIO_SPI0_LCD_CS         31

#define LCD_INIT_DELAY(t) nrf_delay_ms(t)/**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

int adventure_temp = 5;
int state_temp = 0;
int enemy_num = 0;
int state = 0;
int enemy_temp = 160;
bool hit = false;
int hp = 3;
int attack_temp = 50;
int lv = 1;
int full = 3;
int tired = 0;
int experience = 0;
int select = 0;
bool adventure_ing = false;

void select_state();
void clear();
void jump();
void attack();
void bow();
void arrow();
void attack_go();
void white(int i, int j);
void heart();
void enemy();
void w_b_w();
void b_w_b();
void display_hp();
void gameover();

BLE_LBS_DEF(m_lbs);                                                             /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static unsigned char rx_data;

APP_TIMER_DEF(m_led_a_timer_id);

uint8_t i, j;

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
	void *                    p_context)
{
	st7586_spi_xfer_done = true;
}

void st7586_write(const uint8_t category, const uint8_t data){
	int err_code;
	nrf_gpio_pin_write(RATIO_SPI0_LCD_A0, category);

	st7586_spi_xfer_done = false;
	err_code = nrf_drv_spi_transfer(&st7586_spi, &data, 1, &rx_data, 0);
	APP_ERROR_CHECK(err_code);
	while (!st7586_spi_xfer_done) {
		__WFE();
	}
	nrf_delay_us(10);
}

static inline void st7586_pinout_setup(){
	int err_code;
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin = RATIO_SPI0_LCD_CS;
	spi_config.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
	spi_config.mosi_pin = RATIO_SPI0_LCD_MOSI;
	spi_config.sck_pin = RATIO_SPI0_LCD_SCK;
	spi_config.frequency = NRF_SPI_FREQ_1M;
	spi_config.mode = NRF_DRV_SPI_MODE_3;

	err_code = nrf_drv_spi_init(&st7586_spi, &spi_config, spi_event_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_gpio_pin_set(RATIO_SPI0_LCD_A0);
	nrf_gpio_cfg_output(RATIO_SPI0_LCD_A0);

	nrf_gpio_pin_clear(RATIO_SPI0_LCD_BSTB);
	nrf_gpio_cfg_output(RATIO_SPI0_LCD_BSTB);
}

bool main_context(void){
	static const uint8_t ISR_NUMBER_THREAD_MODE = 0;
	uint8_t isr_number = __get_IPSR();
	if ((isr_number) == ISR_NUMBER_THREAD_MODE)
	{
		return true;
	}
	else
	{
		return false;
	}
}

static void timer_handler(void * p_context){
	nrf_drv_gpiote_out_toggle(LED_1_PIN);
	if (main_context())
	{
		nrf_drv_gpiote_out_clear(LED_3_PIN);
		NRF_LOG_INFO("in main context...\r\n");
	}
	else
	{
		nrf_drv_gpiote_out_set(LED_3_PIN);
		NRF_LOG_INFO("in interrupt context...\r\n");
	}
}

// Create timers
static void create_timers(){
	uint32_t err_code;

	// Create timers
	err_code = app_timer_create(&m_led_a_timer_id,
		APP_TIMER_MODE_REPEATED,
		timer_handler);
	APP_ERROR_CHECK(err_code);
}

void button_handler(nrf_drv_gpiote_pin_t pin){
	uint32_t err_code;
	int32_t TICKS = APP_TIMER_TICKS(200);

	switch (pin)
	{
	case BUTTON_1_PIN:
		adventure_temp = 1;
		if(select == 0)
			select = 3;
		else
			select--;
		err_code = app_timer_start(m_led_a_timer_id, TICKS, NULL);
		APP_ERROR_CHECK(err_code);
		break;

	case BUTTON_2_PIN:
		adventure_temp = 2;
		select++;
		err_code = app_timer_start(m_led_a_timer_id, TICKS, NULL);
		APP_ERROR_CHECK(err_code);
		break;

	case BUTTON_3_PIN:
		state_temp = select+1;
	  if(!adventure_ing){
			adventure_temp = 3;
		}
		err_code = app_timer_start(m_led_a_timer_id, TICKS, NULL);
		APP_ERROR_CHECK(err_code);
		break;

	case BUTTON_4_PIN:
		err_code = app_timer_start(m_led_a_timer_id, TICKS, NULL);
		APP_ERROR_CHECK(err_code);
		break;

	default:
		break;
	}

}

void button_scheduler_event_handler(void *p_event_data, uint16_t event_size){
	 button_handler(*((nrf_drv_gpiote_pin_t*)p_event_data));
}

void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
	app_sched_event_put(&pin, sizeof(pin), button_scheduler_event_handler);
}

static void gpio_config(){
	ret_code_t err_code;

	// Initialze driver.
	err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);

	// Configure 3 output pins for LED's.
	nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
	err_code = nrf_drv_gpiote_out_init(LED_1_PIN, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(LED_2_PIN, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(LED_3_PIN, &out_config);
	APP_ERROR_CHECK(err_code);

	// Set output pins (this will turn off the LED's).
	nrf_drv_gpiote_out_set(LED_1_PIN);
	nrf_drv_gpiote_out_set(LED_2_PIN);
	nrf_drv_gpiote_out_set(LED_3_PIN);

	// Make a configuration for input pints. This is suitable for both pins in this example.
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	in_config.pull = NRF_GPIO_PIN_PULLUP;

	// Configure input pins for buttons, with separate event handlers for each button.
	err_code = nrf_drv_gpiote_in_init(BUTTON_1_PIN, &in_config, gpiote_event_handler);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_in_init(BUTTON_2_PIN, &in_config, gpiote_event_handler);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_in_init(BUTTON_3_PIN, &in_config, gpiote_event_handler);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_in_init(BUTTON_4_PIN, &in_config, gpiote_event_handler);
	APP_ERROR_CHECK(err_code);

	// Enable input pins for buttons.
	nrf_drv_gpiote_in_event_enable(BUTTON_1_PIN, true);
	nrf_drv_gpiote_in_event_enable(BUTTON_2_PIN, true);
	nrf_drv_gpiote_in_event_enable(BUTTON_3_PIN, true);
	nrf_drv_gpiote_in_event_enable(BUTTON_4_PIN, true);
}

static void lfclk_request(void){
	uint32_t err_code = nrf_drv_clock_init();
	APP_ERROR_CHECK(err_code);
	nrf_drv_clock_lfclk_request(NULL);
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name){
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void leds_init(void){
	bsp_board_leds_init();
}

static void gap_params_init(void){
	ret_code_t              err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
		(const uint8_t *)DEVICE_NAME,
		strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

static void gatt_init(void){
	ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
	APP_ERROR_CHECK(err_code);
}

static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state){
	if (led_state)
	{
		bsp_board_led_on(LEDBUTTON_LED);
		NRF_LOG_INFO("Received LED ON!");
	}
	else
	{
		bsp_board_led_off(LEDBUTTON_LED);
		NRF_LOG_INFO("Received LED OFF!");
	}
}

static void services_init(void){
	ret_code_t     err_code;
	ble_lbs_init_t init;

	init.led_write_handler = led_write_handler;

	err_code = ble_lbs_init(&m_lbs, &init);
	APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt){
	ret_code_t err_code;

	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
	{
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}

static void conn_params_error_handler(uint32_t nrf_error){
	APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void){
	ret_code_t             err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail = false;
	cp_init.evt_handler = on_conn_params_evt;
	cp_init.error_handler = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}

static void advertising_start(void){
	ret_code_t           err_code;
	ble_gap_adv_params_t adv_params;

	// Start advertising
	memset(&adv_params, 0, sizeof(adv_params));

	adv_params.type = BLE_GAP_ADV_TYPE_ADV_IND;
	adv_params.p_peer_addr = NULL;
	adv_params.fp = BLE_GAP_ADV_FP_ANY;
	adv_params.interval = APP_ADV_INTERVAL;
	adv_params.timeout = APP_ADV_TIMEOUT_IN_SECONDS;

	err_code = sd_ble_gap_adv_start(&adv_params, APP_BLE_CONN_CFG_TAG);
	APP_ERROR_CHECK(err_code);
	bsp_board_led_on(ADVERTISING_LED);
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context){
	ret_code_t err_code;

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		NRF_LOG_INFO("Connected");
		bsp_board_led_on(CONNECTED_LED);
		bsp_board_led_off(ADVERTISING_LED);
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

		err_code = app_button_enable();
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		NRF_LOG_INFO("Disconnected");
		bsp_board_led_off(CONNECTED_LED);
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		err_code = app_button_disable();
		APP_ERROR_CHECK(err_code);
//		advertising_start();
		break;

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		// Pairing not supported
		err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
			BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
			NULL,
			NULL);
		APP_ERROR_CHECK(err_code);
		break;

#ifndef S140
	case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
	{
		NRF_LOG_DEBUG("PHY update request.");
		ble_gap_phys_t const phys =
		{
			.rx_phys = BLE_GAP_PHY_AUTO,
			.tx_phys = BLE_GAP_PHY_AUTO,
		};
		err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
		APP_ERROR_CHECK(err_code);
	} break;
#endif

	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		// No system attributes have been stored.
		err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTC_EVT_TIMEOUT:
		// Disconnect on GATT Client timeout event.
		NRF_LOG_DEBUG("GATT Client Timeout.");
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
			BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_TIMEOUT:
		// Disconnect on GATT Server timeout event.
		NRF_LOG_DEBUG("GATT Server Timeout.");
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
			BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_EVT_USER_MEM_REQUEST:
		err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
	{
		ble_gatts_evt_rw_authorize_request_t  req;
		ble_gatts_rw_authorize_reply_params_t auth_reply;

		req = p_ble_evt->evt.gatts_evt.params.authorize_request;

		if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
		{
			if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ) ||
				(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
				(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
			{
				if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
				{
					auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
				}
				else
				{
					auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
				}
				auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
				err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
					&auth_reply);
				APP_ERROR_CHECK(err_code);
			}
		}
	} break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

	default:
		// No implementation needed.
		break;
	}
}

static void ble_stack_init(void){
	ret_code_t err_code;

	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void lcd_init(void) {
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();

	st7586_pinout_setup();

	NRF_LOG_INFO("SPI example.");
	nrf_gpio_pin_write(RATIO_SPI0_LCD_BSTB, 0);
	LCD_INIT_DELAY(10);
	nrf_gpio_pin_write(RATIO_SPI0_LCD_BSTB, 1);

	LCD_INIT_DELAY(120);

	st7586_write(ST_COMMAND, 0xD7);  // Disable Auto Read
	st7586_write(ST_DATA, 0x9F);
	st7586_write(ST_COMMAND, 0xE0);  // Enable OTP Read
	st7586_write(ST_DATA, 0x00);
	LCD_INIT_DELAY(10);
	st7586_write(ST_COMMAND, 0xE3);  // OTP Up-Load
	LCD_INIT_DELAY(20);
	st7586_write(ST_COMMAND, 0xE1);  // OTP Control Out
	st7586_write(ST_COMMAND, 0x11);  // Sleep Out
	st7586_write(ST_COMMAND, 0x28);  // Display OFF
	LCD_INIT_DELAY(50);

	st7586_write(ST_COMMAND, 0xC0);
	st7586_write(ST_DATA, 0x53);
	st7586_write(ST_DATA, 0x01);
	st7586_write(ST_COMMAND, 0xC3);
	st7586_write(ST_DATA, 0x02);
	st7586_write(ST_COMMAND, 0xC4);
	st7586_write(ST_DATA, 0x06);

	st7586_write(ST_COMMAND, 0xD0);  // Enable Analog Circuit
	st7586_write(ST_DATA, 0x1D);
	st7586_write(ST_COMMAND, 0xB5);  // N-Line = 0
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_COMMAND, 0x39);  // Monochrome Mode
	st7586_write(ST_COMMAND, 0x3A);  // Enable DDRAM Interface
	st7586_write(ST_DATA, 0x02);
	st7586_write(ST_COMMAND, 0x36);  // Scan Direction Setting
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_COMMAND, 0xB0);  // Duty Setting
	st7586_write(ST_DATA, 0x9F);
	st7586_write(ST_COMMAND, 0x20);  // Display Inversion OFF
	st7586_write(ST_COMMAND, 0x2A);  // Column Address Setting
	st7586_write(ST_DATA, 0x00);  // SEG0 -> SEG384
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x7F);
	st7586_write(ST_COMMAND, 0x2B);  // Row Address Setting
	st7586_write(ST_DATA, 0x00);  // COM0 -> COM160
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x9F);
	st7586_write(ST_COMMAND, 0x29);  // Display ON
	LCD_INIT_DELAY(100);
	clear();
}

void start_point(int start_x, int start_y, int end_x, int end_y){
	//SEGGER_RTT_WriteString(0, "In address\r\n");
	st7586_write(ST_COMMAND, 0x2A);//set col
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, start_x);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, end_x);

	st7586_write(ST_COMMAND, 0x2B);//set row
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, start_y);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, end_y);
}

void clear(void){
	start_point(0, 0, 43, 160);
	st7586_write(ST_COMMAND, 0x2C);
	white(44, 160);
}

void white(int x, int y) {
	for (int i = 0; i < x; i++) {
		for (int j = 0; j < y; j++) {
			st7586_write(ST_DATA, 0x00);
		}
	}
}

void black(int x, int y) {
	for (int i = 0; i < x; i++) {
		for (int j = 0; j < y; j++) {
			st7586_write(ST_DATA, 0xff);
		}
	}
}

void b_w_b(int x, int y, int z) {
	for (int i = 0; i < x; i++) {
		st7586_write(ST_DATA, 0xff);
	}
	for (int i = 0; i < y; i++) {
		st7586_write(ST_DATA, 0x00);
	}
	for (int i = 0; i < z; i++) {
		st7586_write(ST_DATA, 0xff);
	}
}
void w_b_w(int x, int y, int z) {
	for (int i = 0; i < x; i++) {
		st7586_write(ST_DATA, 0x00);
	}
	for (int i = 0; i < y; i++) {
		st7586_write(ST_DATA, 0xff);
	}
	for (int i = 0; i < z; i++) {
		st7586_write(ST_DATA, 0x00);
	}
}

void display_head(){
	  start_point(0, 40, 20, 120);
   	st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 5, 4);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 5, 4);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 5, 4);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 5, 4);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   /////////////////////////////
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 4);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 5, 4);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 5, 4);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 5, 4);
   b_w_b(5, 3, 0);
      
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
      
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);

		for(int i=0;i<100;i++){
			select_state();
			LCD_INIT_DELAY(1);
      app_sched_execute();
		}
	  start_point(0, 40, 20, 120);
  st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 7, 2);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 7, 2);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 7, 2);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 7, 2);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   /////////////////////////////
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   w_b_w(4, 14, 3);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 2);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 7, 2);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 7, 2);
   b_w_b(5, 3, 0);
   
   w_b_w(4, 7, 2);
   b_w_b(5, 3, 0);
      
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
      
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);

		for(int i=0;i<100;i++){
			LCD_INIT_DELAY(1);
			select_state();
      app_sched_execute();
		}
}
void adventure_start(){
	start_point(0, 0, 43, 160);
	st7586_write(ST_COMMAND, 0x2C);
	black(44, 160);
	LCD_INIT_DELAY(500);
	start_point(0, 0, 43, 160);
	st7586_write(ST_COMMAND, 0x2C);
	white(44, 160);
}
void adventure() {
	state = 0;
		start_point(0, 0, 20, 50);
   st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(4, 5, 12);
   
   w_b_w(4, 5, 12);

   w_b_w(3, 7, 11);
   
   w_b_w(3, 7, 11);
   
   w_b_w(2, 9, 10);
   
   w_b_w(2, 9, 10);
   
   w_b_w(1, 11, 9);
   
   w_b_w(1, 11, 9);
   
   w_b_w(1, 11, 9);
   
   w_b_w(1, 11, 9);
   
   w_b_w(0, 13, 8);
   
   w_b_w(0, 13, 8);
     
   w_b_w(0, 13, 8);
   
   w_b_w(0, 13, 8);
   
   w_b_w(0, 13, 8);
         
   w_b_w(0, 13, 1);
   w_b_w(0, 2 ,5);
   
   w_b_w(0, 17, 4);
   
	 w_b_w(0, 17, 4);

	 w_b_w(0, 18, 3);

	 w_b_w(0, 18, 3);
   
   w_b_w(0, 18, 3);
   
	 w_b_w(0, 18, 3);
   
	 w_b_w(0, 18, 3);
   
   w_b_w(0, 18, 3);
   
   w_b_w(0, 18, 3);
   
   w_b_w(0, 18, 3);
   
   w_b_w(0, 18, 3);
   
   w_b_w(0, 17, 4);
   
   w_b_w(0, 17, 4);
   
   w_b_w(0, 13, 1);
   w_b_w(0, 2 ,5);
	 
   w_b_w(0, 7, 0);
	 w_b_w(3, 3, 8);
   
   w_b_w(0, 7, 0);
	 w_b_w(3, 3, 8);
   
   w_b_w(0, 7, 0);
	 w_b_w(3, 3, 8);
   
   w_b_w(0, 7, 0);
	 w_b_w(3, 3, 8);
   
   w_b_w(0, 7, 0);
	 w_b_w(3, 3, 8);
   
   w_b_w(0, 7, 0);
	 w_b_w(3, 3, 8);
      
   w_b_w(1, 6, 0);
	 w_b_w(3, 2, 9);

   w_b_w(1, 11, 9);
   
   w_b_w(1, 11, 9);
   
   w_b_w(1, 11, 9);

	 w_b_w(2, 9, 10);
   
   w_b_w(2, 9, 10);
   
   w_b_w(3, 7, 11);
   
   w_b_w(3, 7, 11);
      
   w_b_w(4, 5, 12);
   
   w_b_w(4, 5, 12);
   

		for(int i=0;i<200;i++){
			enemy();
			LCD_INIT_DELAY(1);
      app_sched_execute();
			if(adventure_temp != 0)
				break;
		}
}

void jump() {
	state = 1;
	start_point(0, 0, 20, 50);
	st7586_write(ST_COMMAND, 0x2C);
   white(21,4);

	 b_w_b(6, 1, 0);
   w_b_w(0, 5, 9);
   
	 b_w_b(6, 1, 0);
   w_b_w(0, 5, 9);

	 b_w_b(5, 1, 0);
   w_b_w(0, 7, 8);
   
   w_b_w(6, 7, 8);
   
   w_b_w(5, 9, 7);
   
   w_b_w(5, 9, 7);
   
	 b_w_b(3, 0, 0);
   w_b_w(1, 11, 6);
   
	 b_w_b(3, 0, 0);
   w_b_w(1, 11, 6);
   
	 b_w_b(3, 0, 0);
   w_b_w(1, 11, 6);
   
   w_b_w(4, 11, 6);
   
   w_b_w(3, 13, 5);
   
   w_b_w(3, 13, 5);
     
	 b_w_b(2, 1, 0);
   w_b_w(0, 13, 5);
   
	 b_w_b(2, 1, 0);
   w_b_w(0, 13, 5);
   
	 b_w_b(2, 1, 0);
   w_b_w(0, 13, 5);
         
   w_b_w(3, 13, 1);
   w_b_w(0, 2 ,2);
   
   w_b_w(3, 17, 1);
   
	 w_b_w(3, 17, 1);

   b_w_b(2, 1, 0);
	 w_b_w(0, 18, 0);

   b_w_b(2, 1, 0);
	 w_b_w(0, 18, 0);
   
   b_w_b(2, 1, 0);
   w_b_w(0, 18, 0);
   
	 w_b_w(3, 18, 0);
   
	 w_b_w(3, 18, 0);
   
   w_b_w(3, 18, 0);
   
   b_w_b(2, 1, 0);
   w_b_w(0, 18, 0);
   
   b_w_b(2, 1, 0);
   w_b_w(0, 18, 0);
   
   b_w_b(2, 1, 0);
   w_b_w(0, 18, 0);
   
   w_b_w(3, 17, 1);
   
   w_b_w(3, 17, 1);
   
   w_b_w(3, 13, 1);
   w_b_w(0, 2 ,2);
	// 
	 b_w_b(2, 1, 7);
	 w_b_w(3, 3, 5);
   
	 b_w_b(2, 1, 7);
	 w_b_w(3, 3, 5);
   
	 b_w_b(2, 1, 7);
	 w_b_w(3, 3, 5);
   
   w_b_w(3, 7, 0);
	 w_b_w(3, 3, 5);
   
   w_b_w(3, 7, 0);
	 w_b_w(3, 3, 5);
   
   w_b_w(3, 7, 0);
	 w_b_w(3, 3, 5);
      
	 b_w_b(3, 1, 6);
	 w_b_w(3, 2, 6);

	 black(3, 1);
   w_b_w(1, 11, 6);
   
	 black(3, 1);
   w_b_w(1, 11, 6);
   
   w_b_w(4, 11, 6);

	 w_b_w(5, 9, 7);
   
   w_b_w(5, 9, 7);
  // 
	 black(5, 1);
   w_b_w(1, 7, 8);
   
	 black(5, 1);
   w_b_w(1, 7, 8);
      
	 black(6, 1);
   w_b_w(1, 5, 9);
   
   w_b_w(7, 5,9);

	for(int i=0;i<200;i++){
		enemy();
		LCD_INIT_DELAY(1);
    app_sched_execute();
	}
}

void bow() {
	state = 2;
	
	start_point(0, 0, 20, 50);
	st7586_write(ST_COMMAND, 0x2C);
   white(21,4);

   w_b_w(0, 3, 18);
   
   w_b_w(0, 3, 18);

   w_b_w(0, 4, 17);
   
   w_b_w(0, 4, 17);
   
   w_b_w(0, 5, 16);
   
   w_b_w(0, 5, 16);
   
   w_b_w(0, 6, 15);
   
   w_b_w(0, 6, 15);
   
   w_b_w(0, 6, 15);
   
   w_b_w(0, 6, 15);
   
   w_b_w(0, 7, 14);
   
   w_b_w(0, 7, 14);
     
   w_b_w(0, 7, 14);
   
   w_b_w(0, 7, 14);
   
   w_b_w(0, 7, 14);
         
   w_b_w(0, 7, 1);
   w_b_w(0, 2 ,11);
   
   w_b_w(0, 11, 10);
   
   w_b_w(0, 11, 10);

	 w_b_w(0, 12, 9);

	 w_b_w(0, 12, 9);
   
	 w_b_w(0, 12, 9);
   
	 w_b_w(0, 12, 9);
   
	 w_b_w(0, 12, 9);
   
	 w_b_w(0, 12, 9);
   
	 w_b_w(0, 12, 9);
   
	 w_b_w(0, 12, 9);
   
	 w_b_w(0, 12, 9);
   
   w_b_w(0, 11, 10);
   
   w_b_w(0, 11, 10);
   
   w_b_w(0, 7, 1);
   w_b_w(0, 2 ,11);
	// 
	 b_w_b(0, 0, 1);
	 w_b_w(3, 3, 14);
   
	 b_w_b(0, 0, 1);
	 w_b_w(3, 3, 14);
   
	 b_w_b(0, 0, 1);
	 w_b_w(3, 3, 14);
   
   w_b_w(0, 1, 0);
	 w_b_w(3, 3, 14);
   
   w_b_w(0, 1, 0);
	 w_b_w(3, 3, 14);
   
   w_b_w(0, 1, 0);
	 w_b_w(3, 3, 14);
      
	 b_w_b(0, 0, 1);
	 w_b_w(3, 2, 15);

	 w_b_w(0, 6, 15);
   
	 w_b_w(0, 6, 15);
   
	 w_b_w(0, 6, 15);

	 w_b_w(0, 5, 16);
   
	 w_b_w(0, 5, 16);
  // 
   w_b_w(0, 4, 17);
   
   w_b_w(0, 4, 17);
      
   w_b_w(0, 3, 18);
   
   w_b_w(0, 3, 18);

	for(int i=0;i<200;i++){
		enemy();
		LCD_INIT_DELAY(1);
    app_sched_execute();
	}
}

void hited(){
		start_point(0, 0, 20, 50);
   st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(4, 5, 12);
   
   w_b_w(4, 5, 12);

   w_b_w(3, 7, 11);
   
   w_b_w(3, 7, 11);
   
   w_b_w(2, 9, 10);
   
   w_b_w(2, 9, 10);
   
   w_b_w(1, 11, 9);
   
   w_b_w(1, 11, 9);
   
   w_b_w(1, 11, 9);
   
   w_b_w(1, 11, 9);
   
   w_b_w(0, 13, 8);
   
   w_b_w(0, 13, 8);
     
   w_b_w(0, 13, 8);
   
   w_b_w(0, 13, 8);
   
   w_b_w(0, 13, 8);
         
   w_b_w(0, 13, 1);
   w_b_w(0, 2 ,5);
   
   w_b_w(0, 17, 4);
   
	 w_b_w(0, 17, 4);

	 w_b_w(0, 18, 3);

	 w_b_w(0, 18, 3);
   
   w_b_w(0, 18, 3);
   
	 w_b_w(0, 18, 3);
   
	 w_b_w(0, 18, 3);
   
   w_b_w(0, 18, 3);
   
   w_b_w(0, 18, 3);
   
   w_b_w(0, 18, 3);
   
   w_b_w(0, 18, 3);
   
   w_b_w(0, 17, 4);
   
   w_b_w(0, 17, 4);
   
   w_b_w(0, 13, 1);
   w_b_w(0, 2 ,5);
	 
   w_b_w(0, 7, 0);
	 st7586_write(ST_DATA, 0x00);
 	 st7586_write(ST_DATA, 0xff);
 	 st7586_write(ST_DATA, 0x00);
	 w_b_w(0, 3, 8);
   
   w_b_w(0, 7, 0);
	 st7586_write(ST_DATA, 0xc0);
 	 st7586_write(ST_DATA, 0x18);
 	 st7586_write(ST_DATA, 0x0f);
	 w_b_w(0, 3, 8);
   
   w_b_w(0, 7, 0);
	 st7586_write(ST_DATA, 0xf8);
 	 st7586_write(ST_DATA, 0x00);
 	 st7586_write(ST_DATA, 0x1f);
	 w_b_w(0, 3, 8);
   
   w_b_w(0, 7, 0);
	 st7586_write(ST_DATA, 0xff);
 	 st7586_write(ST_DATA, 0x00);
 	 st7586_write(ST_DATA, 0xff);
	 w_b_w(0, 3, 8);
   
   w_b_w(0, 7, 0);
	 st7586_write(ST_DATA, 0xf8);
 	 st7586_write(ST_DATA, 0x00);
 	 st7586_write(ST_DATA, 0x1f);
	 w_b_w(0, 3, 8);
   
   w_b_w(0, 7, 0);
	 st7586_write(ST_DATA, 0xc0);
 	 st7586_write(ST_DATA, 0x18);
 	 st7586_write(ST_DATA, 0x0f);
	 w_b_w(0, 3, 8);
      
   w_b_w(1, 6, 0);
	 st7586_write(ST_DATA, 0x00);
 	 st7586_write(ST_DATA, 0xff);
 	 st7586_write(ST_DATA, 0x00);
	 w_b_w(0, 2, 9);

   w_b_w(1, 11, 9);
   
   w_b_w(1, 11, 9);
   
   w_b_w(1, 11, 9);

	 w_b_w(2, 9, 10);
   
   w_b_w(2, 9, 10);
   
   w_b_w(3, 7, 11);
   
   w_b_w(3, 7, 11);
      
   w_b_w(4, 5, 12);
   
   w_b_w(4, 5, 12);
		LCD_INIT_DELAY(500);
}

void enemy(){
	if(enemy_temp>-37){
		if(enemy_num == 0){
			start_point(0, enemy_temp, 2, enemy_temp + 36);
			arrow();
			if(enemy_temp < 50){
				if(state != 1){
					if(!hit){
						hit = true;
						hp--;
						display_hp();
						hited();
						experience--;
					}
				}
			}
		}else if(enemy_num == 1){	
			start_point(13, enemy_temp, 15, enemy_temp + 36);
			arrow();		
			if(enemy_temp < 50){
				if(state != 2){
					if(!hit){
						hit = true;
						hp--;
						display_hp();
						hited();
						experience--;
					}
				}
			}
		}else if(enemy_num == 2){
			//small enemy
		}
		if(hp == 0){
			experience++;
			gameover();
		}
	}
	enemy_temp--;
	if(enemy_temp == -500){
		experience++;
		enemy_temp = 160;
		hit = false;
		if(enemy_num == 0)
			enemy_num = 1;
		else
			enemy_num = 0;
	}
}

void gameover(){
	enemy_temp = 160;
	state_temp = 0;
	adventure_temp = 5;
	hp = 3;

	clear();
	start_point(10, 19, 33, 141);
	st7586_write(ST_COMMAND, 0x2C);
	for(int i=0;i<6;i++){
		b_w_b(10, 4, 10);
	}
	for(int i=0;i<7;i++){
		b_w_b(2, 6, 2);
		white(4, 1);
		b_w_b(2, 6, 2);
	}
	for(int i=0;i<7;i++){
		b_w_b(2, 6, 2);
		white(4, 1);
		b_w_b(2, 1, 2);
		b_w_b(0, 3, 2);
	}
	for(int i=0;i<6;i++){
		b_w_b(10, 4, 5);
		b_w_b(0, 1, 4);
	}
	white(24, 4);
	w_b_w(9, 1, 4);
	black(1, 10);
	w_b_w(8, 2, 4);
	black(1, 10);
	w_b_w(7, 3, 4);
	black(1, 10);
	w_b_w(6, 4, 4);
	black(1, 10);
	w_b_w(5, 5, 4);
	black(1, 10);
	w_b_w(4, 6, 4);
	black(1, 10);
	w_b_w(3, 6, 5);
	white(1, 3);
	b_w_b(2, 3, 2);	
	w_b_w(2, 6, 6);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(1, 6, 7);
	white(1, 3);
	b_w_b(2, 3, 2);	
	w_b_w(0, 6, 8);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(0, 5, 9);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(0, 4, 10);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(0, 3, 11);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(0, 3, 11);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(0, 4, 10);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(0, 5, 9);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(0, 6, 8);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(1, 6, 7);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(2, 6, 6);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(3, 6, 5);
	white(1, 3);
	b_w_b(2, 3, 2);
	w_b_w(4, 6, 4);
	black(1, 10);
	w_b_w(5, 5, 4);
	black(1, 10);
	w_b_w(6, 4, 4);
	black(1, 10);
	w_b_w(7, 3, 4);
	black(1, 10);
	w_b_w(8, 2, 4);
	black(1, 10);
	w_b_w(9, 1, 4);
	black(1, 10);

	white(24, 4);

	for(int i=0;i<6;i++){
		b_w_b(10, 4, 10);
	}
	for(int i=0;i<4;i++){
		b_w_b(2, 2, 2);
		w_b_w(2, 2, 4);
		b_w_b(0, 8, 2);
	}
	for(int i=0;i<6;i++){
		b_w_b(2, 2, 2);
		w_b_w(2, 2, 4);
		black(1, 10);
	}
	for(int i=0;i<4;i++){
		b_w_b(2, 2, 2);
		w_b_w(2, 2, 4);
		b_w_b(0, 8, 2);
	}
	for(int i=0;i<6;i++){
		b_w_b(2, 2, 2);
		w_b_w(2, 2, 4);
		black(1, 10);
	}
	white(24, 4);

	for(int i=0;i<6;i++){
		b_w_b(10, 4, 10);
	}
	for(int i=0;i<10;i++){
		w_b_w(4 ,2 ,2);
		w_b_w(0, 2, 4);
		b_w_b(2 ,2 ,2);
		w_b_w(2 ,2, 0);
	}
	w_b_w(3 ,3 ,2);
	w_b_w(0, 2, 4);
	b_w_b(2 ,2 ,2);
	w_b_w(2 ,2, 0);	

	w_b_w(2 ,4 ,2);
	w_b_w(0, 2, 4);
	b_w_b(2 ,2 ,2);
	w_b_w(2 ,2, 0);	

	w_b_w(1 ,5 ,2);
	w_b_w(0, 2, 4);
	b_w_b(2 ,2 ,2);
	w_b_w(2 ,2, 0);	

	w_b_w(0 ,6 ,2);
	w_b_w(0, 2, 4);
	b_w_b(2 ,2 ,2);
	w_b_w(2 ,2, 0);	

	w_b_w(0, 10, 4);
	b_w_b(2 ,2 ,2);
	w_b_w(2 ,2, 0);	

	w_b_w(0, 10, 4);
	b_w_b(2 ,2 ,2);
	w_b_w(2 ,2, 0);	

	w_b_w(0 ,3 ,1);
	w_b_w(0, 6, 4);
	b_w_b(2 ,2 ,2);
	w_b_w(2 ,2, 0);

	w_b_w(0 ,2 ,2);
	w_b_w(0, 6, 4);
	b_w_b(2 ,2 ,2);
	w_b_w(2 ,2, 0);

	w_b_w(0 ,1 ,3);
	w_b_w(0, 6, 4);
	b_w_b(2 ,2 ,2);
	w_b_w(2 ,2, 0);

	w_b_w(0 ,0 ,4);
	w_b_w(0, 6, 4);
	b_w_b(2 ,2 ,2);
	w_b_w(2 ,2, 0);	
	LCD_INIT_DELAY(1000);
	start_point(0, 0, 44, 160);
	st7586_write(ST_COMMAND, 0x2C);
	white(100, 160);
	adventure_ing = false;
}

void arrow(){
	st7586_write(ST_COMMAND, 0x2C);
	white(1, 1);
	st7586_write(ST_DATA, 0x18);
	white(1, 1);
	white(1, 1);
	st7586_write(ST_DATA, 0x18);
	white(1, 1);
	white(1, 1);
	st7586_write(ST_DATA, 0xff);
	white(1, 1);
	white(1, 1);
	st7586_write(ST_DATA, 0xff);
	white(1, 1);
	st7586_write(ST_DATA, 0x03);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x03);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x3f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xfc);
	st7586_write(ST_DATA, 0x3f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xfc);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	for(int i=0;i<15;i++){
		white(1, 1);
		st7586_write(ST_DATA, 0xff);
		white(1, 1);
	}
	st7586_write(ST_DATA, 0x03);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x03);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	for(int i=0;i<8;i++){
		st7586_write(ST_DATA, 0x3f);
		st7586_write(ST_DATA, 0xff);
		st7586_write(ST_DATA, 0xfc);
	}
	white(3, 1);
}

void display_hp(){
	for(int i = 0; i < hp; i++){
		start_point(38, 17*i, 42, 17*i + 14);
		heart();
	}
	for(int i = hp; i < 3; i++){
		start_point(38, 17*i, 42, 17*i + 14);
		st7586_write(ST_COMMAND, 0x2C);
		white(5, 15);
	}
}


void heart(){
	st7586_write(ST_COMMAND, 0x2C);
	white(2, 1);
	st7586_write(ST_DATA, 0x03);
	black(1, 1);
	st7586_write(ST_DATA, 0xc0);
	
	white(2, 1);
	black(2, 1);
	st7586_write(ST_DATA, 0xd8);

	white(1, 1);
	st7586_write(ST_DATA, 0x03);
	black(3, 1);
	
	white(1, 1);
	st7586_write(ST_DATA, 0x1f);
	black(3, 1);
	
	white(1, 1);
	black(3, 1);
	st7586_write(ST_DATA, 0xf8);
	
	st7586_write(ST_DATA, 0x03);
	black(3, 1);
	st7586_write(ST_DATA, 0xf8);

	st7586_write(ST_DATA, 0x1f);
	black(3, 1);
	st7586_write(ST_DATA, 0xc0);
	
	black(3, 1);
	st7586_write(ST_DATA, 0xf8);
	white(1, 1);
	
	st7586_write(ST_DATA, 0x1f);
	black(3, 1);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x03);
	black(3, 1);
	st7586_write(ST_DATA, 0xf8);

	white(1, 1);
	black(3, 1);
	st7586_write(ST_DATA, 0xf8);
	
	white(1, 1);
	st7586_write(ST_DATA, 0x1f);
	black(3, 1);

	white(1, 1);
	st7586_write(ST_DATA, 0x03);
	black(3, 1);
	
	white(2, 1);
	black(2, 1);
	st7586_write(ST_DATA, 0xd8);

	white(2, 1);
	st7586_write(ST_DATA, 0x03);
	black(1, 1);
	st7586_write(ST_DATA, 0xc0);
}


void zero(int x, int y){
	start_point(x, y, x+3, y+8);
	st7586_write(ST_COMMAND, 0x2C);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

}
void one(int x, int y){
	start_point(x, y, x+3, y+35);
	st7586_write(ST_COMMAND, 0x2C);
	white(4, 3);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	white(4, 31);
}
void two(int x, int y){
	start_point(x, y, x+3, y+35);
	st7586_write(ST_COMMAND, 0x2C);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	white(4, 28);
}
void three(int x, int y){
	start_point(x, y, x+3, y+35);
	st7586_write(ST_COMMAND, 0x2C);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
white(4, 28);
}
void max(int x, int y){
	start_point(x, y, x+3, y+35);
	st7586_write(ST_COMMAND, 0x2C);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xc0);

}
void display_state(){
	start_point(27, 0, 42, 30);
	st7586_write(ST_COMMAND, 0x2C);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	//
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	//
	//
		st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	//
		st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	//
			st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	//
			st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	//
			st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	//
			st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	white(16, 2);
		st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	white(4, 1);
	//
		st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	white(4, 1);
	//
	
		st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	white(4, 1);
//
		st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	white(4, 1);
//
		st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	white(4, 1);
	//
			st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	white(4, 1);
//
		st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	white(4, 1);
	//
	white(8, 2);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

//

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
//

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
//

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
//

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
//

	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
//

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
//

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);

	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
//

	white(8, 2);

	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	
	if(lv == 1){
		one(39, 32);
	}else if(lv == 2){
		two(39, 32);
	}else if(lv == 3){
		three(39, 32);
	}else{
		max(39, 32);
	}
	
	if(experience == 0){
		zero(35, 32);
	}else if(experience == 1){
		one(35, 32);
	}else if(experience == 2){
		two(35, 32);
	}
	if(full == 0){
		zero(31, 32);
	}else if(full == 1){
		one(31, 32);
	}else if(full == 2){
		two(31, 32);
	}else{
		max(31, 32);
	}
	if(tired == 0){
		zero(27, 32);
	}else if(tired == 1){
		one(27, 32);
	}else if(tired == 2){
		two(27, 32);
	}else{
		max(27, 32);
	}
}

void select_state(){
	select = select%4;
	start_point(30, 100, 35, 160);
	st7586_write(ST_COMMAND, 0x2C);
	if(select == 0){
						black(6, 3);
		for(int i=0;i<10;i++){
			b_w_b(1, 2, 1);
			b_w_b(0, 1, 1);
		}
		white(6, 2);
		black(6, 3);
		for(int i=0;i<7;i++){
			w_b_w(2, 1, 2);
			black(1, 1);
		}
		black(6, 3);
		white(6, 2);
		for(int i=0;i<5;i++){
			b_w_b(0, 5, 1);
		}
		black(6, 3);
		for(int i=0;i<5;i++){
			b_w_b(0, 5, 1);
		}

	}else if(select == 1){

		
				b_w_b(1, 2, 3);
		b_w_b(1, 2, 3);
		b_w_b(1, 2, 3);
		for(int i=0;i<7;i++){
			b_w_b(1, 2, 1);
			b_w_b(0, 1, 1);
		}
		b_w_b(4, 1, 1);
		b_w_b(4, 1, 1);
		b_w_b(4, 1, 1);
		white(6, 2);
		black(6, 3);
		for(int i=0;i<10;i++){
			b_w_b(1, 5, 0);
		}		
		white(6, 2);
		black(6, 3);
		for(int i=0;i<7;i++){
			w_b_w(3, 1, 1);
			black(1, 1);
		}
		w_b_w(3, 3, 0);
		w_b_w(3, 3, 0);
		w_b_w(3, 3, 0);

	}else if(select == 2){
		black(6, 3);
		for(int i=0;i<7;i++){
			w_b_w(3, 1, 2);
		}
		black(6, 3);
		white(6, 2);
		black(6, 3);
		for(int i=0;i<10;i++){
			b_w_b(1, 5, 0);
		}		
		white(6, 2);
		for(int i=0;i<5;i++){
			b_w_b(0, 5, 1);
		}
		black(6, 3);
		for(int i=0;i<5;i++){
			b_w_b(0, 5, 1);
		}

	}else if(select == 3){
				black(6, 3);
		for(int i=0;i<7;i++){
			w_b_w(2, 1, 2);
			black(1, 1);
		}
		black(6, 3);
		white(6, 2);
		
		black(6, 3);
		for(int i=0;i<7;i++){
			b_w_b(1, 4, 1);
		}
		black(6, 3);		
		
		white(6, 2);
		w_b_w(5, 1, 0);
		w_b_w(4, 2, 0);
		w_b_w(3, 3, 0);
		w_b_w(2, 3, 1);
		w_b_w(1, 3, 2);
		w_b_w(0, 3, 3);
		w_b_w(0, 2, 4);
		w_b_w(0, 3, 3);
		w_b_w(1, 3, 2);
		w_b_w(2, 3, 1);
		w_b_w(3, 3, 0);
		w_b_w(4, 2, 0);
		w_b_w(5, 1, 0);
	}
}
void eating(){
  start_point(0, 40, 20, 120);
  st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   //////chopsticks
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 6, 2);
   b_w_b(5, 4, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 6, 2);
   b_w_b(6, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1,2,2);
   b_w_b(2,2,6);
   white(3,1);
   
   b_w_b(1,1,1);
   w_b_w(1,2,2);
   b_w_b(2,2,6);
   white(3,1);
   
   w_b_w(0,8,1);
   b_w_b(9,3,0);
   
   w_b_w(0,8,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,1,4);
   b_w_b(9,3,0);
   
   w_b_w(0,8,1);
   b_w_b(9,3,0);
   
   w_b_w(0,8,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,1,4);
   b_w_b(9,3,0);
   
   w_b_w(0,8,1);
   b_w_b(9,3,0);
   /////////////////////////////
   w_b_w(0,8,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,1,4);
   b_w_b(9,3,0);
   
   w_b_w(0,8,1);
   b_w_b(9,3,0);
   
   w_b_w(0,8,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,1,4);
   b_w_b(9,3,0);
   
   w_b_w(0,8,1);
   b_w_b(9,3,0);
   
   w_b_w(0,8,1);
   b_w_b(9,3,0);
   
   
   b_w_b(1,1,1);
   w_b_w(1,2,2);
   b_w_b(2,2,6);
   white(3,1);
   
   b_w_b(1,1,1);
   w_b_w(1,2,2);
   b_w_b(2,2,6);
   white(3,1);
   
   b_w_b(1,1,1);
   w_b_w(1, 6, 2);
   b_w_b(6, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   ///////////////////////
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);   
   w_b_w(1, 6, 2);
   b_w_b(5, 4, 0);
   
   b_w_b(1,1,1);   
   w_b_w(1, 6, 2);
   b_w_b(5, 4, 0);
   
   b_w_b(1,1,1);   
   w_b_w(1, 6, 2);
   b_w_b(5, 4, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 15, 2);
   
   b_w_b(1,1,1);
   w_b_w(1, 15, 2);
   
   b_w_b(1,1,1);
   w_b_w(1, 16, 1);
   
   b_w_b(1,1,1);
   w_b_w(1, 16, 1);
   
   b_w_b(1,1,1);
   w_b_w(1, 16, 1);
   
  b_w_b(1,1,1);   
   w_b_w(2, 16, 0);
   
   b_w_b(1,1,1);   
   w_b_w(2, 16, 0);
   
   b_w_b(1,1,1);   
   w_b_w(2, 16, 0);
   
   b_w_b(1,1,1);
   w_b_w(2, 9, 0);
   black(6,1);
   white(1,1);
   
   b_w_b(1,1,1);
   w_b_w(3, 9, 0);
   black(5,1);
   white(1,1);
   
   b_w_b(1,1,1);
   w_b_w(3, 9, 0);
   black(5,1);
   white(1,1);
   
   b_w_b(1,1,1);
   w_b_w(4, 7, 2);
   black(3,1);
   white(2,1);
   
   b_w_b(1,1,1);
   w_b_w(4, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
	LCD_INIT_DELAY(500);
	 start_point(0, 40, 20, 120);
  st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   //////chopsticks
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   
   w_b_w(4, 6, 2);
   b_w_b(5, 4, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 6, 2);
   b_w_b(5, 4, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 6, 2);
   b_w_b(6, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1,2,2);
   b_w_b(2,2,6);
   white(3,1);
   
   b_w_b(1,1,1);
   w_b_w(1,2,2);
   b_w_b(2,2,6);
   white(3,1);
   
   b_w_b(1,1,1);
   w_b_w(1,4,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,4,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,1,4);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,4,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,4,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,1,4);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,4,1);
   b_w_b(9,3,0);
   /////////////////////////////
   b_w_b(1,1,1);
   w_b_w(1,4,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,1,4);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,4,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,4,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,1,4);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,4,1);
   b_w_b(9,3,0);
   
   b_w_b(1,1,1);
   w_b_w(1,4,1);
   b_w_b(9,3,0);
   
   
   b_w_b(1,1,1);
   w_b_w(1,2,2);
   b_w_b(2,2,6);
   white(3,1);
   
   b_w_b(1,1,1);
   w_b_w(1,2,2);
   b_w_b(2,2,6);
   white(3,1);
   
   b_w_b(1,1,1);
   w_b_w(1, 6, 2);
   b_w_b(6, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   ///////////////////////
   b_w_b(1,1,1);
   w_b_w(1, 5, 4);
   b_w_b(5, 3, 0);
   
   b_w_b(1,1,1);   
   w_b_w(1, 6, 2);
   b_w_b(5, 4, 0);
   
   b_w_b(1,1,1);   
   w_b_w(1, 6, 2);
   b_w_b(5, 4, 0);
   
   b_w_b(1,1,1);   
   w_b_w(1, 6, 2);
   b_w_b(5, 4, 0);
   
   b_w_b(1,1,1);
   w_b_w(1, 15, 2);
   
   b_w_b(1,1,1);
   w_b_w(1, 15, 2);
   
   b_w_b(1,1,1);
   w_b_w(1, 16, 1);
   
   b_w_b(1,1,1);
   w_b_w(1, 16, 1);
   
   b_w_b(1,1,1);
   w_b_w(1, 16, 1);
   
  b_w_b(1,1,1);   
   w_b_w(2, 16, 0);
   
   b_w_b(1,1,1);   
   w_b_w(2, 16, 0);
   
   b_w_b(1,1,1);   
   w_b_w(2, 16, 0);
   
   b_w_b(1,1,1);
   w_b_w(2, 9, 0);
   black(6,1);
   white(1,1);
   
   b_w_b(1,1,1);
   w_b_w(3, 9, 0);
   black(5,1);
   white(1,1);
   
   b_w_b(1,1,1);
   w_b_w(3, 9, 0);
   black(5,1);
   white(1,1);
   
   b_w_b(1,1,1);
   w_b_w(4, 7, 2);
   black(3,1);
   white(2,1);
   
   b_w_b(1,1,1);
   w_b_w(4, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
LCD_INIT_DELAY(500);

}
void display_z(){
	start_point(24, 118, 29, 133);
	st7586_write(ST_COMMAND, 0x2C);
	white(6, 16);
}
void display_zZ(){
	start_point(24, 118, 29, 133);
	st7586_write(ST_COMMAND, 0x2C);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

}
void sleep(){
	  start_point(0, 40, 20, 120);
  st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   /////////////////////////////
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 1, 3);
	 w_b_w(0, 10, 3);
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
	 
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
      
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
      
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
      
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
      
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
      
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
      
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);

	start_point(20, 110, 23, 118);
	st7586_write(ST_COMMAND, 0x2C);
	b_w_b(1, 2, 1);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xc0);
	w_b_w(1, 1, 0);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xf8);
	w_b_w(1, 1, 0);

	b_w_b(2, 1, 1);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xc0);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xf8);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x1f);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x0f);
	st7586_write(ST_DATA, 0xff);

	st7586_write(ST_DATA, 0xff);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, 0xff);

	display_z();
	LCD_INIT_DELAY(500);
	
	  start_point(0, 40, 20, 120);
  st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   /////////////////////////////
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 2, 1);
	 w_b_w(0, 11, 3);
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
	 
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
      
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
      
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
      
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
      
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
      
   w_b_w(4, 6, 1);
   b_w_b(7, 3, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 6, 1);
   b_w_b(6, 4, 0);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
      
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
	display_zZ();

	LCD_INIT_DELAY(500);
}
void health(){  start_point(0, 40, 20, 160);
  st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   //////////////////////
   w_b_w(4, 7, 1);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   ///////////////////////////
         w_b_w(4,2,1);
         w_b_w(0, 11, 3);
         
         w_b_w(4,2,1);
         w_b_w(0, 11, 3);
         
         w_b_w(4,3,1);
         w_b_w(0, 10, 3);
         
         w_b_w(4,3,1);
         w_b_w(0, 10, 3);
         
         w_b_w(4,3,1);
         w_b_w(0, 10, 3);
         
         w_b_w(4,3,1);
         w_b_w(0, 10, 3);
         
         w_b_w(4,3,1);
         w_b_w(0, 10, 3);
         
         w_b_w(4,3,1);
         w_b_w(0, 10, 3);
         
         w_b_w(4,3,1);
         w_b_w(0, 10, 3);
         
         w_b_w(4,3,1);
         w_b_w(0, 10, 3);
         
         w_b_w(4,2,1);
         w_b_w(0, 11, 3);
         
         w_b_w(4,2,1);
         w_b_w(0, 11, 3);
   /////////////////////////////////
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
      
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   white(21,5);
   
   w_b_w(6,3,12);
   w_b_w(6,3,12);
   white(21,1);
   for(int i= 0 ; i<5; i++){
      w_b_w(5,5,11);
   }
   
   for(int i= 0 ; i< 10; i ++){
      w_b_w(7,1,13);
   }
   for(int i= 0 ; i<5; i++){
      w_b_w(5,5,11);
   }
   white(21,1);
   w_b_w(6,3,12);
   w_b_w(6,3,12);
LCD_INIT_DELAY(500);
	 start_point(0, 40, 20, 160);
  st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   //////////////////////
   w_b_w(4, 7, 1);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   ///////////////////////////
         w_b_w(4,2,1);
         w_b_w(0, 11, 3);
         
         w_b_w(4,2,1);
         w_b_w(0, 11, 3);
         
         for(int i= 0 ; i< 8 ; i++){
            w_b_w(4,1,3);
           w_b_w(0, 10, 3);
         }
         w_b_w(4,2,1);
         w_b_w(0, 11, 3);
         
         w_b_w(4,2,1);
         w_b_w(0, 11, 3);
   /////////////////////////////////
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
      
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   white(21,5);
   
   w_b_w(8,3,10);
   w_b_w(8,3,10);
   white(21,1);
   for(int i= 0 ; i<5; i++){
      w_b_w(7,5,9);
   }
   
   for(int i= 0 ; i< 10; i ++){
      w_b_w(9,1,11);
   }
   for(int i= 0 ; i<5; i++){
      w_b_w(7,5,9);
   }
   white(21,1);
   w_b_w(8,3,10);
   w_b_w(8,3,10);
	LCD_INIT_DELAY(500);
}
void reject(){
  start_point(0, 40, 20, 160);
  st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   //////////////////////
   w_b_w(4, 7, 1);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   ///////////////////////////
      
         for(int i= 0 ; i< 9; i++){
            w_b_w(4,14,3);
         
         }
   /////////////////////////////////
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   w_b_w(4,14,3);
   w_b_w(4,14,3);
   w_b_w(4,14,3);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
      
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
LCD_INIT_DELAY(500);   
  start_point(0, 40, 20, 160);
  st7586_write(ST_COMMAND, 0x2C);
   white(21,4);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   //////////////////////
   w_b_w(4,14,3);
   w_b_w(4,14,3);
   w_b_w(4,14,3);
   
   w_b_w(4, 7, 1);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(5, 4, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   ///////////////////////////
      
         for(int i= 0 ; i< 9; i++){
            w_b_w(4,14,3);
         
         }
   /////////////////////////////////
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 5, 3);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   w_b_w(4, 7, 1);
   b_w_b(6, 3, 0);
   
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 15, 2);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
   
   w_b_w(4, 16, 1);
      
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 16, 0);
   
   w_b_w(5, 9, 0);
   black(6,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(6, 9, 0);
   black(5,1);
   white(1,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(7, 7, 2);
   black(3,1);
   white(2,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
   
   w_b_w(8, 5, 4);
   black(1,1);
   white(3,1);
 LCD_INIT_DELAY(500);
}
int main(void)
{
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

	lfclk_request();
	gpio_config();
	app_timer_init();
	lcd_init();
	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
	conn_params_init();
	create_timers();

	leds_init();
	advertising_start();
	while (1)
	{
		if(state_temp == 0){
			display_state();
			while(experience>2){
				experience -= 3;
				lv++;
			}
			display_head();
		}else if(state_temp == 1){
			if(full!=3){
				for(int i=0;i<3;i++){
					eating();
				}
				full++;
			}else{
				for(int i=0;i<3;i++){
					reject();
				}
			}
			clear();
			state_temp = 0;
		}else if(state_temp == 2){
			if(tired != 0){
				for(int i=0;i<3;i++){
					sleep();
				}
				tired--;
			}else{
				for(int i=0;i<3;i++){
					reject();
				}
			}
			clear();
			state_temp = 0;
		}else if(state_temp == 3){
			if((tired!=3)&&(full!=0)){
				for(int i=0;i<3;i++){
					health();
				}
				tired++;
				full--;
				experience++;
			}else{
				for(int i=0;i<3;i++){
					reject();
				}
			}
			clear();
			state_temp = 0;
		}else if(state_temp == 4){
			if(adventure_temp == 3){
				adventure_start();
				adventure_temp = 0;
				display_hp();
			}else if(adventure_temp == 0){
				adventure();
			}else if(adventure_temp == 1){
				adventure_temp = 0;
				jump();
			}else if(adventure_temp == 2){
				adventure_temp = 0;
				bow();
			}
		}
	}
}
