/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "softdevice_handler.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_util_platform.h"
#include "ble.h"
#include "ble_advertising.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
//#include "ble_lbs.h"
//#include "ble_aps.h"
//#include "ble_imus.h"
#include "ble_smss.h"
#include "ble_gap.h"
#include "bsp.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"
#include "ms58.h"
#include "sms_imu.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define ADVERTISING_LED_PIN             BSP_BOARD_LED_0                             /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN               BSP_BOARD_LED_1                             /**< Is on when device has connected. */

#define LEDBUTTON_LED_PIN               BSP_BOARD_LED_2                             /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_LED_PIN2				BSP_BOARD_LED_3
#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_0                                /**< Button that will trigger the notification event with the LED Button Service */
#define LEDBUTTON_BUTTON_PIN2			BSP_BUTTON_1

#define DEVICE_NAME                     "SABRE_SMS"                             	/**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED       /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            6                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


static uint8_t							* m_sys_attibutes = NULL;
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
//static ble_lbs_t                        m_lbs;                                      /**< LED Button Service instance. */
//static ble_aps_t						m_aps;
//static ble_imus_t						m_imus;
static ble_smss_t						m_smss;


#define SPI_INSTANCE  0																/**< SPI instance index. */
static const nrf_drv_spi_t spi_master_instance = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  	/**< SPI instance. */
static volatile bool spi_xfer_done;													/**< Flag used to indicate that SPI instance completed the transfer. */
#define SPI_MAX_LENGTH					100
static uint8_t       					m_tx_buf[SPI_MAX_LENGTH] = {0};				/**< TX buffer. */
static uint8_t							m_rx_buf[sizeof(SPI_MAX_LENGTH) + 1];		/**< RX buffer. */

struct ms58_output_s					ms58_output;

extern struct bno055_config_s bno055_config;
extern struct bno055_output_s bno055_output;
extern struct bno055_interrupt_s bno055_interrupt;

APP_TIMER_DEF(pressure_poll_int_id);
static volatile bool pressure_poll_int_done;
APP_TIMER_DEF(imu_poll_int_id);
APP_TIMER_DEF(micros_cnt_id);
static volatile uint32_t micros_cnt_overflow = 0;

#define TIMER_INSTANCE 1
const nrf_drv_timer_t TIMER_DELTA_US = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE);
uint32_t old_cap = 0;
uint32_t old_cap1 = 0;



/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_leds_init();
}

static void timer_delta_us_handler(nrf_timer_event_t event_type, void * p_context)
{
	nrf_drv_timer_clear(&TIMER_DELTA_US);
	bsp_board_led_invert(LEDBUTTON_LED_PIN);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize application timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	
	// Initialize the timer driver to count microseconds
	SEGGER_RTT_printf(0, "init timer...\n");
	nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
	timer_config.frequency = NRF_TIMER_FREQ_1MHz;
	uint32_t err_code = nrf_drv_timer_init(&TIMER_DELTA_US, &timer_config, timer_delta_us_handler);
	APP_ERROR_CHECK(err_code);
}

static void pressure_poll_int_handler(void * p_context)
{
	pressure_poll_int_done = true;
}

static void imu_poll_int_handler(void * p_context)
{
	bno055_interrupt.new_int = true;
}

static void micros_cnt_handler(void * p_context)
{
	micros_cnt_overflow++;
}

static void timer_create(void)
{
	uint32_t err_code;
	err_code = app_timer_create(&pressure_poll_int_id, APP_TIMER_MODE_REPEATED, pressure_poll_int_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&imu_poll_int_id, APP_TIMER_MODE_REPEATED, imu_poll_int_handler);
	APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&micros_cnt_id, APP_TIMER_MODE_REPEATED, micros_cnt_handler);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
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
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	uint32_t err_code;
	
	switch(ble_adv_evt)
	{
		case BLE_ADV_EVT_FAST:
			bsp_board_led_on(ADVERTISING_LED_PIN);
			break;
		case BLE_ADV_EVT_IDLE:
//			sleep_mode_enter();
			break;
		default:
			break;
	}
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
 	ble_adv_modes_config_t options;
   
	memset(&advdata, 0, sizeof(advdata));
	
	advdata.name_type = BLE_ADVDATA_FULL_NAME;
	advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	
	memset(&options, 0, sizeof(options));
	options.ble_adv_fast_enabled = true;
	options.ble_adv_fast_interval = APP_ADV_INTERVAL;
	options.ble_adv_fast_timeout = APP_ADV_TIMEOUT_IN_SECONDS;
	
	err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(ble_smss_t * p_smss, uint8_t led_state)
{
    if (led_state)
    {
        bsp_board_led_on(LEDBUTTON_LED_PIN);
        NRF_LOG_INFO("Received LED ON!\r\n");
    }
    else
    {
        bsp_board_led_off(LEDBUTTON_LED_PIN);
        NRF_LOG_INFO("Received LED OFF!\r\n");
    }
}

static void pressure_write_handler(ble_smss_t * p_smss, uint8_t pressure_value)
{
}

static void imu_write_handler(ble_smss_t * p_smss, uint8_t imu_value)
{
}

static void button_write_handler(ble_smss_t * p_smss, uint8_t button_state)
{
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	ble_smss_init(&m_smss);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

///**@brief Function for starting advertising.
// */
//static void advertising_start(void)
//{
//    uint32_t             err_code;
//    ble_gap_adv_params_t adv_params;

//    // Start advertising
//    memset(&adv_params, 0, sizeof(adv_params));

//    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
//    adv_params.p_peer_addr = NULL;
//    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
//    adv_params.interval    = APP_ADV_INTERVAL;
//    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

//    err_code = sd_ble_gap_adv_start(&adv_params);
//    APP_ERROR_CHECK(err_code);
//    bsp_board_led_on(ADVERTISING_LED_PIN);
//}


/**@brief Function for handling the Application's BLE stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
	uint16_t *attr_len;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");
            bsp_board_led_on(CONNECTED_LED_PIN);
            bsp_board_led_off(ADVERTISING_LED_PIN);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

			NRF_LOG_INFO("Received handle 0x%04x\n", m_conn_handle);
		
			NRF_LOG_INFO("Updating persistent system attribute informations\n");
			err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
			APP_ERROR_CHECK(err_code);
		
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
            bsp_board_led_off(CONNECTED_LED_PIN);
//			err_code = sd_ble_gatts_sys_attr_get(m_conn_handle, NULL, attr_len, 0);
//			APP_ERROR_CHECK(err_code);
//			NRF_LOG_INFO("attr_len = %d\n");

//			err_code = sd_ble_gatts_sys_attr_get(m_conn_handle, m_sys_attibutes, attr_len, 0);
//			APP_ERROR_CHECK(err_code);
		
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

            ble_advertising_start(BLE_ADV_MODE_FAST);
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
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

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_smss_on_ble_evt(&m_smss, p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN:
            NRF_LOG_INFO("Send button state change.\r\n");            
            err_code = ble_smss_on_button_change(&m_smss, button_action);
//			err_code = ble_smss_on_press_value(&m_smss, button_action);
			if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

 		case LEDBUTTON_BUTTON_PIN2:
			NRF_LOG_INFO("Send button 2 state change.\r\n");
			button_action |= 0x10;
//			err_code = ble_smss_on_button_change(&m_smss, button_action);
			err_code = ble_smss_on_press_value(&m_smss, button_action);
			if(err_code != NRF_SUCCESS &&
				 err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
				 err_code != NRF_ERROR_INVALID_STATE)
			{
				APP_ERROR_CHECK(err_code);
			}
			break;

		default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    uint32_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler},
 		{LEDBUTTON_BUTTON_PIN2, false, BUTTON_PULL, button_event_handler}
   };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
//    NRF_LOG_INFO("Transfer completed.\r\n");
//    if (m_rx_buf[0] != 0)
//    {
//        NRF_LOG_INFO(" Received: \r\n");
//        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
}


void pin_change_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	bno055_interrupt.new_int = true;
//	bsp_board_led_invert(LEDBUTTON_LED_PIN);
}


static void gpio_init(void)
{
	ret_code_t err_code;
	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;
	err_code = nrf_drv_gpiote_in_init(DRDY_INT_PIN, &in_config, pin_change_handler);
	APP_ERROR_CHECK(err_code);
	
	bno055_interrupt.new_int = false;
	nrf_drv_gpiote_in_event_enable(DRDY_INT_PIN, true);
}

/**@brief Function for initializing the SPI module.
 */
static void spi_init(void)
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin = SPI_SS_PIN;
	spi_config.miso_pin = SPI_MISO_PIN;
	spi_config.mosi_pin = SPI_MOSI_PIN;
	spi_config.sck_pin = SPI_SCK_PIN;
	spi_config.mode = NRF_DRV_SPI_MODE_0;
	spi_config.frequency = NRF_DRV_SPI_FREQ_500K;
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi_master_instance, &spi_config, spi_event_handler));
}



/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}


static void ms58_reset(void)
{
//	NRF_LOG_INFO("Resetting MS58\n\r");
	uint8_t tx_len = 1;
	uint8_t rx_len = 0;
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_RESET;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	nrf_delay_ms(4);
}

static void ms58_read_prom(void)
{
	uint8_t tx_len = 1;
	uint8_t rx_len = 3;
	
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_1;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[1] = (m_rx_buf[1] << 8 | m_rx_buf[2]);
		
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_2;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[2] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_3;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[3] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_4;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[4] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_5;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[5] = (m_rx_buf[1] << 8 | m_rx_buf[2]);

	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_PROM_READ_6;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
	ms58_output.prom_values[6] = (m_rx_buf[1] << 8 | m_rx_buf[2]);


	// Send the first conversion command
	tx_len = 1;
	rx_len = 0;
	m_tx_buf[0] = MS58_CONV_D1_4096;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
}

static void ms58_read_data(void)
{
	uint8_t tx_len = 4;
	uint8_t rx_len = 4;
	
	memset(m_rx_buf, 0, rx_len);
	m_tx_buf[0] = MS58_ADC_READ;
	m_tx_buf[1] = MS58_ADC_READ;
	m_tx_buf[2] = MS58_ADC_READ;
	m_tx_buf[3] = MS58_ADC_READ;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};

	if(ms58_output.complete) {
		ms58_output.adc_values[MS58_TYPE_PRESS] = ((m_rx_buf[1] << 16) | (m_rx_buf[2] << 8) | (m_rx_buf[3]));
//		SEGGER_RTT_printf(0, "ADC value[MS58_TYPE_TEMP]: 0x%02x %02x %02x\n", m_rx_buf[1], m_rx_buf[2], m_rx_buf[3]);
		m_tx_buf[0] = MS58_CONV_D2_4096;
	}
	else {
		ms58_output.adc_values[MS58_TYPE_TEMP] = ((m_rx_buf[1] << 16) | (m_rx_buf[2] << 8) | (m_rx_buf[3]));
//		SEGGER_RTT_printf(0, "ADC value[MS58_TYPE_PRESS]: 0x%02x %02x %02x\n", m_rx_buf[1], m_rx_buf[2], m_rx_buf[3]);
		m_tx_buf[0] = MS58_CONV_D1_4096;
	}
	tx_len = 1;
	rx_len = 0;
	spi_xfer_done = false;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_instance, m_tx_buf, tx_len, m_rx_buf, rx_len));
	while(!spi_xfer_done) {};
}

static void ms58_calculate(void)
{
    /***************************************************************************
    * Calculated values...
    * Note: - ms58_prom_values[] are uint16_t
    *       - ms58_adc_values[] are uint32_t
    * Typical values given on the MS5003-01BA datasheets (March 25, 2013) are:
    * -------------------------------------------------------------------------
    * ms58_prom_values[]          |   ms58_adc_values[]
    * - C1 = 40'127 (SENSt1)      |   - D1 = 9'085'466 (Digital pressure)
    * - C2 = 36'924 (OFFt1)       |   - D2 = 8'569'150 (Digital temperature)
    * - C3 = 23'317 (TCS)         |
    * - C4 = 23'282 (TCO)         |
    * - C5 = 33'464 (Tref)        |
    * - C6 = 28'312 (TEMPSENS)    |
    **************************************************************************/
    int32_t deltaT;
    int64_t offset, sensitivity, tv1, tv2, tv3;

    /***************************
    * Temperature calculation *
    ***************************/
    /* dT = D2 - Tref = D2 - C5*2^8 */
    /* tv1: 33464 * 2^8 = 8566784 */
    tv1 = ((int64_t)(ms58_output.prom_values[5]) << 8);
    /* deltaT: 8569150 - 8566784 = 2366 */
    deltaT = (int32_t)((int64_t)ms58_output.adc_values[MS58_TYPE_TEMP] - tv1);

    /* TEMP = 20°C + dT*TEMPSENS = 2000 + dT * C6/2^23 */
    /* tv1: 28312 * 2366 = 66986192 */
    tv1 = ((int64_t)ms58_output.prom_values[6] * (int64_t)deltaT);
    /* tv2: 66986192 / 2^23 = 7(.985376358) */
    tv2 = (tv1 >> 23);
    /* temp: 7 + 2000 = 2007 */
    ms58_output.temperature = (int32_t)(tv2 + 2000);

    /************************
    * Pressure calculation *
    ************************/
    /* OFF = OFFt1 + TCO*dT = C2*2^16 + (C4*dT)/2^7 */
    /* tv1: 36924 * 2^16 = 2419851264 */
    tv1 = ((int64_t)(ms58_output.prom_values[2]) << 16);
    /* tv2: 23282 * 2366 = 55085212 */
    tv2 = ((int64_t)ms58_output.prom_values[4] * (int64_t)deltaT);
    /* tv3: 55085212 / 2^7 = 430353(.21875) */
    tv3 = (tv2 >> 7);
    /* offset: 2419851264 + 430353 = 2420281617 */
    offset = (tv1 + tv3);

    /* SENS = SENSt1 + TCS*dT = C1*2^15 + (C3*dT)/2^8 */
    /* tv1: 40127 * 2^15 = 1314881536 */
    tv1 = ((int64_t)(ms58_output.prom_values[1]) << 15);
    /* tv2: 23317 * 2366 = 55168022 */
    tv2 = ((int64_t)ms58_output.prom_values[3] * (int64_t)deltaT);
    /* tv3: 55168022 / 2^8 = 215500(.0859375) */
    tv3 = (tv2 >> 8);
    /* sensitivity: 1314881536 + 215500 = 1315097036 */
    sensitivity = (tv1 + tv3);

    /* P = D1*SENS - OFF = (D1*SENS/2^21 - OFF)/2^15 */
    /* tv1: (9085466 * 1315097036) / 2^21 = 5697378829(.612148284) */
    tv1 = (((int64_t)ms58_output.adc_values[MS58_TYPE_PRESS] * sensitivity) >> 21);
    /* tv2: 5697378829 - 2420281617 = 3277097212 */
    tv2 = tv1 - offset;
    /* press: 3277097212 / 2^15 = 100009(.070190) */
    ms58_output.pressure = (int32_t)(tv2 >> 15);
}



/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    
    // Initialize.
    leds_init();
    timers_init();
	timer_create();
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    buttons_init();

	SEGGER_RTT_printf(0, "Initializing BLE...\n");
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
	
	spi_init();
	ms58_reset();
	ms58_read_prom();
	ms58_output.complete = false;

	twi_init();
	bno055_reset();
	nrf_delay_ms(500);
	int ret = bno055_check();
	if(!ret)
	{
		SEGGER_RTT_printf(0, "Whole BNO055 device is present\n");
		nrf_delay_ms(1000);
		bno055_test();
		nrf_delay_ms(1000);
		bno055_init_config_values();
//		bno055_calibrate_accel_gyro(bno055_config.accel_bias, bno055_config.gyro_bias);
//		nrf_delay_ms(1000);
//		bno055_calibrate_mag(bno055_config.mag_bias);
//		nrf_delay_ms(1000);
		
		// Check calibration status of the sensors
		uint8_t calstat = readByte(BNO055_ADDRESS, BNO055_CALIB_STAT);
		SEGGER_RTT_printf(0, "Not calibrated = 0, fully calibrated = 3\n");
		SEGGER_RTT_printf(0, "System calibration status: %d\n", ((0xC0 & calstat) >> 6));
		SEGGER_RTT_printf(0, "Gyro   calibration status: %d\n", ((0x30 & calstat) >> 4));
		SEGGER_RTT_printf(0, "Accel  calibration status: %d\n", ((0x0C & calstat) >> 2));
		SEGGER_RTT_printf(0, "Mag    calibration status: %d\n", ((0x03 & calstat) >> 0));

		// Initialize bno055
		bno055_initialize();
		SEGGER_RTT_printf(0, "BNO055 initialized for sensor mode....");
	}
	else {
		if((ret & 0x08) == 0x08) {
			SEGGER_RTT_printf(0, "NO DEVICE AT ALL!!\n");
		}
		else {
			if((ret & 0x04) == 0x04) {
				SEGGER_RTT_printf(0, "No accelerometer\n");
			}
			if((ret & 0x02) == 0x02) {
				SEGGER_RTT_printf(0, "No magnetometer\n");
			}
			if((ret & 0x01) == 0x01) {
				SEGGER_RTT_printf(0, "No gyroscope\n");
			}
		}
	}
	
	gpio_init();
	
	app_timer_start(pressure_poll_int_id, APP_TIMER_TICKS(MSEC_TO_UNITS(67, UNIT_1_00_MS),0), NULL);
	app_timer_start(imu_poll_int_id, APP_TIMER_TICKS(MSEC_TO_UNITS(71, UNIT_1_00_MS),0), NULL);
	app_timer_start(micros_cnt_id, 0xffffffff, NULL);
	nrf_drv_timer_compare(&TIMER_DELTA_US, NRF_TIMER_CC_CHANNEL0, 1000000, true);
	nrf_drv_timer_enable(&TIMER_DELTA_US);
	

    // Start execution.
    NRF_LOG_INFO("Blinky Start!\r\n");
	bsp_board_led_on(ADVERTISING_LED_PIN);
	ble_advertising_start(BLE_ADV_MODE_FAST);

    // Enter main loop.
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
		
		if(pressure_poll_int_done) {
			pressure_poll_int_done = false;
			ms58_read_data();
			if(ms58_output.complete) {
				ms58_calculate();
//				SEGGER_RTT_printf(0, "Pressure: %ld, Temperature: %ld\n", ms58_output.pressure, ms58_output.temperature);
//				SEGGER_RTT_printf(0, "Sending: %d\n", aps_cnt);
//				err_code = ble_aps_on_new_value(&m_aps, aps_cnt);
//				if (err_code != NRF_SUCCESS &&
//					err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//					err_code != NRF_ERROR_INVALID_STATE)
//				{
//					APP_ERROR_CHECK(err_code);
//				}
//				aps_cnt += 1;
				ms58_output.complete = false;
			}
			else {
				ms58_output.complete = true;
			}
		}
		
		if(bno055_interrupt.new_int) {
//			SEGGER_RTT_printf(0, "BNO055 interrupt!\n");
			bno055_interrupt.new_int = false;
			bno055_poll_data();
			bno055_int_reset();
		}
    }
}


/**
 * @}
 */
