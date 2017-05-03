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
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "bsp.h"
#include "ble_gap.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_twi.h"
#include "ble_smss.h"
#include "ble_advertising.h"

#include "sms_pressure.h"
#include "sms_imu.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define CENTRAL_LINK_COUNT              0										/**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1										/**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT					/**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2	/**< Reply when unsupported features are requested. */

#define ADVERTISING_LED_PIN             BSP_BOARD_LED_1							/**< Is on when device is advertising. */
//#define CONNECTED_LED_PIN               BSP_BOARD_LED_1							/**< Is on when device has connected. */
#define SENDING_LED_PIN					BSP_BOARD_LED_0

//#define LEDBUTTON_LED_PIN               BSP_BOARD_LED_2							/**< LED to be toggled with the help of the LED Button Service. */
//#define LEDBUTTON_LED1_PIN_NO			BSP_BOARD_LED_3
#define LEDBUTTON_BUTTON1_PIN           BSP_BUTTON_0							/**< Button that will trigger the notification event with the LED Button Service */
#define LEDBUTTON_BUTTON2_PIN			BSP_BUTTON_1

#define DEVICE_NAME                     "SMS_sensors"                             /**< Name of device. Will be included in the advertising data. */
//#define DEVICE_NAME                     "Nordic_Blinky"						/**< Name of device. Will be included in the advertising data. */
#define BOOTLOADER_RESET_3MIN			(0x15C75ABE)							/**<Command to reset device and keep the bootloader active for 3 minutes */
#define BOOTLOADER_DFU_START			(0xB1)

#define APP_ADV_INTERVAL                64										/**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED	/**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define APP_TIMER_PRESCALER             0										/**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            6										/**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4										/**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)			/**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(50, UNIT_1_25_MS)			/**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0										/**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)			/**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER)	/**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)	/**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3										/**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1										/**< Maximum number of users of the GPIOTE handler. */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(5, APP_TIMER_PRESCALER)	/**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF								/**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;/**< Handle of the current connection. */
//static ble_lbs_t                        m_lbs;								/**< LED Button Service instance. */
ble_smss_t								m_smss_service;

/* ====================================================================
 * VARIABLES
 * --------------------------------------------------------------------
 *
 * ==================================================================== */
// Timers
// app timer definitions
APP_TIMER_DEF(pressure_poll_int_id);
APP_TIMER_DEF(imu_poll_int_id);
//APP_TIMER_DEF(micros_cnt_id);
// drv timer instantiation
#define TIMER_INSTANCE 1
const nrf_drv_timer_t TIMER_DELTA_US = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE);
// counter overflow for us timer
volatile uint32_t micros_cnt_overflow = 0;

// SPI
#define SPI_INSTANCE 0
const nrf_drv_spi_t spi_master_instance = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
volatile bool spi_xfer_done;
#define SPI_MAX_LENGTH 100
uint8_t m_tx_buf[SPI_MAX_LENGTH] = {0};
uint8_t m_rx_buf[SPI_MAX_LENGTH + 1];

// TWI
#define TWI_INSTANCE 1
const nrf_drv_twi_t twi_master_instance = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);
volatile bool twi_xfer_done;

// Pressure
extern ms58_output_s ms58_output;
extern ms58_config_s ms58_config;
extern ms58_interrupt_s ms58_interrupt;

// IMU
extern bno055_output_s bno055_output;
extern bno055_config_s bno055_config;
extern bno055_interrupt_s bno055_interrupt;

/* ====================================================================
 * FUNCTIONS DECLARATIONS
 * --------------------------------------------------------------------
 *
 * ==================================================================== */
void advertising_start(void);
void timers_start(void);


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

/* ====================================================================
 * HANDLERS
 * --------------------------------------------------------------------
 * Has to be defined first to be called by the init functions
 * ==================================================================== */
// Timer interrupts
static void pressure_poll_int_handler(void * p_context)
{
	ms58_interrupt.new_value = true;
}

static void imu_poll_int_handler(void * p_context)
{
	bno055_interrupt.new_value = true;
}

static void timer_delta_us_handler(nrf_timer_event_t event_type, void * p_context)
{
	nrf_drv_timer_clear(&TIMER_DELTA_US);
//	bsp_board_led_invert(LEDBUTTON_LED_PIN);
}


// Hardware interrupts
/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;
	static uint16_t send_value = 0;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON1_PIN:
			if(button_action) send_value |= 0xFF;
			else send_value &= 0xFF00;
            NRF_LOG_INFO("Bt1... sending %#x\r\n", send_value);
//			int32_t * tosend1;
//			tosend1 = &ms58_output.pressure;
//			err_code = ble_smss_on_press_value(&m_smss_service, tosend1);
            err_code = ble_smss_on_button_change(&m_smss_service, send_value);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

		case LEDBUTTON_BUTTON2_PIN:
			if(button_action) send_value |= 0xFF00;
			else send_value &= 0x00FF;
			NRF_LOG_INFO("Bt2... sending %#x\r\n", send_value);
//			uint32_t * tosend2;
//			tosend2 = (uint32_t*)&bno055_output.grv[0].b;
//			err_code = ble_smss_on_imu_value(&m_smss_service, tosend2);
			err_code = ble_smss_on_button_change(&m_smss_service, send_value);
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



void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
}
void twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
	twi_xfer_done = true;
	switch(p_event->type)
	{
		case NRF_DRV_TWI_EVT_DONE:
			twi_xfer_done = true;
			break;
		default:
			break;
	}	
}
// BLE
/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
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

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
//            sleep_mode_enter();
            break;
        default:
            break;
    }
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

/**@brief Function for handling the Application's BLE stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");
//            bsp_board_led_on(CONNECTED_LED_PIN);
            bsp_board_led_off(ADVERTISING_LED_PIN);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
			
			timers_start();
		
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
//            bsp_board_led_off(CONNECTED_LED_PIN);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

            advertising_start();
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
    ble_smss_on_ble_evt(&m_smss_service, p_ble_evt);
}
/* ====================================================================
 * INITIALIZATIONS
 * --------------------------------------------------------------------
 *
 * ==================================================================== */
// Board
/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_leds_init();
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    uint32_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON1_PIN, false, BUTTON_PULL, button_event_handler},
		{LEDBUTTON_BUTTON2_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}

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

/**@brief Function for initializing the TWI module.
 */
void twi_init(void)
{
	ret_code_t err_code;
	const nrf_drv_twi_config_t twi_config = {
		.scl				= TWI_SCL_PIN,
		.sda				= TWI_SDA_PIN,
//		.frequency			= NRF_TWI_FREQ_100K,
		.frequency			= NRF_TWI_FREQ_400K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init		= false
		};
	
		err_code = nrf_drv_twi_init(&twi_master_instance, &twi_config, twi_event_handler, NULL);
		APP_ERROR_CHECK(err_code);
		
		nrf_drv_twi_enable(&twi_master_instance);
}

// Drivers
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
	uint32_t err_code;

    // Initialize application timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	
	// Initialize the timer driver to count microseconds
	nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
	timer_config.frequency = NRF_TIMER_FREQ_1MHz;
	err_code = nrf_drv_timer_init(&TIMER_DELTA_US, &timer_config, timer_delta_us_handler);
	APP_ERROR_CHECK(err_code);
}

static void bootloader_start(uint16_t conn_handle)
{
	uint32_t err_code;
	
	/* Force disconnect, disable softdevice, and then reset */
	sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	// The below requires at least bootloader 3.1
	err_code = sd_power_gpregret_set(BOOTLOADER_DFU_START);
	APP_ERROR_CHECK(err_code);
	
	sd_softdevice_disable();
	
	nrf_delay_us(500 * 1000);
	
	//reset system to start the bootloader
	NVIC_SystemReset();
}

	


// BLE
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

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void app_update_function(ble_smss_t * p_smss, uint8_t *data)
{
	uint32_t command = (data[0] +
						(data[1] << 8) +
						(data[2] << 16) +
						(data[3] << 24));
	NRF_LOG_INFO("Received app update command: %#x\n\r", command);
	if(command == 0x1c575abe) {
		NRF_LOG_INFO("Restarting device with 3 min bootloader...\n\r");
		bootloader_start(p_smss->conn_handle);
	}
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	ble_smss_init_t init;
	init.app_update_function = app_update_function;
	ble_smss_init(&m_smss_service, &init);
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
/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
//static void led_write_handler(ble_smss_t * p_smss, uint8_t led_state)
//{
//    if (led_state)
//    {
//        bsp_board_led_on(LEDBUTTON_LED_PIN);
//        NRF_LOG_INFO("Received LED ON!\r\n");
//    }
//    else
//    {
//        bsp_board_led_off(LEDBUTTON_LED_PIN);
//        NRF_LOG_INFO("Received LED OFF!\r\n");
//    }
//}
//static void led1_write_handler(ble_smss_t * p_smss, uint8_t led_state)
//{
//	if (led_state)
//	{
//		for(int i=0; i<10;i++)
//		{
//			nrf_gpio_pin_toggle(LEDBUTTON_LED1_PIN_NO);
//			nrf_delay_us(100000);
//		}
//	}
//	else
//	{
//		for(int i=0; i<24;i++)
//		{
//			nrf_gpio_pin_toggle(LEDBUTTON_LED1_PIN_NO);
//			nrf_delay_us(40000);
//		}
//	}
//}









/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    bsp_board_led_on(ADVERTISING_LED_PIN);
}




/* ====================================================================
 * RUN-TIMER FUNCTIONS
 * --------------------------------------------------------------------
 *
 * ==================================================================== */
static void timers_create(void)
{
	uint32_t err_code;
	err_code = app_timer_create(&pressure_poll_int_id,
								APP_TIMER_MODE_REPEATED,
								pressure_poll_int_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&imu_poll_int_id,
								APP_TIMER_MODE_REPEATED,
								imu_poll_int_handler);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}


void timers_start(void)
{
	NRF_LOG_INFO("Starting poll timers...\n\r");
	app_timer_start(pressure_poll_int_id,
					APP_TIMER_TICKS(MSEC_TO_UNITS(71, UNIT_1_00_MS), 0),
					NULL);
	app_timer_start(imu_poll_int_id,
					APP_TIMER_TICKS(MSEC_TO_UNITS(50, UNIT_1_00_MS), 0),
					NULL);
}

/* ====================================================================
 * MAIN
 * ==================================================================== */
/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    
    // Initialize
    leds_init();
    timers_init();
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    buttons_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

	spi_init();
	twi_init();
	
	// Instantiate
	timers_create();
	
	// Initialize & configure peripherals
	pressure_startup();
	NRF_LOG_INFO("MS58 enabled? %d\r\n\n", ms58_config.dev_en);
	
	imu_startup();
	NRF_LOG_INFO("BNO055 enabled? %d\r\n\n", bno055_config.dev_en);
	if(bno055_config.dev_en) {
		imu_configure();
		imu_check_cal();
		NRF_LOG_INFO("System calibration: %d\n\r",
					((0xC0 & bno055_config.cal_state) >> 6));
		NRF_LOG_INFO("Gyro   calibration: %d\n\r",
					((0x30 & bno055_config.cal_state) >> 4));
		NRF_LOG_INFO("Accel  calibration: %d\n\r",
					((0x0C & bno055_config.cal_state) >> 2));
		NRF_LOG_INFO("Mag    calibration: %d\n\r",
					((0x03 & bno055_config.cal_state) >> 0));
		imu_initialize();
	}
	
	
	// Start advertising
    NRF_LOG_INFO("Starting SMS sensors!\r\n");
    bsp_board_led_on(ADVERTISING_LED_PIN);
	err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
		
		if(ms58_interrupt.new_value)
		{
			ms58_interrupt.new_value = false;
			pressure_read_data();
			if(ms58_output.complete) {
				ms58_output.complete = false;
				pressure_calculate();
//				NRF_LOG_INFO("Press/Temp: %#x/%#x\n\r",
//								ms58_output.pressure,
//								ms58_output.temperature);
				ms58_interrupt.rts = true;
			}
			else {
				ms58_output.complete = true;
			}
		}
		
		if(bno055_interrupt.new_value) {
			bno055_interrupt.new_value = false;
			imu_poll_data();
//			NRF_LOG_INFO("Quat: %d %d %d %d\n\r",
//						(int32_t)(bno055_output.quat[0].val * 1000000),
//						(int32_t)(bno055_output.quat[1].val * 1000000),
//						(int32_t)(bno055_output.quat[2].val * 1000000),
//						(int32_t)(bno055_output.quat[3].val * 1000000));
			bno055_interrupt.rts = true;
		}
		
		if(ms58_interrupt.rts) {
			ms58_interrupt.rts = false;
			int32_t * tosend;
			tosend = &ms58_output.pressure;
		    bsp_board_led_on(SENDING_LED_PIN);
			err_code = ble_smss_on_press_value(&m_smss_service, tosend);
			bsp_board_led_off(SENDING_LED_PIN);
			if(	(err_code != 0x3401) &&
				(err_code != 0x0008) &&
				(err_code != 0) )
			{
				APP_ERROR_CHECK(err_code);
			}
		}
		if(bno055_interrupt.rts) {
			bno055_interrupt.rts = false;
			uint32_t * tosend;
			tosend = (uint32_t*)&bno055_output.quat[0].b;
		    bsp_board_led_on(SENDING_LED_PIN);
			err_code = ble_smss_on_imu_value(&m_smss_service, tosend);
		    bsp_board_led_off(SENDING_LED_PIN);
			if(	(err_code != 0x3401) &&
				(err_code != 0x0008) &&
				(err_code != 0) )
			{
				APP_ERROR_CHECK(err_code);
			}
		}
    }
}


/**
 * @}
 */
