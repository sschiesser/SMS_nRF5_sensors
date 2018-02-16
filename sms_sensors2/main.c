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

/* ====================================================================
 * PRE-PROCESSING VALUES
 * --------------------------------------------------------------------
 * Includes
 * ==================================================================== */
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
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_twi.h"
#include "ble_smss.h"
#include "ble_bas.h"
#include "ble_advertising.h"

#include "sms_pressure.h"
#include "sms_imu.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

 /*--------------------------------------------------------------------
 * BLE/connection defines
 * -------------------------------------------------------------------- */
#define CENTRAL_LINK_COUNT              0										/**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1										/**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#if (NRF_SD_BLE_API_VERSION == 3)
	#define NRF_BLE_MAX_MTU_SIZE		GATT_MTU_SIZE_DEFAULT					/**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif
#define SMS_ADV_INTERVAL                320 //64										/**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define SMS_ADV_TIMEOUT_IN_SECONDS      30										/**< The advertisement timeout value in seconds */
#define SMS_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2	/**< Reply when unsupported features are requested. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)			/**< Minimum acceptable connection interval (15 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)			/**< Maximum acceptable connection interval (20 ms). */
#define SLAVE_LATENCY                   0										/**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(2000, UNIT_10_MS)			/**< Connection supervisory time-out (2 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER)	/**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (20 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)	/**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3										/**< Number of attempts before giving up the connection parameter negotiation. */

 /*--------------------------------------------------------------------
 * Application defines
 * -------------------------------------------------------------------- */
 // General app values
#define SMS_VERSION_ID					0x0101									/**< SMS Version:	MSB --> device ID (01 -> sensors, 02 -> remote, 03 -> central)
																									LSB --> version.subversion (2E -> 2.15) */
#define SMS_RELEASE_ID					001																									
#define SMS_DEVICE_NAME					"SMS_sensors"                             /**< Name of device. Will be included in the advertising data. */
#define BOOTLOADER_RESET_3MIN			(0x15C75ABE)							/**<Command to reset device and keep the bootloader active for 3 minutes */
#define BOOTLOADER_DFU_START			(0xB1)
#define DEAD_BEEF                       0xDEADBEEF								/**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
// LEDs/buttons
#define SMS_CONN_LED_PIN				BSP_BOARD_LED_0							/**< Is on when device is advertising. */
#define SMS_DATA_LED_PIN				BSP_BOARD_LED_1
#define SMS_BUTTON1_PIN					BSP_BUTTON_0							/**< Button that will trigger the notification event with the LED Button Service */
#define SMS_BUTTON2_PIN					BSP_BUTTON_1
#define SMS_BUTTON_DETECTION_DELAY		APP_TIMER_TICKS(5, APP_TIMER_PRESCALER)	/**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
#define SMS_BUTTON_LONG_PRESS_MS		3000
#define SMS_LED_BLINK_ULTRA_MS			80
#define SMS_LED_BLINK_FAST_MS			200
#define SMS_LED_BLINK_MEDIUM_MS			1400
#define SMS_LED_BLINK_SLOW_MS			10000
#define SMS_LED_BLINK_CONNECT			10
// Timers
#define APP_TIMER_PRESCALER             0										/**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            10										/**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         8										/**< Size of timer operation queues. */
// Batgauge/SAADC
#define SMS_BATGAUGE_SAMPLE_RATE_MS		1000									/* Time between each ADC sample */
#define ADC_SAMPLES_IN_BUFFER 			20										/* Number of ADC samples before a batgauge averaging is called */
#define BAT_CONV_ADC0					151
#define BAT_CONV_ADC100					165
#define BAT_CONV_DELTA					(100/(BAT_CONV_ADC100-BAT_CONV_ADC0))
#define BAT_CONV_OFFSET					(-BAT_CONV_DELTA * BAT_CONV_ADC0)



/* ====================================================================
 * VARIABLES
 * --------------------------------------------------------------------
 *
 * ==================================================================== */
// States
enum sms_running_states {
	SMS_OFF = 0,
	SMS_ADV_START,
	SMS_ADVERTISING,
	SMS_ADV_TIMEOUT,
	SMS_CONNECTING,
	SMS_CALIBRATING,
	SMS_RUNNING
};
enum led_states {
	LED_ON,
	LED_OFF,
	LED_ADVERTISING,
	LED_CONNECTING,
	LED_CONNECTED,
	LED_DISCONNECTED,
	LED_GAP_TIMEOUT,
	LED_RUNNING,
	LED_CALIB_ACCEL,
	LED_CALIB_COMP,
	LED_SWITCHING_OFF,
	LED_ERROR
};
struct batgauge_status_s {
	bool start;
	bool init_ok;
	bool enabled;
	bool new_value;
	uint32_t bat_level;
};
// Application state
typedef struct {
	enum sms_running_states sms;
	enum led_states led[2];
	struct batgauge_status_s batgauge;
}app_state_t;
app_state_t m_app_state;

// LED/buttons
static uint8_t 							m_button_mask = 0;							/* Button mask to remember previous pressing state */
static uint8_t							m_led_blink_cnt = 0;
// Batgauge/SAADC
static nrf_saadc_value_t     			m_buffer_pool[2][ADC_SAMPLES_IN_BUFFER];	/* Buffers to store n ADC samples (averaging) */
static uint32_t							m_adc_evt_counter;							/* ADC event counter (debug) */
// BLE/connection
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;	/* Handle of the current connection */
ble_smss_t								m_smss_service;								/* Structure used to identify the SMS service */
static ble_bas_t 						m_bas;                                   	/* Structure used to identify the battery service */


// Timers
APP_TIMER_DEF(pressure_poll_int_id);
APP_TIMER_DEF(imu_poll_int_id);
APP_TIMER_DEF(button_press_timer_id);
APP_TIMER_DEF(saadc_timer_id);
APP_TIMER_DEF(led_conn_timer_id);
APP_TIMER_DEF(led_data_timer_id);

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
void sensors_stop(void);

//#define DEBUG

/**@brief Application error handler overwitten.
 *
 * @param[in] id
 * @param[in] pc
 * @param[in] info
 */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
//NRF_LOG_INFO("ERROR! ID: %d, PC: %d, INFO: %d\n\r", id, pc, info);
#ifndef DEBUG
	NVIC_SystemReset();
#else
	NVIC_SystemReset();
//	while(1){};
#endif //DEBUG
}

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


/**@brief Blink both LEDs in alternance to signal switch-off procedure.
 *
 * @details For simplification reasons, this function uses a blocking delay. Since everything has
 *			already been disabled before (BLE, PWM, timers) this is not an issue.
 */
static void leds_blink_off(void)
{
	bsp_board_leds_off();

	bsp_board_led_on(SMS_DATA_LED_PIN);
	nrf_delay_ms(150);
	bsp_board_led_on(SMS_CONN_LED_PIN);
	nrf_delay_ms(500);
	bsp_board_led_off(SMS_CONN_LED_PIN);
	nrf_delay_ms(150);
	bsp_board_led_off(SMS_DATA_LED_PIN);
	
	bsp_board_leds_off();
}


/**@brief Switch off SMS device.
 *
 * @details	Stop BLE advertising or disconnect, and disable running tasks (PWM, timers).
 *			Only the buttons remain active (for restarting).
 *
* @param[in] restart	If restart is set, reinitialise everything and start advertising (TBD!).
 */
static void sms_switch_off(bool restart)
{
	uint32_t err_code;
	
	NRF_LOG_DEBUG("Switching-off SMS sensors...\n\r");
	
	bno055_interrupt.new_value = false;
	bno055_interrupt.rts = false;
	bno055_interrupt.enabled = false;
	
	ms58_interrupt.new_value = false;
	ms58_interrupt.rts = false;
	ms58_interrupt.enabled = false;
	
	m_app_state.batgauge.new_value = false;
	m_app_state.batgauge.enabled = false;
	
	if(m_app_state.sms == SMS_ADVERTISING) {
		err_code = sd_ble_gap_adv_stop();
		NRF_LOG_DEBUG("Adv stop err_code: 0x%04x\n\r", err_code);
		if(err_code != 0x0008) {
			APP_ERROR_CHECK(err_code);
		}
		m_app_state.sms = SMS_OFF;
	}
	else if(m_app_state.sms == SMS_RUNNING) {
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		NRF_LOG_DEBUG("Disconnect err_code: 0x%04x\n\r", err_code);
		if(!restart) {
			m_app_state.sms = SMS_OFF;
		}
	}
	else if(m_app_state.sms == SMS_ADV_TIMEOUT) {
		m_app_state.sms = SMS_OFF;
	}
	app_timer_stop_all();
	
	leds_blink_off();	
	
	m_conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Switch on SMS device.
 *
 * @details	Start advertising for 120 seconds.
 */
static void sms_switch_on(void)
{
	NRF_LOG_DEBUG("Switching-on SMS sensors...\n\r");
	advertising_start();
}


/* ====================================================================
 * HANDLERS
 * --------------------------------------------------------------------
 * Has to be defined first to be called by the init functions
 * ==================================================================== */
/**@brief Pressure polling handler.
 *
 * @details	Set the new value flag.
 *
 * @param[in] p_context	unused
 */
static void pressure_poll_int_handler(void * p_context)
{
	ms58_interrupt.new_value = true;
}

/**@brief IMU polling handler.
 *
 * @details	Set the new value flag.
 *
 * @param[in] p_context	unused
 */
static void imu_poll_int_handler(void * p_context)
{
	bno055_interrupt.new_value = true;
}

/**@brief Button pressed timeout handler.
 *
 * @details	1. check if and which button is pressed
 *			2. compare the current state with the button mask
 *			3. handle according to the comparison
 *
 * @param[in] p_context	unused
 */
static void button_press_timeout_handler(void * p_context)
{
	uint8_t button_state = 0;
	UNUSED_PARAMETER(p_context);
	
	if(app_button_is_pushed(0)) button_state |= 0x01;
	if(app_button_is_pushed(1)) button_state |= 0x10;
	NRF_LOG_DEBUG("Button long press timeout!! Button state = 0x%02x\n\r", button_state);

	if(button_state == m_button_mask) {
		if(button_state == 0x11) {
			NRF_LOG_DEBUG("DOUBLE long press!! SMS state: %d\n\r", m_app_state.sms);
			if((m_app_state.sms == SMS_ADVERTISING) || (m_app_state.sms == SMS_RUNNING)) {
				sms_switch_off(false);
			}
		}
		else if((button_state == 0x01) || (button_state == 0x10)) {
			NRF_LOG_DEBUG("SINGLE long press!! SMS state: %d\n\r", m_app_state.sms);
			if(m_app_state.sms == SMS_OFF) {
				sms_switch_on();
			}
		}
	}
	// Reset button mask to force complete button release before next detection
	m_button_mask = 0;
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
        case SMS_BUTTON1_PIN:
			if(button_action) send_value |= 0xFF;
			else send_value &= 0xFF00;
			NRF_LOG_DEBUG("Bt1 pressed\r\n", send_value);
			err_code = ble_smss_on_button_change(&m_smss_service, send_value);
			if((err_code != NRF_SUCCESS) &&							// 0x0000
				(err_code != NRF_ERROR_INVALID_STATE) &&			// 0x0008
				(err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&		// 0x3002
				(err_code != BLE_ERROR_NO_TX_PACKETS) &&			// 0x3004
				(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))		// 0x3401
			{
				APP_ERROR_CHECK(err_code);
			}
			
			if(button_action) m_button_mask |= 0x01;
			else m_button_mask &= 0xF0;
			
			NRF_LOG_DEBUG("Stopping button timer... mask = 0x%02x\n\r", m_button_mask);
			app_timer_stop(button_press_timer_id);
			if(m_button_mask != 0) {
				NRF_LOG_DEBUG("Re-starting button timer...\n\r");
				app_timer_start(button_press_timer_id,
					APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_BUTTON_LONG_PRESS_MS, UNIT_1_00_MS), 0),
					NULL);		
			}
			
            break;

		case SMS_BUTTON2_PIN:
			if(button_action) send_value |= 0xFF00;
			else send_value &= 0x00FF;
			NRF_LOG_DEBUG("Bt2 pressed\r\n", send_value);
			err_code = ble_smss_on_button_change(&m_smss_service, send_value);
			if((err_code != NRF_SUCCESS) &&							// 0x0000
				(err_code != NRF_ERROR_INVALID_STATE) &&			// 0x0008
				(err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&		// 0x3002
				(err_code != BLE_ERROR_NO_TX_PACKETS) &&			// 0x3004
				(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))		// 0x3401
			{
				APP_ERROR_CHECK(err_code);
			}

			if(button_action) m_button_mask |= 0x10;
			else m_button_mask &= 0x0F;
			
			NRF_LOG_DEBUG("Stopping button timer... mask = 0x%02x\n\r", m_button_mask);
			app_timer_stop(button_press_timer_id);
			if(m_button_mask != 0) {
				NRF_LOG_DEBUG("Re-starting button timer...\n\r");
				app_timer_start(button_press_timer_id,
					APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_BUTTON_LONG_PRESS_MS, UNIT_1_00_MS), 0),
					NULL);		
			}

			break;
		
        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}



/**@brief Function for handling an SPI event.
 *
 * @details	Set the xfer done flag.
 *
 * @param[in] p_event	unused
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
	UNUSED_PARAMETER(p_event);
    spi_xfer_done = true;
}
/**@brief Function for handling a TWI event.
 *
 * @details	Set the xfer done flag.
 *
 * @param[in] p_event	Select which TWI event has called the handler
 * @param[in] p_context	Unused
 */
void twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
	UNUSED_PARAMETER(p_context);
	
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
/**@brief Function for handling a batgauge event.
 *
 * @details	After ADC_SAMPLES_IN_BUFFER ADC sampling, the batgauge handler is called to calculate a
 *			battery level value and set the new value flag.
 *
 * @param[in] p_event	Select which SAADC event has called the handler
 */
void batgauge_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
	NRF_LOG_DEBUG("Batgauge event!!\n\r");
	if(p_event->type == NRF_DRV_SAADC_EVT_DONE)
	{
		ret_code_t err_code;
		err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, ADC_SAMPLES_IN_BUFFER);
		APP_ERROR_CHECK(err_code);
		
		uint8_t i;
		uint32_t sum = 0;
		NRF_LOG_DEBUG("ADC event number: %d\n\r", (int)m_adc_evt_counter);
		for(i = 0; i < ADC_SAMPLES_IN_BUFFER; i++)
		{
			m_buffer_pool[0][i] = p_event->data.done.p_buffer[i];
			NRF_LOG_DEBUG("%d\r\n", m_buffer_pool[0][i]);
			sum += m_buffer_pool[0][i];
		}
		float adc = (float)sum / ADC_SAMPLES_IN_BUFFER;
		float level = (float)BAT_CONV_DELTA * adc + (float)BAT_CONV_OFFSET;
		if(level > 100) level = 100;
		m_app_state.batgauge.bat_level = (uint32_t)level;
//		m_app_state.batgauge.bat_level = (uint32_t)adc;
		NRF_LOG_DEBUG("Batgauge avg: %d (sum: %d)\n\r", m_app_state.batgauge.bat_level, sum);
		m_adc_evt_counter++;
		m_app_state.batgauge.new_value = true;
	}
}

/**@brief Function for handling the saadc timer.
 *
 * @details	Start a new SAADC sampling sequence.
 *
 * @param[in] p_context	unused
 */
static void saadc_timer_handler(void * p_context)
{
	NRF_LOG_DEBUG("saadc timer done\n\r");
	nrf_drv_saadc_sample();
}

/**@brief Function for handling the PWM event for connection LED.
 *
 * @details	Depending on the PWM instance and the current LED state, toggle the
 *			PWM or switch it off.
 *
 * @param[in] p_context	Select the PWM instance which has called the handler.
 */
static void led_conn_handler(void)
{
	bool lstate = bsp_board_led_state_get(bsp_board_pin_to_led_idx(LED_1));
	NRF_LOG_INFO("LED0 timeout... app_state = %d, led state = %d\n\r", m_app_state.sms, lstate);
	switch(m_app_state.sms) {
		case SMS_ADVERTISING:
			if(m_app_state.led[0] == LED_ON) {
				m_app_state.led[0] = LED_OFF;
				bsp_board_led_off(SMS_CONN_LED_PIN);
				app_timer_start(led_conn_timer_id,
					APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_LED_BLINK_MEDIUM_MS, UNIT_1_00_MS), 0),
					NULL);
			}
			else {
				m_app_state.led[0] = LED_ON;
				bsp_board_led_on(SMS_CONN_LED_PIN);
				app_timer_start(led_conn_timer_id,
					APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_LED_BLINK_FAST_MS, UNIT_1_00_MS), 0),
					NULL);
			}
			break;
			
		case SMS_RUNNING:
			m_led_blink_cnt++;
			if(m_led_blink_cnt < SMS_LED_BLINK_CONNECT) {
				bsp_board_led_invert(SMS_CONN_LED_PIN);
				app_timer_start(led_conn_timer_id,
					APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_LED_BLINK_ULTRA_MS, UNIT_1_00_MS), 0),
					NULL);
			}
			else {
				NRF_LOG_DEBUG("Connect blinking done\n\r");
				bsp_board_led_off(SMS_CONN_LED_PIN);
				app_timer_stop(led_conn_timer_id);
				m_app_state.led[1] = LED_ON;
				bsp_board_led_on(SMS_DATA_LED_PIN);
				app_timer_start(led_data_timer_id,
					APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_LED_BLINK_ULTRA_MS, UNIT_1_00_MS), 0),
					NULL);
				ms58_config.dev_start = true;
				bno055_config.dev_start = true;
				m_app_state.batgauge.start = true;
			}
			break;
			
		default:
			break;
	}
//	static uint16_t change_cnt;
//	static uint8_t blink_cnt;
//	uint32_t max_changes = 0;
//	uint8_t max_blinks = 0;
//	
//	uint32_t err_code;

//	low_power_pwm_t * pwm_instance = (low_power_pwm_t*)p_context;
//	
//	// Commands for LED_0
//	if(pwm_instance->bit_mask == BSP_LED_0_MASK)
//	{
//		switch(m_app_state.led[0]) {
//			case LED_ADVERTISING:
//				max_changes = SMS_LED_BLINK_MEDIUM_TICKS;
//				max_blinks = 0;
//				break;
//			case LED_CONNECTING:
//				max_changes = SMS_LED_BLINK_ULTRA_TICKS;
//				max_blinks = 4;
//				break;
//			default:
//				break;
//		}

//		change_cnt++;
//		
//		if(change_cnt > max_changes) {
//			switch(m_app_state.led[0]) {
//				case LED_ADVERTISING:
//					if(pwm_instance->duty_cycle > 0) {
//						err_code = low_power_pwm_duty_set(pwm_instance, 0);
//					}
//					else {
//						err_code = low_power_pwm_duty_set(pwm_instance, SMS_LED_ON_DUTY);
//					}
//					APP_ERROR_CHECK(err_code);
//					break;
//					
//				case LED_CONNECTING:
//					if(pwm_instance->duty_cycle > 0) {
//						blink_cnt++;
//						err_code = low_power_pwm_duty_set(pwm_instance, 0);
//					}
//					else {
//						err_code = low_power_pwm_duty_set(pwm_instance, SMS_LED_ON_DUTY);
//					}
//					APP_ERROR_CHECK(err_code);
//					
//					if(blink_cnt > max_blinks) {
//						blink_cnt = 0;
//						m_app_state.pwm[0] = PWM_OFF;
//						m_app_state.led[0] = LED_CONNECTED;
//						m_app_state.sms = SMS_RUNNING;
//						ms58_config.dev_start = true;
//						bno055_config.dev_start = true;
//						m_app_state.batgauge.start = true;
//					}
//					break;
//						
//				default:
//					break;
//			}

//			change_cnt = 0;
//		}
//	}
}

/**@brief Function for handling the PWM event for data LED.
 *
 * @details	Depending on the PWM instance and the current LED state, toggle the
 *			PWM or switch it off.
 *
 * @param[in] p_context	Select the PWM instance which has called the handler.
 */
static void led_data_handler(void)
{
	if(m_app_state.led[1] == LED_ON) {
		m_app_state.led[1] = LED_OFF;
		bsp_board_led_off(SMS_DATA_LED_PIN);
		app_timer_start(led_data_timer_id,
			APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_LED_BLINK_SLOW_MS, UNIT_1_00_MS), 0),
			NULL);
	}
	else{
		m_app_state.led[1] = LED_ON;
		bsp_board_led_on(SMS_DATA_LED_PIN);
		app_timer_start(led_data_timer_id,
			APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_LED_BLINK_ULTRA_MS, UNIT_1_00_MS), 0),
			NULL);
	}
//	static uint16_t change_cnt;
//	static uint8_t duty_cnt;
//	uint32_t max_changes = 0;
//	uint8_t max_duty = 0;
//	
//	uint32_t err_code;
//	
//	low_power_pwm_t * pwm_instance = (low_power_pwm_t*)p_context;

//	switch(m_app_state.led[1]) {
//		case LED_RUNNING:
//			max_changes = SMS_LED_BLINK_FAST_TICKS;
//			max_duty = 100/SMS_LED_RUNNING_DUTY;
//			break;
//		case LED_CALIB_ACCEL:
//			max_changes = SMS_LED_BLINK_MEDIUM_TICKS;
//			break;
//		default:
//			break;
//	}
//	
//	change_cnt++;
//		
//	if(change_cnt > max_changes) {
//		switch(m_app_state.led[1]) {
//			case LED_RUNNING:
//				if(pwm_instance->duty_cycle > 0) {
//					duty_cnt = 0;
//					err_code = low_power_pwm_duty_set(pwm_instance, 0);
//				}
//				else {
//					duty_cnt++;
//					if(duty_cnt > max_duty) {
//						err_code = low_power_pwm_duty_set(pwm_instance, SMS_LED_ON_DUTY);
//					}
//					else {
//						err_code = low_power_pwm_duty_set(pwm_instance, 0);
//					}
//				}
//				APP_ERROR_CHECK(err_code);
//				break;
//				
//			case LED_CALIB_ACCEL:
//				if(pwm_instance->duty_cycle > 0) {
//					err_code = low_power_pwm_duty_set(pwm_instance, 0);
//				}
//				else {
//					err_code = low_power_pwm_duty_set(pwm_instance, SMS_LED_ON_DUTY);
//				}
//				APP_ERROR_CHECK(err_code);
//				break;
//					
//			case LED_DISCONNECTED:
//				m_app_state.pwm[1] = PWM_OFF;
//				break;
//			
//			default:
//				break;
//		}

//		change_cnt = 0;
//	}
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
//    uint32_t err_code;
	
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
//            break;
        case BLE_ADV_EVT_IDLE:
//            sleep_mode_enter();
//            break;
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
	NRF_LOG_DEBUG("On BLE event: 0x%04x\n\r", p_ble_evt->header.evt_id);
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
			NRF_LOG_DEBUG("Connected\r\n");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			m_app_state.sms = SMS_CONNECTING;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
			NRF_LOG_DEBUG("Disconnected\r\n");
			sensors_stop();
			if(m_app_state.sms == SMS_RUNNING) {
				m_app_state.sms = SMS_ADV_START;
			}

            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST
		
		case BLE_GAP_EVT_TIMEOUT:
			NRF_LOG_DEBUG("GAP Event Timeout.\r\n");
			m_app_state.led[0] = LED_GAP_TIMEOUT;
			m_app_state.sms = SMS_ADV_TIMEOUT;
			sms_switch_off(false);
			break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
			NRF_LOG_INFO("GATT Client Timeout.\r\n");
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
                    auth_reply.params.write.gatt_status = SMS_FEATURE_NOT_SUPPORTED;
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
    ble_smss_on_ble_evt(&m_smss_service, p_ble_evt);
	ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
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
//	uint32_t err_code;
//	low_power_pwm_config_t low_power_pwm_config;
//	low_power_pwm_config.active_high = false;
//	low_power_pwm_config.period = SMS_PWM_PERIOD;
//	low_power_pwm_config.bit_mask = BSP_LED_0_MASK;
//	low_power_pwm_config.p_timer_id = &led_conn_timer_id;
//	low_power_pwm_config.p_port = NRF_GPIO;
//	
//	m_app_state.pwm[0] = PWM_OFF;
//	err_code = low_power_pwm_init((&low_power_pwm_conn), &low_power_pwm_config, pwm_conn_handler);
//	APP_ERROR_CHECK(err_code);
//	err_code = low_power_pwm_duty_set(&low_power_pwm_conn, SMS_LED_ON_DUTY);
//	APP_ERROR_CHECK(err_code);
//	
//	low_power_pwm_config.active_high = false;
//	low_power_pwm_config.period = SMS_PWM_PERIOD;
//	low_power_pwm_config.bit_mask = BSP_LED_1_MASK;
//	low_power_pwm_config.p_timer_id = &led_data_timer_id;
//	low_power_pwm_config.p_port = NRF_GPIO;
//	
//	m_app_state.pwm[1] = PWM_OFF;
//	err_code = low_power_pwm_init((&low_power_pwm_data), &low_power_pwm_config, pwm_data_handler);
//	APP_ERROR_CHECK(err_code);
//	err_code = low_power_pwm_duty_set(&low_power_pwm_data, SMS_LED_ON_DUTY);
//	APP_ERROR_CHECK(err_code);
}

/**@brief Function to initialize the sensors' power supplies ports.
 *
 * @details Set up supply ports to output and to low
 */
static void supplies_init(void)
{
//	NRF_LOG_INFO("Preparing supply ports as outputs\r\n");
	nrf_gpio_cfg_output(SMS_PRESSURE_SUPPLY_PIN);
	nrf_gpio_pin_write(SMS_PRESSURE_SUPPLY_PIN, SMS_PRESSURE_SW_OFF);
	nrf_gpio_cfg_output(SMS_IMU_SUPPLY_PIN);
	nrf_gpio_pin_write(SMS_IMU_SUPPLY_PIN, SMS_IMU_SW_OFF);
}

/**@brief Function to initialize the battery gauge.
 *
 * @details Configure the SAADC channel and the sample buffer. But DON'T START!
 */
static void batgauge_init(void)
{
	NRF_LOG_DEBUG("Preparing AIN0 as battery level input\r\n");
	ret_code_t err_code;
	nrf_saadc_channel_config_t channel_config =
		NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
	
	err_code = nrf_drv_saadc_init(NULL, batgauge_event_handler);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_saadc_channel_init(0, &channel_config);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], ADC_SAMPLES_IN_BUFFER);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], ADC_SAMPLES_IN_BUFFER);
	APP_ERROR_CHECK(err_code);
	
	m_app_state.batgauge.init_ok = true;
	m_app_state.batgauge.enabled = false;
	m_app_state.batgauge.start = false;
	m_app_state.batgauge.new_value = false;
}




/**@brief Function for initialising the button handler module.
 */
static void buttons_init(void)
{
    uint32_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {SMS_BUTTON1_PIN, false, BUTTON_PULL, button_event_handler},
		{SMS_BUTTON2_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               SMS_BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initialising the SPI module.
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

/**@brief Function for initialising the TWI module.
 *
 * @details	Enable module after initialization.
 */
void twi_init(void)
{
	ret_code_t err_code;
	const nrf_drv_twi_config_t twi_config = {
		.scl				= TWI_SCL_PIN,
		.sda				= TWI_SDA_PIN,
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
 * @details Create the timer modules. BUT DON'T START!
 */
static void timers_init(void)
{
	uint32_t err_code;

    // Initialize application timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	
	// Create timers...
	// - pressure polling
	err_code = app_timer_create(&pressure_poll_int_id,
								APP_TIMER_MODE_REPEATED,
								pressure_poll_int_handler);
	APP_ERROR_CHECK(err_code);

	// - IMU polling
	err_code = app_timer_create(&imu_poll_int_id,
								APP_TIMER_MODE_REPEATED,
								imu_poll_int_handler);
	APP_ERROR_CHECK(err_code);
	
	// - button long pressing timer
	err_code = app_timer_create(&button_press_timer_id,
								APP_TIMER_MODE_SINGLE_SHOT,
								button_press_timeout_handler);
	APP_ERROR_CHECK(err_code);
	
	// - SAADC polling
	err_code = app_timer_create(&saadc_timer_id,
								APP_TIMER_MODE_REPEATED,
								saadc_timer_handler);
	APP_ERROR_CHECK(err_code);

	// - LEDs blinking timers
	err_code = app_timer_create(&led_conn_timer_id,
								APP_TIMER_MODE_SINGLE_SHOT,
								(app_timer_timeout_handler_t)led_conn_handler);
	APP_ERROR_CHECK(err_code);
	err_code = app_timer_create(&led_data_timer_id,
								APP_TIMER_MODE_SINGLE_SHOT,
								(app_timer_timeout_handler_t)led_data_handler);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting the bootloader for 3 minutes
 *
 * @details	Set the new value in GPREGRET, disable sd and reset
 *
 * @param[in] conn_handle	The current BLE connection handle needed to disconnect.
 */
static void bootloader_start(uint16_t conn_handle)
{
	uint32_t err_code;
	
	/* Force disconnect, disable softdevice, and then reset */
	sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	// The below requires at least bootloader 3.1
//	err_code = sd_power_gpregret_set(BOOTLOADER_DFU_START, 0x000000FF);
	err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
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
                                          (const uint8_t *)SMS_DEVICE_NAME,
                                          strlen(SMS_DEVICE_NAME));
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
    options.ble_adv_fast_interval = SMS_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = SMS_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the reception of a write command.
 *
 * @details	Concatenate the received data into a 32bit value and check
 * 			check for the corresponding command.
 *
 * @param[in] p_smss	SMS service struct containing the connection handle
 * @param[in] data		Pointer to the command data
 */
static void app_update_function(ble_smss_t * p_smss, uint8_t *data)
{
	uint32_t command =	(data[0] + (data[1] << 8) +
						(data[2] << 16) + (data[3] << 24));
	NRF_LOG_DEBUG("Received app update command: %#x\n\r", command);
	if(command == 0x1c57b007) {
		NRF_LOG_DEBUG("Restarting device with 3 min bootloader...\n\r");
		bootloader_start(p_smss->conn_handle);
	}
	else if(command == 0x1c57ca1b) {
		NRF_LOG_DEBUG("Received compass calibration command\n\r");
		bno055_interrupt.enabled = false;
		bno055_interrupt.new_value = false;
		bno055_interrupt.rts = false;
		bsp_board_leds_on();
		bno055_calibrate_accel_gyro(bno055_config.accel_bias, bno055_config.gyro_bias);
		bsp_board_led_off(SMS_CONN_LED_PIN);
		bno055_calibrate_mag(bno055_config.mag_bias);
		nrf_delay_ms(1000);
		imu_check_cal();
		NRF_LOG_DEBUG("System calibration: %d\n\r",
					((0xC0 & bno055_config.cal_state) >> 6));
		NRF_LOG_DEBUG("Gyro   calibration: %d\n\r",
					((0x30 & bno055_config.cal_state) >> 4));
		NRF_LOG_DEBUG("Accel  calibration: %d\n\r",
					((0x0C & bno055_config.cal_state) >> 2));
		NRF_LOG_DEBUG("Mag    calibration: %d\n\r",
					((0x03 & bno055_config.cal_state) >> 0));
		bno055_config.dev_calib_done = true;
	}
	else if(command == 0x11223344) {
		NRF_LOG_DEBUG("Closed loop latency test\n\r");
		ble_smss_on_button_change(&m_smss_service, 0xFFFF);
	}
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	uint32_t err_code;
	
	ble_smss_init_t init;
	init.app_update_function = app_update_function;
	ble_smss_init(&m_smss_service, &init);
	 
    ble_bas_init_t bas_init;

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

	// Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
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


/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    uint32_t             err_code;

	NRF_LOG_INFO("Starting advertising for %d seconds\n\r", SMS_ADV_TIMEOUT_IN_SECONDS);

	m_app_state.sms = SMS_ADVERTISING;
	m_app_state.led[0] = LED_ON;
	bsp_board_led_on(SMS_CONN_LED_PIN);
	err_code = app_timer_start(led_conn_timer_id,
		APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_LED_BLINK_FAST_MS, UNIT_1_00_MS), 0),
		NULL);
	
	err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(err_code);
}


/* ====================================================================
 * RUN-TIME FUNCTIONS
 * --------------------------------------------------------------------
 *
 * ==================================================================== */
/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}


/**@brief Stop all sensors.
 *
 * @details	1. switch off the power supplies (pressure, imu)
 *			2. stop the polling timer (pressure, imu, batgauge)
 *			3. clear the enabled flag for all sensors
 */
void sensors_stop(void)
{
	NRF_LOG_DEBUG("Stopping sensors...\n\r");
	NRF_LOG_DEBUG("...switch off supply\n\r");
	nrf_gpio_pin_write(SMS_PRESSURE_SUPPLY_PIN, SMS_PRESSURE_SW_OFF);
	nrf_gpio_pin_write(SMS_IMU_SUPPLY_PIN, SMS_IMU_SW_OFF);
	
	NRF_LOG_DEBUG("...polling timers\n\r");
	app_timer_stop(pressure_poll_int_id);
	app_timer_stop(imu_poll_int_id);
	app_timer_stop(saadc_timer_id);
	
	ms58_interrupt.enabled = false;
	bno055_interrupt.enabled = false;
	m_app_state.batgauge.enabled = false;
}

/**@brief Start the button push timer in order to detect a long push.
 */
void button_press_timer_start(void)
{
	app_timer_start(button_press_timer_id,
					APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_BUTTON_LONG_PRESS_MS, UNIT_1_00_MS), 0),
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
    
    // Setup log
//	err_code = NRF_LOG_INIT(NULL);
//	APP_ERROR_CHECK(err_code);
//	uint8_t fw_msb, fw_lsb;
//	fw_msb = ((SMS_VERSION_ID & 0xFF) > 8);
//	fw_lsb = (SMS_VERSION_ID & 0x0F);
//	NRF_LOG_INFO("===============================\n\r");
//	NRF_LOG_INFO("SMS sensors firmware v%d.%d, r%03d\n\r", fw_msb, fw_lsb, SMS_RELEASE_ID);
//	NRF_LOG_INFO("===============================\n\n\r");

	// Initialize hardware & services
    timers_init();
    leds_init();
	supplies_init();
    buttons_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

	spi_init();
	twi_init();
	batgauge_init();
	
	// Initialize & configure peripherals
	ms58_config.dev_start = false;
	bno055_config.dev_start = false;
//	bno055_config.dev_calib_done = false;
//	m_app_state.batgauge.start = false;
//	m_app_state.batgauge.new_value = false;
	
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
	
	// Start advertising
	m_app_state.sms = SMS_ADV_START;

    // Enter main loop.
    for (;;)
    {
		switch(m_app_state.sms) {
			case SMS_ADV_START:
				bsp_board_led_off(SMS_DATA_LED_PIN);
				app_timer_stop(led_data_timer_id);
				advertising_start();
				break;
			
			case SMS_CONNECTING:
				app_timer_stop(led_conn_timer_id);
				m_app_state.sms = SMS_RUNNING;
				m_led_blink_cnt = 0;
				bsp_board_led_invert(SMS_CONN_LED_PIN);
				app_timer_start(led_conn_timer_id,
					APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_LED_BLINK_ULTRA_MS, UNIT_1_00_MS), 0),
					NULL);
				break;
			
			default:
				break;
		}
		

		// Start flag of the pressure (MS58) sensor
		if(ms58_config.dev_start)
		{
			nrf_gpio_pin_write(SMS_PRESSURE_SUPPLY_PIN, SMS_PRESSURE_SW_ON);
			pressure_enable();
			ms58_config.dev_en = true;
			app_timer_start(pressure_poll_int_id,
							APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_PRESSURE_POLL_MS, UNIT_1_00_MS), 0),
							NULL);
			ms58_interrupt.enabled = true;
			ms58_config.dev_start = false;
		}
		
		// Start flag of the IMU (BNO055) sensor
		if(bno055_config.dev_start)
		{
			nrf_gpio_pin_write(SMS_IMU_SUPPLY_PIN, SMS_IMU_SW_ON);
			imu_enable();
			bno055_config.dev_en = true;
			app_timer_start(imu_poll_int_id,
							APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_IMU_POLL_MS, UNIT_1_00_MS), 0),
							NULL);
			bno055_interrupt.enabled = true;
			bno055_config.dev_start = false;
		}
		
		// Start flag of the battery gauge (SAADC)
		if(m_app_state.batgauge.start)
		{
			nrf_drv_saadc_sample();
			app_timer_start(saadc_timer_id,
							APP_TIMER_TICKS(MSEC_TO_UNITS(SMS_BATGAUGE_SAMPLE_RATE_MS, UNIT_1_00_MS), 0),
							NULL);
			m_app_state.batgauge.enabled = true;
			m_app_state.batgauge.start = false;
		}

		// BNO055 calibration has been done, polling can continue
		if(bno055_config.dev_calib_done) {
			m_app_state.led[1] = LED_RUNNING;
			m_app_state.sms = SMS_RUNNING;
			bno055_interrupt.enabled = true;
			bno055_config.dev_calib_done = false;
		}
		
		// New value flag of the pressure sensor
		if((ms58_interrupt.enabled) && (ms58_interrupt.new_value))
		{
//			nrf_gpio_pin_write(DBG1_PIN, 1);
			ms58_interrupt.new_value = false;
			pressure_poll_data();
//			nrf_gpio_pin_write(DBG1_PIN, 0);
		}
		
		// New value flag of the IMU
		if((bno055_interrupt.enabled) && (bno055_interrupt.new_value))
		{
//			nrf_gpio_pin_write(DBG2_PIN, 1);
//			NRF_LOG_INFO("BNO055 interrupt\n\r");
			bno055_interrupt.new_value = false;
			imu_poll_data(SMS_IMU_DATAMSK_QUAT);
//			nrf_gpio_pin_write(DBG2_PIN, 0);
		}
		
		// New value flag of the battery gauge... sends notification automatically
		if(m_app_state.batgauge.new_value)
		{
			m_app_state.batgauge.new_value = false;
			uint32_t err_code;
			err_code = ble_bas_battery_level_update(&m_bas, (uint8_t)m_app_state.batgauge.bat_level);
			if((err_code != NRF_SUCCESS) &&							// 0x0000
				(err_code != NRF_ERROR_INVALID_STATE) &&			// 0x0008
				(err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&		// 0x3002
				(err_code != BLE_ERROR_NO_TX_PACKETS) &&			// 0x3004
				(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))		// 0x3401
			{
				APP_ERROR_HANDLER(err_code);
			}
		}
		
		// RTS (ready-to-send) for ms58
		if((ms58_interrupt.enabled) && (ms58_interrupt.rts))
		{
			ms58_interrupt.rts = false;
			int32_t * tosend;
			tosend = &ms58_output.pressure;
			err_code = ble_smss_on_press_value(&m_smss_service, tosend);
			if((err_code != NRF_SUCCESS) &&							// 0x0000
				(err_code != NRF_ERROR_INVALID_STATE) &&			// 0x0008
				(err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&		// 0x3002
				(err_code != BLE_ERROR_NO_TX_PACKETS) &&			// 0x3004
				(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))		// 0x3401
			{
				APP_ERROR_CHECK(err_code);
			}
		}
		

		// RTS (ready-to-send) for BNO055
		if((bno055_interrupt.enabled) && (bno055_interrupt.rts))
		{
			bno055_interrupt.rts = false;
			uint32_t * tosend;
			tosend = (uint32_t*)&bno055_output.quat[0].b;
			err_code = ble_smss_on_imu_value(&m_smss_service, tosend);
			if((err_code != NRF_SUCCESS) &&							// 0x0000
				(err_code != NRF_ERROR_INVALID_STATE) &&			// 0x0008
				(err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&		// 0x3002
				(err_code != BLE_ERROR_NO_TX_PACKETS) &&			// 0x3004
				(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))		// 0x3401
			{
				APP_ERROR_CHECK(err_code);
			}
		}
		
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }

	}
}


/**
 * @}
 */
