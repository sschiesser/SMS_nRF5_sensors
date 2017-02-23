#ifndef _SMS_SENSORS_H_
#define _SMS_SENSORS_H_

#define SMS_S_UUID_BASE				{0x1C, 0x57, 0x5A, 0xBE, 0x03, 0x53, 0x55, 0x55, \
																		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#define SMS_S_UUID_BUTTON_SERVICE	0x1523
#define SMS_S_UUID_PRESS_SERVICE	0x1524
#define SMS_S_UUID_IMU_SERVICE		0x1525
#define SMS_S_UUID_BUTTON_CHAR 		0x1623
#define SMS_S_UUID_PRESS_CHAR		0x1624
#define SMS_S_UUID_IMU_CHAR			0x1625


typedef struct sms_sensors_s {
	ble_lbs_t	sms_lbs_t;
}sms_sensors_t;

typedef void (*ble_lbs_led_write_handler_t) (sms_sensors_t.sms_lbs_t * p_lbs, uint8_t new_state);

#endif