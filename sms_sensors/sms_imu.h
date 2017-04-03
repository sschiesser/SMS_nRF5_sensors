#ifndef SMS_IMU_H_
#define SMS_IMU_H_

#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
#include "softdevice_handler.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_drv_timer.h"
#include "SEGGER_RTT.h"
#include "bno055.h"

#define PI					(3.1415926535897932384626433832795)

int bno055_check(void);
int bno055_test(void);
void bno055_calibrate(float *dest1, float *dest2);
//void mpu9250_calibrate(float *dest1, float *dest2);
//void mpu9250_initialize(void);
//int mpu9250_comp_check(void);
//void mpu9250_comp_initialize(float *destination);
//int mpu9250_poll_data(void);

//void read_accel_data(int16_t *destination);
//void read_gyro_data(int16_t *destination);
//void read_comp_data(int16_t *destination);
//int16_t read_temp_data(void);
//float get_Mres(uint8_t m_scale);
//float get_Gres(uint8_t g_scale);
//float get_Ares(uint8_t a_scale);

void twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void twi_init(void);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);

void madgwick_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
void mahony_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

#endif
