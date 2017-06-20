#ifndef SMS_IMU_H_
#define SMS_IMU_H_

#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
//#include "softdevice_handler.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
//#include "app_timer.h"
#include "nrf_drv_timer.h"
//#include "SEGGER_RTT.h"
#include "bno055.h"

#define SMS_IMU_CHAR_LEN		16

#define SMS_IMU_DATAMSK_GRV		(1 << 6)
#define SMS_IMU_DATAMSK_LIA		(1 << 5)
#define SMS_IMU_DATAMSK_EULER	(1 << 4)
#define SMS_IMU_DATAMSK_QUAT	(1 << 3)
#define SMS_IMU_DATAMSK_MAG		(1 << 2)
#define SMS_IMU_DATAMSK_GYRO	(1 << 1)
#define SMS_IMU_DATAMSK_ACCEL	(1 << 0)

#define SMS_IMU_POLL_MS			19


#define PI						(3.1415926535897932384626433832795)

extern bool appDebug;

//void bno055_reset(void);
//int bno055_check(void);
void imu_startup(void);
void imu_configure(void);
void imu_check_cal(void);
void imu_initialize(void);
// IMU data mask:
// bit:  7  -  6  -  5  -   4   -   3  -  2  -   1  -   0 
// val: N/A - grv - lia - euler - quat - mag - gyro - accel
void imu_poll_data(uint8_t data_msk);


//int bno055_test(void);
//void bno055_init_config_values(void);
//void bno055_calibrate_accel_gyro(float *dest1, float *dest2);
//void bno055_calibrate_mag(float *dest);
void bno055_int_reset(void);

//void read_accel_data(int16_t *destination);
//void read_gyro_data(int16_t *destination);
//int16_t read_gyrotemp_data(void);
//void read_mag_data(int16_t *destination);
//void read_quat_data(int16_t * destination);
//void read_euler_data(int16_t * destination);
//void read_lia_data(int16_t * destination);
//void read_grv_data(int16_t * destination);

void twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void twi_init(void);
//void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
//uint8_t readByte(uint8_t address, uint8_t subAddress);
//void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);

void madgwick_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
void mahony_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

#endif
