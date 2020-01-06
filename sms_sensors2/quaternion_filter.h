#ifndef QUATERNION_FILTER_H_
#define QUATERNION_FILTER_H_

#include <stdint.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "sms_imu.h"

void madgwick_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void mahony_quaternion_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

#endif
