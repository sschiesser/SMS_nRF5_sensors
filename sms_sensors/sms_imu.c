#include "sms_imu.h"

#define TWI_INSTANCE 1
const nrf_drv_twi_t twi_master_instance = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);
volatile bool twi_xfer_done;

struct bno055_config_s bno055_config;
struct bno055_output_s bno055_output;
struct bno055_interrupt_s bno055_interrupt;

extern const nrf_drv_timer_t TIMER_DELTA_US;

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

void bno055_reset(void)
{
	writeByte(BNO055_ADDRESS, BNO055_SYS_TRIGGER, 0x20);
}


int bno055_check(void)
{
	int ret = 0x0F;
	uint8_t c = readByte(BNO055_ADDRESS, BNO055_CHIP_ID);
	SEGGER_RTT_printf(0, "Device returned 0x%02x\n", c);
	if(c == 0xA0) ret = 0x07;
	nrf_delay_ms(500);
	c = readByte(BNO055_ADDRESS, BNO055_ACC_ID);
	SEGGER_RTT_printf(0, "Accelerometer returned 0x%02x\n", c);
	if(c == 0xFB) ret = 0x03;
	nrf_delay_ms(500);
	c = readByte(BNO055_ADDRESS, BNO055_MAG_ID);
	SEGGER_RTT_printf(0, "Magnetometer returned 0x%02x\n", c);
	if(c == 0x32) ret = 0x01;
	nrf_delay_ms(500);
	c = readByte(BNO055_ADDRESS, BNO055_GYRO_ID);
	SEGGER_RTT_printf(0, "Gyroscope returned 0x%02x\n", c);
	if(c == 0x0F) ret = 0;
	
	return ret;
}

int bno055_test(void)
{
	// Check software revision ID
	uint8_t swlsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_LSB);
	uint8_t swmsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_MSB);
	SEGGER_RTT_printf(0, "Software revision ID: %02x.%02x\n", swmsb, swlsb);
	
	// Check bootloader version
	uint8_t blid = readByte(BNO055_ADDRESS, BNO055_BL_REV_ID);
	SEGGER_RTT_printf(0, "Bootloader version: %d\n", blid);
	
	// Check self-test results
	uint8_t selftest = readByte(BNO055_ADDRESS, BNO055_ST_RESULT);
	if(selftest & 0x01) {
		SEGGER_RTT_printf(0, "Accelerometer passed selftest\n");
	}
	if(selftest & 0x02) {
		SEGGER_RTT_printf(0, "Magnetometer passed selftest\n");
	}
	if(selftest & 0x04) {
		SEGGER_RTT_printf(0, "Gyroscope passed selftest\n");
	}
	if(selftest & 0x08) {
		SEGGER_RTT_printf(0, "MCU passed selftest\n");
	}
}




void bno055_init_config_values(void)
{
	bno055_config.g_pwr_mode = NormalG;	// Gyro power mode
	bno055_config.g_scale = GFS_250DPS;	// Gyro full scale
	bno055_config.g_bw = GBW_523Hz;		// Gyro bandwidth
	bno055_config.a_pwr_mode = NormalA;	// Accel power mode
	bno055_config.a_scale = AFS_2G;		// Accel full scale
	bno055_config.a_bw = ABW_1000Hz;	// Accel bandwidth, accel sample rate divided by ABW_divx
	bno055_config.m_pwr_mode = Normal;	// Select magnetometer power mode
	bno055_config.m_op_mode = Regular;	// Select magnetometer perfomance mode
	bno055_config.m_odr = MODR_30Hz;	// Select magnetometer ODR when in BNO055 bypass mode
	bno055_config.pwr_mode = Normalpwr;	// Select BNO055 power mode
	bno055_config.opr_mode = NDOF;		// specify operation mode for sensors
	SEGGER_RTT_printf(0, "BNO055 config values initialized\n");
}

void bno055_calibrate_accel_gyro(float *dest1, float *dest2)
{
	uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii = 0, sample_count = 0;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	SEGGER_RTT_printf(0, "Accel/Gyro Calibration: Put device on a level surface and keep motionless! Wait......");
	nrf_delay_ms(4000);
  
	// Select page 0 to read sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
	// Select BNO055 system operation mode as AMG for calibration
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	nrf_delay_ms(25);
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, AMG);
   
	// In NDF fusion mode, accel full scale is at +/- 4g, ODR is 62.5 Hz, set it the same here
	writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, bno055_config.a_pwr_mode << 5 | bno055_config.a_bw << 2 | AFS_4G );
	sample_count = 256;
	for(ii = 0; ii < sample_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0};
		readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
		accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ; // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
		accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
		accel_bias[0]  += (int32_t) accel_temp[0];
		accel_bias[1]  += (int32_t) accel_temp[1];
		accel_bias[2]  += (int32_t) accel_temp[2];
		nrf_delay_ms(20);  // at 62.5 Hz ODR, new accel data is available every 16 ms
	}
	accel_bias[0]  /= (int32_t) sample_count;  // get average accel bias in mg
	accel_bias[1]  /= (int32_t) sample_count;
	accel_bias[2]  /= (int32_t) sample_count;
    
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) 1000;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) 1000;}

	dest1[0] = (float) accel_bias[0];  // save accel biases in mg for use in main program
	dest1[1] = (float) accel_bias[1];  // accel data is 1 LSB/mg
	dest1[2] = (float) accel_bias[2];          

	// In NDF fusion mode, gyro full scale is at +/- 2000 dps, ODR is 32 Hz
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, bno055_config.g_bw << 3 | GFS_2000DPS );
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, bno055_config.g_pwr_mode);
	
	for(ii = 0; ii < sample_count; ii++) {
		int16_t gyro_temp[3] = {0, 0, 0};
		readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
		gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;  // Form signed 16-bit integer for each sample in FIFO
		gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
		gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
		nrf_delay_ms(35);  // at 32 Hz ODR, new gyro data available every 31 ms
	}
	gyro_bias[0]  /= (int32_t) sample_count;  // get average gyro bias in counts
	gyro_bias[1]  /= (int32_t) sample_count;
	gyro_bias[2]  /= (int32_t) sample_count;

	dest2[0] = (float) gyro_bias[0]/16.;  // save gyro biases in dps for use in main program
	dest2[1] = (float) gyro_bias[1]/16.;  // gyro data is 16 LSB/dps
	dest2[2] = (float) gyro_bias[2]/16.;          

	// Return to config mode to write accelerometer biases in offset register
	// This offset register is only used while in fusion mode when accelerometer full-scale is +/- 4g
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	nrf_delay_ms(25);

	//write biases to accelerometer offset registers ad 16 LSB/dps
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB, (int16_t)accel_bias[0] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB, ((int16_t)accel_bias[0] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB, (int16_t)accel_bias[1] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB, ((int16_t)accel_bias[1] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB, (int16_t)accel_bias[2] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB, ((int16_t)accel_bias[2] >> 8) & 0xFF);

	// Check that offsets were properly written to offset registers
//	SEGGER_RTT_printf(0, "Average accelerometer bias = %d, %d, %d\n",\
//						(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB)),\
//						(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB)),\
//						(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB)));

	//write biases to gyro offset registers
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB, (int16_t)gyro_bias[0] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB, ((int16_t)gyro_bias[0] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB, (int16_t)gyro_bias[1] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB, ((int16_t)gyro_bias[1] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB, (int16_t)gyro_bias[2] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB, ((int16_t)gyro_bias[2] >> 8) & 0xFF);

	// Select BNO055 system operation mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, bno055_config.opr_mode );

	// Check that offsets were properly written to offset registers
//	SEGGER_RTT_printf(0, "Average gyro bias = %d, %d, %d\n",\
//						(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB)),\
//						(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB)),\
//						(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB)));

	SEGGER_RTT_printf(0, "Accel/Gyro Calibration done!\n");
}

void bno055_calibrate_mag(float *dest1)
{
	uint8_t data[6]; // data array to hold mag x, y, z, data
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0};
	int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

	SEGGER_RTT_printf(0, "Mag Calibration: Wave device in a figure eight until done!");
	nrf_delay_ms(4000);

	// Select page 0 to read sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
	// Select BNO055 system operation mode as NDOF for calibration
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	nrf_delay_ms(25);
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, AMG );

	// In NDF fusion mode, mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
	sample_count = 256;
	for(ii = 0; ii < sample_count; ii++) {
		int16_t mag_temp[3] = {0, 0, 0};
		readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
		mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
		mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
		mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
		for (int jj = 0; jj < 3; jj++) {
			if (ii == 0) {
				mag_max[jj] = mag_temp[jj]; // Offsets may be large enough that mag_temp[i] may not be bipolar! 
				mag_min[jj] = mag_temp[jj]; // This prevents max or min being pinned to 0 if the values are unipolar...
			} else {
				if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
				if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
			}
		}
		nrf_delay_ms(105);  // at 10 Hz ODR, new mag data is available every 100 ms
	}

	SEGGER_RTT_printf(0, "mag x min/max: %d, %d\n", mag_min[0], mag_max[0]);
	SEGGER_RTT_printf(0, "mag y min/max: %d, %d\n", mag_min[1], mag_max[1]);
	SEGGER_RTT_printf(0, "mag z min/max: %d, %d\n", mag_min[2], mag_max[2]);

	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	dest1[0] = (float) mag_bias[0] / 1.6;  // save mag biases in mG for use in main program
	dest1[1] = (float) mag_bias[1] / 1.6;  // mag data is 1.6 LSB/mg
	dest1[2] = (float) mag_bias[2] / 1.6;          

	// Return to config mode to write mag biases in offset register
	// This offset register is only used while in fusion mode when magnetometer sensitivity is 16 LSB/microTesla
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	nrf_delay_ms(25);

	//write biases to accelerometer offset registers as 16 LSB/microTesla
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB, (int16_t)mag_bias[0] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB, ((int16_t)mag_bias[0] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB, (int16_t)mag_bias[1] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB, ((int16_t)mag_bias[1] >> 8) & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB, (int16_t)mag_bias[2] & 0xFF);
	writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB, ((int16_t)mag_bias[2] >> 8) & 0xFF);

	// Check that offsets were properly written to offset registers
//	SEGGER_RTT_printf(0, "Average magnetometer bias = %d, %d, %d\n",\
//						(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB)),\
//						(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB)),\
//						(int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB)));

	// Select BNO055 system operation mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, bno055_config.opr_mode);
	nrf_delay_ms(25);

	SEGGER_RTT_printf(0, "Mag Calibration done!");
}


void bno055_initialize(void)
{
	// Select BNO055 config mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
	nrf_delay_ms(25);
	// Select page 1 to configure sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x01);
	// Configure ACC
	writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, bno055_config.a_pwr_mode << 5 | bno055_config.a_bw << 2 | bno055_config.a_scale);
	// Configure GYR
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, bno055_config.g_bw << 3 | bno055_config.g_scale);
	writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, bno055_config.g_pwr_mode);
	// Configure MAG
	writeByte(BNO055_ADDRESS, BNO055_MAG_CONFIG, bno055_config.m_pwr_mode << 5 | bno055_config.m_op_mode << 3 | bno055_config.m_odr);

//	// Configure interrupt settings...
//	// Accelerometer any motion interrupt (4:2)
//	writeByte(BNO055_ADDRESS, BNO055_ACC_INT_SETTINGS, 0x1C);
//	writeByte(BNO055_ADDRESS, BNO055_ACC_AM_THRES, 0x00);
//	// Gyroscope any motion interrupt (
//	writeByte(BNO055_ADDRESS, BNO055_GYR_INT_SETTINGS, 0x47);
//	writeByte(BNO055_ADDRESS, BNO055_GYR_AM_THRESH, 0x00);
//	writeByte(BNO055_ADDRESS, BNO055_GYR_AM_SET, 0x00);
//	// Set interrupt mask (6 -> ACC_AM, 2 -> GYR_AM)
//	writeByte(BNO055_ADDRESS, BNO055_INT_MSK, 0x44);
//	// Enable interrupt
//	writeByte(BNO055_ADDRESS, BNO055_INT_EN, 0x44);
	
	// Select page 0 to read sensors
	writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);

	// Select BNO055 gyro temperature source 
	writeByte(BNO055_ADDRESS, BNO055_TEMP_SOURCE, 0x01 );

	// Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
	writeByte(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01 );
	
	// Select BNO055 system power mode
	writeByte(BNO055_ADDRESS, BNO055_PWR_MODE, bno055_config.pwr_mode);

	// Select BNO055 system operation mode
	writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, bno055_config.opr_mode);
	nrf_delay_ms(25);
}

void bno055_int_reset(void)
{
	writeByte(BNO055_ADDRESS, BNO055_SYS_TRIGGER, 0x40);
}


void bno055_poll_data(void)
{
	// read raw data storage
	int16_t data3[3], data4[4];
	float ax, ay, az, gx, gy, gz, mx, my, mz;
	float q[4];
	float yaw, pitch, roll;
	float lia[3], grv[3];
	uint8_t i;
	
    read_accel_data(data3);  // Read the x/y/z adc values
    // Now we'll calculate the accleration value into actual mg's
    ax = (float)data3[0]; // - accelBias[0];  // subtract off calculated accel bias
    ay = (float)data3[1]; // - accelBias[1];
    az = (float)data3[2]; // - accelBias[2];
	// Fill the sensor output table for later sending
	bno055_output.accel[0].val = ax;
	bno055_output.accel[1].val = ay;
	bno055_output.accel[2].val = az;
//	SEGGER_RTT_printf(0, "Raw acceleration: %x, %x, %x\n", (int32_t)(ax*10000.), (int32_t)(ay*10000.), (int32_t)(az*10000.));
	
	
    read_gyro_data(data3);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gx = (float)data3[0]/16.; // - gyroBias[0];  // subtract off calculated gyro bias
    gy = (float)data3[1]/16.; // - gyroBias[1];  
    gz = (float)data3[2]/16.; // - gyroBias[2];  
	// Fill the sensor output table for later sending
	bno055_output.gyro[0].val = gx;
	bno055_output.gyro[1].val = gy;
	bno055_output.gyro[2].val = gz;
//	SEGGER_RTT_printf(0, "Raw gyroscope   : %ld, %ld, %ld\n", (int32_t)(gx*10000.), (int32_t)(gy*10000.), (int32_t)(gz*10000.));

    read_mag_data(data3);  // Read the x/y/z adc values   
    // Calculate the magnetometer values in milliGauss
    mx = (float)data3[0]/1.6; // - magBias[0];  // get actual magnetometer value in mGauss 
    my = (float)data3[1]/1.6; // - magBias[1];  
    mz = (float)data3[2]/1.6; // - magBias[2];   
	// Fill the sensor output table for later sending
	bno055_output.mag[0].val = mx;
	bno055_output.mag[1].val = my;
	bno055_output.mag[2].val = mz;
//	SEGGER_RTT_printf(0, "Raw magnetometer: %ld, %ld, %ld\n", (int32_t)(mx*10000.), (int32_t)(my*10000.), (int32_t)(mz*10000.));
    
    read_quat_data(data4);  // Read the x/y/z adc values   
    // Calculate the quaternion values  
    q[0] = (float)(data4[0])/16384.;    
    q[1] = (float)(data4[1])/16384.;  
    q[2] = (float)(data4[2])/16384.;   
    q[3] = (float)(data4[3])/16384.;   
	// Fill the sensor output table for later sending
	bno055_output.quat[0].val = q[0];
	bno055_output.quat[1].val = q[1];
	bno055_output.quat[2].val = q[2];
	bno055_output.quat[3].val = q[3];
// 	SEGGER_RTT_printf(0, "Raw quaternions : %ld, %ld, %ld, %ld\n", (int32_t)(q[0]*10000.), (int32_t)(q[1]*10000.), (int32_t)(q[2]*10000.), (int32_t)(q[3]*10000.));
   
    read_euler_data(data3);  // Read the x/y/z adc values   
    // Calculate the Euler angles values in degrees
    yaw = (float)data3[0]/16.;  
    roll = (float)data3[1]/16.;  
    pitch = (float)data3[2]/16.;   
	// Fill the sensor output table for later sending
	bno055_output.yaw.val = yaw;
	bno055_output.roll.val = roll;
	bno055_output.pitch.val = pitch;
// 	SEGGER_RTT_printf(0, "Raw euler angles: %ld, %ld, %ld\n", (int32_t)(yaw*10000.), (int32_t)(roll*10000.), (int32_t)(pitch*10000.));

    read_lia_data(data3);  // Read the x/y/z adc values   
    // Calculate the linear acceleration (sans gravity) values in mg
    lia[0] = (float)data3[0];  
    lia[1] = (float)data3[1];  
    lia[2] = (float)data3[2];   
	// Fill the sensor output table for later sending
	bno055_output.lia[0].val = lia[0];
	bno055_output.lia[1].val = lia[1];
	bno055_output.lia[2].val = lia[2];
//	SEGGER_RTT_printf(0, "Raw LIA         : %ld, %ld, %ld\n", (int32_t)(lia[0]*10000.), (int32_t)(lia[1]*10000.), (int32_t)(lia[2]*10000.));

    read_grv_data(data3);  // Read the x/y/z adc values   
    // Calculate the linear acceleration (sans gravity) values in mg
    grv[0] = (float)data3[0];  
    grv[1] = (float)data3[1];  
    grv[2] = (float)data3[2];   
	// Fill the sensor output table for later sending
	bno055_output.grv[0].val = grv[0];
	bno055_output.grv[1].val = grv[1];
	bno055_output.grv[2].val = grv[2];
//	SEGGER_RTT_printf(0, "Raw gravity     : %ld, %ld, %ld\n", (int32_t)(grv[0]*10000.), (int32_t)(grv[1]*10000.), (int32_t)(grv[2]*10000.));

	// Chose (once) between unprecise but low-power ms app timer
	// and precise current-demanding us drv timer 
//	static uint32_t last_time_ms = 0;
//	uint32_t now_ms = app_timer_cnt_get();
//	uint32_t delta_ms = (now_ms - last_time_ms)/33;
//	last_time_ms = now_ms;
	static uint32_t last_time_us = 0;
	uint32_t now_us = nrf_drv_timer_capture(&TIMER_DELTA_US, NRF_TIMER_CC_CHANNEL0);
	uint32_t delta_us = now_us - last_time_us;
	last_time_us = now_us;
	// Fill the sensor output table for later sending
	bno055_output.ts_us = now_us;
	
//	float deltat = (float)delta_us/1000000.;

//	mahony_quaternion_update(ax, ay, az, gx*PI/180.0, gy*PI/180.0, gz*PI/180.0, my, mx, mz, deltat);
//	madgwick_quaternion_update(ax, ay, az, gx*PI/180.0, gy*PI/180.0, gz*PI/180.0, my, mx, mz, deltat);
//	mahony_quaternion_update(ax, ay, az, gx*PI/180.0, gy*PI/180.0, gz*PI/180.0, my, mx, mz, deltat);
//	madgwick_quaternion_update(ax, ay, az, gx*PI/180.0, gy*PI/180.0, gz*PI/180.0, my, mx, mz, deltat);
}


// Read accel data
void read_accel_data(int16_t *destination)
{
	// x/y/z accel register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers into data array
	readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];      
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4]; 
}
// Read gyro data
void read_gyro_data(int16_t *destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4]; 
}
// Read temperature data
int16_t read_gyrotemp_data(void)
{
  return readByte(BNO055_ADDRESS, BNO055_TEMP);  // Read the two raw data registers sequentially into data array 
}

// Read magnetometer data
void read_mag_data(int16_t *destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

// Read quaternion data
void read_quat_data(int16_t * destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[8];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_QUA_DATA_W_LSB, 8, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	destination[3] = ((int16_t)rawData[7] << 8) | rawData[6];
}

// Read Euler angle data
void read_euler_data(int16_t * destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_EUL_HEADING_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

// Read linear acceleration data
void read_lia_data(int16_t * destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_LIA_DATA_X_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

// Read gravity vector
void read_grv_data(int16_t * destination)
{
	// x/y/z gyro register data stored here
	uint8_t rawData[6];
	// Read the six raw data registers sequentially into data array
	readBytes(BNO055_ADDRESS, BNO055_GRV_DATA_X_LSB, 6, &rawData[0]);
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
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

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	uint8_t reg[2] = {subAddress, data};
	twi_xfer_done = false;
	ret_code_t err_code = nrf_drv_twi_tx(&twi_master_instance, address, reg, sizeof(reg), false);
	APP_ERROR_CHECK(err_code);
	while(!twi_xfer_done) {};
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t reg[1] = {subAddress};
	twi_xfer_done = false;
	ret_code_t err_code = nrf_drv_twi_tx(&twi_master_instance, address, reg, sizeof(reg), false);
	APP_ERROR_CHECK(err_code);
	while(!twi_xfer_done);
		
	uint8_t data;
	twi_xfer_done = false;
	err_code = nrf_drv_twi_rx(&twi_master_instance, address, &data, sizeof(data));
	APP_ERROR_CHECK(err_code);
	while(!twi_xfer_done);
	
	return data;
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
	uint8_t reg[1] = {subAddress};
	twi_xfer_done = false;
	ret_code_t err_code = nrf_drv_twi_tx(&twi_master_instance, address, reg, sizeof(reg), false);
	APP_ERROR_CHECK(err_code);
	while(!twi_xfer_done);
	
	twi_xfer_done = false;
	err_code = nrf_drv_twi_rx(&twi_master_instance, address, dest, count);
	APP_ERROR_CHECK(err_code);
	while(!twi_xfer_done);
}
