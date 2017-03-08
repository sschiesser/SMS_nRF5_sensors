#include "sms_imu.h"
#include "mpu9250.h"

#define TWI_INSTANCE 1
const nrf_drv_twi_t twi_master_instance = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);
volatile bool twi_xfer_done;

struct mpu9250_config_s mpu9250_config;
struct mpu9250_output_s mpu9250_output;
struct mpu9250_interrupt_s mpu9250_interrupt;


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

int mpu9250_check(void)
{
	int ret = -1;
	uint8_t c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	if(c == 0x71) ret = 0;

	return ret;
}


void mpu9250_calibrate(float *dest1, float *dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0};
	int32_t accel_bias[3] = {0, 0, 0};
	
	// reset device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	nrf_delay_ms(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	nrf_delay_ms(200);

	// Configure device for bias calculation
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	nrf_delay_ms(15);
	
	// Configure MPU9250 gyro and accelerometer for bias calculation
	writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
	
	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	nrf_delay_ms(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, data); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	
	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, data); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
		
		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
		
	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;
	
	if(accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}
	
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;
	
	// Push gyro biases to hardware registers
	writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
	
	// Output scaled gyro biases for display in the main program
	dest1[0] = (float)((float)gyro_bias[0]/(float)gyrosensitivity);
	dest1[1] = (float)((float)gyro_bias[1]/(float)gyrosensitivity);
	dest1[2] = (float)((float)gyro_bias[2]/(float)gyrosensitivity);


	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, data); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, data);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, data);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	
	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
	
	for(ii = 0; ii < 3; ii++) {
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}
	
	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);
	
	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	
	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

	// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void mpu9250_initialize(void)
{
	// wake up device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	nrf_delay_ms(100); // Wait for all registers to reset

	// get stable time source
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	nrf_delay_ms(200);
	
	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	 //writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
	writeByte(MPU9250_ADDRESS, CONFIG, 0x05);		// gyro bandwidth = 10 Hz, delay = 17.85 ms -> max rate = 56 Hz

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	//writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x0A);  	// Use a 90 Hz rate; a rate consistent with the filter update rate
	// // determined inset in CONFIG above
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x63);  	// Use a 10 Hz rate; a rate consistent with the filter update rate
	
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | (mpu9250_config.g_scale << 3); // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
	
	// Set accelerometer full-scale range configuration
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | (mpu9250_config.a_scale << 3); // Set full scale range for the accelerometer
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	// c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz, delay 11.8 ms
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, send 50 us interrupt pulses,
	// clear on ANY read, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	//writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x12);
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	nrf_delay_ms(100);
}


int mpu9250_comp_check(void)
{
	int ret = -1;
	uint8_t d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
	if(d == 0x48) ret = 0;
	
	return ret;
}

void mpu9250_comp_initialize(float *destination)
{
	mpu9250_config.m_scale = MFS_16BITS;	// Choose either 14-bit or 16-bit magnetometer resolution
	mpu9250_config.m_mode = MODE_CONT1;	// CONT1 (2) for 8 Hz, CONT2 (6) for 100 Hz continuous magnetometer data read

	/* !!! SET MAGNETOMETER BIAS VALUES !!! SHOULD BE CALCULATED AUTOMATICALLY !!! */
	mpu9250_config.mag_bias[0] = 470.0;
	mpu9250_config.mag_bias[1] = 120.0;
	mpu9250_config.mag_bias[2] = 125.0;
	/* !!! SET MAGNETOMETER BIAS VALUES !!! SHOULD BE CALCULATED AUTOMATICALLY !!! */

	// First extract the factory calibration for each magnetometer axis
	uint8_t data[3];  // x/y/z gyro calibration data stored here
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	nrf_delay_ms(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	nrf_delay_ms(10);
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, data);  // Read the x-, y-, and z-axis calibration values
	destination[0] =  (float)(data[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (float)(data[1] - 128)/256. + 1.;
	destination[2] =  (float)(data[2] - 128)/256. + 1.;
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	nrf_delay_ms(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS, AK8963_CNTL, mpu9250_config.m_scale << 4 | mpu9250_config.m_mode); // Set magnetometer data resolution and sample ODR
	nrf_delay_ms(10);
}



int mpu9250_poll_data(void)
{
	read_accel_data(mpu9250_output.raw_accel);
	float a_res = get_Ares(mpu9250_config.a_scale);
	float ax = ((float)mpu9250_output.raw_accel[0]) * a_res;
	float ay = ((float)mpu9250_output.raw_accel[1]) * a_res;
	float az = ((float)mpu9250_output.raw_accel[2]) * a_res;
	
	read_gyro_data(mpu9250_output.raw_gyro);
	float g_res = get_Gres(mpu9250_config.g_scale);
	float gx = ((float)mpu9250_output.raw_gyro[0]) * g_res;
	float gy = ((float)mpu9250_output.raw_gyro[1]) * g_res;
	float gz = ((float)mpu9250_output.raw_gyro[2]) * g_res;
	
	read_comp_data(mpu9250_output.raw_compass);
	float m_res = get_Mres(mpu9250_config.m_scale);
	float mx = ( ((float)mpu9250_output.raw_compass[0]) * m_res * mpu9250_config.mag_calibration[0] ) - mpu9250_config.mag_bias[0];
	float my = ( ((float)mpu9250_output.raw_compass[1]) * m_res * mpu9250_config.mag_calibration[1] ) - mpu9250_config.mag_bias[1];
	float mz = ( ((float)mpu9250_output.raw_compass[2]) * m_res * mpu9250_config.mag_calibration[2] ) - mpu9250_config.mag_bias[2];
	
//	static uint32_t last_time = 0;
//	const uint32_t cnt_max = 0xffffffff/SMS_DUALTIMER_LOAD_US;
//	uint32_t now = (uint32_t)(dualtimer_get_value(timer1_instance.id)/SMS_DUALTIMER_LOAD_US);
//	uint32_t deltati = ((now < last_time) ? (last_time - now) : (cnt_max - now + last_time));
//	last_time = now;
//	float deltatf = (float)deltati / 1000000.0;
//	mahony_quaternion_update(ax, ay, az, gx*PI/180.0, gy*PI/180.0, gz*PI/180.0, my, mx, mz, deltatf);
//	//madgwick_quaternion_update(ax, ay, az, gx*PI/180.0, gy*PI/180.0, gz*PI/180.0, my, mx, mz, deltatf);
//	
//	if(imu_device.config.ahrs) {
//		ahrs_calculation(imu_device.output.q);
//	}
	return 0;
}

/* Read accel data */
void read_accel_data(int16_t *destination)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}
/* Read gyro data */
void read_gyro_data(int16_t *destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}
/* Read compass data */
void read_comp_data(int16_t *destination)
{
	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
		}
	}
}
/* Read temperature data */
int16_t read_temp_data(void)
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
	return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

/* Utility functions */
float get_Mres(uint8_t m_scale)
{
	float retVal = 0;
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	switch(m_scale) {
		case MFS_14BITS:
		retVal = 10.0 * 4912.0 / 8190.0;
		break;
		
		case MFS_16BITS:
		retVal = 10.0 * 4912.0 / 32760.0;
		break;
		
		default:
		break;
	}
	return retVal;
}
float get_Gres(uint8_t g_scale)
{
	float retVal = 0;
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	switch(g_scale) {
		case GFS_250DPS:
		retVal = 250.0 / 32768.0;
		break;
		
		case GFS_500DPS:
		retVal = 500.0 / 32768.0;
		break;
		
		case GFS_1000DPS:
		retVal = 1000.0 / 32768.0;
		break;
		
		case GFS_2000DPS:
		retVal = 2000.0 / 32768.0;
		break;
		
		default:
		break;
	}
	return retVal;
}
float get_Ares(uint8_t a_scale)
{
	float retVal = 0.0;
	switch(a_scale) {
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
		retVal = 2.0/32768.0;
		break;
		case AFS_4G:
		retVal = 4.0/32768.0;
		break;
		case AFS_8G:
		retVal = 8.0/32768.0;
		break;
		case AFS_16G:
		retVal = 16.0/32768.0;
		break;
	}
	return retVal;
}


/**@brief Function for initializing the TWI module.
 */
void twi_init(void)
{
	ret_code_t err_code;
	const nrf_drv_twi_config_t twi_config = {
		.scl				= TWI_SCL_PIN,
		.sda				= TWI_SDA_PIN,
		.frequency			= NRF_TWI_FREQ_100K,
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
