/*
* bno055.h
*
* Created: 03.04.2017 10:03:54
* Author: sschies1
*/ 

#ifndef BNO055_H_
#define BNO055_H_

/* === Includes ============================================================= */
#include <stdbool.h>
#include <stdint.h>

// BNO055 Register Map
// http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_10_Release.pdf

#define ADO 0
#if ADO
#define BNO055_ADDRESS 0x29   //  Device address of BNO055 when ADO = 1
#else
#define BNO055_ADDRESS 0x28   //  Device address of BNO055 when ADO = 0
#endif  


// BNO055 Page 0
#define BNO055_CHIP_ID          0x00    // should be 0xA0              
#define BNO055_ACC_ID           0x01    // should be 0xFB              
#define BNO055_MAG_ID           0x02    // should be 0x32              
#define BNO055_GYRO_ID          0x03    // should be 0x0F              
#define BNO055_SW_REV_ID_LSB    0x04                                                                          
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_MAG_DATA_X_MSB   0x0F
#define BNO055_MAG_DATA_Y_LSB   0x10
#define BNO055_MAG_DATA_Y_MSB   0x11
#define BNO055_MAG_DATA_Z_LSB   0x12
#define BNO055_MAG_DATA_Z_MSB   0x13
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_GYR_DATA_X_MSB   0x15
#define BNO055_GYR_DATA_Y_LSB   0x16
#define BNO055_GYR_DATA_Y_MSB   0x17
#define BNO055_GYR_DATA_Z_LSB   0x18
#define BNO055_GYR_DATA_Z_MSB   0x19
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_QUA_DATA_W_MSB   0x21
#define BNO055_QUA_DATA_X_LSB   0x22
#define BNO055_QUA_DATA_X_MSB   0x23
#define BNO055_QUA_DATA_Y_LSB   0x24
#define BNO055_QUA_DATA_Y_MSB   0x25
#define BNO055_QUA_DATA_Z_LSB   0x26
#define BNO055_QUA_DATA_Z_MSB   0x27
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_LIA_DATA_X_MSB   0x29
#define BNO055_LIA_DATA_Y_LSB   0x2A
#define BNO055_LIA_DATA_Y_MSB   0x2B
#define BNO055_LIA_DATA_Z_LSB   0x2C
#define BNO055_LIA_DATA_Z_MSB   0x2D
#define BNO055_GRV_DATA_X_LSB   0x2E
#define BNO055_GRV_DATA_X_MSB   0x2F
#define BNO055_GRV_DATA_Y_LSB   0x30
#define BNO055_GRV_DATA_Y_MSB   0x31
#define BNO055_GRV_DATA_Z_LSB   0x32
#define BNO055_GRV_DATA_Z_MSB   0x33
#define BNO055_TEMP             0x34
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A
#define BNO055_UNIT_SEL         0x3B
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_ACC_RADIUS_MSB   0x68
#define BNO055_MAG_RADIUS_LSB   0x69
#define BNO055_MAG_RADIUS_MSB   0x6A

// BNO055 Page 1
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_CONFIG       0x08
#define BNO055_MAG_CONFIG       0x09
#define BNO055_GYRO_CONFIG_0    0x0A
#define BNO055_GYRO_CONFIG_1    0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK          0x0F
#define BNO055_INT_EN           0x10
#define BNO055_ACC_AM_THRES     0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION  0x13
#define BNO055_ACC_HG_THRESH    0x14
#define BNO055_ACC_NM_THRESH    0x15
#define BNO055_ACC_NM_SET       0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET     0x18
#define BNO055_GYR_DUR_X        0x19
#define BNO055_GYR_HR_Y_SET     0x1A
#define BNO055_GYR_DUR_Y        0x1B
#define BNO055_GYR_HR_Z_SET     0x1C
#define BNO055_GYR_DUR_Z        0x1D
#define BNO055_GYR_AM_THRESH    0x1E
#define BNO055_GYR_AM_SET       0x1F


// Set initial input parameters
enum Ascale {  // ACC Full Scale
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum Abw { // ACC Bandwidth
	ABW_7_81Hz = 0,
	ABW_15_63Hz,
	ABW_31_25Hz,
	ABW_62_5Hz,
	ABW_125Hz,    
	ABW_250Hz,
	ABW_500Hz,     
	ABW_1000Hz,    //0x07
};

enum APwrMode { // ACC Pwr Mode
	NormalA = 0,  
	SuspendA,
	LowPower1A,
	StandbyA,        
	LowPower2A,
	DeepSuspendA
};

enum Gscale {  // gyro full scale
	GFS_2000DPS = 0,
	GFS_1000DPS,
	GFS_500DPS,
	GFS_250DPS,
	GFS_125DPS    // 0x04
};

enum GPwrMode { // GYR Pwr Mode
	NormalG = 0,
	FastPowerUpG,
	DeepSuspendedG,
	SuspendG,
	AdvancedPowerSaveG
};

enum Gbw { // gyro bandwidth
	GBW_523Hz = 0,
	GBW_230Hz,
	GBW_116Hz,
	GBW_47Hz,
	GBW_23Hz,
	GBW_12Hz,
	GBW_64Hz,
	GBW_32Hz
};

enum OPRMode {  // BNO-55 operation modes
	CONFIGMODE = 0x00,
	// Sensor Mode
	ACCONLY,
	MAGONLY,
	GYROONLY,
	ACCMAG,
	ACCGYRO,
	MAGGYRO,
	AMG,            // 0x07
	// Fusion Mode
	IMU,
	COMPASS,
	M4G,
	NDOF_FMC_OFF,
	NDOF            // 0x0C
};

enum PWRMode {
	Normalpwr = 0,   
	Lowpower,       
	Suspendpwr       
};

enum Modr {         // magnetometer output data rate  
	MODR_2Hz = 0,     
	MODR_6Hz,
	MODR_8Hz,
	MODR_10Hz,  
	MODR_15Hz,
	MODR_20Hz,
	MODR_25Hz, 
	MODR_30Hz 
};

enum MOpMode { // MAG Op Mode
	LowPower = 0,
	Regular,
	EnhancedRegular,
	HighAccuracy
};

enum MPwrMode { // MAG power mode
	Normal = 0,   
	Sleep,     
	Suspend,
	ForceMode  
};

typedef union float2bytes{
	float val;
	uint8_t b[4];
} f2b_t;

typedef struct {
	enum Ascale a_scale;
	enum Abw a_bw;
	enum APwrMode a_pwr_mode;
	enum Gscale g_scale;
	enum Gbw g_bw;
	enum GPwrMode g_pwr_mode;
	enum MOpMode m_op_mode;
	enum MPwrMode m_pwr_mode;
	enum Modr m_odr;
	enum PWRMode pwr_mode;
	enum OPRMode opr_mode;
	//	enum Mscale m_scale;
	//	enum Mmode m_mode;
	//	bool init_ok;
	//	float self_test[6];
	float gyro_bias[3];
	float accel_bias[3];
	float mag_bias[3];
	//	float mag_calibration[3];
	//	bool ahrs;
	uint8_t cal_state;
	bool dev_start;
	bool init_ok;
	bool dev_en;
	uint8_t comp_mask;
}bno055_config_s;

typedef struct {
	f2b_t accel[3];
	f2b_t gyro[3];
	f2b_t mag[3];
	f2b_t temp;
	f2b_t quat[4];
	f2b_t yaw;
	f2b_t roll;
	f2b_t pitch;
	f2b_t lia[3];
	f2b_t grv[3];
	uint32_t ts_us;
//	// The float variables for internal processing
//	float accel[3];
//	float gyro[3];
//	float mag[3];
//	float temp;
//	float quat[4];
//	float yaw;
//	float pitch;
//	float roll;
//	float lia[3];
//	float grv[3];
//	// The "sliced" 8-bit integers for serial transmision
//	uint8_t accel_i[3][4];
//	uint8_t gyro_i[3][4];
//	uint8_t mag_i[3][4];
//	uint8_t temp_i[4];
//	uint8_t quat_i[4][4];
//	uint8_t yaw_i[4];
//	uint8_t roll_i[4];
//	uint8_t pitch_i[4];
//	uint8_t lia_i[3][4];
//	uint8_t grv_i[3][4]
}bno055_output_s;

typedef struct {
	bool enabled;
	volatile bool new_value;
	volatile bool rts;
}bno055_interrupt_s;



#endif // BNO055_H_
