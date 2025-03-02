
#ifndef DRIVERS_MPU_9250_H_
#define DRIVERS_MPU_9250_H_

/*********************
 *      INCLUDES
 *********************/

#include <stdint.h>
#include "main.h"

/*********************
 *      DEFINES
 *********************/

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C<<1
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL1     0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2	 0x0B
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define     Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)
//MPU9250 register map
#define MPU9250_SELF_TEST_X_GYRO        0x00        /*!< Gyroscope self-test registers */
#define MPU9250_SELF_TEST_Y_GYRO        0x01
#define MPU9250_SELF_TEST_Z_GYRO        0x02
#define MPU9250_SELF_TEST_X_ACCEL       0x0D        /*!< Accelerometer self-test registers */
#define MPU9250_SELF_TEST_Y_ACCEL       0x0E
#define MPU9250_SELF_TEST_Z_ACCEL       0x0F
#define MPU9250_XG_OFFSET_H             0x13        /*!< Gyroscope offset registers */
#define MPU9250_XG_OFFSET_L             0x14
#define MPU9250_YG_OFFSET_H             0x14
#define MPU9250_YG_OFFSET_L             0x16
#define MPU9250_ZG_OFFSET_H             0x17
#define MPU9250_ZG_OFFSET_L             0x18
#define MPU9250_SMPRT_DIV               0x19        /*!< Sample rate divider */
#define MPU9250_CONFIG                  0x1A        /*!< Configuration */
#define MPU9250_GYRO_CONFIG             0x1B        /*!< Gyroscope configuration */
#define MPU9250_ACCEL_CONFIG            0x1C        /*!< Accelerometer configuration */
#define MPU9250_ACCEL_CONFIG2           0x1D        /*!< Accelerometer configuration 2 */
#define MPU9250_LP_ACCEL_ODR            0x1E        /*!< Low power accelerometer ODR control */
#define MPU9250_WOM_THR                 0x1F        /*!< Wake-on motion threshold */
#define MPU9250_FIFO_EN                 0x23        /*!< FIFO enable */
#define MPU9250_I2C_MST_CTRL            0x24        /*!< I2C master control */
#define MPU9250_I2C_SLV0_ADDR           0x25        /*!< I2C slave 0 control */
#define MPU9250_I2C_SLV0_REG            0x26
#define MPU9250_I2C_SLV0_CTRL           0x27
#define MPU9250_I2C_SLV1_ADDR           0x28        /*!< I2C slave 1 control */
#define MPU9250_I2C_SLV1_REG            0x29
#define MPU9250_I2C_SLV1_CTRL           0x2A
#define MPU9250_I2C_SLV2_ADDR           0x2B        /*!< I2C slave 2 control */
#define MPU9250_I2C_SLV2_REG            0x2C
#define MPU9250_I2C_SLV2_CTRL           0x2D
#define MPU9250_I2C_SLV3_ADDR           0x2E        /*!< I2C slave 3 control */
#define MPU9250_I2C_SLV3_REG            0x2F
#define MPU9250_I2C_SLV3_CTRL           0x30
#define MPU9250_I2C_SLV4_ADDR           0x31        /*!< I2C slave 4 control */
#define MPU9250_I2C_SLV4_REG            0x32
#define MPU9250_I2C_SLV4_DO             0x33
#define MPU9250_I2C_SLV4_CTRL           0x34
#define MPU9250_I2C_SLV4_DI             0x35
#define MPU9250_I2C_MST_STATUS          0x36        /*!< I2C master status */
#define MPU9250_INT_PIN_CFG             0x37        /*!< Interrupt pin/bypass enable configuration */
#define MPU9250_INT_ENABLE              0x38        /*!< Interrupt enable */
#define MPU9250_INT_STATUS              0x3A        /*!< Interrupt status */
#define MPU9250_ACCEL_XOUT_H            0x3B        /*!< Accelerometer measurements */
#define MPU9250_ACCEL_XOUT_L            0x3C
#define MPU9250_ACCEL_YOUT_H            0x3D
#define MPU9250_ACCEL_YOUT_L            0x3E
#define MPU9250_ACCEL_ZOUT_H            0x3F
#define MPU9250_ACCEL_ZOUT_L            0x40
#define MPU9250_TEMP_OUT_H              0x41        /*!< Temperature measurements */
#define MPU9250_TEMP_OUT_L              0x42
#define MPU9250_GYRO_XOUT_H             0x43        /*!< Gyroscope measurements */
#define MPU9250_GYRO_XOUT_L             0x44
#define MPU9250_GYRO_YOUT_H             0x45
#define MPU9250_GYRO_YOUT_L             0x46
#define MPU9250_GYRO_ZOUT_H             0x47
#define MPU9250_GYRO_ZOUT_L             0x48
#define MPU9250_EXT_SENS_DATA_00        0x49        /*!< External sensor data */
#define MPU9250_EXT_SENS_DATA_01        0x4A
#define MPU9250_EXT_SENS_DATA_02        0x4B
#define MPU9250_EXT_SENS_DATA_03        0x4C
#define MPU9250_EXT_SENS_DATA_04        0x4D
#define MPU9250_EXT_SENS_DATA_05        0x4E
#define MPU9250_EXT_SENS_DATA_06        0x4F
#define MPU9250_EXT_SENS_DATA_07        0x50
#define MPU9250_EXT_SENS_DATA_08        0x51
#define MPU9250_EXT_SENS_DATA_09        0x52
#define MPU9250_EXT_SENS_DATA_10        0x53
#define MPU9250_EXT_SENS_DATA_11        0x54
#define MPU9250_EXT_SENS_DATA_12        0x55
#define MPU9250_EXT_SENS_DATA_13        0x56
#define MPU9250_EXT_SENS_DATA_14        0x57
#define MPU9250_EXT_SENS_DATA_15        0x58
#define MPU9250_EXT_SENS_DATA_16        0x59
#define MPU9250_EXT_SENS_DATA_17        0x5A
#define MPU9250_EXT_SENS_DATA_18        0x5B
#define MPU9250_EXT_SENS_DATA_19        0x5C
#define MPU9250_EXT_SENS_DATA_20        0x5D
#define MPU9250_EXT_SENS_DATA_21        0x5E
#define MPU9250_EXT_SENS_DATA_22        0x5F
#define MPU9250_EXT_SENS_DATA_23        0x60
#define MPU9250_I2C_SLV0_DO             0x63        /*!< I2C slave 0 data out */
#define MPU9250_I2C_SLV1_DO             0x64        /*!< I2C slave 1 data out */
#define MPU9250_I2C_SLV2_DO             0x65        /*!< I2C slave 2 data out */
#define MPU9250_I2C_SLV3_DO             0x66        /*!< I2C slave 3 data out */
#define MPU9250_I2C_MST_DELAY_CTRL      0x67        /*!< I2C master delay control */
#define MPU9250_SIGNAL_PATH_RESET       0x68        /*!< Signal path reset */
#define MPU9250_MOT_DETECT_CTRL         0x69        /*!< Acelerometer interrupt control */
#define MPU9250_USER_CTRL               0x6A        /*!< User control */
#define MPU9250_PWR_MGMT_1              0x6B        /*!< Power management 1 */
#define MPU9250_PWR_MGMT_2              0x6C        /*!< Power management 2 */
#define MPU9250_FIFO_COUNTH             0x72        /*!< FIFO counter registers */
#define MPU9250_FIFO_COUNTL             0x73
#define MPU9250_FIFP_R_W                0x74        /*!< FIFO read write */
#define MPU9250_WHO_AM_I                0x75        /*!< Who am I */
#define MPU9250_XA_OFFSET_H             0x77        /*!< Accelerometer offset registers */
#define MPU9250_XA_OFFSET_L             0x78
#define MPU9250_YA_OFFSET_H             0x7A
#define MPU9250_YA_OFFSET_L             0x7B
#define MPU9250_ZA_OFFSET_H             0x7D
#define MPU9250_ZA_OFFSET_L             0x7E
#define MPU9250_ADDR                    (0x68<<1)   /*!< MPU9250 Address */
//#define MPU9250_INT_PIN_CFG             0x37        /*!< Interrupt pin/bypass enable configuration */
//#define MPU9250_ADDR                    (0x68<<1)   /*!< MPU9250 Address */

#define DEG_TO_RAD   3.141592f / 180
/**********************
 *      TYPEDEFS
 **********************/
// float LSB_Sensitivity_ACC;
// float LSB_Sensitivity_GYRO;
// uint16_t error;
extern I2C_HandleTypeDef hi2c1;

typedef struct _MPU9250
{
	short acc_x_raw;
	short acc_y_raw;
	short acc_z_raw;

	short temperature_raw;

	short gyro_x_raw;
	short gyro_y_raw;
	short gyro_z_raw;

	short mag_x_raw;
	short mag_y_raw;
	short mag_z_raw;

	float acc_x;
	float acc_y;
	float acc_z;

	float temperature;

	float gyro_x;
	float gyro_y;
	float gyro_z;

	float mag_x;
	float mag_y;
	float mag_z;

  float Filt_accx;
	float Filt_accy;
	float Filt_accz;

  float Filt_gyx;
	float Filt_gyy;
	float Filt_gyz;

  float cal_gyx;
	float cal_gyy;
	float cal_gyz;
} MPU9250_t;

typedef struct imu_9250 imu_9250_t;

struct imu_9250
{
	//	I2C_HandleTypeDef *hi2c;
	void (*get_data)(imu_9250_t *const imu_p);
	MPU9250_t pt1_p;
};

/**********************
 *     OPERATION
 **********************/

imu_9250_t *IMU_9250_Create();
void IMU_9250_Destroy(imu_9250_t *const imu_p);

void MPU9250_Writebyte(uint8_t device_addr,uint8_t reg_addr, uint8_t val);
void MPU9250_Writebytes(uint8_t device_addr,uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU9250_Readbyte(uint8_t device_addr,uint8_t reg_addr, uint8_t* data);
void MPU9250_Readbytes(uint8_t device_addr,uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU9250_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC);
void MPU9250_ProcessData(imu_9250_t *mpu9250);
void MPU9250_Get6AxisRawData(imu_9250_t *mpu9250);
void MPU9250_GetMagRawData(imu_9250_t *mpu9250);
void MPU9250_DataConvert(imu_9250_t *mpu9250);
void calibrateGyro(imu_9250_t *mpu9250, uint16_t numCalPoints);

#endif /* DRIVERS_MPU_9250_H_ */
