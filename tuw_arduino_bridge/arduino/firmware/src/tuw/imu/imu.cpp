#include "imu.h"
#include "Wire.h"
#include "i2c.h"
#include <math.h>

using namespace tuw;

IMU::IMU() {

}

void IMU::init ( void ) {
	Wire.begin();
	// TODO: Set I2C speed to 400kHz

	i2c_buf[0] = PRESSURE_CTRL1_NORMAL_MODE | PRESSURE_CTRL1_ODR_7HZ;
	I2C_WriteBytes(PRESSURE_I2C_ADDRESS, i2c_buf, PRESSURE_REG_CTRL1, 1);

	i2c_buf[0] = GYROSCOPE_CTRL1_NORMAL_MODE | GYROSCOPE_CTRL1_EN_X_AXIS |
	             GYROSCOPE_CTRL1_EN_Y_AXIS | GYROSCOPE_CTRL1_EN_Z_AXIS |
	             GYROSCOPE_CTRL1_ODR_100HZ;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0x00;
	i2c_buf[3] = GYROSCOPE_CTRL4_RANGE_500DPS;
	i2c_buf[4] = GYROSCOPE_CTRL5_OUTPUT_LPF2;
	I2C_WriteBytes(GYROSCOPE_I2C_ADDRESS, i2c_buf, REG_AUTO_INCREMENT | GYROSCOPE_REG_CTRL1, 5);

	i2c_buf[0] = ACCELEROMETER_CTRL1_ODR_100HZ | ACCELEROMETER_CTRL1_EN_X_AXIS |
	             ACCELEROMETER_CTRL1_EN_Y_AXIS | ACCELEROMETER_CTRL1_EN_Z_AXIS;
	i2c_buf[1] = ACCELEROMETER_CTRL2_RANGE_4G;
	i2c_buf[2] = 0x00;
	i2c_buf[3] = 0x00;
	i2c_buf[4] = MAGNETOMETER_CTRL5_RES_HIGH | MAGNETOMETER_CTRL5_ODR_100HZ;
	i2c_buf[5] = 0x00;
	i2c_buf[6] = 0x00; // ACCELEROMETER_CTRL7_EN_LPF;
	I2C_WriteBytes(ACCELEROMETER_I2C_ADDRESS, i2c_buf, REG_AUTO_INCREMENT | ACCELEROMETER_REG_CTRL1, 7);
}

void IMU::update ( void ) {
	int32_t temp_i32;

	// PRESSURE SENSOR
	I2C_ReadBytes(PRESSURE_I2C_ADDRESS, i2c_buf, REG_AUTO_INCREMENT | PRESSURE_REG_STATUS, 6);
	uint32_t pressure_raw = ((uint32_t)i2c_buf[3] << 16) | ((uint16_t)i2c_buf[2] << 8) | ((uint8_t) i2c_buf[1]);

	pressure = ((pressure_raw >> 12) * 100) + ((pressure_raw & 0x0FFF) / 40.96);

	temperature = (i2c_buf[5] << 8) | i2c_buf[4];
	temperature /= 48;
	temperature += 425;

	// GYRO
	I2C_ReadBytes(GYROSCOPE_I2C_ADDRESS, i2c_buf, REG_AUTO_INCREMENT | GYROSCOPE_REG_STATUS, 7);
	gyro[AXIS_X] = (i2c_buf[2] << 8) | i2c_buf[1];
	temp_i32 = gyro[AXIS_X] * 500L;
	gyro[AXIS_X] = temp_i32 >> 15;
	gyro[AXIS_Y] = (i2c_buf[4] << 8) | i2c_buf[3];
	temp_i32 = gyro[AXIS_Y] * 500L;
	gyro[AXIS_Y] = temp_i32 >> 15;
	gyro[AXIS_Z] = (i2c_buf[6] << 8) | i2c_buf[5];
	temp_i32 = gyro[AXIS_Z] * 500L;
	gyro[AXIS_Z] = temp_i32 >> 15;

	// ACCELEROMETER
	I2C_ReadBytes(ACCELEROMETER_I2C_ADDRESS, i2c_buf, REG_AUTO_INCREMENT | ACCELEROMETER_REG_STATUS, 7);
	accel[AXIS_X] = (i2c_buf[2] << 8) | i2c_buf[1];
	temp_i32 = accel[AXIS_X] * 4L * 981L;
	accel[AXIS_X] = temp_i32 >> 15;
	accel[AXIS_Y] = (i2c_buf[4] << 8) | i2c_buf[3];
	temp_i32 = accel[AXIS_Y] * 4L * 981L;
	accel[AXIS_Y] = temp_i32 >> 15;
	accel[AXIS_Z] = (i2c_buf[6] << 8) | i2c_buf[5];
	temp_i32 = accel[AXIS_Z] * 4L * 981L;
	accel[AXIS_Z] = temp_i32 >> 15;

	// MAGNETOMETER
	I2C_ReadBytes(MAGNETOMETER_I2C_ADDRESS, i2c_buf, REG_AUTO_INCREMENT | MAGNETOMETER_REG_STATUS, 7);
	magnet[AXIS_X] = (i2c_buf[2] << 8) | i2c_buf[1];
	temp_i32 = magnet[AXIS_X] * 2000L;
	magnet[AXIS_X] = temp_i32 >> 15;
	magnet[AXIS_Y] = (i2c_buf[4] << 8) | i2c_buf[3];
	temp_i32 = magnet[AXIS_Y] * 2000L;
	magnet[AXIS_Y] = temp_i32 >> 15;
	magnet[AXIS_Z] = (i2c_buf[6] << 8) | i2c_buf[5];
	temp_i32 = magnet[AXIS_Z] * 2000L;
	magnet[AXIS_Z] = temp_i32 >> 15;

	compass = 180 * atan2(magnet[AXIS_Y], magnet[AXIS_X])/M_PI;
	if (compass < 0) compass += 360;
}



uint32_t IMU::get_pressure(void) {
	return pressure;
}

int16_t IMU::get_temperature(void) {
	return temperature;
}

int16_t IMU::get_accel(uint8_t axis) {
	return accel[axis];
}

int16_t IMU::get_gyro(uint8_t axis) {
	return gyro[axis];
}

int16_t IMU::get_magnet(uint8_t axis) {
	return magnet[axis];
}

int16_t IMU::get_compass(void) {
	return compass;
}


