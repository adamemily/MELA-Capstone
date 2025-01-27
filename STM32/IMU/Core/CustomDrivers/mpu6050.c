
/*

#include "mpu6050_reg_map.h"
#include "mpu6050.h"

#include <string.h>
#include <stdbool.h>


#define MPU6050_ADDRESS MPU6050_DEV_ADD_AD0_LOW // Device address when AD0 is set to low.
#define WHO_AM_I_VALUE 0x68u  // Value of WHO_AM_I register data.

static bool device_init_complete = false;

bool get_who_am_i(void);
bool write_accel_config(void);
uint8_t read_regsiter(uint8_t reg_add);
bool write_regsiter(uint8_t reg_add, uint8_t reg_val);

bool mpu6050_init(void)
{

	// Take the accelerometer out of sleep mode.
	uint8_t data = 0x00;
	(bool)write_regsiter(MPU6050_REG_PWR_MGMT_1, data);

    // Set data rate to 1KHz by writing MPU6050_REG_SMPRT_DIV register.
	data = 0x07;
	(bool)write_regsiter(MPU6050_REG_SMPRT_DIV, data);

	// Set accelerometer configuration to +-4g.
	data = 0x10;
	(bool)write_regsiter(MPU6050_REG_ACCEL_CONFIG, data);

	device_init_complete = get_who_am_i();

	return device_init_complete;
}


int16_t x_axis_acc_data(void)
{
	uint16_t x_acc= 0u;
	if(device_init_complete)
	{
		uint8_t x_acc_l = read_regsiter(MPU6050_REG_ACCEL_XOUT_L);
		uint8_t x_acc_h = read_regsiter(MPU6050_REG_ACCEL_XOUT_H);
		x_acc = ((int16_t)x_acc_h << 8) | x_acc_l;
	}
	return x_acc;
}

int16_t y_axis_acc_data(void)
{
	uint16_t y_acc = 0u;
	if(device_init_complete)
	{
		uint8_t y_acc_l = read_regsiter(MPU6050_REG_ACCEL_YOUT_L);
		uint8_t y_acc_h = read_regsiter(MPU6050_REG_ACCEL_YOUT_H);
		y_acc= ((int16_t)y_acc_h << 8) | y_acc_l;
	}
	return y_acc;
}


int16_t z_axis_acc_data(void)
{
	uint16_t z_acc = 0u;
	if(device_init_complete)
	{
		uint8_t z_acc_l = read_regsiter(MPU6050_REG_ACCEL_ZOUT_L);
		uint8_t z_acc_h = read_regsiter(MPU6050_REG_ACCEL_ZOUT_H);
		z_acc = ((int16_t)z_acc_h << 8) | z_acc_l;
	}
	return z_acc;
}

int16_t x_axis_gyro_data(void)
{
	uint16_t x_gyro = 0u;
	if(device_init_complete)
	{
		uint8_t x_gyro_l = read_regsiter(MPU6050_REG_GYRO_XOUT_L);
		uint8_t x_gyro_h = read_regsiter(MPU6050_REG_GYRO_XOUT_H);
		x_gyro = ((int16_t)x_gyro_h << 8) | x_gyro_l;
	}
	return x_gyro;
}

int16_t y_axis_gyro_data(void)
{
	uint16_t y_gyro = 0u;
	if(device_init_complete)
	{
		uint8_t y_gyro_l = read_regsiter(MPU6050_REG_GYRO_YOUT_L);
		uint8_t y_gyro_h = read_regsiter(MPU6050_REG_GYRO_YOUT_H);
		y_gyro = ((int16_t)y_gyro_h << 8) | y_gyro_l;
	}
	return y_gyro;
}


int16_t z_axis_gyro_data(void)
{
	uint16_t z_gyro = 0u;
	if(device_init_complete)
	{
		uint8_t z_gyro_l = read_regsiter(MPU6050_REG_GYRO_ZOUT_L);
		uint8_t z_gyro_h = read_regsiter(MPU6050_REG_GYRO_ZOUT_H);
		z_gyro = ((int16_t)z_gyro_h << 8) | z_gyro_l;
	}
	return z_gyro;
}


int16_t temperature_data(void)
{
	uint16_t temperature = 0u;
	if(device_init_complete)
	{
		uint8_t temperature_l = read_regsiter(MPU6050_REG_TEMP_OUT_L);
		uint8_t temperature_h = read_regsiter(MPU6050_REG_TEMP_OUT_H);
		temperature = ((int16_t)temperature_h << 8) | temperature_l;
	}
	return temperature;
}


//verify that the MPU6050 address is being read-in correctly from its known address
bool get_who_am_i(void)
{
	uint8_t reg_val = 0x00;
	reg_val = read_regsiter(MPU6050_REG_WHO_AM_I);

	return reg_val == WHO_AM_I_VALUE;
}


//simple function to read register value
uint8_t read_regsiter(uint8_t reg_add)
{
	uint8_t reg_val = 0x00;
	(void)periph_i2c_rx(MPU6050_ADDRESS, reg_add, &reg_val);
	return reg_val;
}

//simple function to write to a register and return confirmation of write
bool write_regsiter(uint8_t reg_add, uint8_t reg_val)
{
	bool reg_write_okay = periph_i2c_rx(MPU6050_ADDRESS, reg_add, &reg_val);
	return reg_write_okay;
}
*/
