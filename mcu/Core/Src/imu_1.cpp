#include "imu_1.h"

uint8_t IMU_1::init(I2C_HandleTypeDef &i2c) {
	uint8_t read_dev_id = 0;
    uint8_t stat = HAL_I2C_Mem_Read(&i2c, (IMU_I2C_ADDRESS << 1), IMU_WHO_AM_I_REGISTER, 1, 0, 1, HAL_MAX_DELAY);

    if (stat != HAL_OK) {
    	return stat;
    }

    if (read_dev_id == IMU_DEVICE_ID) {
    	return 0;
    }

    return -1;
}
