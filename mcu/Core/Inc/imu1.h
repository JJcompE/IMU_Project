#include <stdint.h>

const uint8_t IMU1_I2C_ADDRESS = 0x6B;
const uint8_t IMU1_I2C_ADDRESS_WR = 0xD7;
const uint8_t IMU1_WHO_AM_I_REGISTER = 0x0F;
const uint8_t IMU1_DEVICE_ID = 0x70;
const uint8_t IMU1_Acc_Cntr1 = 0x10;
const uint8_t IMU1_Gyro_Cntr2 = 0x11;
const uint8_t IMU1_STATUS_REG = 0x1E;
const uint8_t IMU1_CTRL3_REG = 0x12;
const uint8_t IMU1_FUNC_CFG_ACCESS = 0x01;
const uint8_t IMU1_CTRL6 = 0x15;

const float GRAVITY = 9.81;
const float ACCEL_SENS = 0.061;
const float GYRO_SENS = 4.375;

typedef enum{
    IMU1_HAL_OK,
	IMU1_HAL_ERROR,
	IMU1_HAL_BUSY,
	IMU1_HAL_TIMEOUT,
    BAD_ID,
    CONNECTED,
    UNKNOWN
} IMU1_Status;

#include <stdint.h>

int16_t convert_to_int16(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

void convert_accel_data(int16_t *raw_accel_ini, const uint8_t *a_buff) {
    for (int i = 0; i < 3; ++i) {
        raw_accel_ini[i] = convert_to_int16(a_buff[2 * i + 1], a_buff[2 * i]);
    }
}
