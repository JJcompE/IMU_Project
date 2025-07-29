#ifndef IMU1_H
#define IMU1_H

#include <stdint.h>

static const uint8_t IMU1_I2C_ADDRESS = 0x6B;
static const uint8_t IMU1_I2C_ADDRESS_WR = 0xD7;
static const uint8_t IMU1_WHO_AM_I_REGISTER = 0x0F;
static const uint8_t IMU1_DEVICE_ID = 0x70;
static const uint8_t IMU1_Acc_Cntr1 = 0x10;
static const uint8_t IMU1_Gyro_Cntr2 = 0x11;
static const uint8_t IMU1_STATUS_REG = 0x1E;
static const uint8_t IMU1_CTRL3_REG = 0x12;
static const uint8_t IMU1_FUNC_CFG_ACCESS = 0x01;
static const uint8_t IMU1_CTRL6 = 0x15;

static const float GRAVITY = 9.81f;
static const float ACCEL_SENS = 0.061f;
static const float GYRO_SENS = 4.375f;

typedef enum{
    IMU1_HAL_OK,
    IMU1_HAL_ERROR,
    IMU1_HAL_BUSY,
    IMU1_HAL_TIMEOUT,
    BAD_ID,
    CONNECTED,
    UNKNOWN
} IMU1_Status;

// Inline or static function definitions
static inline int16_t convert_to_int16(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

static inline void convert_accel_data(int16_t *raw_accel_ini, const uint8_t *a_buff) {
    for (int i = 0; i < 3; ++i) {
        raw_accel_ini[i] = convert_to_int16(a_buff[2 * i + 1], a_buff[2 * i]);
    }
}

#endif // IMU1_H
