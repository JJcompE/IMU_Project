#include <cstdint>
#include <iostream>
#include <string>

using namespace std;

class IMU_1 {
private:
    const uint8_t IMU_I2C_ADDRESS = 0x6B;
    const uint8_t IMU_I2C_ADDRESS_WR = 0xD7;
    const uint8_t IMU_WHO_AM_I_REGISTER = 0xf;
    const uint8_t IMU_DEVICE_ID = 0x70;

    const uint8_t IMU_Acc_Cntr1 = 0x10;
    const uint8_t IMU_Gyro_Cntr2 = 0x11;
    const uint8_t IMU_Status = 0x1E;
    const uint8_t IMU_CTRL3 = 0x12;
    const uint8_t IMU_FUNC_CFG_ACCESS = 0x01;
    const uint8_t IMU_CTRL6 = 0x15;

public:
    bool init();

};
