//
// Created by qjy on 2026/10/18.
//
#include <cstdint>
#ifndef SPI_IMU_IMU_H
#define SPI_IMU_IMU_H



class IMU {
public:
    void acc_calculate();
    void gyro_calculate();





private:
    //读取寄存器数据
    std::uint8_t rx_acc_data[6]{};
    std::uint8_t rx_gyro_data[6]{};
    //拼接后的三轴数据
    std::int16_t acc_raw[3]{};
    std::int16_t gyro_raw[3]{};

    // X、Y、Z 轴加速度，单位 m/s²
    float accel_[3]{};
    float gyro_[3]{};

    //量程系数
    float acc_scale_ = 0.0f;
    float gyro_scale_ = 0.0f;





};



#endif //SPI_IMU_IMU_H
