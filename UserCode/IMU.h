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
    void begin_gyro_calibration();//开始校准
    void add_sample_gyro();//新增一组最新的gyro_数据
    bool finish_gyro_calibration();//求平均，保存零偏





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
    //X、Y、Z轴零偏
    float gyro_bias_[3] = {};

    float gyro_sum_[3]{};//三轴角速度累计和
    std::uint32_t count_ = 0;//已累计样本数量








};



#endif //SPI_IMU_IMU_H
