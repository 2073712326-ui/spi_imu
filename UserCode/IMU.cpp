//
// Created by qjy on 2026/10/18.
//

#include "IMU.h"
#include <sys/_stdint.h>
#include "bmi088.h"
#include <cmath>

uint8_t rx_acc_data[6]{};//加速度计接收缓冲区
uint8_t rx_gyro_data[6]{};//陀螺仪接收缓冲区
//将两个字节拼成一个数据
int16_t acc_raw[3]{};
int16_t gyro_raw[3]{};




void IMU::acc_calculate()
{
    //1.设置/读取acc0x41寄存器中的量程range参数，并换算为量程系数
    std::uint8_t raw_range = 0;
    bmi088_accel_read_reg(0x41,&raw_range,1);

    //2.根据编码，确定对应的换算系数
    raw_range &= 0x03;
    float range_g = 3.0f * (1U << raw_range);
    float g = 9.80665f;
    acc_scale_ = range_g * g / 32768.0f;

    //3.读取acc0x12寄存器中的6位acc数据,高字节在前,低字节在后
    bmi088_gyro_read_reg(0x12,&raw_range,6);
    acc_raw[0] = static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(rx_acc_data[1]) << 8)
        | rx_acc_data[0]);//x轴数据


    acc_raw[1] = static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(rx_acc_data[3]) << 8)
        | rx_acc_data[2]);//y轴数据

    acc_raw[2] = static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(rx_acc_data[5]) << 8)
        | rx_acc_data[4]);//z轴数据

    //4.用量程系数将原始数据转换为常用单位m/s²
    accel_[0] = acc_raw[0] * acc_scale_;
    accel_[1] = acc_raw[1] * acc_scale_;
    accel_[2] = acc_raw[2] * acc_scale_;

}

void IMU::gyro_calculate()
{
    //读取量程编码
    std::uint8_t raw_range = 0;
    bmi088_gyro_read_reg(0x12,&raw_range,1);
    //仅0-4是有效编码
    if (raw_range > 4)
    {
        return;
    }
    //根据编码计算量程
    float range_dps = 2000.0f * (1U << raw_range);
    float pi = 3.14f;

    //换算系数
    float DEG_TO_RAD = pi / 180.0f;
    gyro_scale_ = range_dps / 32768.0f * DEG_TO_RAD;

    //读取三轴数据，共6字节
    std::uint8_t rx_gyro_data[6]{};
    bmi088_gyro_read_reg(0x02,&rx_gyro_data[0],6);

    //拼接为三个有符号的16为原始值
    std::int16_t gyro_raw[3]{};

    gyro_raw[0] = static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(rx_gyro_data[1]) << 8)
        | rx_gyro_data[0]);

    gyro_raw[1] = static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(rx_gyro_data[3]) << 8)
        | rx_gyro_data[2]);

    gyro_raw[2] = static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(rx_gyro_data[5]) << 8)
        | rx_gyro_data[4]);

    //换算为角速度
    gyro_[0] = gyro_raw[0] * gyro_scale_;
    gyro_[1] = gyro_raw[1] * gyro_scale_;
    gyro_[2] = gyro_raw[2] * gyro_scale_;

}
void IMU::begin_gyro_calibration()
{
    gyro_sum_[0] = 0.0f;
    gyro_sum_[1] = 0.0f;
    gyro_sum_[2] = 0.0f;

    count_ = 0;
}

void IMU::add_sample_gyro(){
    gyro_sum_[0] += gyro_[0];
    gyro_sum_[1] += gyro_[1];
    gyro_sum_[2] += gyro_[2];
    count_++;
}

bool IMU::finish_gyro_calibration(){
    if (count_ == 0)
    {
        return false;

    }
    gyro_bias_[0] = gyro_sum_[0] / count_;
    gyro_bias_[1] = gyro_sum_[1] / count_;
    gyro_bias_[2] = gyro_sum_[2] / count_;
    return true;
}

bool IMU::calculate_acc_angle(){
    //取出三轴加速度
    float ax = accel_[0];
    float ay = accel_[1];
    float az = accel_[2];
    //计算倾角
    roll_acc_ = std::atan2(ay,az);
    pitch_acc_ = std::atan2(-ax,std::sqrt(ay * ay + az * az));
    return true;

}
