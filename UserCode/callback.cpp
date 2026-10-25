#include "tim.h"
#include "bmi088.h"
#include "IMU.h"
#include "imu_app.h"
//
// Created by qjy on 2026/10/18.
//定义四个计数器
volatile uint32_t imu_callback_count = 0;
volatile uint32_t imu_sample_count = 0;//记录主循环读取一组传感器数据的次数
volatile uint32_t imu_update_count = 0;//记录成功解算的次数
volatile uint32_t imu_update_fail_count = 0;//记录失败次数

IMU imu;//创建一个名叫imu的对象


volatile float imu_roll_deg = 0.0f;
volatile float imu_pitch_deg = 0.0f;
volatile float imu_yaw_deg = 0.0f;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)//检查中断是否来自TIM6
    {
        ++imu_callback_count;
    }
}

void imu_poll(void)
{
    static uint32_t last_sample_tick = 0;
    static uint32_t last_update_tick = 0;
    const uint32_t sample_tick = imu_callback_count;
    if (sample_tick == last_sample_tick)
    {
        return;
    }
    last_sample_tick = sample_tick;
    ++imu_sample_count;


    const uint32_t spi_errors_before = bmi088_spi_error_count;
    imu.acc_calculate();
    imu.gyro_calculate();
    if (bmi088_spi_error_count != spi_errors_before)
    {
        ++imu_update_fail_count;
        return;
    }


    const float dt = (sample_tick - last_update_tick) * 0.002f;
    if (!imu.update_attitude(dt))
    {
        ++imu_update_fail_count;
        return;
    }
    last_update_tick = sample_tick;
    ++imu_update_count;
}
