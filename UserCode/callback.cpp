#include "tim.h"
#include "bmi088.h"
//
// Created by qjy on 2026/10/18.
//
uint8_t rx_data[8]{};
int16_t accel_raw[3]{};
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM6)
    {
        bmi088_accel_read_reg(0x12,rx_data,6);
        accel_raw[0] = (int16_t)((uint16_t)rx_data[1] << 8 | rx_data[0]);
        accel_raw[1] = (int16_t)((uint16_t)rx_data[3] << 8 | rx_data[2]);
        accel_raw[2] = (int16_t)((uint16_t)rx_data[5] << 8 | rx_data[4]);





    }
}

