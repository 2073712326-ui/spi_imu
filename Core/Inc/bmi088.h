//
// Created by qjy on 2026/10/18.
//

#ifndef SPI_IMU_BMI088_H
#define SPI_IMU_BMI088_H
#include <stdint.h>



#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t bmi088_spi_error_count;
extern volatile uint32_t bmi088_last_spi_error;
extern volatile uint8_t bmi088_accel_id;
extern volatile uint8_t bmi088_gyro_id;
// 0: OK; 1/2: accel/gyro ID; 3: power; 4/5: accel/gyro config; 6: SPI.
extern volatile uint8_t bmi088_init_error;

void bmi088_write_byte(uint8_t tx_data);
void bmi088_read_byte(uint8_t* rx_data,uint8_t length);
void bmi088_write_reg(uint8_t reg,uint8_t data);
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
uint8_t bmi088_init(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_accel_read_reg(uint8_t reg,uint8_t* data,uint8_t length);
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_gyro_read_reg(uint8_t reg,uint8_t* data,uint8_t length);

#ifdef __cplusplus
}
#endif



#endif //SPI_IMU_BMI088_H
