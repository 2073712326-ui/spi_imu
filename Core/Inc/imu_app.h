#ifndef IMU_APP_H
#define IMU_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Call from the main loop. TIM6 only records when a sample is due.
void imu_poll(void);

extern volatile uint32_t imu_callback_count;
extern volatile uint32_t imu_sample_count;
extern volatile uint32_t imu_update_count;
extern volatile uint32_t imu_update_fail_count;
extern volatile float imu_roll_deg;
extern volatile float imu_pitch_deg;
extern volatile float imu_yaw_deg;

#ifdef __cplusplus
}
#endif

#endif
