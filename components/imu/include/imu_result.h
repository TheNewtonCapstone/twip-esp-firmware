#pragma once

#include <stdio.h>

typedef uint8_t imu_result_t;

#define IMU_OK 0
#define IMU_NULL_HANDLE 1
#define IMU_CONFIG_FAIL 2
#define IMU_WAKE_UP_FAIL 3
#define IMU_TIMER_FAIL 4
#define IMU_TIMER_START_FAIL 5
#define IMU_I2C_BUS_FAIL 6
#define IMU_I2C_DEVICE_FAIL 7