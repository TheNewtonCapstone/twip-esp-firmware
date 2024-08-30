#pragma once

#include <mpu6050.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "imu_result.h"

typedef void (*imu_data_cb_t)(void* imu_data_ptr);

typedef struct {
  double ax;
  double ay;
  double az;
  double angx;
  double angy;
  double angz;
  double yaw;
  double pitch;
  double roll;
} imu_data_t;

typedef struct {
  i2c_master_dev_handle_t dev_handle;
  mpu6050_handle_t imu_handle;
  esp_timer_handle_t timer_handle;

  imu_data_cb_t
      data_cb;  // imu data is scoped to the imu_cb function, copy it if needed

  uint64_t timer_period_ns;
} imu_t;

// Creates an IMU object: if it can't allocate memory, it returns NULL
imu_t* create_imu(uint64_t timer_period_ns, imu_data_cb_t data_cb);
// Deletes an IMU object: IMU cannot be used afterwards
void delete_imu(imu_t* imu);

imu_result_t init_imu(uint8_t sda, uint8_t scl, int8_t i2c_address, imu_t* imu);

void imu_cb(void* imu_ptr);