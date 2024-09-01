#include "imu.h"

#include <stdio.h>

#include "driver/i2c_master.h"
#include "imu_result.h"
#include "mpu6050.h"

imu_t *create_imu(uint64_t timer_period_ns, imu_data_cb_t data_cb)
{
  imu_t *imu = (imu_t *)malloc(sizeof(imu_t));
  if (imu == NULL)
  {
    return NULL;
  }

  imu->data_cb = data_cb;
  imu->timer_period_ns = timer_period_ns;

  imu->imu_handle = NULL;
  imu->timer_handle = NULL;

  return imu;
}

void delete_imu(imu_t *imu)
{
  if (imu->imu_handle != NULL)
  {
    mpu6050_delete(imu->imu_handle);
  }

  if (imu->timer_handle != NULL)
  {
    esp_timer_stop(imu->timer_handle);
    esp_timer_delete(imu->timer_handle);
  }

  if (imu->dev_handle != NULL)
  {
    i2c_master_bus_rm_device(imu->dev_handle);
  }

  free(imu);

  imu = NULL;
}

imu_result_t init_imu(uint8_t sda, uint8_t scl, int8_t i2c_address,
                      imu_t *imu)
{
  i2c_master_bus_config_t bus_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .sda_io_num = sda,
      .scl_io_num = scl,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  i2c_master_bus_handle_t bus_handle;
  esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
  if (err != ESP_OK)
  {
    return IMU_I2C_BUS_FAIL;
  }

  i2c_device_config_t device_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = i2c_address,
      .scl_speed_hz = 400000,
  };

  err = i2c_master_bus_add_device(bus_handle, &device_config, &imu->dev_handle);
  if (err != ESP_OK)
  {
    return IMU_I2C_DEVICE_FAIL;
  }

  imu->imu_handle = mpu6050_create(&imu->dev_handle);
  if (imu->imu_handle == NULL)
  {
    return IMU_NULL_HANDLE;
  }

  err = mpu6050_config(imu->imu_handle, ACCE_FS_4G, GYRO_FS_500DPS);
  if (err != ESP_OK)
  {
    return IMU_CONFIG_FAIL;
  }

  err = mpu6050_wake_up(imu->imu_handle);
  if (err != ESP_OK)
  {
    return IMU_WAKE_UP_FAIL;
  }

  const esp_timer_create_args_t timer_args = {
      .callback = imu_cb,
      .arg = imu,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "imu_timer",
  };

  err = esp_timer_create(&timer_args, &imu->timer_handle);
  if (err != ESP_OK)
  {
    return IMU_TIMER_FAIL;
  }

  err = esp_timer_start_periodic(imu->timer_handle, imu->timer_period_ns);
  if (err != ESP_OK)
  {
    return IMU_TIMER_START_FAIL;
  }

  return IMU_OK;
}

void imu_cb(void *imu_ptr)
{
  imu_t *imu = (imu_t *)imu_ptr;

  mpu6050_acce_value_t acce;
  mpu6050_raw_acce_value_t raw_acce;
  mpu6050_gyro_value_t gyro;
  mpu6050_raw_gyro_value_t raw_gyro;
  complimentary_angle_t angle;

  mpu6050_get_acce(imu->imu_handle, &acce);
  // mpu6050_get_raw_acce(imu->imu_handle, &raw_acce);
  mpu6050_get_gyro(imu->imu_handle, &gyro);
  // mpu6050_get_raw_gyro(imu->imu_handle, &raw_gyro);
  mpu6050_complimentory_filter(imu->imu_handle, &acce, &gyro, &angle);

  imu_data_t imu_data = {
      .ax = acce.acce_x,
      .ay = acce.acce_y,
      .az = acce.acce_z,
      /* .ax = raw_acce.raw_acce_x,
      .ay = raw_acce.raw_acce_y,
      .az = raw_acce.raw_acce_z, */
      .angx = gyro.gyro_x,
      .angy = gyro.gyro_y,
      .angz = gyro.gyro_z,
      /* .angx = raw_gyro.raw_gyro_x,
      .angy = raw_gyro.raw_gyro_y,
      .angz = raw_gyro.raw_gyro_z, */
      .pitch = angle.pitch,
      .roll = angle.roll,
  };

  imu->data_cb(&imu_data);
}
