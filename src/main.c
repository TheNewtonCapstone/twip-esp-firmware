#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "string.h"

// DEFINES

static imu_data_t last_imu_data;

void imu_data_cb(void* imu_data_ptr) {
  imu_data_t* imu_data = (imu_data_t*)imu_data_ptr;
  last_imu_data = *imu_data;
}

void imu_task() {
  // Initialize the IMU
  imu_t* imu = create_imu(5000, imu_data_cb);
  if (imu == NULL) {
    ESP_LOGE("IMU", "Failed to create IMU object");
    return;
  }

  imu_result_t result =
      init_imu(GPIO_NUM_21, GPIO_NUM_22, MPU6050_I2C_ADDRESS, imu);
  if (result != IMU_OK) {
    ESP_LOGE("IMU", "Failed to initialize IMU: %d", result);
    delete_imu(imu);
    return;
  }

  while (1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("Accel: %f, %f, %f\n", last_imu_data.ax, last_imu_data.ay,
           last_imu_data.az);
    printf("Ang: %f, %f, %f\n", last_imu_data.angx, last_imu_data.angy,
           last_imu_data.angz);
    printf("Yaw: %f, Pitch: %f, Roll: %f\n", last_imu_data.yaw,
           last_imu_data.pitch, last_imu_data.roll);
  }
}

void app_main() { xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL); }