/*
 * Display.c
 *
 *  Created on: 14.08.2017
 *      Author: darek
 */
// #include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "imu.h"
#include "sdkconfig.h"

extern "C" {
void app_main(void);
}

static IMUData imu_data;

void imu_cb(IMUData data) { imu_data = data; }

void task_display(void *) {
  IMU imu = IMU(0x68);
  imu.init();
  imu.subscribe("imu", imu_cb);
  imu.start(100);

  while (true) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI("IMU", "Gravity: %f %f %f", imu_data.gravity.x, imu_data.gravity.y,
             imu_data.gravity.z);
    ESP_LOGI("IMU", "Linear Acc: %d %d %d", imu_data.linear_acc.x,
             imu_data.linear_acc.y, imu_data.linear_acc.z);
    ESP_LOGI("IMU", "Angular Vel: %d %d %d", imu_data.ang_vel.x,
             imu_data.ang_vel.y, imu_data.ang_vel.z);
    ESP_LOGI("IMU", "Orientation: %f %f %f", imu_data.orientation.x,
             imu_data.orientation.y, imu_data.orientation.z);
  }

  imu.stop();
  imu.unsubscribe("imu");
}

void app_main() {
  // xTaskCreate(task_initI2C, "task_initI2C", 2048, NULL, 5, NULL);
  xTaskCreate(task_display, "task_display", 4096, NULL, 5, NULL);
}