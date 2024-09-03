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
#include <esp_system.h>
#include "imu.h"
#include "sdkconfig.h"
#include <cstring>
#include <array>
#include <string.h>
#include "driver/uart.h"
#include <driver/ledc.h>
#include <esp32/gpio.h>

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

#define PACKET_SIZE 16
const int DELAY_MS = 30;

extern "C" {
  void app_main(void);
}

// MOTOR CONSTANTS
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50000;
const int PWM_RESOLUTION = 8;
const float NO_LOAD_SPEED = 7.50f;
const float STALL_TORQUE = 1.05f;

struct Motor {
  gpio_num_t encoder;  // yellow
  gpio_num_t pwm;// blue
  gpio_num_t dir; // white
};

const Motor motor1 = { GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_19 };   // left
const Motor motor2 = { GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27 };   // right


static IMUData imu_data;


void init_uart(void);
void init_motors(void);
void imu_cb(IMUData data) { imu_data = data; }
void tx_task(void); // read imu_data and send it over uart
void set_torque(float torque);


void app_main() {
  init_uart();
  // xTaskCreate(task_initI2C, "task_initI2C", 2048, NULL, 5, NULL);
  xTaskCreate(tx_task, "task_display", 4096, NULL, 5, NULL);
}



static void rx_task(void* arg) {
  static const char* RX_TASK_TAG = "RX_TASK";
  esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
  uint8_t* data = (uint8_t*)malloc(RX_BUF_SIZE + 1);
  float motor_cmd_1;
  float motor_cmd_2;
  while (1) {
    const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, DELAY_MS / portTICK_PERIOD_MS);
    if (rxBytes > 0) {
      ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
      ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
      const
    }
  }
  free(data);
}


void tx_task(void*) {
  IMU imu = IMU(0x68);
  imu.init();
  imu.subscribe("imu", imu_cb);
  imu.start(100);


  // uart 
  char send_buf[PACKET_SIZE];

  while (true) {

    // clear 
    for (size_t i = 0; i < PACKET_SIZE; i++) {
      send_buf[i] = 0x00;
    }

    send_buf[0] = 's';  // starting character
    send_buf[1] = 'f'; // the type of data being sent
    send_buf[2] = 2; // sending 2 values, roll and angular velocity
    memcpy(send_buf + 3, &imu_data.gravity.x, sizeof(float)); // TODO : SEND PITCH
    memcpy(send_buf + 7, &imu_data.ang_vel.z, sizeof(float));

    ESP_LOGI("IMU", "Gravity: %f %f %f", imu_data.gravity.x, imu_data.gravity.y,
      imu_data.gravity.z);
    ESP_LOGI("IMU", "Linear Acc: %d %d %d", imu_data.linear_acc.x,
      imu_data.linear_acc.y, imu_data.linear_acc.z);
    ESP_LOGI("IMU", "Angular Vel: %d %d %d", imu_data.ang_vel.x,
      imu_data.ang_vel.y, imu_data.ang_vel.z);
    ESP_LOGI("IMU", "Orientation: %f %f %f", imu_data.orientation.x,
      imu_data.orientation.y, imu_data.orientation.z);

    vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
  }

  imu.stop();
  imu.unsubscribe("imu");
}

void init_uart(void) {

  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  // We won't use a buffer for sending data.
  uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
void set_torque(float torque, const Motor& motor) {
  float speed = NO_LOAD_SPEED * (torque) / STALL_TORQUE;
  gpio_set_level(motor.dir, (speed > 0) ? GPIO_LEVEL_LOW : GPIO_LEVEL_HIGH);
  int pwm = (int)((abs(speed) / NO_LOAD_SPEED * 150.f) + 95.f);
  ledcWrite(motor.pwm, pwm);
  vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
}

void init_motors() {
  // left
  pinMode(motor1.encoder, INPUT_PULLUP);
  pinMode(motor2.encoder, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motor1.encoder), ISR_L_ENCODER_CB, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2.encoder), ISR_R_ENCODER_CB, RISING);

  ledcAttach(motor1.pwm, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motor2.pwm, PWM_FREQ, PWM_RESOLUTION);

  pinMode(motor1.dir, OUTPUT);
  pinMode(motor2.dir, OUTPUT);

  ledcWrite(motor1.pwm, 255);
  ledcWrite(motor2.pwm, 255);

  delay(1000);

  ledcWrite(motor1.pwm, 0);
  ledcWrite(motor2.pwm, 0);
}