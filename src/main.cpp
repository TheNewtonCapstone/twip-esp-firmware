/*
 * Display.c
 *
 *  Created on: 14.08.2017
 *      Author: darek
 */
// #include <driver/i2c.h>
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "imu.h"
#include "sdkconfig.h"
#include <cstring>
#include <array>
#include <string.h>
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/gpio.h"


const int DELAY_MS = 50;

// uart communication 
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define TX_BUFFER_SIZE 16 // sending packet
static const int RX_BUF_SIZE = 1024;

// MOTOR CONSTANTS
const float NO_LOAD_SPEED = 7.50f;
const float STALL_TORQUE = 1.05f;
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define DUTY_RES                LEDC_TIMER_8_BIT// Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (50000) // Frequency in Hertz. Set frequency at 4 kHz


struct Motor {
  gpio_num_t encoder;  // yellow
  gpio_num_t pwm;// blue
  gpio_num_t dir; // white
};

const Motor motor1 = { GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_19 };   // left
const Motor motor2 = { GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27 };   // right


static IMUData imu_data;


extern "C" {
  void app_main(void);
}

void init_uart(void);
void init_motors(void);
void imu_cb(IMUData data) { imu_data = data; }
static void tx_task(void*); // read imu_data and send it over uart
static void rx_task(void*); // receive motor_cmd and set torque of the motors
void set_torque(float torque, const Motor& motor);


void app_main() {
  init_uart();
  init_motors();
  // xTaskCreate(task_initI2C, "task_initI2C", 2048, NULL, 5, NULL);
  xTaskCreate(tx_task, "tx_task", 4096, NULL, 5, NULL);
  xTaskCreate(rx_task, "rx_task", 4096, NULL, 5, NULL);
}

static void rx_task(void* arg) {
  static const char* RX_TASK_TAG = "RX_TASK";
  esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
  char read_buf[RX_BUF_SIZE];
  float motor_cmd_1;
  float motor_cmd_2;
  while (1) {
    // clear buffer
    for (size_t i = 0; i < RX_BUF_SIZE; i++) {
      read_buf[i] = 0x00;
    }
    const int rxBytes = uart_read_bytes(UART_NUM_1, read_buf, RX_BUF_SIZE, DELAY_MS / portTICK_PERIOD_MS);

    if (rxBytes <= 0) {
      ESP_LOGI(RX_TASK_TAG, "No bytes, received\n");
      continue;
    }
    ESP_LOGI(RX_TASK_TAG, "Motor cmd : %f\t%f", motor_cmd_1, motor_cmd_2);
    // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, read_buf);


    memcpy(&motor_cmd_1, read_buf + 3, sizeof(float));
    memcpy(&motor_cmd_2, read_buf + 7, sizeof(float));

    set_torque(motor_cmd_1, motor1);
    set_torque(motor_cmd_2, motor2);

  }
}


static void tx_task(void* arg) {
  // read imu_value and send them
  static const char* TX_TASK_TAG = "TX_TASK";
  esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
  IMU imu = IMU(0x68);
  imu.init();
  imu.subscribe("imu", imu_cb);
  imu.start(100);


  // uart 
  char send_buf[TX_BUFFER_SIZE];

  while (true) {
    vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);

    // clear  biffer
    for (size_t i = 0; i < TX_BUFFER_SIZE; i++) {
      send_buf[i] = 0x00;
    }


    ESP_LOGE("IMU", "Gravity: %f %f %f", imu_data.gravity.x, imu_data.gravity.y,
      imu_data.gravity.z);
    ESP_LOGE("IMU", "Linear Acc: %d %d %d", imu_data.linear_acc.x,
      imu_data.linear_acc.y, imu_data.linear_acc.z);
    ESP_LOGE("IMU", "Angular Vel: %d %d %d", imu_data.ang_vel.x,
      imu_data.ang_vel.y, imu_data.ang_vel.z);
    ESP_LOGE(TX_TASK_TAG, "Orientation: %f %f %f", imu_data.orientation.x,
      imu_data.orientation.y, imu_data.orientation.z);

    send_buf[0] = 's';  // starting character
    send_buf[1] = 'f'; // the type of data being sent
    send_buf[2] = 2; // sending 2 values, roll and angular velocity

    // send values first 4 bytes are for the roll and next 4 bytes are for the angular velocity
    memcpy(send_buf + 3, &imu_data.orientation.x, sizeof(float)); // TODO : SEND PITCH
    memcpy(send_buf + 7, &imu_data.orientation.z, sizeof(float));
    uart_write_bytes(UART_NUM_1, &send_buf, sizeof(send_buf));
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
  torque = std::max(-STALL_TORQUE, std::min(STALL_TORQUE, torque));
  gpio_set_level(motor.dir, (torque > 0) ? 1 : 0);
  float speed = NO_LOAD_SPEED * (std::abs(torque) / STALL_TORQUE);
  uint32_t duty = (uint32_t)(speed / NO_LOAD_SPEED) * ((1 << DUTY_RES) - 1);


    // int pwm = (int)((abs(speed) / NO_LOAD_SPEED * 150.f) + 95.f);
  ledc_set_duty(LEDC_MODE, motor.pwm == motor1.pwm ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1, duty);
  ledc_update_duty(LEDC_MODE, motor.pwm == motor1.pwm ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1);
  vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
}

void init_motors() {

  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = DUTY_RES,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = LEDC_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
  };

  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the motor PWM channel configuration
  ledc_channel_config_t m1_channel = {
      .gpio_num = motor1.pwm,
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER,
      .duty = 0, // Set duty to 0%
      .hpoint = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&m1_channel));

  ledc_channel_config_t m2_channel = {
      .gpio_num = motor2.pwm,
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL_1,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER,
      .duty = 0, // Set duty to 0%
      .hpoint = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&m2_channel));

      // Configure direction pins
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << motor1.dir) | (1ULL << motor2.dir),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&io_conf));


  set_torque(0, motor1);
  set_torque(0, motor1);

  ESP_LOGI("Motoros", "GPIO DUMP : \n");
  gpio_dump_io_configuration(stdout, (1ULL << 18) | (1ULL << 5) | (1ULL << 19));
  gpio_dump_io_configuration(stdout, (1ULL << 12) | (1ULL << 14) | (1ULL << 27));

}