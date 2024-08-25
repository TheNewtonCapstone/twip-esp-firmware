#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

static const int RX_BUF_SIZE = 256;
// static const int TX_BUF_SIZE = 256;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

typedef struct {
  double roll;
  double ang_vel;
} IMUMessage;

typedef struct {
  double left_torque;
  double right_torque;
} MotorMessage;

void init(void) {
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  // We won't use a buffer for sending data.
  uart_driver_install(UART_NUM_1, RX_BUF_SIZE, 0, 0, NULL, 0);
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
}

int sendData(const char *logName, const void *data, const size_t len) {
  const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
  ESP_LOGI(logName, "Wrote %d bytes", txBytes);
  return txBytes;
}

static void tx_task(void *arg) {
  static const char *TX_TASK_TAG = "TX_TASK";
  esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
  // IMUMessage data = {1.0, 2.0};
  int data = 69;

  // ESP_LOGI(TX_TASK_TAG, "Attempting to send: (%f, %f)", data.roll,
  // data.ang_vel);

  while (1) {
    sendData(TX_TASK_TAG, &data, sizeof(data));
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

static void rx_task(void *arg) {
  static const char *RX_TASK_TAG = "RX_TASK";
  esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
  MotorMessage *data = (MotorMessage *)malloc(RX_BUF_SIZE);
  while (1) {
    const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE,
                                        1000 / portTICK_PERIOD_MS);

    if (rxBytes > 0) {
      ESP_LOGI(RX_TASK_TAG, "Read %d bytes, (%f, %f)", rxBytes,
               data->left_torque, data->right_torque);
      ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
    }
  }
  free(data);
}

void app_main() {
  init();
  xTaskCreate(rx_task, "uart_rx_task", 2048, NULL, configMAX_PRIORITIES - 1,
              NULL);
  xTaskCreate(tx_task, "uart_tx_task", 2048, NULL, configMAX_PRIORITIES - 1,
              NULL);
}