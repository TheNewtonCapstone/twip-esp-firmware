#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main() {
  char str[100];

  while (true) {
    scanf("%s", str);
    printf("Hello world! %s\n", str);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}