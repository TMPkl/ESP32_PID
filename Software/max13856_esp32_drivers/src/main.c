#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max31865.h"

void app_main(void) {
    max31865_init(HSPI_HOST, GPIO_NUM_5);

    while (1) {
        float temp = max31865_read_temperature();
        printf("Temp: %.2f C\n", temp);

        uint8_t fault = max31865_read_fault();
        if (fault) {
            printf("Fault detected: 0x%02X\n", fault);
            max31865_clear_fault();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
