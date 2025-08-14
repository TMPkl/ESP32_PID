// Example usage of MAX31865 with ESP32 using your pinout:
// SCK -> GPIO19, CS -> GPIO5, SDO(MISO) -> GPIO4, SDI(MOSI) -> GPIO23

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max31865.h"

void app_main(void) {
    // Define pin mapping per your note: CK/SCK=D19, CS=D5, SDO=D4 (MISO), SDI=D23 (MOSI)
    max31865_pinout_t pinout = {
        .cs = GPIO_NUM_5,
        .d_rdy_pin = (gpio_num_t)-1, // not used in this example
        .spi_pinout = {
            .mosi = GPIO_NUM_23,
            .miso = GPIO_NUM_4,
            .sck  = GPIO_NUM_19,
        },
    };

    // Use VSPI host; choose PT type (100 or 1000). We'll assume PT100 here.
    if (max31865_init(VSPI_HOST, 500, pinout) != ESP_OK) {
        printf("MAX31865 init failed\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

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
