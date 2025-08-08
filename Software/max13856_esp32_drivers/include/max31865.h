#ifndef MAX31865_H
#define MAX31865_H

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sck;
} SPI_Pinout;

typedef struct {
    gpio_num_t cs;
    gpio_num_t d_rdy_pin; 
    SPI_Pinout spi_pinout;
} max31865_pinout_t;

esp_err_t max31865_init(spi_host_device_t host, int pt_type, max31865_pinout_t pinout);
float max31865_read_temperature();
uint8_t max31865_read_fault();
void max31865_clear_fault();

#endif
