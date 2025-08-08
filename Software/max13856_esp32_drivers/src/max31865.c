#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_heap_caps.h" // jeśli korzystasz z pamięci DMA
#include "max31865.h"

#define TAG "MAX31865"

#define MAX31865_CONFIG_REG  0x00
#define MAX31865_RTD_MSB_REG 0x01
#define MAX31865_RTD_LSB_REG 0x02
#define MAX31865_FAULT_REG   0x07

    
static spi_device_handle_t spi = NULL; // Handle for the SPI device
static int pt_type; 


esp_err_t max31865_init(spi_host_device_t host, int pt_type_, max31865_pinout_t pinout) {
    pt_type = pt_type_;

    spi_bus_config_t buscfg = {
        .mosi_io_num = pinout.spi_pinout.mosi,
        .miso_io_num = pinout.spi_pinout.miso,
        .sclk_io_num = pinout.spi_pinout.sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 1,
        .spics_io_num = pinout.cs,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;

    ret = spi_bus_add_device(host, &devcfg, &spi);
    if (ret != ESP_OK) return ret;

    uint8_t config = 0b11000010;  // Configuration register: VBIAS ON + Auto conversion + filtr 50 Hz + tryb 2 przewody
    uint8_t data[2] = {MAX31865_CONFIG_REG & 0x7F, config};

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = data,
    };

    return spi_device_transmit(spi, &t);
}

static uint8_t read_register(uint8_t reg) {
    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2];
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(spi, &t);
    return rx[1];
}

static void write_register(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg & 0x7F, val};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    spi_device_transmit(spi, &t);
}

uint16_t max31865_read_rtd_raw() {
    uint8_t msb = read_register(MAX31865_RTD_MSB_REG);
    uint8_t lsb = read_register(MAX31865_RTD_LSB_REG);
    return ((msb << 8) | lsb) >> 1;
}

double max31865_read_temperature() {
    #define A  3.9083e-3
    #define B -5.775e-7
    #define C -4.183e-12

    #define R0 500.0f
    uint16_t rtd = max31865_read_rtd_raw();

    double t = (rtd / R0 - 1) / A;
    double t_prev;
    int max_iter = 10;
    double epsilon = 1e-5;  

    for (int i = 0; i < max_iter; i++) {
        t_prev = t;

        double f = R0 * (1 + A * t + B * t * t + C * (t - 100) * t * t * t) - rtd;

        double df = R0 * (A + 2 * B * t + C * (4 * t * t - 300 * t + 30000));

        t = t - f / df;

        if (fabs(t - t_prev) < epsilon) {
            break;  // zbieżność osiągnięta
        }
    }
    return t;
}

uint8_t max31865_read_fault() {
    return read_register(MAX31865_FAULT_REG);
}

void max31865_clear_fault() {
    uint8_t config = read_register(MAX31865_CONFIG_REG);
    config |= 0x02;
    write_register(MAX31865_CONFIG_REG, config);
}
