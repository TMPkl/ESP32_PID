#include "max31865.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "MAX31865"

#define MAX31865_CONFIG_REG  0x00
#define MAX31865_RTD_MSB_REG 0x01
#define MAX31865_RTD_LSB_REG 0x02
#define MAX31865_FAULT_REG   0x07

#define PT1000_TYPE 0
#define PT100_TYPE 1   
#define PT500_TYPE 2
    
static spi_device_handle_t spi;
static int pt_type; 
static int R_ref; // Reference resistance for PT1000, PT100, or PT500
static gpio_num_t cs; // Chip select pin
esp_err_t max31865_init(spi_host_device_t host, int pt_type_, int R_ref_, struct max31865_pinout pinout) {
    pt_type = pt_type_;
    R_ref = R_ref_;
    cs = pinout.cs;

    spi_bus_config_t buscfg = {
        .mosi_io_num = pinout.mosi,
        .miso_io_num = pinout.miso,
        .sclk_io_num = pinout.sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 1,
        .spics_io_num = cs,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;

    ret = spi_bus_add_device(host, &devcfg, &spi);
    if (ret != ESP_OK) return ret;

    uint8_t config = 0b11000010;
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

float max31865_read_temperature() {
    uint16_t rtd = max31865_read_rtd_raw();
    float resistance = rtd * R_ref / 32768.0; // dla Rref = 400Ω
    float temp = (resistance - 100.0) / 0.385; // dla PT100//////////////////////////////////////todo, dopisać poprawną kalkulację dla róznych typów czujników
    return temp;
}

uint8_t max31865_read_fault() {
    return read_register(MAX31865_FAULT_REG);
}

void max31865_clear_fault() {
    uint8_t config = read_register(MAX31865_CONFIG_REG);
    config |= 0x02;
    write_register(MAX31865_CONFIG_REG, config);
}
