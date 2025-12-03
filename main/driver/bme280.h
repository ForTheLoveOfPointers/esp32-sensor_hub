#pragma once
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define SDA_IO GPIO_NUM_21
#define SCL_IO GPIO_NUM_22
#define BME280_SENSOR_ADDR 0x77 // Default in the BME280
#define SCL_SPEED_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 2000
#define TAG "bme_driver"

typedef struct bme280_DeviceBus
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
} bme280_DeviceBus;

typedef struct {
    float temperature;
    float humidity;
    float pressure;
} bme280_data_t;



bme280_DeviceBus* bme_init(void);