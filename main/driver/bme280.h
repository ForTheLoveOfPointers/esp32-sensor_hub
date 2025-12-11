#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "../queue_shared.h"

#define SDA_IO GPIO_NUM_21
#define SCL_IO GPIO_NUM_22
#define BME280_SENSOR_ADDR 0x77 // Default in the BME280
#define SCL_SPEED_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 2000
#define TAG "bme_driver"


typedef struct {

    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

} bme280_calib_data_t;




typedef struct {
    float temperature;
    float humidity;
    float pressure;
} bme280_data_t;


void bme_loop_read(void);
void bme_init(void);