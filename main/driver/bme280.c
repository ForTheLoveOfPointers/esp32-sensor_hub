#include "bme280.h"


static esp_err_t bme280_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void i2c_install(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = SDA_IO,
        .scl_io_num = SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .allow_pd = true
    };

    ESP_ERROR_CHECK( i2c_new_master_bus(&bus_cfg, bus_handle) );

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_addr = BME280_SENSOR_ADDR,
        .scl_speed_hz = SCL_SPEED_HZ
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

void bme_init(void) {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_install(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

}