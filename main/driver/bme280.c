#include "bme280.h"


/**
 * This function uses legacy method calls, becaus the BME280 needs code that's very fine tuned. 
 * WILL CHANGE IF API GET'S BETTER.
 * At this stage, the higher level ESP32 API doesn't provide enough options to do that
 * REPEATED START
 * that the IC asks for
 */
static esp_err_t bme280_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Step 1: Send the register address (write mode)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Step 2: Repeated start to switch to read mode
    i2c_master_start(cmd);

    // Step 3: Read bytes
    i2c_master_write_byte(cmd, (BME280_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);

    // Stop condition
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    return err;
}

/**
 * This function reads 8 bytes, that is:
 * TEMPERATURE: 3 BYTES
 * PRESSURE: 3 BYTES
 * HUMIDITY: 2 BYTES
 */
void bme280_read_from_device(i2c_master_dev_handle_t dev_handle) {
    uint8_t rx_data[8];
    if(bme280_register_read(0xF7, rx_data, 8) != ESP_OK) {
        ESP_LOGE(TAG, "Error reading sensor data");
    }

    int32_t adc_P = (rx_data[0] << 12) | (rx_data[1] << 4) | (rx_data[2] >> 4);
    int32_t adc_T = (rx_data[3] << 12) | (rx_data[4] << 4) | (rx_data[5] >> 4);
    int32_t adc_H = (rx_data[6] << 8)  |  rx_data[7];

}


static void i2c_install(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
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

bme280_DeviceBus* bme_init(void) {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_install(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    bme280_DeviceBus d_b_handle = {
        .bus_handle = bus_handle,
        .dev_handle = dev_handle
    };

    return &d_b_handle;
}