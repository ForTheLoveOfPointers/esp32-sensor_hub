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
 * This is the real difficult part. Compensation formulas must be applied to the data you read, so I should get that,
 * read the compensation registers and then use the compensated values to output the correct measurements. 
 * A few days may be needed, as I lack the required time, but the BME datasheet provides pretty much everything.
 */

static void bme280_read_calibration(bme280_calib_data_t *cal) {

    uint8_t buf1[25];
    uint8_t buf2[7];

    // T and P calibration (0x88–0xA1)
    bme280_register_read(0x88, buf1, 25);

    // H calibration part 1 (0xA1)
    cal->dig_H1 = buf1[25];

    // H calibration part 2 (0xE1–0xE7)
    bme280_register_read(0xE1, buf2, 7);

    cal->dig_T1 = (buf1[1] << 8)  | buf1[0];
    cal->dig_T2 = (buf1[3] << 8)  | buf1[2];
    cal->dig_T3 = (buf1[5] << 8)  | buf1[4];

    cal->dig_P1 = (buf1[7] << 8)  | buf1[6];
    cal->dig_P2 = (buf1[9] << 8)  | buf1[8];
    cal->dig_P3 = (buf1[11] << 8) | buf1[10];
    cal->dig_P4 = (buf1[13] << 8) | buf1[12];
    cal->dig_P5 = (buf1[15] << 8) | buf1[14];
    cal->dig_P6 = (buf1[17] << 8) | buf1[16];
    cal->dig_P7 = (buf1[19] << 8) | buf1[18];
    cal->dig_P8 = (buf1[21] << 8) | buf1[20];
    cal->dig_P9 = (buf1[23] << 8) | buf1[22];

    cal->dig_H2 = (buf2[1] << 8) | buf2[0];
    cal->dig_H3 = buf2[2];
    cal->dig_H4 = (buf2[3] << 4) | (buf2[4] & 0x0F);
    cal->dig_H5 = (buf2[5] << 4) | (buf2[4] >> 4);
    cal->dig_H6 = buf2[6];

} 


static int32_t t_fine;
static int32_t BME280_compensate_T_int32(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) – ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) – ((int32_t)dig_T1)) * ((adc_T>>4) – ((int32_t)dig_T1)))
    >> 12) *
    ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
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

   int32_t adc_P = ((int32_t)rx_data[0] << 12) | ((int32_t)rx_data[1] << 4) | ((int32_t)rx_data[2] >> 4);

    int32_t adc_T = ((int32_t)rx_data[3] << 12) | ((int32_t)rx_data[4] << 4) | ((int32_t)rx_data[5] >> 4);

    int32_t adc_H = ((int32_t)rx_data[6] << 8) |  (int32_t)rx_data[7];
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