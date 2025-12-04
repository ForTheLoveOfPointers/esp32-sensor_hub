#include "bme280.h"


static bme280_calib_data_t cal;

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

static void bme280_read_calibration(void) {

    uint8_t buf1[26];
    uint8_t buf2[7];

    // T and P calibration (0x88–0xA1)
    bme280_register_read(0x88, buf1, 26);

    // H calibration part 1 (0xA1)
    cal.dig_H1 = buf1[25];

    // H calibration part 2 (0xE1–0xE7)
    bme280_register_read(0xE1, buf2, 7);

    cal.dig_T1 = (buf1[1] << 8)  | buf1[0];
    cal.dig_T2 = (buf1[3] << 8)  | buf1[2];
    cal.dig_T3 = (buf1[5] << 8)  | buf1[4];

    cal.dig_P1 = (buf1[7] << 8)  | buf1[6];
    cal.dig_P2 = (buf1[9] << 8)  | buf1[8];
    cal.dig_P3 = (buf1[11] << 8) | buf1[10];
    cal.dig_P4 = (buf1[13] << 8) | buf1[12];
    cal.dig_P5 = (buf1[15] << 8) | buf1[14];
    cal.dig_P6 = (buf1[17] << 8) | buf1[16];
    cal.dig_P7 = (buf1[19] << 8) | buf1[18];
    cal.dig_P8 = (buf1[21] << 8) | buf1[20];
    cal.dig_P9 = (buf1[23] << 8) | buf1[22];

    cal.dig_H2 = (buf2[1] << 8) | buf2[0];
    cal.dig_H3 = buf2[2];
    cal.dig_H4 = (buf2[3] << 4) | (buf2[4] & 0x0F); // For later: stored as dig_H4 [11:4] / [3:0]
    cal.dig_H5 = (buf2[5] << 4) | (buf2[4] >> 4);
    cal.dig_H6 = buf2[6];

} 


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
static int32_t t_fine;
static int32_t BME280_compensate_T_int32(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) – ((int32_t)cal.dig_T1<<1))) * ((int32_t)cal.dig_T2)) >> 11;
    var2 = (((((adc_T>>4) – ((int32_t)cal.dig_T1)) * ((adc_T>>4) – ((int32_t)cal.dig_T1)))
    >> 12) *
    ((int32_t)cal.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P_int64(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) – 128000;
    var2 = var1 * var1 * (int64_t)cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)cal.dig_P5)<<17);
    var2 = var2 + (((int64_t)cal.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)cal.dig_P3)>>8) + ((var1 * (int64_t)cal.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)cal.dig_P1)>>33;
    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)cal.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)cal.dig_P7)<<4);
    return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine – ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) – (((int32_t)cal.dig_H4) << 20) – (((int32_t)cal.dig_H5) *
    v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
    ((int32_t)cal.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)cal.dig_H3)) >> 11) +
    ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)cal.dig_H2) +
    8192) >> 14));
    v_x1_u32r = (v_x1_u32r – (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
    ((int32_t)cal.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}

/**
 * This function reads 8 bytes, that is:
 * TEMPERATURE: 3 BYTES
 * PRESSURE: 3 BYTES
 * HUMIDITY: 2 BYTES
 */
static void bme280_read_from_device(bme280_comp_data_t *bme_data) {
    uint8_t rx_data[8];
    if(bme280_register_read(0xF7, rx_data, 8) != ESP_OK) {
        ESP_LOGE(TAG, "Error reading sensor data");
    }

    int32_t adc_P = ((int32_t)rx_data[0] << 12) | ((int32_t)rx_data[1] << 4) | ((int32_t)rx_data[2] >> 4);

    int32_t adc_T = ((int32_t)rx_data[3] << 12) | ((int32_t)rx_data[4] << 4) | ((int32_t)rx_data[5] >> 4);

    int32_t adc_H = ((int32_t)rx_data[6] << 8) |  (int32_t)rx_data[7];


    int32_t comp_T = BME280_compensate_T_int32(adc_T);
    uint32_t comp_P = BME280_compensate_P_int64(adc_P);
    uint32_t comp_H = bme280_compensate_H_int32(adc_H);

    bme_data->comp_T = comp_T;
    bme_data->comp_P = comp_P;
    bme_data->comp_H = comp_H;


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

    bme280_comp_data_t bme_data;

    while(true) {
        bme280_read_from_device(&bme280_data_t);
        ESP_LOGI(TAG, "Data read as T: %d, P: %u, H: %u", bme_data.comp_T, bme_data.comp_P, bme_data.comp_H);
        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }

    return &d_b_handle;
}