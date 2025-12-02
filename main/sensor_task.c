#include "sensor_task.h"

void read_bme(void) {
    bme280_DeviceBus* buses = bme_init();

    while (1)
    {
        bme280_register_read(buses.dev_handle, BME280_SENSOR_ADDR, )
    }
    
}