#include <stdio.h>
#include "freertos/FreeRTOS.h"

#include "driver/bme280.h"

void bme_task(void* parameter) {
    bme_init();
    while(true) {
        bme_loop_read();
    }
}

void app_main(void)
{
    xTaskCreatePinnedToCore(
        bme_task,
        "BME Task",
        4096,               
        NULL,               
        1,                  
        NULL,
        1
    );
}
