#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "queue_shared.h"
#include "http_send_data.h"

#include "driver/bme280.h"

QueueHandle_t bme_queue;

void bme_task(void* parameter) {
    bme_init();
    while(true) {
        bme_loop_read();
    }
}

void app_main(void)
{

    bme_queue = xQueueCreate(2, sizeof(bme280_comp_data_t));

    xTaskCreatePinnedToCore(
        bme_task,
        "BME_Task",
        4096,               
        NULL,               
        2,                  
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        send_data_task,
        "Send_Data_Task",
        4096,
        NULL,
        2,
        NULL,
        0
    );
}
