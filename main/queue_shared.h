#pragma once
#include "freertos/queue.h"

typedef struct {

    int32_t comp_T;
    uint32_t comp_P;
    uint32_t comp_H;

} bme280_comp_data_t;

extern QueueHandle_t bme_queue;