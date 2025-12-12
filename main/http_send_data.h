#pragma once
#include <stdio.h>
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "queue_shared.h"


void send_data_task(void);