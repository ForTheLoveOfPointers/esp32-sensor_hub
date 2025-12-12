#include "http_send_data.h"

#define API_TOKEN "API Token"
#define INFLUX_DB_URL "url" // Ideally, have this with the queries or construct the query using definitions

static const char* TAG = "HTTP Client";

QueueHandle_t bme_queue;

static esp_http_client_handle_t create_config() {

    esp_http_client_config_t cfg = {
        .url = INFLUX_DB_URL,
        .method = HTTP_METHOD_POST,

    };

    esp_http_client_handle_t client_hdl = esp_http_client_init(&cfg);
    char token_header[128];
    int j = snprintf(token_header, "Token %s\0", API_TOKEN);
    if(j < 0) {
        ESP_LOGE(TAG, "Could not write the 'Authorization' token into buffer");
        return NULL;
    }
    if(esp_http_client_set_header(client_hdl, "Authorization", token_header) != ESP_OK) {
        ESP_LOGE(TAG, "Could not set the request 'Authorization' header");
        return NULL
    }
    return client_hdl;

}

void send_data_task(void) {
    esp_http_client_handle_t client_hdl = create_config();
    if(client_hdl == NULL) {
        return;
    }

    char time_data[512];
    while(true) {
        time_data = {0};
        if(xQueueReceive(bme_queue, &time_data, (TickType_t)5)) {
            esp_http_client_write(client_hdl, time_data, sizeof(time_data));
        }
        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}