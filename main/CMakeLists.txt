idf_component_register(SRCS "http_send_data.c" "sensor_node.c" "driver/bme280.c" "http_send_data.c"
                    INCLUDE_DIRS "." "./driver"
                    REQUIRES driver esp_driver_gpio nvs_flash esp_http_client
                    )
