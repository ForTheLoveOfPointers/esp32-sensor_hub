idf_component_register(SRCS "sensor_node.c" "driver/bme280.c"
                    INCLUDE_DIRS "." "./driver"
                    REQUIRES driver esp_driver_gpio nvs_flash
                    )
