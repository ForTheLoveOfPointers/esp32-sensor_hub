idf_component_register(SRCS "sensor_task.c" "sensor_node.c" "driver/bme280.c"
                    INCLUDE_DIRS "." "./driver"
                    REQUIRES esp_driver_gpio
                    )
