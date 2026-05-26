idf_component_register(SRCS "i2c_control.c" 
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_driver_i2c esp_driver_gpio vfs esp_driver_uart)
