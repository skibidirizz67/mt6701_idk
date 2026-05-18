idf_component_register(SRCS "cli.c" 
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES i2c_mt6701 esp_driver_i2c esp_driver_gpio vfs esp_driver_uart)
