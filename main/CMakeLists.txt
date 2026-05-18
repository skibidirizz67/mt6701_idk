idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES cli i2c_mt6701 esp_driver_i2c esp_driver_gpio
                       INCLUDE_DIRS "")