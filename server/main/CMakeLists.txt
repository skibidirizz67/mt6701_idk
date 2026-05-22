idf_component_register(SRCS "main.c" "shared.c"
                       PRIV_REQUIRES esp_driver_uart cli i2c_mt6701 esp_driver_i2c esp_driver_gpio
                       INCLUDE_DIRS "include")