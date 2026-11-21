idf_component_register(
    SRCS "CC1101Radio.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_spi esp_timer esp_rom freertos)
