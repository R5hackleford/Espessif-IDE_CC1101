idf_component_register(
    SRCS "main.cpp" "../../../CC1101Radio.cpp"
    INCLUDE_DIRS "../../.."
    REQUIRES driver esp_timer esp_rom freertos)
