idf_component_register(
    SRCS "ota_manager.cpp"
    INCLUDE_DIRS "."
    REQUIRES app_update bootloader_support esp_partition common
    PRIV_REQUIRES log
)
