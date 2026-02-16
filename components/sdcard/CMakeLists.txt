idf_component_register(
    SRCS "sdcard_manager.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver sdmmc fatfs common esp_vfs_console
)
