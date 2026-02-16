idf_component_register(
    SRCS
        "main.cpp"
        "waveshare_display.cpp"
        "ublox_gps.cpp"
        "finish_line.cpp"
        "lap_storage.cpp"
        "serial_commands.cpp"
        "../font/generated/share_tech_mono_24.c"
        "../font/generated/share_tech_mono_32.c"
        "../font/generated/share_tech_mono_56.c"
    INCLUDE_DIRS
        "."
        "../font/generated"
    REQUIRES
        driver
        esp_adc
        esp_timer
        nvs_flash
        spiffs
        esp_lcd
        lvgl
        geo
        track
        timing
        modes
        common
        wifi_portal
        rtc
        imu
        sdcard
        ota
        ble_ota
)

# Font files need this to find lvgl.h properly
target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_LVGL_H_INCLUDE_SIMPLE)
