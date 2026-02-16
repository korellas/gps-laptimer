idf_component_register(
    SRCS "ble_ota.cpp"
    INCLUDE_DIRS "."
    REQUIRES bt common ota
    PRIV_REQUIRES log wifi_portal esp_timer
)
