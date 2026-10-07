idf_component_register(
    SRCS "my_project.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_driver_gpio
        esp_event
        esp_netif
        esp_timer
        esp_wifi
        esp_adc
        esp_driver_ledc
        espressif__cjson
        mqtt
        nvs_flash
)
