idf_component_register(
    SRCS "main.c" "network.c"
    INCLUDE_DIRS "."
    REQUIRES esp_netif esp_eth esp_wifi nvs_flash driver
)
