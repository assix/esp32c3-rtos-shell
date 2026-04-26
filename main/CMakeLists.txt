idf_component_register(SRCS "c3-shell-os.c"
                    INCLUDE_DIRS "."
                    REQUIRES spiffs vfs nvs_flash esp_wifi esp_netif 
                             esp_http_client console esp_event lwip
                             esp_driver_i2c esp_driver_gpio esp_timer)