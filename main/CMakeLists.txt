idf_component_register(
                    SRCS "robot_io.c" "wifi_servo_server.c" "main.c"
                    
                    INCLUDE_DIRS "."
                    
                    EMBED_TXTFILES "certs/servercert.pem"
                                   "certs/prvtkey.pem"

                    REQUIRES esp_wifi esp_http_server esp_event esp_netif esp_driver_gpio esp_driver_ledc nvs_flash spiffs json driver)

spiffs_create_partition_image(spiffs ../spiffs FLASH_IN_PROJECT) # Create memory for SPIFFS