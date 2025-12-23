idf_component_register(
                    SRCS "main.c" "robot_io/robot_io.c" "uart_receive/uart_receive.c" "wifi_server/wifi_server.c" "cmd_control/cmd_control.c" "rt_stats/rt_stats.c"

                    INCLUDE_DIRS "." "robot_io" "uart_receive" "wifi_server" "cmd_control" "rt_stats"

                    EMBED_TXTFILES "certs/servercert.pem"
                                    "certs/prvtkey.pem"

                    REQUIRES esp_wifi esp_http_server esp_event esp_netif esp_driver_gpio esp_driver_ledc nvs_flash spiffs json driver console vfs esp_timer mdns)

target_compile_options(${COMPONENT_LIB} PRIVATE # Optimalization for reducing IRAM
    "-mlongcalls"
    "-ffunction-sections" 
    "-fdata-sections"
)

target_link_options(${COMPONENT_LIB} PRIVATE  # Optimalization for reducing IRAM
    "-Wl,--gc-sections"
)

target_compile_options(${COMPONENT_LIB} PRIVATE # Optimalization for reducing FLASH
    "-Os"
)

spiffs_create_partition_image(spiffs ../spiffs FLASH_IN_PROJECT) # Create memory for SPIFFS