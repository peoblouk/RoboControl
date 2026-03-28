idf_component_register(
                    SRCS "main.c" "robot_io/robot_io.c" "wifi_server/wifi_server.c" "wifi_server/ws_handlers.c" "wifi_server/http_handlers.c" "wifi_server/file_manager.c" "cmd_control/cmd_control.c" "rt_stats/rt_stats.c" "gcode/gcode.c" "status_led/status_led.c" "can_communication/can_communication.c"

                    INCLUDE_DIRS "." "robot_io" "wifi_server" "cmd_control" "rt_stats" "gcode" "status_led" "can_communication"

                    EMBED_TXTFILES "certs/servercert.pem"
                                    "certs/prvtkey.pem"

                    REQUIRES esp_wifi esp_http_server esp_event esp_netif esp_driver_gpio esp_driver_ledc nvs_flash spiffs json driver console vfs esp_timer mdns esp_adc led_strip)

idf_build_get_property(idf_target IDF_TARGET)

target_compile_options(${COMPONENT_LIB} PRIVATE # Optimalization for reducing IRAM
    "-ffunction-sections"
    "-fdata-sections"
)

if(idf_target STREQUAL "esp32" OR idf_target STREQUAL "esp32s2" OR idf_target STREQUAL "esp32s3")
    target_compile_options(${COMPONENT_LIB} PRIVATE
        "-mlongcalls"
    )
endif()

target_link_options(${COMPONENT_LIB} PRIVATE  # Optimalization for reducing IRAM
    "-Wl,--gc-sections"
)

target_compile_options(${COMPONENT_LIB} PRIVATE # Optimalization for reducing FLASH
    "-Os"
)

spiffs_create_partition_image(spiffs ../spiffs FLASH_IN_PROJECT) # Create memory for SPIFFS
