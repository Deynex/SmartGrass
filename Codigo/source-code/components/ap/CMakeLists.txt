idf_component_register(SRCS "ap.c"
                    PRIV_REQUIRES nvs_flash
                                  esp_wifi
                    INCLUDE_DIRS "include")
