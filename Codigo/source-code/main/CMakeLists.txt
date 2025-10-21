idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES esp_driver_i2c
                                  esp_driver_ledc
                                  esp_driver_gpio
                                  a4988
                                  i2c
                                  mpu6050
                                  esp_timer
                    INCLUDE_DIRS ".")