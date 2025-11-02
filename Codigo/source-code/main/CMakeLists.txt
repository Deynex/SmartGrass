idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES esp_driver_i2c
                                  esp_driver_ledc
                                  esp_driver_gpio
                                  esp_timer
                                  a4988
                                  ap
                                  i2c
                                  mks_servo
                                  mpu6050
                                  server
                                  uart
                    INCLUDE_DIRS ".")