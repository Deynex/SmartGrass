idf_component_register(SRCS "mpu6050.c"
                    PRIV_REQUIRES esp_driver_i2c
                                  i2c
                    INCLUDE_DIRS "include")
