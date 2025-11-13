idf_component_register(SRCS "main.c"
                    REQUIRES
                        esp_timer
                        a4988
                        i2c
                        mks_servo42c
                        mpu6050
                        uart
                    INCLUDE_DIRS ".")