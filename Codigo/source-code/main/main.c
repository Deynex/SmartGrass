#include <stdio.h>
#include "i2c.h"
#include "mpu6050.h"
#include "a4988.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"


// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Main
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
void app_main(void)
{
    // Manejador del bus I2C e inicialización
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_init(&bus_handle);

    // Manejador del dispositivo MPU6050 y adición al bus I2C
    i2c_master_dev_handle_t mpu6050_handle;
    i2c_add_device(bus_handle, &mpu6050_handle, MPU6050_ADDR, MPU6050_I2C_FREQ_HZ);

    // Inicialización del dispositivo MPU6050
    mpu6050_init(mpu6050_handle);

    // Configuración del rango del acelerómetro
    mpu6050_set_accel_range(mpu6050_handle, MPU6050_ACCEL_RANGE_2G);

    // Configuración del rango del giroscopio
    mpu6050_set_gyro_range(mpu6050_handle, MPU6050_GYRO_RANGE_250DPS);

    // Variables para almacenar los valores de los sensores
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    int16_t temp_raw;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;

    int64_t start_time, end_time;
    while (true)
    {
        start_time = esp_timer_get_time();

        mpu6050_read_all_raw(mpu6050_handle, &accel_x_raw, &accel_y_raw, &accel_z_raw, &temp_raw, &gyro_x_raw, &gyro_y_raw, &gyro_z_raw); // 511 us

        end_time = esp_timer_get_time();

        // Impresión de los valores crudos
        printf("Acelerómetro: X = %d, Y = %d, Z = %d\n", accel_x_raw, accel_y_raw, accel_z_raw);
        printf("Temperatura: %d\n", temp_raw);
        printf("Giroscopio: X = %d, Y = %d, Z = %d\n", gyro_x_raw, gyro_y_raw, gyro_z_raw);
        printf("Tiempo entre lecturas: %lld us\n", end_time - start_time);

        // Borra las últimas líneas
        printf("\033[A\033[K\033[A\033[K\033[A\033[K\033[A\033[K");

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}