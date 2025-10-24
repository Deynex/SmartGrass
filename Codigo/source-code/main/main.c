#include <stdio.h>
#include "i2c.h"
#include "mpu6050.h"
#include "a4988.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Variables
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
static const char *TAG = "MAIN";
i2c_master_bus_handle_t bus_handle;     // Manejador del bus I2C
i2c_master_dev_handle_t mpu6050_handle; // Manejador del dispositivo MPU6050
int16_t accel_x_raw, accel_y_raw, accel_z_raw;
int16_t temp_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
int64_t start_time, end_time; // Variables para medir el tiempo de ejecución
SemaphoreHandle_t mutex;      // Mutex para proteger el acceso a la variable compartida
int data = 0;                 // Variable compartida entre tareas
a4988_handle_t *motor = NULL; // Manejador del motor A4988

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Tareas
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// static const char *TAG_CORE_0 = "Control";
void control(void *pvParameters)
{
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

// static const char *TAG_CORE_1 = "Motores";
void motors(void *pvParameters)
{
    // Implementar lógica de motores aquí
    while (true)
    {
        ESP_LOGI(TAG, "Iniciando rotación continua a 180 RPM en sentido CW");
        a4988_run_continuous(motor, 180.0, A4988_DIR_CW);
        ESP_LOGI(TAG, "El motor está girando continuamente...");
        vTaskDelay(pdMS_TO_TICKS(50000)); // Girar durante 50 segundos

        ESP_LOGI(TAG, "Deteniendo motor...");
        a4988_stop(motor);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void pwm_init(void)
{
    // Configuración del motor
    a4988_config_t motor_config = {
        .step_pin = A4988_STEP_PIN,
        .dir_pin = A4988_DIR_PIN,
        .enable_pin = A4988_ENABLE_PIN,
        .steps_per_rev = TOTAL_STEPS,
        .timer_num = A4988_TIMER,
        .channel_num = A4988_CHANNEL};

    // Inicializar el motor
    esp_err_t ret = a4988_init(&motor_config, &motor);
    if (ret != ESP_OK)
    {
        printf("Error al inicializar el motor A4988\n");
        return;
    }

    // Habilitar el motor
    a4988_enable(motor);
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Main
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
void app_main(void)
{
    // Inicialización del bus I2C y del dispositivo MPU6050
    i2c_master_bus_init(&bus_handle);
    i2c_add_device(bus_handle, &mpu6050_handle, MPU6050_ADDR, MPU6050_I2C_FREQ_HZ);

    // Inicialización del dispositivo MPU6050
    mpu6050_init(mpu6050_handle);

    // Configuración del rango del acelerómetro
    mpu6050_set_accel_range(mpu6050_handle, MPU6050_ACCEL_RANGE_2G);

    // Configuración del rango del giroscopio
    mpu6050_set_gyro_range(mpu6050_handle, MPU6050_GYRO_RANGE_250DPS);

    // Set the LEDC peripheral configuration
    pwm_init();

    // Crear mutex
    mutex = xSemaphoreCreateMutex();

    if (mutex == NULL)
    {
        ESP_LOGE(TAG, "Error al crear mutex");
        return;
    }

    xTaskCreatePinnedToCore(control, "Tarea_Core_0", 2048, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(motors, "Tarea_Core_1", 2048, NULL, 10, NULL, 1);
}