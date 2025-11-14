#include <stdio.h>
#include "i2c.h"
#include "mpu6050.h"
#include "a4988.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "uart.h"
#include "mks_servo42c.h"
#include "soft_ap.h"

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Definiciones
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// Pines para el driver dual (4 motores: 2 izquierda, 2 derecha)
#define VEHICLE_STEP_PIN GPIO_NUM_19  // STEP compartido para los 4 motores
#define VEHICLE_DIR_LEFT GPIO_NUM_4   // DIR para motores izquierdos
#define VEHICLE_DIR_RIGHT GPIO_NUM_18 // DIR para motores derechos
#define VEHICLE_ENABLE_PIN GPIO_NUM_5 // ENABLE compartido para los 4 motores

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Variables
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
static const char *TAG = "MAIN";
// static const char *TAG_CORE_0 = "Control";
// static const char *TAG_CORE_1 = "Motores";
i2c_master_bus_handle_t i2c_bus_handle; // Manejador del bus I2C
i2c_master_dev_handle_t mpu6050_handle; // Manejador del dispositivo MPU6050
int16_t accel_x_raw, accel_y_raw, accel_z_raw;
int16_t temp_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
int64_t start_time, end_time;               // Variables para medir el tiempo de ejecución
SemaphoreHandle_t mutex;                    // Mutex para proteger el acceso a variables compartidas
a4988_dual_handle_t *vehicle_handle = NULL; // Manejador del driver dual para el vehículo

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Core 0: Control
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
void control(void *pvParameters)
{
    while (true)
    {
        start_time = esp_timer_get_time();

        mpu6050_read_all_raw(mpu6050_handle, &accel_x_raw, &accel_y_raw, &accel_z_raw, &temp_raw, &gyro_x_raw, &gyro_y_raw, &gyro_z_raw); // ~573 us

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

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Core 1: Motores
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
void motors(void *pvParameters)
{
    // Secuencia de prueba del vehículo
    while (true)
    {
        // Avanzar
        a4988_dual_move(vehicle_handle, A4988_DUAL_FORWARD, 180.0);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Detener
        a4988_dual_stop(vehicle_handle);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Retroceder
        a4988_dual_move(vehicle_handle, A4988_DUAL_BACKWARD, 180.0);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Apagar motores
        a4988_dual_disable(vehicle_handle);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Encender motores
        a4988_dual_enable(vehicle_handle);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Girar a la izquierda
        a4988_dual_move(vehicle_handle, A4988_DUAL_LEFT, 180.0);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Detener
        a4988_dual_stop(vehicle_handle);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Girar a la derecha
        a4988_dual_move(vehicle_handle, A4988_DUAL_RIGHT, 180.0);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Detener y pausa larga
        a4988_dual_stop(vehicle_handle);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Funciones
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
static void vehicle_init(void)
{
    // Configuración del driver dual para el vehículo (4 motores)
    a4988_dual_config_t vehicle_config = {
        .step_pin = VEHICLE_STEP_PIN,
        .dir_pin_left = VEHICLE_DIR_LEFT,
        .dir_pin_right = VEHICLE_DIR_RIGHT,
        .enable_pin = VEHICLE_ENABLE_PIN,
        .steps_per_rev = TOTAL_STEPS,
        .timer_num = A4988_TIMER,
        .channel_num = A4988_CHANNEL,
    };

    // Inicializar el driver dual
    esp_err_t ret = a4988_dual_init(&vehicle_config, &vehicle_handle);
    if (ret != ESP_OK)
    {
        return;
    }

    // Habilitar los motores
    a4988_dual_enable(vehicle_handle);
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Main
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
void app_main(void)
{
    // Inicialización del bus I2C y del dispositivo MPU6050
    i2c_master_bus_init(&i2c_bus_handle);
    i2c_add_device(i2c_bus_handle, &mpu6050_handle, MPU6050_ADDR, MPU6050_I2C_FREQ_HZ);

    // Inicialización del dispositivo MPU6050
    mpu6050_init(mpu6050_handle);

    // Configuración del rango del acelerómetro
    mpu6050_set_accel_range(mpu6050_handle, MPU6050_ACCEL_RANGE_2G);

    // Configuración del rango del giroscopio
    mpu6050_set_gyro_range(mpu6050_handle, MPU6050_GYRO_RANGE_250DPS);

    // Inicializar NVS, WiFi y servidor HTTP
    nvs_init();
    wifi_init();

    // Inicialización de los motores del vehículo
    vehicle_init();

    // Crear mutex
    mutex = xSemaphoreCreateMutex();

    if (mutex == NULL)
    {
        ESP_LOGE(TAG, "Error al crear mutex");
        return;
    }

    xTaskCreatePinnedToCore(control, "control", 4096, NULL, 10, NULL, PRO_CPU_NUM); // Core 0: Control
    xTaskCreatePinnedToCore(motors, "motors", 4096, NULL, 10, NULL, APP_CPU_NUM);   // Core 1: Motors
}