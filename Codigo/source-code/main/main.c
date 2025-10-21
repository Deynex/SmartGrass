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
i2c_master_bus_handle_t bus_handle;     // Manejador del bus I2C
i2c_master_dev_handle_t mpu6050_handle; // Manejador del dispositivo MPU6050
int16_t accel_x_raw, accel_y_raw, accel_z_raw;
int16_t temp_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
int64_t start_time, end_time; // Variables para medir el tiempo de ejecución
SemaphoreHandle_t mutex;      // Mutex para proteger el acceso a la variable compartida
int data = 0;                 // Variable compartida entre tareas

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_1_BIT // Set duty resolution to 1 bits
#define LEDC_DUTY 1                    // Set duty to 50%. (2 ** 1) * 50% = 1
#define LEDC_FREQUENCY 500             // Frequency in Hertz. Set frequency at 500 Hz

#define MOTOR_STEP_PIN GPIO_NUM_19  // Pin STEP del A4988
#define MOTOR_DIR_PIN GPIO_NUM_18   // Pin DIR del A4988
#define MOTOR_ENABLE_PIN GPIO_NUM_5 // Pin ENABLE del A4988

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
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void pwm_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_STEP_PIN,
        .duty = 0, // Set duty to 0%
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Main
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
static const char *TAG = "MAIN";
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
    // Set duty to 0.1%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

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