#include <stdio.h>
#include <inttypes.h>
#include "i2c.h"
#include "mpu6050.h"
#include "a4988.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "uart.h"
#include "mks_servo42c.h"
#include "soft_ap.h"
#include "vehicle.h"
#include "server.h"
#include "ws.h"

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Definiciones
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// Bus I2C
#define I2C_MASTER_SCL 22
#define I2C_MASTER_SDA 21
#define I2C_MASTER_NUM I2C_NUM_0

// Pines del Vehículo para el A4988 Dual
#define VEHICLE_STEP_PIN GPIO_NUM_23
#define VEHICLE_DIR_LEFT GPIO_NUM_19
#define VEHICLE_DIR_RIGHT GPIO_NUM_4
#define VEHICLE_ENABLE_PIN GPIO_NUM_18

// Configuración de hardware
#define VEHICLE_LEDC_TIMER LEDC_TIMER_0
#define VEHICLE_LEDC_CHANNEL LEDC_CHANNEL_0
#define VEHICLE_STEPS_PER_REV 200
#define VEHICLE_MICROSTEPS 16
#define VEHICLE_TOTAL_STEPS (VEHICLE_STEPS_PER_REV * VEHICLE_MICROSTEPS)

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Variables
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
static const char *TAG = "MAIN";
i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t mpu6050_handle;
vehicle_handle_t vehicle_handle = NULL;

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Core 0: Control
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
void control_telemetry_task(void *pvParameters)
{
    // Variables para almacenar los datos
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    int16_t temp_raw;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    mks_encoder_data_t enc_data;
    mks_status_t status;
    float shaft_error;

    // Buffer para construir el string JSON
    char json_buffer[512]; // Aumentado para asegurar que quepa todo

    while (true)
    {
        // Leer todos los datos de los sensores
        mpu6050_read_all_raw(mpu6050_handle, &accel_x_raw, &accel_y_raw, &accel_z_raw, &temp_raw, &gyro_x_raw, &gyro_y_raw, &gyro_z_raw);
        mks_read_encoder(&enc_data);
        mks_read_shaft_error_angle(&shaft_error);
        mks_get_status(&status);

        // Formatear los datos como un string JSON
        snprintf(json_buffer, sizeof(json_buffer),
                 "{\"accel_x\": %d, \"accel_y\": %d, \"accel_z\": %d, "
                 "\"gyro_x\": %d, \"gyro_y\": %d, \"gyro_z\": %d, "
                 "\"temp\": %d, \"enc_angle\": %.2f, \"enc_total\": %.2f, "
                 "\"enc_carry\": %" PRId32 ", \"shaft_error\": %.2f, "
                 "\"motor_speed\": %d, \"motor_current\": %d}",
                 accel_x_raw, accel_y_raw, accel_z_raw,
                 gyro_x_raw, gyro_y_raw, gyro_z_raw,
                 temp_raw, enc_data.angle_deg, enc_data.total_angle,
                 enc_data.carry, shaft_error,
                 status.speed, status.current);

        // 3. Enviar el JSON a todos los clientes WebSocket conectados
        ws_server_send_text_all(json_buffer);

        // 4. Esperar al siguiente ciclo
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Main
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
void app_main(void)
{
    // Inicialización de I2C y MPU6050
    esp_err_t ret;
    ret = i2c_master_bus_init(&i2c_bus_handle, I2C_MASTER_SDA, I2C_MASTER_SCL, I2C_MASTER_NUM);

    if (ret == ESP_OK)
    {
        ret = i2c_add_device(i2c_bus_handle, &mpu6050_handle, MPU6050_ADDR, MPU6050_I2C_FREQ_HZ);
    }

    if (ret == ESP_OK)
    {
        ret = mpu6050_init(mpu6050_handle);
    }

    if (ret == ESP_OK)
    {
        mpu6050_set_accel_range(mpu6050_handle, MPU6050_ACCEL_RANGE_2G);
        mpu6050_set_gyro_range(mpu6050_handle, MPU6050_GYRO_RANGE_250DPS);
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "¡Falló la inicialización de I2C/MPU6050!");
        return;
    }

    // Inicializar UART y MKS Servo
    uart_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    mks_init();

    // Inicializar Red y Servidor
    nvs_init();
    wifi_init();

    // Calibrar Encoder MKS
    if (!mks_calibrate_encoder())
    {
        ESP_LOGE(TAG, "Error durante la calibración del encoder");
    }
    else
    {
        ESP_LOGI(TAG, "Calibración del encoder exitosa");
    }

    // Inicializar Vehículo
    a4988_dual_config_t stepper_config = {
        .step_pin = VEHICLE_STEP_PIN,
        .dir_left_pin = VEHICLE_DIR_LEFT,
        .dir_right_pin = VEHICLE_DIR_RIGHT,
        .enable_pin = VEHICLE_ENABLE_PIN,
        .steps_per_rev = VEHICLE_TOTAL_STEPS,
        .timer_num = VEHICLE_LEDC_TIMER,
        .channel_num = VEHICLE_LEDC_CHANNEL,
    };
    vehicle_config_t vehicle_config = {
        .stepper_config = stepper_config,
    };
    ret = vehicle_init(&vehicle_config, &vehicle_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "¡Falló la inicialización del componente de vehículo!");
        return;
    }
    vehicle_enable(vehicle_handle);

    // Pasamos el vehicle_handle al servidor
    // El servidor, a su vez, se lo pasará al módulo WebSocket
    http_server_init(vehicle_handle);

    // Iniciar Tareas
    ESP_LOGI(TAG, "Iniciando tareas de aplicación...");
    xTaskCreatePinnedToCore(control_telemetry_task, "control_telemetry", 4096, NULL, 5, NULL, PRO_CPU_NUM); // Core 0: Control & Telemetría

    // La tarea 'motors' ya no se inicia, el control es ahora por WebSocket
    // xTaskCreatePinnedToCore(motors, "motors", 4096, NULL, 5, NULL, APP_CPU_NUM);
}