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
#include "vehicle.h"

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Definiciones
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// Bus I2C maestro para el MPU6050
#define I2C_MASTER_SCL 22
#define I2C_MASTER_SDA 21
#define I2C_MASTER_NUM I2C_NUM_0

// Pines para el driver dual (4 motores: 2 izquierda, 2 derecha)
#define VEHICLE_STEP_PIN GPIO_NUM_23   // STEP compartido para los 4 motores
#define VEHICLE_DIR_LEFT GPIO_NUM_19   // DIR para motores izquierdos
#define VEHICLE_DIR_RIGHT GPIO_NUM_4   // DIR para motores derechos
#define VEHICLE_ENABLE_PIN GPIO_NUM_18 // ENABLE compartido para los 4 motores

// Configuración de hardware para ESTE vehículo
#define VEHICLE_LEDC_TIMER LEDC_TIMER_0
#define VEHICLE_LEDC_CHANNEL LEDC_CHANNEL_0
#define VEHICLE_STEPS_PER_REV 200 // Pasos por revolución (ej. 200 para 1.8°)
#define VEHICLE_MICROSTEPS 1      // Microstepping (ej. 1 para full-step)
#define VEHICLE_TOTAL_STEPS (VEHICLE_STEPS_PER_REV * VEHICLE_MICROSTEPS)

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
int64_t start_time, end_time;           // Variables para medir el tiempo de ejecución
vehicle_handle_t vehicle_handle = NULL; // Handle del vehículo
mks_encoder_data_t enc_data;            // Datos del encoder
mks_status_t status;                    // Estado del motor
int32_t pulses_received;                // Pulsos recibidos del encoder
float shaft_error;                      // Error angular del eje

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Core 0: Control
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
void control(void *pvParameters)
{
    while (true)
    {
        start_time = esp_timer_get_time();

        // Leer datos del MPU6050
        mpu6050_read_all_raw(mpu6050_handle, &accel_x_raw, &accel_y_raw, &accel_z_raw, &temp_raw, &gyro_x_raw, &gyro_y_raw, &gyro_z_raw); // ~573 us

        // Leer datos del encoder
        mks_read_encoder(&enc_data);
        mks_read_shaft_error_angle(&shaft_error);
        mks_read_pulses_received(&pulses_received);
        mks_get_status(&status);

        end_time = esp_timer_get_time();

        // Impresión de los valores crudos
        printf("Acelerómetro: X = %d, Y = %d, Z = %d\n", accel_x_raw, accel_y_raw, accel_z_raw);
        printf("Temperatura: %d\n", temp_raw);
        printf("Giroscopio: X = %d, Y = %d, Z = %d\n", gyro_x_raw, gyro_y_raw, gyro_z_raw);
        printf("Encoder: carry=%" PRId32 ", value=%u, angle=%.2f°, total_angle=%.2f°\n", enc_data.carry, enc_data.value, enc_data.angle_deg, enc_data.total_angle);
        printf("Shaft error: %.2f°\n", shaft_error);
        printf("Pulses received: %" PRId32 "\n", pulses_received);
        printf("Current status: mstep=%d, current=%d mA, enable=%d, speed=%d RPM\n", status.mstep, status.current, status.enable, status.speed);
        printf("Tiempo entre lecturas: %lld us\n", end_time - start_time);

        // Borra las últimas líneas
        printf("\033[A\033[K\033[A\033[K\033[A\033[K\033[A\033[K\033[A\033[K\033[A\033[K\033[A\033[K\033[A\033[K");

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Core 1: Movimiento
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
void motors(void *pvParameters)
{
    // Secuencia de prueba del vehículo
    while (true)
    {
        // Avanzar
        vehicle_move(vehicle_handle, A4988_DUAL_FORWARD, 180.0);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Detener
        vehicle_stop(vehicle_handle);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Retroceder
        vehicle_move(vehicle_handle, A4988_DUAL_BACKWARD, 180.0);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Apagar motores
        vehicle_disable(vehicle_handle);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Encender motores
        vehicle_enable(vehicle_handle);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Girar a la izquierda
        vehicle_move(vehicle_handle, A4988_DUAL_LEFT, 180.0);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Detener
        vehicle_stop(vehicle_handle);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Girar a la derecha
        vehicle_move(vehicle_handle, A4988_DUAL_RIGHT, 180.0);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Detener y pausa larga
        vehicle_stop(vehicle_handle);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void encoder_task(void *arg)
{
    while (true)
    {
        // Iniciar movimiento
        mks_run(1, 10);
        vTaskDelay(pdMS_TO_TICKS(10000));

        // Detener motor
        mks_stop();
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Iniciar movimiento en sentido opuesto
        mks_run(0, 10);
        vTaskDelay(pdMS_TO_TICKS(10000));

        // Detener motor
        mks_stop();
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Prueba de diferentes velocidades
        mks_run(1, 15);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(1, 30);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(1, 60);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(1, 90);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(1, 120);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(1, 150);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(1, 180);
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Detener motor antes de repetir

        mks_stop();
        vTaskDelay(pdMS_TO_TICKS(100));

        // Prueba de diferentes velocidades en sentido opuesto
        mks_run(0, 15);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(0, 30);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(0, 60);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(0, 90);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(0, 120);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(0, 150);
        vTaskDelay(pdMS_TO_TICKS(3000));
        mks_run(0, 180);
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Detener motor antes de repetir
        mks_stop();
        vTaskDelay(pdMS_TO_TICKS(100));

        // --- INICIO DEL PASO DE CORRECCIÓN ---
        // Al final del ciclo, leemos el desfase de pulsos y lo corregimos.

        int32_t pulse_drift = 0;
        if (mks_read_pulses_received(&pulse_drift))
        {
            ESP_LOGI("encoder_task", "Fin de ciclo. Desfase de pulsos: %" PRId32, pulse_drift);

            if (pulse_drift != 0)
            {
                // Determinar la dirección de corrección
                // Si el desfase es positivo (>0), movemos en reversa (dir 0)
                // Si el desfase es negativo (<0), movemos en adelante (dir 1)
                uint8_t dir = (pulse_drift > 0) ? 0 : 1;

                // El comando de pulsos espera un valor positivo
                uint32_t pulses_to_fix = (pulse_drift > 0) ? pulse_drift : -pulse_drift;

                ESP_LOGI("encoder_task", "Corrigiendo desfase... Moviendo %" PRIu32 " pulsos en dirección %d", pulses_to_fix, dir);

                // Usamos la nueva función de movimiento relativo
                // Usamos una velocidad baja (ej. 10 RPM) para una corrección precisa
                if (mks_move_relative_pulses(dir, 10, pulses_to_fix))
                {
                    ESP_LOGI("encoder_task", "Corrección completada. Pulsos deberían ser 0.");
                }
                else
                {
                    ESP_LOGE("encoder_task", "¡Fallo al corregir el desfase!");
                }
            }
        }
        else
        {
            ESP_LOGE("encoder_task", "No se pudo leer el desfase de pulsos.");
        }

        // Espera larga antes de iniciar el próximo ciclo
        ESP_LOGI("encoder_task", "Iniciando nuevo ciclo en 10 segundos...");
        vTaskDelay(pdMS_TO_TICKS(10000));
        // --- FIN DEL PASO DE CORRECCIÓN ---
    }
}

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Funciones
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------


// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
// MARK: Main
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
void app_main(void)
{
    esp_err_t ret; // Variable para comprobar los retornos

    // Inicialización del bus I2C y del dispositivo MPU6050
    ESP_LOGI(TAG, "Inicializando bus I2C...");
    ret = i2c_master_bus_init(&i2c_bus_handle, I2C_MASTER_SDA, I2C_MASTER_SCL, I2C_MASTER_NUM);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "¡Falló la inicialización del bus I2C!");
        return;
    }

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Añadiendo MPU6050 al bus I2C...");
        ret = i2c_add_device(i2c_bus_handle, &mpu6050_handle, MPU6050_ADDR, MPU6050_I2C_FREQ_HZ);
    }

    // Inicializar MPU6050
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Inicializando driver MPU6050...");
        ret = mpu6050_init(mpu6050_handle); // Asumiendo que mpu6050_init devuelve esp_err_t
    }

    // Configurar MPU6050
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Configurando rangos de MPU6050...");
        mpu6050_set_accel_range(mpu6050_handle, MPU6050_ACCEL_RANGE_2G);
        mpu6050_set_gyro_range(mpu6050_handle, MPU6050_GYRO_RANGE_250DPS);
    }

    // Comprobación final
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "¡Falló la inicialización de I2C/MPU6050!");
        return; // Detiene app_main
    }

    ESP_LOGI(TAG, "I2C y MPU6050 inicializados correctamente.");

    // Inicialización del UART
    uart_init();

    // Inicializar MKS SERVO42C
    vTaskDelay(pdMS_TO_TICKS(10000));
    mks_init();

    // Inicializar NVS, WiFi y servidor HTTP
    nvs_init();
    wifi_init();

    ESP_LOGI(TAG, "Inicializando componente de vehículo...");

    // 1. Definir la configuración para el subsistema de steppers
    a4988_dual_config_t stepper_config = {
        .step_pin = VEHICLE_STEP_PIN,
        .dir_left_pin = VEHICLE_DIR_LEFT,
        .dir_right_pin = VEHICLE_DIR_RIGHT,
        .enable_pin = VEHICLE_ENABLE_PIN,
        .steps_per_rev = VEHICLE_TOTAL_STEPS,
        .timer_num = VEHICLE_LEDC_TIMER,
        .channel_num = VEHICLE_LEDC_CHANNEL,
    };

    // 2. Envolverla en la configuración principal del vehículo
    vehicle_config_t vehicle_config = {
        .stepper_config = stepper_config};

    // 3. Inicializar el componente de vehículo
    ret = vehicle_init(&vehicle_config, &vehicle_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "¡Falló la inicialización del componente de vehículo!");
        return;
    }

    // 4. Habilitar los motores
    vehicle_enable(vehicle_handle);

    // Calibrar encoder al inicio
    if (!mks_calibrate_encoder())
    {
        ESP_LOGE("APP", "Error durante la calibración del encoder");
    }
    else
    {
        ESP_LOGI("APP", "Calibración del encoder exitosa");
    }

    xTaskCreatePinnedToCore(control, "control", 4096, NULL, configMAX_PRIORITIES - 1, NULL, PRO_CPU_NUM);           // Core 0: Control
    xTaskCreatePinnedToCore(motors, "motors", 4096, NULL, configMAX_PRIORITIES - 1, NULL, APP_CPU_NUM);             // Core 1: Motors
    //xTaskCreatePinnedToCore(encoder_task, "encoder_task", 4096, NULL, configMAX_PRIORITIES - 2, NULL, APP_CPU_NUM); // Core 1: Encoder
}