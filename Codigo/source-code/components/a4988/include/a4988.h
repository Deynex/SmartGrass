#ifndef A4988_H
#define A4988_H

#include "driver/gpio.h"
#include "driver/ledc.h"

// Configuración PWM
#define A4988_TIMER LEDC_TIMER_0
#define A4988_CHANNEL LEDC_CHANNEL_0

// Configuración del motor
#define STEPS_PER_REVOLUTION 200 // Motor típico de 1.8° (200 pasos/revolución)
#define MICROSTEPS 1             // Configura según los jumpers MS1, MS2, MS3 del A4988. 1 = Full step, 2 = Half step, 4 = 1/4, 8 = 1/8, 16 = 1/16
#define TOTAL_STEPS (STEPS_PER_REVOLUTION * MICROSTEPS)

// Velocidades en Hz (pasos por segundo)
// Equivalentes aproximados a las velocidades anteriores
#define SPEED_SLOW 500    // ~500 pasos/seg (equivalente a 2000μs = 0.5ms delay)
#define SPEED_MEDIUM 1000 // ~1000 pasos/seg (equivalente a 1000μs = 1ms delay)
#define SPEED_FAST 2000   // ~2000 pasos/seg (equivalente a 500μs = 0.5ms delay)

/**
 * @brief Estructura de configuración para el driver dual (vehículo)
 * Controla múltiples motores con STEP y ENABLE compartidos, pero DIR independientes
 */
typedef struct
{
    gpio_num_t step_pin;        // Pin STEP compartido para todos los motores
    gpio_num_t dir_pin_left;    // Pin DIR para motores del lado izquierdo
    gpio_num_t dir_pin_right;   // Pin DIR para motores del lado derecho
    gpio_num_t enable_pin;      // Pin ENABLE compartido (opcional, usar -1 si no se usa)
    uint32_t steps_per_rev;     // Pasos por revolución
    ledc_timer_t timer_num;     // Timer LEDC a usar
    ledc_channel_t channel_num; // Canal LEDC a usar
} a4988_dual_config_t;

/**
 * @brief Handle del driver dual (vehículo)
 */
typedef struct
{
    gpio_num_t step_pin;
    gpio_num_t dir_pin_left;
    gpio_num_t dir_pin_right;
    gpio_num_t enable_pin;
    uint32_t steps_per_rev;
    ledc_timer_t timer_num;
    ledc_channel_t channel_num;
    uint32_t current_frequency; // Frecuencia actual en Hz
    bool enabled;
    volatile bool running; // Flag para controlar la rotación continua
} a4988_dual_handle_t;

/**
 * @brief Dirección del motor
 */
typedef enum
{
    A4988_DIR_CW = 0, // Sentido horario
    A4988_DIR_CCW = 1 // Sentido antihorario
} a4988_direction_t;

/**
 * @brief Dirección del vehículo (modo dual)
 */
typedef enum
{
    A4988_DUAL_FORWARD = 0,  // Avanzar (ambos lados hacia adelante)
    A4988_DUAL_BACKWARD = 1, // Retroceder (ambos lados hacia atrás)
    A4988_DUAL_LEFT = 2,     // Girar a la izquierda (izq atrás, der adelante)
    A4988_DUAL_RIGHT = 3,    // Girar a la derecha (izq adelante, der atrás)
    A4988_DUAL_STOP = 4      // Detener
} a4988_dual_direction_t;

/**
 * @brief Inicializa el driver dual A4988 para vehículo
 *
 * @param config Configuración del driver dual
 * @param handle Puntero al handle que se creará
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_dual_init(const a4988_dual_config_t *config, a4988_dual_handle_t **handle);

/**
 * @brief Habilita los motores del vehículo
 *
 * @param handle Handle del driver dual
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_dual_enable(a4988_dual_handle_t *handle);

/**
 * @brief Deshabilita los motores del vehículo
 *
 * @param handle Handle del driver dual
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_dual_disable(a4988_dual_handle_t *handle);

/**
 * @brief Establece la velocidad del vehículo en RPM
 *
 * @param handle Handle del driver dual
 * @param rpm Velocidad en revoluciones por minuto
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_dual_set_speed(a4988_dual_handle_t *handle, float rpm);

/**
 * @brief Mueve el vehículo en una dirección específica de forma continua
 *
 * @param handle Handle del driver dual
 * @param direction Dirección del movimiento (FORWARD, BACKWARD, LEFT, RIGHT)
 * @param rpm Velocidad en revoluciones por minuto
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_dual_move(a4988_dual_handle_t *handle, a4988_dual_direction_t direction, float rpm);

/**
 * @brief Detiene el vehículo inmediatamente
 *
 * @param handle Handle del driver dual
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_dual_stop(a4988_dual_handle_t *handle);

/**
 * @brief Libera los recursos del driver dual
 *
 * @param handle Handle del driver dual
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_dual_free(a4988_dual_handle_t *handle);

#endif // A4988_H