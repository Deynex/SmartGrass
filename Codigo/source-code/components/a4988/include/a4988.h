#ifndef A4988_H
#define A4988_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Ajusta estos pines según tu conexión ESP32 <-> A4988
#define A4988_STEP_PIN GPIO_NUM_19  // Pin STEP del A4988
#define A4988_DIR_PIN GPIO_NUM_18   // Pin DIR del A4988
#define A4988_ENABLE_PIN GPIO_NUM_5 // Pin ENABLE del A4988 (opcional, usa -1 si no lo usas)

// Configuración PWM
#define A4988_TIMER LEDC_TIMER_0
#define A4988_CHANNEL LEDC_CHANNEL_0

// Configuración del motor
#define STEPS_PER_REVOLUTION 200 // Motor típico de 1.8° (200 pasos/revolución)
#define MICROSTEPS 1             // Configura según los jumpers MS1, MS2, MS3 del A4988. 1 = Full step, 2 = Half step, 4 = 1/4, 8 = 1/8, 16 = 1/16
#define TOTAL_STEPS (STEPS_PER_REVOLUTION * MICROSTEPS)

// Velocidades en Hz (pasos por segundo)
// Equivalentes aproximados a las velocidades anteriores
#define SPEED_SLOW 500       // ~500 pasos/seg (equivalente a 2000μs = 0.5ms delay)
#define SPEED_MEDIUM 1000    // ~1000 pasos/seg (equivalente a 1000μs = 1ms delay)
#define SPEED_FAST 2000      // ~2000 pasos/seg (equivalente a 500μs = 0.5ms delay)


/**
 * @brief Estructura de configuración del driver A4988
 */
typedef struct
{
    gpio_num_t step_pin;        // Pin STEP (pulsos)
    gpio_num_t dir_pin;         // Pin DIR (dirección)
    gpio_num_t enable_pin;      // Pin ENABLE (habilitación, opcional, usar -1 si no se usa)
    uint32_t steps_per_rev;     // Pasos por revolución (típicamente 200 para motor 1.8°)
    ledc_timer_t timer_num;     // Timer LEDC a usar (LEDC_TIMER_0, LEDC_TIMER_1, etc.)
    ledc_channel_t channel_num; // Canal LEDC a usar (LEDC_CHANNEL_0, LEDC_CHANNEL_1, etc.)
} a4988_config_t;

/**
 * @brief Handle del motor A4988
 */
typedef struct
{
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t enable_pin;
    uint32_t steps_per_rev;
    ledc_timer_t timer_num;
    ledc_channel_t channel_num;
    uint32_t current_frequency; // Frecuencia actual en Hz
    bool enabled;
    volatile bool running; // Flag para controlar la rotación continua
} a4988_handle_t;

/**
 * @brief Dirección del motor
 */
typedef enum
{
    A4988_DIR_CW = 0, // Sentido horario
    A4988_DIR_CCW = 1 // Sentido antihorario
} a4988_direction_t;

/**
 * @brief Inicializa el driver A4988
 *
 * @param config Configuración del driver
 * @param handle Puntero al handle que se creará
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_init(const a4988_config_t *config, a4988_handle_t **handle);

/**
 * @brief Habilita el motor
 *
 * @param handle Handle del motor
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_enable(a4988_handle_t *handle);

/**
 * @brief Deshabilita el motor
 *
 * @param handle Handle del motor
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_disable(a4988_handle_t *handle);

/**
 * @brief Establece la dirección del motor
 *
 * @param handle Handle del motor
 * @param direction Dirección (CW o CCW)
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_set_direction(a4988_handle_t *handle, a4988_direction_t direction);

/**
 * @brief Establece la velocidad del motor (frecuencia de pulsos en Hz)
 *
 * @param handle Handle del motor
 * @param frequency_hz Frecuencia en Hz (pasos por segundo)
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_set_speed(a4988_handle_t *handle, uint32_t frequency_hz);

/**
 * @brief Establece la velocidad angular del motor en RPM (revoluciones por minuto)
 *
 * @param handle Handle del motor
 * @param rpm Velocidad en revoluciones por minuto
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_set_angular_speed(a4988_handle_t *handle, float rpm);

/**
 * @brief Mueve el motor un número específico de pasos
 *
 * @param handle Handle del motor
 * @param steps Número de pasos a mover
 * @param direction Dirección del movimiento
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_step(a4988_handle_t *handle, uint32_t steps, a4988_direction_t direction);

/**
 * @brief Rota el motor un número específico de grados
 *
 * @param handle Handle del motor
 * @param degrees Grados a rotar
 * @param direction Dirección del movimiento
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_rotate_degrees(a4988_handle_t *handle, float degrees, a4988_direction_t direction);

/**
 * @brief Rota el motor un número específico de revoluciones
 *
 * @param handle Handle del motor
 * @param revolutions Número de revoluciones
 * @param direction Dirección del movimiento
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_rotate_revolutions(a4988_handle_t *handle, float revolutions, a4988_direction_t direction);

/**
 * @brief Obtiene la velocidad angular actual del motor en RPM
 *
 * @param handle Handle del motor
 * @param rpm Puntero donde se guardará el valor de RPM
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_get_angular_speed(a4988_handle_t *handle, float *rpm);

/**
 * @brief Inicia la rotación continua del motor a una velocidad específica en RPM
 * Esta función inicia una tarea en segundo plano que mantiene el motor girando
 *
 * @param handle Handle del motor
 * @param rpm Velocidad en revoluciones por minuto
 * @param direction Dirección de rotación (CW o CCW)
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_run_continuous(a4988_handle_t *handle, float rpm, a4988_direction_t direction);

/**
 * @brief Detiene inmediatamente la rotación continua del motor (Emergency Stop)
 *
 * @param handle Handle del motor
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_stop(a4988_handle_t *handle);

/**
 * @brief Libera los recursos del driver
 *
 * @param handle Handle del motor
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t a4988_free(a4988_handle_t *handle);

#endif // A4988_H