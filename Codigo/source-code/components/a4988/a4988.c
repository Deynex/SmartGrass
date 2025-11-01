#include <stdio.h>
#include <stdlib.h>
#include "a4988.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "A4988";

// Configuración PWM
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Resolución de 13 bits
#define LEDC_DUTY 4096                  // Duty cycle 50% (2^13 / 2)
#define LEDC_MAX_FREQ 4000             // Frecuencia máxima en Hz

esp_err_t a4988_dual_init(const a4988_dual_config_t *config, a4988_dual_handle_t **handle)
{
    if (config == NULL || handle == NULL)
    {
        ESP_LOGE(TAG, "Parámetros inválidos");
        return ESP_ERR_INVALID_ARG;
    }

    // Reservar memoria para el handle
    *handle = (a4988_dual_handle_t *)malloc(sizeof(a4988_dual_handle_t));
    if (*handle == NULL)
    {
        ESP_LOGE(TAG, "Error al reservar memoria");
        return ESP_ERR_NO_MEM;
    }

    // Copiar configuración
    (*handle)->step_pin = config->step_pin;
    (*handle)->dir_pin_left = config->dir_pin_left;
    (*handle)->dir_pin_right = config->dir_pin_right;
    (*handle)->enable_pin = config->enable_pin;
    (*handle)->steps_per_rev = config->steps_per_rev;
    (*handle)->timer_num = config->timer_num;
    (*handle)->channel_num = config->channel_num;
    (*handle)->current_frequency = 0;
    (*handle)->enabled = false;
    (*handle)->running = false;

    // Configurar timer LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = config->timer_num,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = 1000, // Frecuencia inicial
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al configurar timer LEDC");
        free(*handle);
        return ret;
    }

    // Configurar canal LEDC para el pin STEP compartido
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = config->channel_num,
        .timer_sel = config->timer_num,
        .gpio_num = config->step_pin,
        .duty = 0, // Inicialmente apagado
        .hpoint = 0,
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al configurar canal LEDC");
        free(*handle);
        return ret;
    }

    // Configurar pines DIR (izquierdo y derecho) como GPIO
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << config->dir_pin_left) | (1ULL << config->dir_pin_right),
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al configurar pines DIR");
        free(*handle);
        return ret;
    }
    gpio_set_level(config->dir_pin_left, 0);
    gpio_set_level(config->dir_pin_right, 0);

    // Configurar ENABLE pin compartido (si está definido)
    if (config->enable_pin != -1)
    {
        io_conf.pin_bit_mask = (1ULL << config->enable_pin);
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Error al configurar pin ENABLE");
            free(*handle);
            return ret;
        }
        gpio_set_level(config->enable_pin, 1); // HIGH = deshabilitado
    }

    ESP_LOGI(TAG, "A4988 DUAL inicializado correctamente");
    ESP_LOGI(TAG, "STEP: GPIO%d (compartido), DIR_LEFT: GPIO%d, DIR_RIGHT: GPIO%d, ENABLE: GPIO%d",
             config->step_pin, config->dir_pin_left, config->dir_pin_right, config->enable_pin);
    ESP_LOGI(TAG, "Timer: %d, Canal: %d", config->timer_num, config->channel_num);

    return ESP_OK;
}

esp_err_t a4988_dual_enable(a4988_dual_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->enable_pin != -1)
    {
        gpio_set_level(handle->enable_pin, 0); // LOW = habilitado
        handle->enabled = true;
        ESP_LOGI(TAG, "Motores del vehículo habilitados");
    }

    return ESP_OK;
}

esp_err_t a4988_dual_disable(a4988_dual_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Detener PWM si está corriendo
    if (handle->running)
    {
        a4988_dual_stop(handle);
    }

    if (handle->enable_pin != -1)
    {
        gpio_set_level(handle->enable_pin, 1); // HIGH = deshabilitado
        handle->enabled = false;
        ESP_LOGI(TAG, "Motores del vehículo deshabilitados");
    }

    return ESP_OK;
}

esp_err_t a4988_dual_set_speed(a4988_dual_handle_t *handle, float rpm)
{
    if (handle == NULL || rpm <= 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Calcular frecuencia en Hz desde RPM
    float frequency_hz = (rpm / 60.0f) * handle->steps_per_rev;

    if (frequency_hz > LEDC_MAX_FREQ)
    {
        ESP_LOGW(TAG, "Frecuencia limitada a %d Hz", LEDC_MAX_FREQ);
        frequency_hz = LEDC_MAX_FREQ;
    }

    // Actualizar frecuencia del timer
    esp_err_t ret = ledc_set_freq(LEDC_MODE, handle->timer_num, (uint32_t)frequency_hz);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al establecer frecuencia");
        return ret;
    }

    handle->current_frequency = (uint32_t)frequency_hz;
    ESP_LOGI(TAG, "Velocidad del vehículo: %.2f RPM = %d Hz", rpm, handle->current_frequency);

    return ESP_OK;
}

esp_err_t a4988_dual_move(a4988_dual_handle_t *handle, a4988_dual_direction_t direction, float rpm)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Detener primero si está en movimiento
    if (handle->running)
    {
        a4988_dual_stop(handle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Si la dirección es STOP, solo detener
    if (direction == A4988_DUAL_STOP)
    {
        return a4988_dual_stop(handle);
    }

    // Establecer velocidad
    esp_err_t ret = a4988_dual_set_speed(handle, rpm);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Configurar dirección según el movimiento deseado
    switch (direction)
    {
    case A4988_DUAL_FORWARD:
        // Ambos lados hacia adelante (misma dirección)
        gpio_set_level(handle->dir_pin_left, A4988_DIR_CW);
        gpio_set_level(handle->dir_pin_right, A4988_DIR_CW);
        ESP_LOGI(TAG, "Vehículo AVANZANDO a %.2f RPM", rpm);
        break;

    case A4988_DUAL_BACKWARD:
        // Ambos lados hacia atrás (misma dirección opuesta)
        gpio_set_level(handle->dir_pin_left, A4988_DIR_CCW);
        gpio_set_level(handle->dir_pin_right, A4988_DIR_CCW);
        ESP_LOGI(TAG, "Vehículo RETROCEDIENDO a %.2f RPM", rpm);
        break;

    case A4988_DUAL_LEFT:
        // Giro a la izquierda: izquierda atrás, derecha adelante
        gpio_set_level(handle->dir_pin_left, A4988_DIR_CCW);
        gpio_set_level(handle->dir_pin_right, A4988_DIR_CW);
        ESP_LOGI(TAG, "Vehículo GIRANDO A LA IZQUIERDA a %.2f RPM", rpm);
        break;

    case A4988_DUAL_RIGHT:
        // Giro a la derecha: izquierda adelante, derecha atrás
        gpio_set_level(handle->dir_pin_left, A4988_DIR_CW);
        gpio_set_level(handle->dir_pin_right, A4988_DIR_CCW);
        ESP_LOGI(TAG, "Vehículo GIRANDO A LA DERECHA a %.2f RPM", rpm);
        break;

    default:
        ESP_LOGE(TAG, "Dirección inválida");
        return ESP_ERR_INVALID_ARG;
    }

    // Pequeño delay para estabilizar las señales DIR
    vTaskDelay(pdMS_TO_TICKS(2));

    // Iniciar PWM continuo en el pin STEP compartido
    ledc_set_duty(LEDC_MODE, handle->channel_num, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, handle->channel_num);

    handle->running = true;

    return ESP_OK;
}

esp_err_t a4988_dual_stop(a4988_dual_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Detener PWM inmediatamente
    ledc_set_duty(LEDC_MODE, handle->channel_num, 0);
    ledc_update_duty(LEDC_MODE, handle->channel_num);

    handle->running = false;

    return ESP_OK;
}

esp_err_t a4988_dual_free(a4988_dual_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Detener si está en movimiento
    if (handle->running)
    {
        a4988_dual_stop(handle);
    }

    // Deshabilitar motores antes de liberar
    a4988_dual_disable(handle);

    // Detener el canal LEDC
    ledc_stop(LEDC_MODE, handle->channel_num, 0);

    // Liberar memoria
    free(handle);

    return ESP_OK;
}