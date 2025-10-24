#include <stdio.h>
#include <stdlib.h>
#include "a4988.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "A4988";

// Configuración PWM
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_1_BIT // Resolución de 1 bit (50% duty cycle)
#define LEDC_DUTY 1                    // Duty cycle 50%
#define LEDC_MAX_FREQ 40000            // Frecuencia máxima en Hz

esp_err_t a4988_init(const a4988_config_t *config, a4988_handle_t **handle)
{
    if (config == NULL || handle == NULL)
    {
        ESP_LOGE(TAG, "Parámetros inválidos");
        return ESP_ERR_INVALID_ARG;
    }

    // Reservar memoria para el handle
    *handle = (a4988_handle_t *)malloc(sizeof(a4988_handle_t));
    if (*handle == NULL)
    {
        ESP_LOGE(TAG, "Error al reservar memoria");
        return ESP_ERR_NO_MEM;
    }

    // Copiar configuración
    (*handle)->step_pin = config->step_pin;
    (*handle)->dir_pin = config->dir_pin;
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
        .freq_hz = 1000, // Frecuencia inicial (se cambiará después)
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al configurar timer LEDC");
        free(*handle);
        return ret;
    }

    // Configurar canal LEDC para el pin STEP
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = config->channel_num,
        .timer_sel = config->timer_num,
        .intr_type = LEDC_INTR_DISABLE,
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

    // Configurar pin DIR como GPIO normal
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << config->dir_pin)};
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        free(*handle);
        return ret;
    }
    gpio_set_level(config->dir_pin, 0);

    // Configurar ENABLE pin (si está definido)
    if (config->enable_pin != -1)
    {
        io_conf.pin_bit_mask = (1ULL << config->enable_pin);
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK)
        {
            free(*handle);
            return ret;
        }
        gpio_set_level(config->enable_pin, 1); // HIGH = deshabilitado
    }

    ESP_LOGI(TAG, "A4988 inicializado correctamente");
    ESP_LOGI(TAG, "STEP: GPIO%d, DIR: GPIO%d, ENABLE: GPIO%d",
             config->step_pin, config->dir_pin, config->enable_pin);
    ESP_LOGI(TAG, "Timer: %d, Canal: %d", config->timer_num, config->channel_num);

    return ESP_OK;
}

esp_err_t a4988_enable(a4988_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->enable_pin != -1)
    {
        gpio_set_level(handle->enable_pin, 0); // LOW = habilitado
        handle->enabled = true;
        ESP_LOGI(TAG, "Motor habilitado");
    }

    return ESP_OK;
}

esp_err_t a4988_disable(a4988_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Detener PWM si está corriendo
    if (handle->running)
    {
        a4988_stop(handle);
    }

    if (handle->enable_pin != -1)
    {
        gpio_set_level(handle->enable_pin, 1); // HIGH = deshabilitado
        handle->enabled = false;
        ESP_LOGI(TAG, "Motor deshabilitado");
    }

    return ESP_OK;
}

esp_err_t a4988_set_direction(a4988_handle_t *handle, a4988_direction_t direction)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_set_level(handle->dir_pin, direction);
    vTaskDelay(pdMS_TO_TICKS(1)); // Pequeño delay para estabilizar la señal

    return ESP_OK;
}

esp_err_t a4988_set_speed(a4988_handle_t *handle, uint32_t frequency_hz)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (frequency_hz == 0)
    {
        ESP_LOGE(TAG, "La frecuencia debe ser mayor que 0");
        return ESP_ERR_INVALID_ARG;
    }

    if (frequency_hz > LEDC_MAX_FREQ)
    {
        ESP_LOGW(TAG, "Frecuencia limitada a %d Hz", LEDC_MAX_FREQ);
        frequency_hz = LEDC_MAX_FREQ;
    }

    // Actualizar frecuencia del timer
    esp_err_t ret = ledc_set_freq(LEDC_MODE, handle->timer_num, frequency_hz);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al establecer frecuencia");
        return ret;
    }

    handle->current_frequency = frequency_hz;
    ESP_LOGI(TAG, "Velocidad actualizada: %lu Hz (pasos/segundo)", frequency_hz);

    return ESP_OK;
}

esp_err_t a4988_step(a4988_handle_t *handle, uint32_t steps, a4988_direction_t direction)
{
    if (handle == NULL || steps == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Establecer dirección
    a4988_set_direction(handle, direction);

    // Si no hay frecuencia configurada, usar una por defecto
    if (handle->current_frequency == 0)
    {
        a4988_set_speed(handle, 1000); // 1000 Hz por defecto
    }

    // Calcular tiempo necesario en milisegundos
    uint32_t time_ms = (steps * 1000) / handle->current_frequency;

    // Iniciar PWM
    ledc_set_duty(LEDC_MODE, handle->channel_num, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, handle->channel_num);

    // Esperar el tiempo calculado
    vTaskDelay(pdMS_TO_TICKS(time_ms));

    // Detener PWM
    ledc_set_duty(LEDC_MODE, handle->channel_num, 0);
    ledc_update_duty(LEDC_MODE, handle->channel_num);

    return ESP_OK;
}

esp_err_t a4988_rotate_degrees(a4988_handle_t *handle, float degrees, a4988_direction_t direction)
{
    if (handle == NULL || degrees < 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Calcular número de pasos
    uint32_t steps = (uint32_t)((degrees / 360.0) * handle->steps_per_rev);

    ESP_LOGI(TAG, "Rotando %.2f grados (%lu pasos)", degrees, steps);

    return a4988_step(handle, steps, direction);
}

esp_err_t a4988_rotate_revolutions(a4988_handle_t *handle, float revolutions, a4988_direction_t direction)
{
    if (handle == NULL || revolutions < 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Calcular número de pasos
    uint32_t steps = (uint32_t)(revolutions * handle->steps_per_rev);

    ESP_LOGI(TAG, "Rotando %.2f revoluciones (%lu pasos)", revolutions, steps);

    return a4988_step(handle, steps, direction);
}

esp_err_t a4988_set_angular_speed(a4988_handle_t *handle, float rpm)
{
    if (handle == NULL || rpm <= 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Calcular frecuencia en Hz desde RPM
    // RPM = revoluciones por minuto
    // Frecuencia (Hz) = (RPM / 60) * steps_per_rev

    float frequency_hz = (rpm / 60.0f) * handle->steps_per_rev;

    ESP_LOGI(TAG, "Velocidad angular: %.2f RPM = %.2f Hz", rpm, frequency_hz);

    return a4988_set_speed(handle, (uint32_t)frequency_hz);
}

esp_err_t a4988_get_angular_speed(a4988_handle_t *handle, float *rpm)
{
    if (handle == NULL || rpm == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Calcular RPM desde la frecuencia actual
    // Frecuencia (Hz) = pasos por segundo
    // RPM = (Frecuencia / steps_per_rev) * 60

    *rpm = ((float)handle->current_frequency / handle->steps_per_rev) * 60.0f;

    ESP_LOGI(TAG, "Velocidad angular actual: %.2f RPM (%lu Hz)", *rpm, handle->current_frequency);

    return ESP_OK;
}

esp_err_t a4988_run_continuous(a4988_handle_t *handle, float rpm, a4988_direction_t direction)
{
    if (handle == NULL || rpm <= 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Detener rotación anterior si existe
    if (handle->running)
    {
        ESP_LOGW(TAG, "Deteniendo rotación continua anterior");
        a4988_stop(handle);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Establecer la velocidad angular
    esp_err_t ret = a4988_set_angular_speed(handle, rpm);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Establecer la dirección
    a4988_set_direction(handle, direction);

    // Iniciar PWM continuo
    ledc_set_duty(LEDC_MODE, handle->channel_num, LEDC_DUTY);
    ledc_update_duty(LEDC_MODE, handle->channel_num);

    handle->running = true;

    ESP_LOGI(TAG, "Rotación continua iniciada a %.2f RPM en dirección %s",
             rpm, direction == A4988_DIR_CW ? "CW" : "CCW");

    return ESP_OK;
}

esp_err_t a4988_stop(a4988_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->running)
    {
        ESP_LOGW(TAG, "El motor no está en rotación continua");
        return ESP_OK;
    }

    // Detener PWM inmediatamente
    ledc_set_duty(LEDC_MODE, handle->channel_num, 0);
    ledc_update_duty(LEDC_MODE, handle->channel_num);

    handle->running = false;

    ESP_LOGI(TAG, "¡EMERGENCY STOP! Motor detenido inmediatamente");

    return ESP_OK;
}

esp_err_t a4988_free(a4988_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // Detener rotación continua si está activa
    if (handle->running)
    {
        a4988_stop(handle);
    }

    // Deshabilitar motor antes de liberar
    a4988_disable(handle);

    // Detener el canal LEDC
    ledc_stop(LEDC_MODE, handle->channel_num, 0);

    // Liberar memoria
    free(handle);

    ESP_LOGI(TAG, "Recursos liberados");

    return ESP_OK;
}