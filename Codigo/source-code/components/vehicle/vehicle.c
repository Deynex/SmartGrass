/**
 * @file vehicle.c
 * @brief Implementación de la API de abstracción del vehículo
 * @author DEYNEX
 */

#include "vehicle.h"
#include "a4988.h"
#include "mks_servo42c.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "VEHICLE";

/**
 * @brief Dirección del motor MKS (definido por hardware)
 * Asumimos 0 = CW = Hacia adelante, 1 = CCW = Hacia atrás
 */
#define MKS_DIR_FORWARD 0
#define MKS_DIR_BACKWARD 1

/**
 * @brief Estructura interna del handle del vehículo
 */
struct vehicle_handle_s
{
    /** @brief Handle para el subsistema de 3 steppers NEMA (Lado Izq + Trasero Der) */
    a4988_dual_handle_t stepper_handle;

    /** @brief Último RPM configurado */
    float current_rpm;

    /** @brief Última dirección configurada */
    a4988_dual_direction_t current_direction;

    /** @brief Estado de habilitación */
    bool enabled;
};

// --- Implementación de Funciones Públicas ---

esp_err_t vehicle_init(const vehicle_config_t *config, vehicle_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL)
    {
        ESP_LOGE(TAG, "Parámetros de init inválidos");
        return ESP_ERR_INVALID_ARG;
    }

    // 1. Reservar memoria para el handle del vehículo
    vehicle_handle_t handle = (vehicle_handle_t)calloc(1, sizeof(struct vehicle_handle_s));
    if (handle == NULL)
    {
        ESP_LOGE(TAG, "Fallo al reservar memoria para el handle del vehículo");
        return ESP_ERR_NO_MEM;
    }

    // 2. Inicializar el subsistema de steppers A4988
    // Esto configurará los 3 motores NEMA
    esp_err_t ret = a4988_dual_init(&config->stepper_config, &handle->stepper_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Fallo al inicializar el subsistema A4988");
        free(handle);
        return ret;
    }

    // 3. Poner el MKS en un estado conocido (detenido y deshabilitado)
    // Asumimos que mks_init() ya fue llamado en app_main
    mks_stop();
    mks_set_state(MKS_DISABLE);

    handle->enabled = false;
    handle->current_rpm = 0;
    handle->current_direction = A4988_DUAL_STOP;

    *out_handle = handle;
    ESP_LOGI(TAG, "Componente de vehículo inicializado correctamente");
    return ESP_OK;
}

esp_err_t vehicle_enable(vehicle_handle_t handle)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;

    // Habilitar steppers NEMA
    esp_err_t ret = a4988_dual_enable(handle->stepper_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Fallo al habilitar steppers A4988");
        return ret;
    }

    // Habilitar servo MKS
    if (!mks_set_state(MKS_ENABLE))
    {
        ESP_LOGE(TAG, "Fallo al habilitar servo MKS");
        return ESP_FAIL;
    }

    handle->enabled = true;
    ESP_LOGI(TAG, "Vehículo habilitado");
    return ESP_OK;
}

esp_err_t vehicle_disable(vehicle_handle_t handle)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;

    // 1. Detener todo movimiento antes de deshabilitar
    vehicle_stop(handle);

    // 2. Deshabilitar steppers NEMA (corta energía)
    a4988_dual_disable(handle->stepper_handle);

    // 3. Deshabilitar servo MKS (corta energía)
    mks_set_state(MKS_DISABLE);

    handle->enabled = false;
    ESP_LOGI(TAG, "Vehículo deshabilitado");
    return ESP_OK;
}

esp_err_t vehicle_stop(vehicle_handle_t handle)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;

    // Detener steppers NEMA (detiene PWM)
    a4988_dual_stop(handle->stepper_handle);

    // Detener servo MKS (comando UART)
    mks_stop();

    handle->current_rpm = 0;
    handle->current_direction = A4988_DUAL_STOP;
    ESP_LOGI(TAG, "Vehículo detenido");
    return ESP_OK;
}

esp_err_t vehicle_move(vehicle_handle_t handle, a4988_dual_direction_t direction, float rpm)
{
    if (handle == NULL)
        return ESP_ERR_INVALID_ARG;

    // Si la dirección es STOP o RPM es 0, usar la función de parada
    if (direction == A4988_DUAL_STOP || rpm <= 0)
    {
        return vehicle_stop(handle);
    }

    // --- 1. Determinar la dirección del motor MKS (Delantero Derecho) ---
    // El MKS debe hacer lo que haga el lado DERECHO del vehículo.
    uint8_t mks_direction;

    switch (direction)
    {
    case A4988_DUAL_FORWARD: // Lado derecho va ADELANTE
    case A4988_DUAL_LEFT:    // Lado derecho va ADELANTE (para girar izq)
        mks_direction = MKS_DIR_FORWARD;
        break;

    case A4988_DUAL_BACKWARD: // Lado derecho va ATRÁS
    case A4988_DUAL_RIGHT:    // Lado derecho va ATRÁS (para girar der)
        mks_direction = MKS_DIR_BACKWARD;
        break;

    default:
        ESP_LOGE(TAG, "Dirección de movimiento inválida: %d", direction);
        return ESP_ERR_INVALID_ARG;
    }

    // --- 2. Enviar comando a los steppers NEMA ---
    // a4988_dual_move se encarga de los 3 motores NEMA
    // (Lado Izq, Trasero Der)
    esp_err_t ret = a4988_dual_move(handle->stepper_handle, direction, rpm);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Fallo al mover subsistema A4988");
        vehicle_stop(handle); // Detener todo si una parte falla
        return ret;
    }

    // --- 3. Enviar comando al servo MKS ---
    // mks_run se encarga del motor Delantero Derecho
    if (!mks_run(mks_direction, (uint16_t)rpm))
    {
        ESP_LOGE(TAG, "Fallo al mover subsistema MKS");
        vehicle_stop(handle); // Detener todo si una parte falla
        return ESP_FAIL;
    }

    // Guardar estado
    handle->current_rpm = rpm;
    handle->current_direction = direction;
    ESP_LOGI(TAG, "Vehículo moviéndose (Dir: %d, RPM: %.1f)", direction, rpm);

    return ESP_OK;
}

esp_err_t vehicle_free(vehicle_handle_t *handle_ptr)
{
    if (handle_ptr == NULL || *handle_ptr == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    vehicle_handle_t handle = *handle_ptr;

    // Poner el hardware en estado seguro
    vehicle_disable(handle);

    // Liberar el handle del subsistema A4988
    a4988_dual_free(&handle->stepper_handle);

    // Liberar la memoria del handle principal del vehículo
    free(handle);

    // Evitar "dangling pointers"
    *handle_ptr = NULL;

    ESP_LOGI(TAG, "Componente de vehículo liberado");
    return ESP_OK;
}