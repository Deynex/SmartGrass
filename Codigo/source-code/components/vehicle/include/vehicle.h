/**
 * @file vehicle.h
 * @brief API de abstracción para controlar el vehículo de 4 motores (3 NEMA + 1 MKS)
 * @author DEYNEX
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include "esp_err.h"
#include "a4988.h"        // Necesario para el tipo a4988_dual_config_t y a4988_dual_direction_t
#include "mks_servo42c.h" // Necesario para las definiciones de MKS

/**
 * @brief Handle opaco para el componente del vehículo
 */
struct vehicle_handle_s;
typedef struct vehicle_handle_s *vehicle_handle_t;

/**
 * @brief Configuración completa del vehículo
 *
 * Contiene la configuración para el subsistema de steppers A4988.
 * El MKS_Servo42c se asume inicializado por separado (como en tu main.c)
 * ya que su API es estática y basada en UART.
 */
typedef struct
{
    /** @brief Configuración para los 3 steppers NEMA controlados por A4988 */
    a4988_dual_config_t stepper_config;

    // NOTA: No se necesita configuración de MKS aquí, ya que mks_init()
    // se llama globalmente en main.c para inicializar el UART y el mutex.

} vehicle_config_t;

/**
 * @brief Inicializa el componente completo del vehículo
 *
 * Configura el subsistema de steppers A4988 y prepara el MKS para comandos.
 *
 * @param config Configuración del vehículo
 * @param out_handle Puntero donde se almacenará el handle del vehículo creado
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t vehicle_init(const vehicle_config_t *config, vehicle_handle_t *out_handle);

/**
 * @brief Habilita todos los motores del vehículo (NEMA y MKS)
 *
 * @param handle Handle del vehículo
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t vehicle_enable(vehicle_handle_t handle);

/**
 * @brief Deshabilita todos los motores del vehículo (NEMA y MKS)
 *
 * Detiene el movimiento y corta la energía (torque) de todos los motores.
 *
 * @param handle Handle del vehículo
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t vehicle_disable(vehicle_handle_t handle);

/**
 * @brief Mueve el vehículo en una dirección y velocidad específicas
 *
 * Esta es la función principal. Traduce una dirección de vehículo
 * (ej. A4988_DUAL_LEFT) en los comandos correctos para:
 * 1. Los 3 steppers NEMA (usando a4988_dual_move)
 * 2. El servo MKS (usando mks_run)
 *
 * @param handle Handle del vehículo
 * @param direction Dirección del movimiento (reutilizamos el enum de A4988)
 * @param rpm Velocidad deseada en RPM (se aplicará a todos los motores)
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t vehicle_move(vehicle_handle_t handle, a4988_dual_direction_t direction, float rpm);

/**
 * @brief Detiene el movimiento del vehículo
 *
 * Detiene los pulsos STEP de los NEMA y envía el comando STOP al MKS.
 * Los motores permanecen habilitados (con torque).
 *
 * @param handle Handle del vehículo
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t vehicle_stop(vehicle_handle_t handle);

/**
 * @brief Libera los recursos del componente del vehículo
 *
 * @param handle_ptr Puntero al handle del vehículo (se establecerá a NULL)
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t vehicle_free(vehicle_handle_t *handle_ptr);

#endif // VEHICLE_H