#pragma once

#include "esp_err.h"

/**
 * @brief Inicializa el servidor HTTP
 * 
 * Esta función configura e inicia el servidor HTTP en el puerto 80.
 * Registra los manejadores de URI para la página principal y el favicon.
 * También registra el manejador de eventos del servidor.
 * 
 * @note Si el servidor ya está iniciado, retorna ESP_OK sin hacer nada
 * @note El servidor se inicia con las siguientes características:
 *       - Puerto: 80
 *       - Máximo de conexiones simultáneas: 7
 *       - Máximo de URIs: 8
 *       - Stack size: 4096 bytes
 * 
 * @return ESP_OK si el servidor se inició correctamente
 * @return ESP_ERR_* en caso de error durante la inicialización
 */
esp_err_t http_server_init(void);

/**
 * @brief Detiene el servidor HTTP
 * 
 * Esta función detiene el servidor HTTP y libera todos los recursos asociados.
 * Desregistra el manejador de eventos y limpia el handle del servidor.
 * 
 * @note Es seguro llamar esta función múltiples veces
 * @note Esta función es idempotente
 */
void http_server_stop(void);

/**
 * @brief Reinicia el servidor HTTP
 * 
 * Esta función detiene y vuelve a iniciar el servidor HTTP.
 * Útil para aplicar cambios de configuración o recuperarse de errores.
 * 
 * @note Incluye un delay de 500ms entre detener e iniciar para asegurar
 *       que todos los recursos se liberen correctamente
 */
void http_server_restart(void);