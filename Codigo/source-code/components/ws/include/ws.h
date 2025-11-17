/* ========================================================================
 * Archivo: ws.h
 * Descripción: API para el servidor WebSocket.
 * ======================================================================== */
#pragma once

#include "esp_http_server.h"

/**
 * @brief Registra los handlers de WebSocket en un servidor HTTP existente.
 * * @param server El handle del servidor HTTP (obtenido de httpd_start).
 * @return ESP_OK si se registra correctamente, o un error en caso contrario.
 */
esp_err_t ws_server_register_handlers(httpd_handle_t server);

/**
 * @brief Envía un mensaje de texto a todos los clientes WebSocket conectados.
 * * Esta función es thread-safe y se puede llamar desde cualquier tarea.
 * Utiliza la cola de trabajo del servidor HTTP para enviar los datos de forma asíncrona.
 * * @param data Puntero al string de texto a enviar.
 * @return ESP_OK si el trabajo de envío se encoló correctamente.
 * @return ESP_ERR_INVALID_ARG si data es NULL.
 * @return ESP_FAIL si no hay clientes conectados o el servidor no está iniciado.
 */
esp_err_t ws_server_send_text_all(const char* data);