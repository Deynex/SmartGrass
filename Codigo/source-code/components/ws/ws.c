/* ========================================================================
 * Archivo: ws.c
 * Descripción: Implementación del servidor WebSocket.
 * ======================================================================== */

#include "ws.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "WEBSOCKET";

// --- Gestión de Clientes ---

#define MAX_CLIENTS 5 // Número máximo de clientes WebSocket simultáneos

// Estructura para gestionar los descriptores de socket (fd) de los clientes
typedef struct {
    httpd_handle_t handle;
    int fds[MAX_CLIENTS];
} ws_clients_t;

// Variable global (estática) para almacenar los clientes
static ws_clients_t s_clients = {
    .handle = NULL,
    .fds = { -1, -1, -1, -1, -1 } // Inicializa todos como -1 (desconectado)
};

// Añade un cliente a la lista
static void add_client(httpd_handle_t handle, int fd)
{
    if (s_clients.handle == NULL) {
        s_clients.handle = handle; // Guarda el handle del servidor la primera vez
    }
    
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients.fds[i] == -1) {
            ESP_LOGI(TAG, "Cliente conectado, fd=%d, asignado al slot %d", fd, i);
            s_clients.fds[i] = fd;
            return;
        }
    }
    ESP_LOGW(TAG, "No hay slots libres para el cliente fd=%d", fd);
    // Opcional: cerrar la conexión si no hay espacio
    // httpd_sess_trigger_close(handle, fd); 
}

// Elimina un cliente de la lista
static void remove_client(int fd)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients.fds[i] == fd) {
            ESP_LOGI(TAG, "Cliente desconectado, fd=%d, liberado del slot %d", fd, i);
            s_clients.fds[i] = -1;
            break; // Salir del bucle una vez encontrado
        }
    }
    
    // Si es el último cliente, limpiar el handle del servidor
    bool any_client_left = false;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients.fds[i] != -1) {
            any_client_left = true;
            break;
        }
    }
    if (!any_client_left) {
        s_clients.handle = NULL;
    }
}


// --- Lógica de Envío Asíncrono ---

// Estructura para pasar datos al worker queue
struct async_send_arg {
    char* data;
    size_t len;
};

// Tarea que se ejecuta en el worker queue del servidor
static void ws_async_send_task(void *arg)
{
    struct async_send_arg *args = (struct async_send_arg *)arg;
    
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)args->data;
    ws_pkt.len = args->len;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // Enviar a TODOS los clientes conectados
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients.fds[i] != -1) {
            esp_err_t ret = httpd_ws_send_frame_async(s_clients.handle, s_clients.fds[i], &ws_pkt);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Error enviando async a fd=%d: %s", s_clients.fds[i], esp_err_to_name(ret));
                // Si falla, puede que el cliente se haya desconectado de forma abrupta
                remove_client(s_clients.fds[i]); // Podríamos querer limpiarlo aquí
            }
        }
    }
    
    free(args->data); // Liberar la copia de los datos
    free(args);       // Liberar el struct de argumentos
}

/**
 * @brief API pública para enviar texto (implementación)
 */
esp_err_t ws_server_send_text_all(const char* data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_clients.handle == NULL) {
        ESP_LOGW(TAG, "No hay servidor o clientes conectados, no se puede enviar.");
        return ESP_FAIL;
    }

    size_t len = strlen(data);
    struct async_send_arg *args = malloc(sizeof(struct async_send_arg));
    if (args == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    args->data = malloc(len + 1); // +1 para NUL
    if (args->data == NULL) {
        free(args);
        return ESP_ERR_NO_MEM;
    }
    
    memcpy(args->data, data, len + 1);
    args->len = len;

    // Encolar el trabajo
    esp_err_t ret = httpd_queue_work(s_clients.handle, ws_async_send_task, args);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_queue_work falló: %s", esp_err_to_name(ret));
        free(args->data);
        free(args);
    }
    
    return ret;
}


// --- Handler Principal de WebSocket ---

/**
 * @brief Manejador para eventos de WebSocket
 */
static esp_err_t ws_handler(httpd_req_t *req)
{
    // === 1. Handshake (Nueva conexión) ===
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake solicitado por cliente");
        add_client(req->handle, httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    // === 2. Recepción de Paquetes ===
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    // 2.1. Obtener la longitud del frame
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        // Error o desconexión abrupta
        ESP_LOGE(TAG, "httpd_ws_recv_frame (len) falló: %s", esp_err_to_name(ret));
        remove_client(httpd_req_to_sockfd(req));
        return ret;
    }

    // 2.2. Manejar paquete de CIERRE (desconexión limpia)
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "Cliente envió paquete CLOSE");
        remove_client(httpd_req_to_sockfd(req));
        // Enviar paquete de cierre de vuelta
        ws_pkt.payload = NULL;
        ws_pkt.len = 0;
        httpd_ws_send_frame(req, &ws_pkt);
        return ESP_OK;
    }
    
    // 2.3. Manejar paquete de TEXTO/DATOS
    if (ws_pkt.len > 0) {
        buf = calloc(1, ws_pkt.len + 1); // +1 para el NUL
        if (buf == NULL) {
            ESP_LOGE(TAG, "Fallo al reservar memoria para el buffer del WS");
            return ESP_ERR_NO_MEM;
        }
        
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame (payload) falló: %s", esp_err_to_name(ret));
            free(buf);
            return ret;
        }

        ESP_LOGI(TAG, "Paquete recibido de fd=%d: [%s]", httpd_req_to_sockfd(req), ws_pkt.payload);

        // --- ¡AQUÍ VA TU LÓGICA! ---
        // Procesa el mensaje recibido en ws_pkt.payload (es un string C)
        
        // Ejemplo: Si recibes "Trigger async", envía un mensaje a todos
        if (strcmp((char*)ws_pkt.payload, "Trigger async") == 0) {
            free(buf); // Libera el buffer antes de llamar a la función que puede tardar
            return ws_server_send_text_all("¡Envío asíncrono disparado!");
        }

        // Lógica de "Echo" (devuelve el mismo mensaje)
        ret = httpd_ws_send_frame(req, &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_send_frame falló: %s", esp_err_to_name(ret));
        }
        
        free(buf);
    }
    
    return ESP_OK;
}

// --- Función de Registro de Handlers ---

/**
 * @brief API pública para registrar handlers (implementación)
 */
esp_err_t ws_server_register_handlers(httpd_handle_t server)
{
    // Definición de la URI para el WebSocket
    static const httpd_uri_t ws_uri = {
        .uri        = "/ws",      // Endpoint (ej. ws://192.168.2.1/ws)
        .method     = HTTP_GET,
        .handler    = ws_handler,
        .user_ctx   = NULL,
        .is_websocket = true     // ¡Importante!
    };
    
    // Inicializa la lista de clientes
    s_clients.handle = server; // Guarda el handle
    for (int i = 0; i < MAX_CLIENTS; i++) {
        s_clients.fds[i] = -1;
    }

    ESP_LOGI(TAG, "Registrando URI /ws para WebSockets");
    return httpd_register_uri_handler(server, &ws_uri);
}