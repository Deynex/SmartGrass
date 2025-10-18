#include "driver/i2c_master.h"

// Bus I2C
#define I2C_MASTER_SCL 22         // Número de GPIO utilizado para el reloj maestro I2C
#define I2C_MASTER_SDA 21         // Número de GPIO utilizado para los datos maestros I2C
#define I2C_MASTER_NUM I2C_NUM_0  // Número de puerto I2C para el dispositivo maestro
#define I2C_MASTER_TIMEOUT_MS 500 // Tiempo de espera del I2C en milisegundos

/**
 * @brief Inicialización del I2C maestro
 * @param bus_handle Puntero al manejador del bus I2C
 * @param dev_handle Puntero al manejador del dispositivo
 * @param dev_addr Dirección del dispositivo
 */
void i2c_master_bus_init(i2c_master_bus_handle_t *bus_handle);

/**
 * @brief Adición de un dispositivo al bus I2C
 * @param bus_handle Manejador del bus I2C
 * @param dev_handle Puntero al manejador del dispositivo
 * @param dev_addr Dirección del dispositivo
 */
void i2c_add_device(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t dev_addr, uint32_t freq_hz);

/**
 * @brief Lectura de un registro del dispositivo hasta otro registro
 * @note Se asume que el registro de inicio es menor que el registro de fin
 * @param dev_handle Manejador del dispositivo
 * @param reg_addr Dirección del registro a leer
 * @param data Puntero al buffer donde se almacenará el valor leído
 * @param len Longitud del buffer
 * @return esp_err_t
 */
esp_err_t i2c_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Escritura de un byte en un registro del dispositivo
 * @param dev_handle Manejador del dispositivo
 * @param reg_addr Dirección del registro a escribir
 * @param data Valor a escribir en el registro
 * @return esp_err_t
 */
esp_err_t i2c_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);