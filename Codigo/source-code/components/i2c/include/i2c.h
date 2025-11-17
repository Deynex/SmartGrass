#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h" // Para esp_err_t
#include <stdint.h>  // Para uint8_t, uint32_t
#include <stddef.h>  // Para size_t

/**
 * @brief Inicialización del bus I2C maestro
 *
 * @param bus_handle Puntero donde se almacenará el handle del bus
 * @param sda_pin Número de GPIO para SDA
 * @param scl_pin Número de GPIO para SCL
 * @param port_num Número de puerto I2C (I2C_NUM_0 o I2C_NUM_1)
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t i2c_master_bus_init(i2c_master_bus_handle_t *bus_handle, int sda_pin, int scl_pin, i2c_port_t port_num);

/**
 * @brief Adición de un dispositivo al bus I2C
 *
 * @param bus_handle Handle del bus I2C (creado con i2c_master_bus_init)
 * @param dev_handle Puntero donde se almacenará el handle del dispositivo
 * @param dev_addr Dirección I2C del dispositivo
 * @param freq_hz Frecuencia de reloj para este dispositivo (ej. 100000 o 400000)
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t i2c_add_device(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t dev_addr, uint32_t freq_hz);

/**
 * @brief Lectura de uno o más registros de un dispositivo I2C
 *
 * @param dev_handle Handle del dispositivo
 * @param reg_addr Dirección del registro a leer
 * @param data Puntero al buffer donde se almacenarán los datos leídos
 * @param len Número de bytes a leer
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t i2c_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Escritura de un byte en un registro de un dispositivo I2C
 *
 * @param dev_handle Handle del dispositivo
 * @param reg_addr Dirección del registro a escribir
 * @param data Byte a escribir
 * @return esp_err_t ESP_OK si tiene éxito
 */
esp_err_t i2c_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data);