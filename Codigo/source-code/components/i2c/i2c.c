#include "i2c.h" // ¡Primero!

// Includes de implementación
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "I2C";

// Definimos el timeout aquí, como un detalle de implementación privado
#define I2C_MASTER_TIMEOUT_MS 500

/**
 * @brief Función de ayuda "privada" para verificar y registrar errores de I2C
 */
static void full_error_check(esp_err_t err)
{
    if (err != ESP_OK)
    {
        char error_msg[64];
        esp_err_to_name_r(err, error_msg, sizeof(error_msg));
        ESP_LOGE(TAG, "Error I2C: %s (%d)", error_msg, err);
        // ESP_ERROR_CHECK_WITHOUT_ABORT(err); // Opcional
    }
}

esp_err_t i2c_master_bus_init(i2c_master_bus_handle_t *bus_handle, int sda_pin, int scl_pin, i2c_port_t port_num)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = port_num,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t err = i2c_new_master_bus(&bus_config, bus_handle);
    full_error_check(err); // Loguea el error si existe
    return err; // Devuelve el estado al llamador
}

esp_err_t i2c_add_device(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t dev_addr, uint32_t freq_hz)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = freq_hz,
    };

    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, dev_handle);
    full_error_check(err);
    return err;
}

esp_err_t i2c_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
    full_error_check(err); // Ahora somos consistentes
    return err;
}

esp_err_t i2c_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    esp_err_t err = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
    full_error_check(err);
    return err;
}