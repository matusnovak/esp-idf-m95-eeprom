#include "m95_eeprom.h"

#include <unistd.h>

#define M95_CMD_ENABLE_WRITE 0x06
#define M95_CMD_DISABLE_WRITE 0x04
#define M95_CMD_READ_STATUS_REG 0x05
#define M95_CMD_WRITE_ARRAY 0x02
#define M95_CMD_READ_ARRAY 0x03

struct m95_eeprom_context_t {
    spi_device_handle_t spi;
    uint8_t address_size;
};

typedef struct m95_eeprom_context_t m95_eeprom_context_t;

esp_err_t m95_eeprom_init(const m95_eeprom_config_t* config, m95_eeprom_handle_t* handle) {
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    m95_eeprom_context_t* ctx = (m95_eeprom_context_t*)malloc(sizeof(m95_eeprom_context_t));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 5 * 1000 * 1000;
    devcfg.spics_io_num = config->cs_pin;
    devcfg.queue_size = 3;
    devcfg.command_bits = 8;
    devcfg.address_bits = 0;

    ctx->address_size = config->address_size;

    esp_err_t ret = spi_bus_add_device(config->host, &devcfg, &ctx->spi);
    if (ret != ESP_OK) {
        free(ctx);
        return ret;
    }

    *handle = ctx;

    return ESP_OK;
}

esp_err_t m95_eeprom_deinit(m95_eeprom_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->spi) {
        return spi_bus_remove_device(handle->spi);
        handle->spi = NULL;
    }

    return ESP_OK;
}

esp_err_t m95_eeprom_enable_write(m95_eeprom_handle_t handle) {
    if (!handle || !handle->spi) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t txd = {};
    txd.cmd = M95_CMD_ENABLE_WRITE;

    esp_err_t ret = spi_device_acquire_bus(handle->spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_device_transmit(handle->spi, &txd);

    spi_device_release_bus(handle->spi);

    return ret;
}

esp_err_t m95_eeprom_disable_write(m95_eeprom_handle_t handle) {
    if (!handle || !handle->spi) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t txd = {};
    txd.cmd = M95_CMD_DISABLE_WRITE;

    esp_err_t ret = spi_device_acquire_bus(handle->spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_device_transmit(handle->spi, &txd);

    spi_device_release_bus(handle->spi);

    return ret;
}

esp_err_t m95_eeprom_read_status_reg(m95_eeprom_handle_t handle, uint8_t* status) {
    if (!handle || !handle->spi) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t txd = {};
    txd.cmd = M95_CMD_READ_STATUS_REG;
    txd.flags = SPI_TRANS_USE_RXDATA;
    txd.length = 1 * 8;
    txd.rxlength = 1 * 8;

    esp_err_t ret = spi_device_acquire_bus(handle->spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_device_transmit(handle->spi, &txd);

    spi_device_release_bus(handle->spi);

    if (ret == ESP_OK) {
        *status = txd.rx_data[0];
    }

    return ret;
}

esp_err_t m95_eeprom_wait(m95_eeprom_handle_t handle) {
    while (true) {
        uint8_t status;
        esp_err_t ret = m95_eeprom_read_status_reg(handle, &status);
        if (ret != ESP_OK) {
            return ret;
        }

        if ((status & M95EEPROM_STATUS_BIT_WIP) == 0) {
            break;
        }

        usleep(1);
    }

    return ESP_OK;
}

esp_err_t m95_eeprom_write_range(m95_eeprom_handle_t handle, uint32_t address, const void* data, uint32_t length) {
    if (!handle || !handle->spi) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_ext_t txd = {};
    txd.base.cmd = M95_CMD_WRITE_ARRAY;
    txd.base.flags = SPI_TRANS_VARIABLE_ADDR;
    txd.base.addr = address;
    txd.base.tx_buffer = data;
    txd.base.rx_buffer = NULL;
    txd.base.length = length * 8;
    txd.base.rxlength = length * 8;
    txd.address_bits = 24;

    esp_err_t ret = spi_device_acquire_bus(handle->spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_device_transmit(handle->spi, (spi_transaction_t*)&txd);

    spi_device_release_bus(handle->spi);

    return ret;
}

esp_err_t m95_eeprom_read_range(m95_eeprom_handle_t handle, uint32_t address, void* data, uint32_t length) {
    if (!handle || !handle->spi) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_ext_t txd = {};
    txd.base.cmd = M95_CMD_READ_ARRAY;
    txd.base.flags = SPI_TRANS_VARIABLE_ADDR;
    txd.base.addr = address;
    txd.base.tx_buffer = NULL;
    txd.base.rx_buffer = data;
    txd.base.length = length * 8;
    txd.base.rxlength = length * 8;
    txd.address_bits = 24;

    esp_err_t ret = spi_device_acquire_bus(handle->spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_device_transmit(handle->spi, (spi_transaction_t*)&txd);

    spi_device_release_bus(handle->spi);

    return ret;
}

esp_err_t m95_eeprom_write_byte(m95_eeprom_handle_t handle, uint32_t address, uint8_t data) {
    if (!handle || !handle->spi) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_ext_t txd = {};
    txd.base.cmd = M95_CMD_WRITE_ARRAY;
    txd.base.flags = SPI_TRANS_VARIABLE_ADDR;
    txd.base.addr = address;
    txd.base.tx_buffer = &data;
    txd.base.rx_buffer = NULL;
    txd.base.length = 1 * 8;
    txd.base.rxlength = 1 * 8;
    txd.address_bits = 24;

    esp_err_t ret = spi_device_acquire_bus(handle->spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_device_transmit(handle->spi, (spi_transaction_t*)&txd);

    spi_device_release_bus(handle->spi);

    return ret;
}

esp_err_t m95_eeprom_read_byte(m95_eeprom_handle_t handle, uint32_t address, uint8_t* data) {
    if (!handle || !handle->spi) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_ext_t txd = {};
    txd.base.cmd = M95_CMD_READ_ARRAY;
    txd.base.flags = SPI_TRANS_VARIABLE_ADDR;
    txd.base.addr = address;
    txd.base.tx_buffer = NULL;
    txd.base.rx_buffer = data;
    txd.base.length = 1 * 8;
    txd.base.rxlength = 1 * 8;
    txd.address_bits = 24;

    esp_err_t ret = spi_device_acquire_bus(handle->spi, portMAX_DELAY);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = spi_device_transmit(handle->spi, (spi_transaction_t*)&txd);

    spi_device_release_bus(handle->spi);

    return ret;
}
