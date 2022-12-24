#pragma once

#include <driver/gpio.h>
#include <driver/spi_master.h>

/**
 * @brief The WIP (write in progress) bit of the status register
 * 
 */
#define M95EEPROM_STATUS_BIT_WIP 0x01

/**
 * @brief The WEL (write enabled) bit of the status register
 * 
 */
#define M95EEPROM_STATUS_BIT_WEL 0x02

/**
 * @brief The m95 eeprom handle. Use this to call the m95_eeprom_xyz functions.
 * 
 */
typedef struct m95_eeprom_context_t* m95_eeprom_handle_t;

/**
 * @brief Configuration init options for the m95 eeprom.
 * 
 */
struct m95_eeprom_config_t {
    /**
     * @brief The SPI host, for example SPI3_HOST
     */
    spi_host_device_t host;

    /**
     * @brief The GPIO pin number for the SPI CS pin.
     */
    int cs_pin;

    /**
     * @brief Number of bits for the address size. Refer to the M95 eeprom
     * datasheet. This is usually 24 for large eeproms (m95m04 for example),
     * or 16 bits for small eeproms (m95040 for example).
     */
    uint8_t address_size;
};

typedef struct m95_eeprom_config_t m95_eeprom_config_t;

/**
 * @brief Initialized m95_eeprom handle with a specific SPI host and a cs pin.
 *        The spi host must be initialized beforehand. This function only
 *        creates the spi device. Use m95_eeprom_remove to deinitialize the eeprom.
 * 
 * @param config The pointer to the init config options.
 * @param handle The pointer to the m95_eeprom_handle_t handle.
 * @return esp_err_t   EPS_OK on success or an error on failure.
 */
esp_err_t m95_eeprom_init(const m95_eeprom_config_t* config, m95_eeprom_handle_t* handle);

/**
 * @brief Deinitializes the eeprom and removes the SPI device from the host.
 * 
 * @param handle The m95_eeprom_handle_t handle.
 * @return esp_err_t EPS_OK on success or an error on failure.
 */
esp_err_t m95_eeprom_deinit(m95_eeprom_handle_t handle);

/**
 * @brief Enables writing to the eeprom. Must be called before each write call.
 * 
 * @param handle The m95_eeprom_handle_t handle.
 * @return esp_err_t EPS_OK on success or an error on failure.
 */
esp_err_t m95_eeprom_enable_write(m95_eeprom_handle_t handle);

/**
 * @brief Disables writing to the eeprom.
 * 
 * @param handle The m95_eeprom_handle_t handle.
 * @return esp_err_t EPS_OK on success or an error on failure.
 */
esp_err_t m95_eeprom_disable_write(m95_eeprom_handle_t handle);

/**
 * @brief Reads the status register of the eeprom. Refer to the datasheet
 *        for the status bits.
 * 
 * @param handle The m95_eeprom_t handle.
 * @return esp_err_t EPS_OK on success or an error on failure.
 */
esp_err_t m95_eeprom_read_status_reg(m95_eeprom_handle_t handle, uint8_t* status);

/**
 * @brief Waits for the status (the WIP bit) to become zero. The WIP bit is 
 *        set to 1 when there is a write in progress. You should call this after
 *        each write call!
 * 
 * @param handle The m95_eeprom_handle_t handle.
 * @return esp_err_t EPS_OK on success or an error on failure.
 */
esp_err_t m95_eeprom_wait(m95_eeprom_handle_t handle);

/**
 * @brief Writes a single byte to the eeprom at a specific address.
 * 
 * @param handle  The m95_eeprom_handle_t handle.
 * @param address The address (not the page) where to write the data to.
 * @param data    The data to write.
 * @return esp_err_t EPS_OK on success or an error on failure.
 */
esp_err_t m95_eeprom_write_byte(m95_eeprom_handle_t handle, uint32_t address, uint8_t data);

/**
 * @brief Reads a single byte from the eeprom at a specific address.
 * 
 * @param handle  The m95_eeprom_handle_t handle.
 * @param address The address (not the page) where to read the data from.
 * @param data    Pointer to the data where the read value will be stored.
 * @return esp_err_t EPS_OK on success or an error on failure.
 */
esp_err_t m95_eeprom_read_byte(m95_eeprom_handle_t handle, uint32_t address, uint8_t* data);

/**
 * @brief Writes a sequence of bytes to the eeprom starting at a specific address.
 * @warn Each time a new data byte is shifted in, the least significant bits of the 
 *       internal address counter are incremented. If more bytes are sent than will 
 *       fit up to the end of the page, a condition known as “roll-over” occurs.
 *       In case of roll-over, the bytes exceeding the page size are overwritten 
 *       from location 0 of the same page.
 * 
 * @param handle The m95_eeprom_handle_t handle.
 * @param address The address where to start the write sequence at.
 * @param data Pointer to the data to be written. The source
 *             must contain at least "length" bytes.
 * @param length Length of the data to write in bytes.
 * @return esp_err_t EPS_OK on success or an error on failure.
 */
esp_err_t m95_eeprom_write_range(m95_eeprom_handle_t handle, uint32_t address, const void* data, uint32_t length);

/**
 * @brief Reads a sequence of bytes from the eeprom starting at a specific address.
 * @note When the highest address is reached, the address counter rolls over to zero, allowing 
 *       the read cycle to be continued indefinitely. The whole memory can, therefore, be read 
 *       with a single read instruction.
 * 
 * @param handle The m95_eeprom_handle_t handle.
 * @param address The address where to start the read sequence at.
 * @param data Pointer to the data where the read data will be stored. The destination
 *             must contain at least "length" bytes.
 * @param length Length of the data to read in bytes.
 * @return esp_err_t EPS_OK on success or an error on failure.
 */
esp_err_t m95_eeprom_read_range(m95_eeprom_handle_t handle, uint32_t address, void* data, uint32_t length);
