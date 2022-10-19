#pragma once

extern "C" {
    #include "m95_eeprom.h"
}

/**
 * @brief A C++ wrapper around the m95_eeprom.h
 *        This library supports EEPROMS m95m0X and m950x0.
 */
class M95EEPROM {
public:
    /**
     * @brief Constructs an instance of this class without initializing 
     *        the SPI device and the EEPROM. You must call the init() function.
     */
    M95EEPROM() = default;

    /**
     * @brief Initialized m95_eeprom handle with a specific SPI host and a cs pin.
     *        The spi host must be initialized beforehand. This function only
     *        creates the spi device. Use m95_eeprom_remove to deinitialize the eeprom.
     * @warning Calling init() is not allowed if this constructor is used.
     * @note This constructor checks for the return value of init() via ESP_ERROR_CHECK.
     * 
     * @param host The SPI host, for example SPI3_HOST
     * @param csPin The GPIO pin number for the SPI CS pin.
     * @param addressSize Number of bits for the address size. Refer to the M95 eeprom
     *                     datasheet. This is usually 24 for large eeproms (m95m04 for example),
     *                     or 16 bits for small eeproms (m95040 for example).
     */
    M95EEPROM(spi_host_device_t host, int csPin, uint8_t addressSize) {
        const esp_err_t ret = init(host, csPin, addressSize);
        ESP_ERROR_CHECK(ret);
    }

    ~M95EEPROM() {
        remove();
    }

    /**
     * @brief Initialized m95_eeprom handle with a specific SPI host and a cs pin.
     *        The spi host must be initialized beforehand. This function only
     *        creates the spi device. Use m95_eeprom_remove to deinitialize the eeprom.
     * 
     * @param host The SPI host, for example SPI3_HOST
     * @param csPin The GPIO pin number for the SPI CS pin.
     * @param addressSize Number of bits for the address size. Refer to the M95 eeprom
     *                     datasheet. This is usually 24 for large eeproms (m95m04 for example),
     *                     or 16 bits for small eeproms (m95040 for example).
     * @return esp_err_t EPS_OK on success or an error on failure.
     */
    esp_err_t init(spi_host_device_t host, int csPin, uint8_t addressSize) {
        return m95_eeprom_init(&handle, host, csPin, addressSize);
    }

    /**
     * @brief einitializes the eeprom and removes the SPI device from the host.
     * @return esp_err_t EPS_OK on success or an error on failure.
     */
    esp_err_t remove() {
        return m95_eeprom_remove(&handle);
    }

    /**
     * @brief Writes a sequence of bytes to the eeprom starting at a specific address.
     * @warn Each time a new data byte is shifted in, the least significant bits of the 
     *       internal address counter are incremented. If more bytes are sent than will 
     *       fit up to the end of the page, a condition known as “roll-over” occurs.
     *       In case of roll-over, the bytes exceeding the page size are overwritten 
     *       from location 0 of the same page.
     * @note This function automatically calls wait and enable write functions.
     * 
     * @param address The address where to start the write sequence at.
     * @param data Pointer to the data to be written. The source
     *             must contain at least "length" bytes.
     * @param length Length of the data to write in bytes.
     * @return esp_err_t EPS_OK on success or an error on failure.
     */
    esp_err_t writeRange(uint32_t address, const void* data, uint32_t length) {
        esp_err_t ret = m95_eeprom_enable_write(&handle);
        if (ret != ESP_OK) {
            return ret;
        }
        
        ret = m95_eeprom_write_range(&handle, address, data, length);
        if (ret != ESP_OK) {
            return ret;
        }

        return m95_eeprom_wait(&handle);
    }

    /**
     * @brief Reads a sequence of bytes from the eeprom starting at a specific address.
     * @note When the highest address is reached, the address counter rolls over to zero, allowing 
     *       the read cycle to be continued indefinitely. The whole memory can, therefore, be read 
     *       with a single read instruction.
     * 
     * @param address The address where to start the read sequence at.
     * @param data Pointer to the data where the read data will be stored. The destination
     *             must contain at least "length" bytes.
     * @param length Length of the data to read in bytes.
     * @return esp_err_t EPS_OK on success or an error on failure.
     */
    esp_err_t readRange(uint32_t address, void* data, uint32_t length) {
        return m95_eeprom_read_range(&handle, address, data, length);
    }

    /**
     * @brief Writes a single byte to the eeprom at a specific address.
     * @note This function automatically calls wait and enable write functions.
     * 
     * @param handle  The pointer to the m95_eeprom_t handle.
     * @param address The address (not the page) where to write the data to.
     * @param data    The data to write.
     * @return esp_err_t EPS_OK on success or an error on failure.
     */
    esp_err_t writeByte(uint32_t address, uint8_t data) {
        esp_err_t ret = m95_eeprom_enable_write(&handle);
        if (ret != ESP_OK) {
            return ret;
        }
        
        ret = m95_eeprom_write_byte(&handle, address, data);
        if (ret != ESP_OK) {
            return ret;
        }

        return m95_eeprom_wait(&handle);
    }

    /**
     * @brief Reads a single byte from the eeprom at a specific address.
     * 
     * @param handle  The pointer to the m95_eeprom_t handle.
     * @param address The address (not the page) where to read the data from.
     * @param data    Pointer to the data where the read value will be stored.
     * @return esp_err_t EPS_OK on success or an error on failure.
     */
    esp_err_t readByte(uint32_t address, uint8_t* data) {
        return m95_eeprom_read_byte(&handle, address, data);
    }

private:
    m95_eeprom_t handle;
};
