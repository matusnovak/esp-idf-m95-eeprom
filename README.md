# ESP-IDF m95 EEPROM Library

This is a simple EEPROM library for the m950x0 and m95mx0 series SPI EEPROM chips. For example: [m95m01](https://www.st.com/resource/en/datasheet/m95m01-r.pdf) chip.
This library targets the [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html)
framework.

This is a C and C++ library available via `m95_eeprom.h` and `m95_eeprom.hpp` respectively.

This code was tested on the `m95m04` chip with the `ESP32-S3-WROOM-1` microcontroller using `SPI2` and `SPI3` buses. Implemented based on the documentation provided in the [m95m04-dr-5 datasheet](https://www.st.com/resource/en/datasheet/m95m04-dr.pdf).

A [PlatformIO](https://platformio.org/) `library.json` is provided. You can add this library as git clone (or as a git submodule) into your own project's `lib` folder.

## Available commands

* Enable write (WEL bit)
* Disable write (WEL bit)
* Read status register
* Wait for write to finish (WIP bit)
* Read one byte
* Write one byte
* Read a range of bytes
* Write a range of bytes

## Not available

* Write status register
* Identification pages
* Locking

## Usage

```c
#include <m95_eeprom.h>

void app_main() {
    spi_bus_config_t bus = {};
    bus.data0_io_num = -1;
    bus.data1_io_num = -1;
    bus.data2_io_num = -1;
    bus.data3_io_num = -1;
    bus.data4_io_num = -1;
    bus.data5_io_num = -1;
    bus.data6_io_num = -1;
    bus.data7_io_num = -1;
    bus.miso_io_num = SPI3_PIN_MISO;
    bus.mosi_io_num = SPI3_PIN_MOSI;
    bus.sclk_io_num = SPI3_PIN_CLK;
    bus.quadhd_io_num = -1;
    bus.quadwp_io_num = -1;
    bus.max_transfer_sz = 4;

    // Initialize SPI host bus
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Init EEPROM device & initializes SPI device
    // Uses 24 bit addressing (check your datasheet!)
    // Tested on m95m04
    m95_eeprom_config_t config = {};
    config.host = SPI3_HOST;
    config.cs_pin = EEPROM_CS_PIN;
    config.address_size = 24;

    // Do the init with the config
    m95_eeprom_handle_t eeprom;
    ret = m95_eeprom_init(config, &eeprom);
    ESP_ERROR_CHECK(ret);

    // Enable write.
    // !!! MUST BE DONE BEFORE EACH WRITE COMMAND !!!
    ret = m95_eeprom_enable_write(eeprom);
    ESP_ERROR_CHECK(ret);

    // Write length of 12 bytes at an address 0x05
    ret = m95_eeprom_write_range(eeprom, 0x05, "Hello World!", 12);
    ESP_ERROR_CHECK(ret);

    // Wait for the read to finish
    ret = m95_eeprom_enable_wait(eeprom);
    ESP_ERROR_CHECK(ret);

    // Read destination
    char buffer[13];
    buffer[12] = '\0'; // Null terminate string
    memset(buffer, 'x', 12); // Optional, for debugging purposes

    // Read length of 12 bytes from an address 0x05
    ret = m95_eeprom_read_range(eeprom, 0x05, buffer, 12);
    ESP_ERROR_CHECK(ret);

    printf("EEPROM read at address 0x05 result: %s\n", buffer);

    // Deinitialize & remove SPI device from the SPI host bus
    // Additinally frees the memory allocated by the init function.
    ret = m95_eeprom_deinit(eeprom);
    ESP_ERROR_CHECK(ret);
}

```
