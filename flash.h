#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stddef.h>
#include <drv_spi.h>

int flash_init(spi_t * driver);

int flash_get_status(spi_t * driver);

int flash_write(spi_t * driver, uint32_t address, void * buffer, size_t buflen , bool blocking);

int flash_read( spi_t * driver, uint32_t address, void * buffer, size_t buflen );

int flash_set_protection( spi_t * driver, uint8_t pattern );

int flash_sector_erase( spi_t * driver, uint32_t address , bool blocking);

#endif

