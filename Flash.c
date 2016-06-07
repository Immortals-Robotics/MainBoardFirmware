#include "flash.h"

#include <drv_spi.h>
#include "devices.h"
#include <time.h>

#define M25PX0_ERR_OK           0       /**<  No error */
#define M25PX0_ERR_SPI          -1      /**<  Could not get control over SPI bus */

#define M25PX0_STAT_WIP         0x01    /**<  Write in progress */
#define M25PX0_STAT_WEL         0x02    /**<  Write enable latch */
#define M25PX0_STAT_BP0         0x04    /**<  Block protect bit 0 */
#define M25PX0_STAT_BP1         0x08    /**<  Block protect bit 1 */
#define M25PX0_STAT_BP2         0x10    /**<  Block protect bit 2 */
#define M25PX0_STAT_SRWD        0x80    /**<  Status register write protect */

#define M25PX0_INSTR_WREN       0x06    // Write enable
#define M25PX0_INSTR_WRDI       0x04    // Write disable
#define M25PX0_INSTR_RDSR       0x05    // Read status register
#define M25PX0_INSTR_WRSR       0x01    // Write status register
#define M25PX0_INSTR_READ       0x03    // Read data bytes
#define M25PX0_INSTR_FAST_READ  0x0B    // Read data bytes at higher speed
#define M25PX0_INSTR_PP         0x02    // Page program
#define M25PX0_INSTR_SE         0xD8    // Sector erase
#define M25PX0_INSTR_BE         0xC7    // Bulk erase
#define M25PX0_INSTR_DP         0xB9    // Deep power-down
#define M25PX0_INSTR_RES        0xAB    // Release from deep power down and optionally read electronic signature

int flash_init(spi_t * driver)
{
    spi_set_baudrate( driver,10000 );
    spi_set_mode( driver, SPI_MODE0 );
    spi_cs_hi( driver );                                                  // Reset chip's SPI FSM
    spi_set_endianess( driver, true );
    spi_cs_lo( driver );
    spi_transceive8( driver, M25PX0_INSTR_RES );
    for ( clock_t tres = clock() + 3 * CLOCKS_PER_SEC / 1000000; clock() < tres; ) __nop();
    int sig = (uint8_t)spi_transceive32( driver, 0xFFFFFFFF ) & 0xFF;
    spi_cs_hi( driver );
    return sig;
}

int flash_get_status(spi_t * driver)
{
    int ret = -1;
    spi_cs_lo( driver );
    spi_transceive8( driver, M25PX0_INSTR_RDSR );
    ret = spi_transceive8( driver, 0xFF );
    spi_cs_hi( driver );
    return ret;
}

int flash_write(spi_t * driver, uint32_t address, void * buffer, size_t buflen , bool blocking)
{
    size_t bytes;
    uint8_t * buf8;
    uint16_t * buf16;
    uint32_t * buf32;
    spi_cs_hi( driver );                                               // Make sure flash memory's SPI FSM is reset
    spi_set_endianess( driver, true );
    spi_cs_lo( driver );
    spi_transceive8(driver, M25PX0_INSTR_WREN );                     // Enable writing
    spi_cs_hi( driver );
    __nop();
    spi_cs_lo( driver );
    spi_transceive32( driver, (M25PX0_INSTR_PP << 24) | address );    // Start the sector erase
    spi_set_endianess( driver, true );                                // Switch endianess to native format
    // Maximize on 256 bytes
    if ( buflen > 256 )
    {
        buflen = 256;
    }

    bytes = buflen;
    buf8 = buffer;

    // Fix 8-bit alignment problems
    if( ((uintptr_t)buf8 & 0x01) && (bytes > 0) )
    {
        spi_transceive8( driver, *buf8++ );
        bytes--;
    }

    // Fix 16-bit alignment problems

    buf16 = (uint16_t *)buf8;

    if( ((uintptr_t)buf16 & 0x02) && (bytes > 1) )
    {
        spi_transceive16( driver, *buf16++ );
        bytes -= 2;
    }

    // Bulk write
    buf32 = (uint32_t *)buf16;
    while( bytes > 3 )
    {
        spi_transceive32( driver, *buf32++ );
        bytes -= 4;
    }

    // Any trailing halfwords?
    buf16 = (uint16_t *)buf32;
    if( bytes & 0x02 )
    {
        spi_transceive16( driver, *buf16++ );
    }

    // Any trailing bytes?
    buf8 = (uint8_t *)buf16;
    if( bytes & 0x01 )
    {
        spi_transceive8( driver, *buf8 );
    }
    spi_cs_hi( driver );

    if(blocking)
    {
       while(flash_get_status(driver) & M25PX0_STAT_WIP)
       {
          __nop();
       }
    }

    return buflen;
}

int flash_read( spi_t * driver, uint32_t address, void * buffer, size_t buflen )
{
    uint8_t * buf8;
    uint16_t * buf16;
    uint32_t * buf32;

    size_t bytes;

    __nop();
        spi_cs_hi( driver );                                          // Reset SPI FSM
        spi_set_endianess( driver, true );                            // Send read command & address
        spi_cs_lo( driver );
        spi_transceive32( driver, (M25PX0_INSTR_READ << 24) | address);
        spi_set_endianess( driver, true );                                // Switch endianess to native format
        bytes = buflen;
        buf8 = buffer;

        // Fix 8-bit alignment problems
        if( ((uintptr_t)buf8 & 0x01) && (bytes > 0) )
        {
            *buf8++ = spi_transceive8( driver, 0xFF );
            bytes--;
        }

        // Fix 16-bit alignment problems

        buf16 = (uint16_t *)buf8;

        if( ((uintptr_t)buf16 & 0x02) && (bytes > 1) )
        {
            *buf16++ = spi_transceive16( driver, 0xFFFF );
            bytes -= 2;
        }

        // Bulk read
        buf32 = (uint32_t *)buf16;
        while( bytes > 3 )
        {
            *buf32++ = spi_transceive32( driver, 0xFFFFFFFF );
            bytes -= 4;
        }

        // Any trailing halfwords?
        buf16 = (uint16_t *)buf32;
        if( bytes & 0x02 )
        {
            *buf16++ = spi_transceive16( driver, 0xFFFF );
        }

        // Any trailing bytes?
        buf8 = (uint8_t *)buf16;
        if( bytes & 0x01 )
        {
            *buf8 = spi_transceive8( driver, 0xFF );
        }
        spi_cs_hi( driver );
     return buflen;
}

int flash_set_protection( spi_t * driver, uint8_t pattern )
{
    int retval = M25PX0_ERR_SPI;
    uint8_t status;
        spi_cs_hi( driver );                                              // Make sure flash memory's SPI FSM is reset
        spi_cs_lo( driver );
        spi_transceive8( driver, M25PX0_INSTR_RDSR );
        status = spi_transceive8( driver, 0xFF );                         // Get current status register
        spi_cs_hi( driver );
        spi_cs_lo( driver );
        spi_transceive8( driver, M25PX0_INSTR_WREN );                     // Enable writing
        spi_cs_hi( driver );
        spi_cs_lo( driver );
        spi_transceive8( driver, M25PX0_INSTR_WRSR );                     // Write new protection bits in status register
        spi_transceive8( driver, (status & ~(M25PX0_STAT_BP2|M25PX0_STAT_BP1|M25PX0_STAT_BP0)) | pattern);
        spi_cs_hi( driver );
        retval = M25PX0_ERR_OK;
  return retval;
}

int flash_sector_erase( spi_t * driver, uint32_t address , bool blocking)
{
    int retval = M25PX0_ERR_SPI;
    spi_cs_hi( driver );                                                  // Make sure flash memory's SPI FSM is reset
    spi_set_endianess( driver, true );
    spi_cs_lo( driver );
    spi_transceive8(driver, M25PX0_INSTR_WREN );                     // Enable writing
    spi_cs_hi( driver );
    spi_cs_lo( driver );
    spi_transceive32( driver, (M25PX0_INSTR_SE << 24) | address );    // Start the sector erase
    spi_cs_hi( driver );
    retval = M25PX0_ERR_OK;
    if(blocking)
    {
       while(flash_get_status(driver) & M25PX0_STAT_WIP)
       {
          __nop();
       }
    }
    return retval;
}

