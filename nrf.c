#include "nrf.h"

spi_t* nrf_spi;
ioport_t * servo; 

void init_spi()
{
    nrf_spi = spi_open(DRV_SPI_2);
}

uint8_t test_spi()
{
    uint8_t ans = 100;
    //if(spi_get_bus(nrf_spi,0))
    {
        spi_cs_lo(nrf_spi);
        ans=spi_transceive8(nrf_spi,255);
        spi_cs_hi(nrf_spi);
        //spi_release_bus(nrf_spi);
    }

    return ans;
}

uint8_t spi1_send_read_byte ( uint8_t in )
{
    return spi_transceive8(nrf_spi,in);
}

void DelayUS ( uint64_t t )
{
    delay_us ( t );
}
