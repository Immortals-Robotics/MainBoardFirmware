#ifndef SPI_HAL_H_
#define SPI_HAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

    bool init_spi(void);

    void shutdown_spi(void);

    int micro_xfer(char *tx_buf, char *rx_buf, unsigned count);

    void enable_drivers(bool enable);

#ifdef __cplusplus
}
#endif

#endif
