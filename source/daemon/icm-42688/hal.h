#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C"
{
#endif
    // Users of the library need to implement these
#if ICM42688_FEATURE_SPI
    extern void icm42688SpiTransfer(uint8_t t_idx, const uint8_t *t_tx_buf, uint8_t *t_rx_buf, size_t t_count);
#endif
#if ICM42688_FEATURE_I2C
    extern void    icm42688I2cWrite(uint8_t t_idx, uint8_t t_data);
    extern uint8_t icm42688I2cRead(uint8_t t_idx);
#endif
#ifdef __cplusplus
}
#endif
