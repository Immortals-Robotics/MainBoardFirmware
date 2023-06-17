#ifndef SPI_HAL_H_
#define SPI_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

bool init_spi();

void shutdown_spi();

void test_4671();

void enable_driver(bool enable);

#ifdef __cplusplus
}
#endif

#endif