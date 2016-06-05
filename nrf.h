#include "devices.h"
#include <drv_spi.h>
#include <drv_ioport.h>
#include <timing.h>

void init_spi(void);
uint8_t test_spi(void);
uint8_t spi1_send_read_byte ( uint8_t in );
void DelayUS ( uint64_t t );
