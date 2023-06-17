#include <cstdio>

#include <pigpiod_if2.h>

#include "spi_hal.h"

extern "C" {
#include "tmc/ic/TMC4671/TMC4671.h"
#include "tmc/ic/TMC6200/TMC6200.h"
}

void motors_test()
{
    init_spi();

    tmc4671_writeInt(3, TMC4671_CHIPINFO_ADDR, 0);

    int result = tmc4671_readInt(3, TMC4671_CHIPINFO_DATA);

    int tmc6200_version = TMC6200_FIELD_READ(3, TMC6200_IOIN_OUTPUT, TMC6200_VERSION_MASK, TMC6200_VERSION_SHIFT);

    int gstat = tmc6200_readInt(3, TMC6200_GSTAT);

    TMC6200_FIELD_UPDATE(3, TMC6200_DRV_CONF, TMC6200_DRVSTRENGTH_MASK, TMC6200_DRVSTRENGTH_SHIFT, 1);

    // set default PWM configuration for evaluation board use with TMC467x-EVAL
    tmc6200_writeInt(3, TMC6200_GCONF, 0x0);

    enable_driver(true);

    shutdown_spi();
}

void micro_test()
{
    const int pi = pigpio_start(0, 0); /* Connect to local Pi. */

    if (pi < 0)
    {
        printf("Can't connect to pigpio daemon\n");
        return;
    }

    char rx_buf[0x100] = {};

    char msg[0x100] = {};
    for (int i = 0; i < 0x100; ++i)
    {
        // bit-inverted from i. The values should be: {0xff, 0xfe, 0xfd...}
        msg[i] = ~i;
    }

    int h = spi_open(pi, 1, 8 * 1000 * 1000, PI_SPI_FLAGS_AUX_SPI(1));

    if (h < 0)
        return;

    while (true)
    {
        spi_xfer(pi, h, msg, rx_buf, 0x100);
        printf("rcv: %s", rx_buf);
    }

    spi_close(pi, h);

    pigpio_stop(pi); /* Disconnect from local Pi. */
}

int main(int argc, char* argv[])
{
    //motors_test();
    micro_test();

    return 0;
}
