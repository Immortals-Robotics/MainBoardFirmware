#include <cstdio>

#include "spi_hal.h"

extern "C" {
#include "tmc/ic/TMC4671/TMC4671.h"
#include "tmc/ic/TMC6200/TMC6200.h"
}

int main(int argc, char* argv[])
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

    return 0;
}
