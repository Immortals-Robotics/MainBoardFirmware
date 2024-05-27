#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <pigpio.h>

int spi_motor_h;
int spi_micro_h;

const uint8_t cs = 8;

const uint8_t mux_a0 = 27;
const uint8_t mux_a1 = 22;
const uint8_t mux_a2 = 5;
const uint8_t mux_a3 = 6;

const uint8_t m_en = 13;

const uint8_t unused_ce_id = 15;

static const uint8_t ctrl_ce_id[5] = {
    0, // Dribbler
    2, // Motor 1
    8, // Motor 2
    6, // Motor 3
    4  // Motor 4
};

static const uint8_t drv_ce_id[5] = {
    1, // Dribbler
    3, // Motor 1
    9, // Motor 2
    7, // Motor 3
    5  // Motor 4
};

bool init_spi(void)
{
    const int result = gpioInitialise();
    if (result < 0)
    {
        printf("Failed to initialize pigpio: %d.\n", result);
        return false;
    }

    gpioSetMode(cs, PI_OUTPUT);

    gpioSetMode(mux_a0, PI_OUTPUT);
    gpioSetMode(mux_a1, PI_OUTPUT);
    gpioSetMode(mux_a2, PI_OUTPUT);
    gpioSetMode(mux_a3, PI_OUTPUT);

    gpioSetMode(m_en, PI_OUTPUT);
    {
        const unsigned spi_flags = PI_SPI_FLAGS_MODE(3) | // mode 3
                                   PI_SPI_FLAGS_RESVD(1); // ce0 not reserved

        spi_motor_h = spiOpen(0, 1 * 1000 * 1000, spi_flags);

        if (spi_motor_h < 0)
        {
            printf("Can't create motor SPI handle\n");
            return false;
        }
    }
    {
        const unsigned spi_flags = PI_SPI_FLAGS_MODE(0) |   // mode 0
                                   PI_SPI_FLAGS_AUX_SPI(1); // spi1

        spi_micro_h = spiOpen(1, 4 * 1000 * 1000, spi_flags);

        if (spi_micro_h < 0)
        {
            printf("Can't create micro SPI handle\n");
            return false;
        }
    }

    return true;
}

void shutdown_spi(void)
{
    gpioWrite(mux_a0, 1);
    gpioWrite(mux_a1, 1);
    gpioWrite(mux_a2, 1);
    gpioWrite(mux_a3, 1);

    spiClose(spi_motor_h);
    spiClose(spi_micro_h);

    gpioTerminate();
}

void select_ce(const uint8_t ce_id)
{
    gpioWrite(mux_a0, (ce_id & 1) >> 0);
    gpioWrite(mux_a1, (ce_id & 2) >> 1);
    gpioWrite(mux_a2, (ce_id & 4) >> 2);
    gpioWrite(mux_a3, (ce_id & 8) >> 3);
}

#if FEATURE_SPI_ARRAY_TRANSFER
void tmc4671_readWriteArray(uint8_t motor, uint8_t *data, size_t length)
{
    const uint8_t ce_id = ctrl_ce_id[motor];
    select_ce(ce_id);

    gpioWrite(cs, 0);

    spiXfer(spi_motor_h, (char *) data, (char *) data, length);

    gpioWrite(cs, 1);
}

void tmc6200_readWriteArray(uint8_t motor, uint8_t *data, size_t length)
{
    const uint8_t ce_id = drv_ce_id[motor];
    select_ce(ce_id);

    gpioWrite(cs, 0);

    spiXfer(spi_motor_h, (char *) data, (char *) data, length);

    gpioWrite(cs, 1);
}
#else
uint8_t tmc4671_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
    const uint8_t ce_id = ctrl_ce_id[motor];
    select_ce(ce_id);

    char rx_buf[64] = {0};

    gpioWrite(cs, 0);

    spiXfer(spi_motor_h, (char *) &data, rx_buf, 1);

    if (lastTransfer)
    {
        gpioWrite(cs, 1);
    }

    return rx_buf[0];
}

uint8_t tmc6200_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
    const uint8_t ce_id = drv_ce_id[motor];
    select_ce(ce_id);

    char rx_buf[64] = {0};

    gpioWrite(cs, 0);

    spiXfer(spi_motor_h, (char *) &data, rx_buf, 1);

    if (lastTransfer)
    {
        gpioWrite(cs, 1);
    }

    return rx_buf[0];
}
#endif

int micro_xfer(char *tx_buf, char *rx_buf, const unsigned count)
{
    return spiXfer(spi_micro_h, tx_buf, rx_buf, count);
}

void enable_drivers(bool enable)
{
    gpioWrite(m_en, enable);
}
