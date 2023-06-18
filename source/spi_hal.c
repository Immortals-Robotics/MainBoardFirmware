#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <pigpiod_if2.h>

int pi_h;
int spi_h;

const uint8_t cs = 8;

const uint8_t mux_a0 = 27;
const uint8_t mux_a1 = 22;
const uint8_t mux_a2 = 5;
const uint8_t mux_a3 = 6;

const uint8_t m_en = 13;

const uint8_t unused_ce_id = 15;

static const uint8_t ctrl_ce_id[5] = 
{
    0, // Dribbler
    2, // Motor 1
    8, // Motor 2
    6, // Motor 3
    4  // Motor 4
};

static const uint8_t drv_ce_id[5] =
{
    1, // Dribbler
    3, // Motor 1
    9, // Motor 2
    7, // Motor 3
    5  // Motor 4
};

bool init_spi()
{
    pi_h = pigpio_start(0, 0); /* Connect to local Pi. */

    if (pi_h < 0)
    {
        printf("Can't connect to pigpio daemon\n");
        return false;
    }

    set_mode(pi_h, cs, PI_OUTPUT);

    set_mode(pi_h, mux_a0, PI_OUTPUT);
    set_mode(pi_h, mux_a1, PI_OUTPUT);
    set_mode(pi_h, mux_a2, PI_OUTPUT);
    set_mode(pi_h, mux_a3, PI_OUTPUT);

    set_mode(pi_h, m_en, PI_OUTPUT);

    const unsigned spi_flags =
        PI_SPI_FLAGS_MODE(3) // mode 3
        | PI_SPI_FLAGS_RESVD(1)   // ce0 not reserved
        ;

    spi_h = spi_open(pi_h, 0, 1000000, spi_flags);

    if (spi_h < 0)
    {
        printf("Can't create SPI handle\n");
        return false;
    }

    return true;
}

void shutdown_spi()
{
    gpio_write(pi_h, mux_a0, 1);
    gpio_write(pi_h, mux_a1, 1);
    gpio_write(pi_h, mux_a2, 1);
    gpio_write(pi_h, mux_a3, 1);

    spi_close(pi_h, spi_h);

    pigpio_stop(pi_h); /* Disconnect from local Pi. */
}

void select_ce(const uint8_t ce_id)
{
    gpio_write(pi_h, mux_a0, (ce_id & 1) >> 0);
    gpio_write(pi_h, mux_a1, (ce_id & 2) >> 1);
    gpio_write(pi_h, mux_a2, (ce_id & 4) >> 2);
    gpio_write(pi_h, mux_a3, (ce_id & 8) >> 3);
}

uint8_t tmc4671_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
    const uint8_t ce_id = ctrl_ce_id[motor];
    select_ce(ce_id);

    char rx_buf[64] = {0};

    gpio_write(pi_h, cs, 0);

    spi_xfer(pi_h, spi_h, &data, rx_buf, 1);

    if (lastTransfer)
    {
        gpio_write(pi_h, cs, 1);
    }

    return rx_buf[0];
}

uint8_t tmc6200_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
    const uint8_t ce_id = drv_ce_id[motor];
    select_ce(ce_id);

    char rx_buf[64] = {};

    gpio_write(pi_h, cs, 0);

    spi_xfer(pi_h, spi_h, &data, rx_buf, 1);

    if (lastTransfer)
    {
        gpio_write(pi_h, cs, 1);
    }

    return rx_buf[0];
}

void enable_driver(bool enable)
{
    gpio_write(pi_h, m_en, enable);
}