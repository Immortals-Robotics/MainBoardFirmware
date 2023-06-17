#include <cstdio>

#include <pigpiod_if2.h>

#include "spi_hal.h"

extern "C" {
#include "tmc/ic/TMC4671/TMC4671.h"
#include "tmc/ic/TMC6200/TMC6200.h"
}

#include "immortals/micro.pb.h"
#include <google/protobuf/util/delimited_message_util.h>

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

    static constexpr size_t buffer_size = 128;

    char rx_buf[buffer_size] = {};
    char tx_buf[buffer_size] = {};

    int h = spi_open(pi, 1, 4 * 1000 * 1000, 
        PI_SPI_FLAGS_AUX_SPI(1) | 
                PI_SPI_FLAGS_MODE(0));

    if (h < 0)
        return;

    while (true)
    {
        Immortals::Protos::MicroCommand command{};

        Immortals::Protos::MikonaCommand* const mikona = command.mutable_mikona();
        mikona->set_charge(true);
        mikona->set_discharge(false);
        mikona->set_kick_a(0);
        mikona->set_kick_b(0);

        Immortals::Protos::LEDCommand *const led = command.mutable_led();
        led->set_wifi_connected(true);
        led->set_wifi_acitivity(false);
        led->set_fault(false);

        command.set_buzzer(false);

        command.SerializeToArray(tx_buf, buffer_size);

        google::protobuf::io::ArrayOutputStream output_stream{tx_buf, buffer_size};
        google::protobuf::util::SerializeDelimitedToZeroCopyStream(command, &output_stream);

        spi_xfer(pi, h, tx_buf, rx_buf, buffer_size);

        Immortals::Protos::MicroStatus status{};
        google::protobuf::io::ArrayInputStream input_stream{rx_buf, buffer_size};
        google::protobuf::util::ParseDelimitedFromZeroCopyStream(&status, &input_stream, nullptr);
        printf("ir: %d\n", status.balldetected());
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
