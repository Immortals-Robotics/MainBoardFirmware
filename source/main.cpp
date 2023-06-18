#include <cstdio>

#include <pigpiod_if2.h>

#include "spi_hal.h"

#include "motor.h"

#include "immortals/micro.pb.h"
#include <google/protobuf/util/delimited_message_util.h>

void velocity_test(const uint8_t motor_id)
{
    Motor motor{motor_id};

    motor.init();

    // Switch to velocity mode
    motor.setMotionMode(Motor::MotionMode::Velocity);

    // Rotate right
    motor.setTargetVelocity(8000);
    time_sleep(3);

    // Rotate left
    motor.setTargetVelocity(-8000);
    time_sleep(3);

    // Stop
    motor.setTargetVelocity(0);
    motor.setMotionMode(Motor::MotionMode::Stopped);
}

void motors_test(const uint8_t motor_id)
{
    enable_driver(true);

    velocity_test(motor_id);
}

void micro_test()
{
    static constexpr size_t buffer_size = 128;

    char rx_buf[buffer_size] = {};
    char tx_buf[buffer_size] = {};

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

        micro_xfer(tx_buf, rx_buf, buffer_size);

        Immortals::Protos::MicroStatus status{};
        google::protobuf::io::ArrayInputStream input_stream{rx_buf, buffer_size};
        google::protobuf::util::ParseDelimitedFromZeroCopyStream(&status, &input_stream, nullptr);
        printf("ir: %d\n", status.balldetected());
    }
}

int main(int argc, char* argv[])
{
    init_spi();

    motors_test(3);
    micro_test();

    shutdown_spi();

    return 0;
}
