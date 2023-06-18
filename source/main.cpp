#include <cstdio>
#include <pigpiod_if2.h>

#include "spi_hal.h"

#include "motor.h"
#include "micro.h"

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
    Micro micro{};

    while (true)
    {
        Immortals::Protos::MicroCommand command{};

        Immortals::Protos::MikonaCommand* const mikona = command.mutable_mikona();
        mikona->set_charge(true);

        micro.sendCommand(command);

        const Immortals::Protos::MicroStatus& status = micro.getStatus();
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
