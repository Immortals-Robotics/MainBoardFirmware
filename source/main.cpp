#include <cstdio>
#include <pigpiod_if2.h>

#include "spi_hal.h"

#include "motor.h"
#include "micro.h"
#include "command.h"

#include "immortals/micro.pb.h"
#include <google/protobuf/util/delimited_message_util.h>

void velocity_test(const uint8_t motor_id)
{
    Immortals::Motor motor{motor_id};

    motor.init();

    // Switch to velocity mode
    motor.setMotionMode(Immortals::Motor::MotionMode::Velocity);

    // Rotate right
    motor.setTargetVelocity(8000);
    time_sleep(3);

    // Rotate left
    motor.setTargetVelocity(-8000);
    time_sleep(3);

    // Stop
    motor.setTargetVelocity(0);
    motor.setMotionMode(Immortals::Motor::MotionMode::Stopped);
}

void motors_test(const uint8_t motor_id)
{
    enable_driver(true);

    velocity_test(motor_id);
}

void micro_test()
{
    Immortals::Micro micro{};

    while (true)
    {
        Immortals::Protos::MicroCommand command{};

        command.mutable_led()->set_wifi_connected(micro.getStatus().button());

        micro.sendCommand(command);

        const Immortals::Protos::MicroStatus& status = micro.getStatus();

        time_sleep(0.01);
    }
}

void commands_test()
{
    Immortals::Command command{};
    command.setId(0);
    command.connect();

    while (true)
    {
        if (command.receive())
        {
            const Immortals::Protos::RobotCommand& robot_command = command.getCommand();

            printf("Received command: %d\n", robot_command.mikona_enabled());
        }
    }
}

int main(int argc, char* argv[])
{
    init_spi();

    motors_test(3);
    micro_test();
    commands_test();

    shutdown_spi();

    return 0;
}
