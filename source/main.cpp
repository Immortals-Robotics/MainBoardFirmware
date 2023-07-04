#include <cstdio>
#include <cmath>
#include <unordered_map>

#include <pigpiod_if2.h>

#include "spi_hal.h"
#include "motor.h"
#include "micro.h"
#include "command.h"

#include "immortals/micro.pb.h"
#include <google/protobuf/util/delimited_message_util.h>

void motors_test()
{
    std::unordered_map<Immortals::Motor::Id, Immortals::Motor> motor_map = 
    {
        //{Immortals::Motor::Id::MD, Immortals::Motor{Immortals::Motor::Id::MD}},
        {Immortals::Motor::Id::M1, Immortals::Motor{Immortals::Motor::Id::M1}},
        {Immortals::Motor::Id::M2, Immortals::Motor{Immortals::Motor::Id::M2}},
        {Immortals::Motor::Id::M3, Immortals::Motor{Immortals::Motor::Id::M3}},
        {Immortals::Motor::Id::M4, Immortals::Motor{Immortals::Motor::Id::M4}},
    };

    enable_drivers(true);

    for (auto& motor : motor_map)
    {
        motor.second.init();
        motor.second.setMotionMode(Immortals::Motor::MotionMode::Velocity);
    }

    // Rotate right
    for (auto& motor : motor_map)
    {
        motor.second.setTargetVelocity(1500);
    }

    time_sleep(1);

    // Rotate left
    for (auto& motor : motor_map)
    {
        motor.second.setTargetVelocity(-1500);
    }

    time_sleep(1);

    // Stop
    for (auto& motor : motor_map)
    {
        motor.second.setTargetVelocity(0);
        motor.second.setMotionMode(Immortals::Motor::MotionMode::Stopped);
    }
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

struct WheelConfig
{
    // wheel radius in meters
    float radius;
    // distance from center of robot to wheel in meters
    float distance;
    // angle of wheel from robot's x (right) vector in radians
    float angle;
};

float compute_wheel_velocity(const Immortals::Protos::MoveLocalVelocity& velocity, const WheelConfig& wheel)
{
    const float vx = -velocity.left();
    const float vy =  velocity.forward();
    const float w  =  velocity.angular();

    return (1.0f / wheel.radius) * (vx * std::cos(wheel.angle) + vy * std::sin(wheel.angle) + w * wheel.distance);
}

void commands_test()
{
    Immortals::Micro micro{};

    static constexpr float wheel_radius = 0.05f;
    static constexpr float wheel_distance = 0.1f;

    const std::unordered_map<Immortals::Motor::Id, WheelConfig> wheel_config =
    {
        {Immortals::Motor::Id::M1, {wheel_radius, wheel_distance,  2.4958208f}}, //  180 - 37
        {Immortals::Motor::Id::M2, {wheel_radius, wheel_distance,  0.6457718f}}, //  37
        {Immortals::Motor::Id::M3, {wheel_radius, wheel_distance, -0.7853982f}}, // -45
        {Immortals::Motor::Id::M4, {wheel_radius, wheel_distance, -2.3561945f}}, // -135
    };

    std::unordered_map<Immortals::Motor::Id, Immortals::Motor> motor_map = 
    {
        {Immortals::Motor::Id::MD, Immortals::Motor{Immortals::Motor::Id::MD}},
        {Immortals::Motor::Id::M1, Immortals::Motor{Immortals::Motor::Id::M1}},
        {Immortals::Motor::Id::M2, Immortals::Motor{Immortals::Motor::Id::M2}},
        {Immortals::Motor::Id::M3, Immortals::Motor{Immortals::Motor::Id::M3}},
        {Immortals::Motor::Id::M4, Immortals::Motor{Immortals::Motor::Id::M4}},
    };

    for (auto& motor : motor_map)
    {
        motor.second.init();
        motor.second.setMotionMode(Immortals::Motor::MotionMode::Velocity);
    }

    micro.requestStatus();

    Immortals::Command command{};
    command.setRobotId(micro.getStatus().robot_id());
    command.connect();

    while (true)
    {
        if (command.receive())
        {
            const Immortals::Protos::RobotCommand& robot_command = command.getCommand();

            Immortals::Protos::MicroCommand micro_command{};

            printf("Received command: %d\n", robot_command.mikona_enabled());

            
            if (robot_command.has_move_command())
            {
                if (robot_command.move_command().has_wheel_velocity())
                {
                    printf("Received wheel velocity");

                    // TODO: take pole count into account

                    const float wheel_velocity[4] = 
                    {
                        robot_command.move_command().wheel_velocity().front_left(),
                        robot_command.move_command().wheel_velocity().front_right(),
                        robot_command.move_command().wheel_velocity().back_right(),
                        robot_command.move_command().wheel_velocity().back_left(),
                    };

                    motor_map[Immortals::Motor::Id::FrontLeft ].setTargetVelocity(robot_command.move_command().wheel_velocity().front_left()  / wheel_config.at(Immortals::Motor::Id::FrontLeft ).radius);
                    motor_map[Immortals::Motor::Id::FrontRight].setTargetVelocity(robot_command.move_command().wheel_velocity().front_right() / wheel_config.at(Immortals::Motor::Id::FrontRight).radius);
                    motor_map[Immortals::Motor::Id::BackRight ].setTargetVelocity(robot_command.move_command().wheel_velocity().back_right()  / wheel_config.at(Immortals::Motor::Id::BackRight ).radius);
                    motor_map[Immortals::Motor::Id::BackLeft  ].setTargetVelocity(robot_command.move_command().wheel_velocity().back_left()   / wheel_config.at(Immortals::Motor::Id::BackLeft  ).radius);

                }
                else if (robot_command.move_command().has_local_velocity())
                {
                    printf("Received local velocity");

                    const Immortals::Protos::MoveLocalVelocity& local_velocity = robot_command.move_command().local_velocity();

                    motor_map[Immortals::Motor::Id::FrontLeft ].setTargetVelocity(compute_wheel_velocity(local_velocity, wheel_config.at(Immortals::Motor::Id::FrontLeft)));
                    motor_map[Immortals::Motor::Id::FrontRight].setTargetVelocity(compute_wheel_velocity(local_velocity, wheel_config.at(Immortals::Motor::Id::FrontRight)));
                    motor_map[Immortals::Motor::Id::BackRight ].setTargetVelocity(compute_wheel_velocity(local_velocity, wheel_config.at(Immortals::Motor::Id::BackRight)));
                    motor_map[Immortals::Motor::Id::BackLeft  ].setTargetVelocity(compute_wheel_velocity(local_velocity, wheel_config.at(Immortals::Motor::Id::BackLeft)));
                }
                else if (robot_command.move_command().has_global_velocity())
                {
                    printf("Received global velocity");
                }
                else
                {
                    printf("Received unknown move command");
                }
            }
        
            micro_command.mutable_mikona()->set_charge(robot_command.mikona_enabled());

            // TODO: convert from m/s to uS pulse width
            if (robot_command.kick_type() == Immortals::Protos::RobotCommand_KickType_Direct)
            {
                micro_command.mutable_mikona()->set_kick_a(robot_command.kick_speed());
            }
            else if (robot_command.kick_type() == Immortals::Protos::RobotCommand_KickType_Chip)
            {
                micro_command.mutable_mikona()->set_kick_b(robot_command.kick_speed());
            }

            motor_map[Immortals::Motor::Id::Dribbler].setTargetVelocity(robot_command.dribbler_speed());
        }
    }
}

int main(int argc, char* argv[])
{
    init_spi();

    motors_test();

    micro_test();
    commands_test();

    shutdown_spi();

    return 0;
}
