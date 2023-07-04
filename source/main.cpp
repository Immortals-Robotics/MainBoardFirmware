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

std::unordered_map<Immortals::Motor::Id, Immortals::Motor> motor_map;

void motors_test()
{
    enable_drivers(true);

    // Rotate right
    for (auto& motor : motor_map)
    {
        motor.second.setTargetVelocityRpm(100);
    }

    time_sleep(1);

    // Rotate left
    for (auto& motor : motor_map)
    {
        motor.second.setTargetVelocityRpm(-100);
    }

    time_sleep(1);

    // Stop
    for (auto& motor : motor_map)
    {
        motor.second.setTargetVelocityRpm(0);
        motor.second.setMotionMode(Immortals::Motor::MotionMode::Stopped);
    }
}

struct WheelConfig
{
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

    return vx * std::cos(wheel.angle) + vy * std::sin(wheel.angle) + w * wheel.distance;
}

int main(int argc, char* argv[])
{
    Immortals::Services::initialize();

    if (!init_spi())
    {
        LOG_CRITICAL("Failed to initialize SPI\n");
        return 1;
    }

    Immortals::Micro micro{};

    static constexpr float wheel_radius = 0.05f;
    static constexpr float wheel_distance = 0.1f;
    static constexpr uint8_t pole_count = 8;

    const std::unordered_map<Immortals::Motor::Id, WheelConfig> wheel_config =
    {
        {Immortals::Motor::Id::M1, {wheel_distance,  2.4958208f}}, //  180 - 37
        {Immortals::Motor::Id::M2, {wheel_distance,  0.6457718f}}, //  37
        {Immortals::Motor::Id::M3, {wheel_distance, -0.7853982f}}, // -45
        {Immortals::Motor::Id::M4, {wheel_distance, -2.3561945f}}, // -135
    };

    motor_map = 
    {
        {Immortals::Motor::Id::MD, Immortals::Motor{Immortals::Motor::Id::MD, 1, 0.01f}},
        {Immortals::Motor::Id::M1, Immortals::Motor{Immortals::Motor::Id::M1, pole_count, wheel_radius}},
        {Immortals::Motor::Id::M2, Immortals::Motor{Immortals::Motor::Id::M2, pole_count, wheel_radius}},
        {Immortals::Motor::Id::M3, Immortals::Motor{Immortals::Motor::Id::M3, pole_count, wheel_radius}},
        {Immortals::Motor::Id::M4, Immortals::Motor{Immortals::Motor::Id::M4, pole_count, wheel_radius}},
    };

    for (auto& motor : motor_map)
    {
        motor.second.init();
        motor.second.setMotionMode(Immortals::Motor::MotionMode::Velocity);
    }

    motors_test();

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

            LOG_DEBUG("Received command\n");

            
            if (robot_command.has_move_command())
            {
                if (robot_command.move_command().has_wheel_velocity())
                {
                    LOG_DEBUG("Received wheel velocity");

                    // TODO: take pole count into account

                    const float wheel_velocity[4] = 
                    {
                        robot_command.move_command().wheel_velocity().front_left(),
                        robot_command.move_command().wheel_velocity().front_right(),
                        robot_command.move_command().wheel_velocity().back_right(),
                        robot_command.move_command().wheel_velocity().back_left(),
                    };

                    motor_map[Immortals::Motor::Id::FrontLeft ].setTargetVelocityMs(robot_command.move_command().wheel_velocity().front_left() );
                    motor_map[Immortals::Motor::Id::FrontRight].setTargetVelocityMs(robot_command.move_command().wheel_velocity().front_right());
                    motor_map[Immortals::Motor::Id::BackRight ].setTargetVelocityMs(robot_command.move_command().wheel_velocity().back_right() );
                    motor_map[Immortals::Motor::Id::BackLeft  ].setTargetVelocityMs(robot_command.move_command().wheel_velocity().back_left()  );

                }
                else if (robot_command.move_command().has_local_velocity())
                {
                    LOG_DEBUG("Received local velocity");

                    const Immortals::Protos::MoveLocalVelocity& local_velocity = robot_command.move_command().local_velocity();

                    motor_map[Immortals::Motor::Id::FrontLeft ].setTargetVelocityMs(compute_wheel_velocity(local_velocity, wheel_config.at(Immortals::Motor::Id::FrontLeft)));
                    motor_map[Immortals::Motor::Id::FrontRight].setTargetVelocityMs(compute_wheel_velocity(local_velocity, wheel_config.at(Immortals::Motor::Id::FrontRight)));
                    motor_map[Immortals::Motor::Id::BackRight ].setTargetVelocityMs(compute_wheel_velocity(local_velocity, wheel_config.at(Immortals::Motor::Id::BackRight)));
                    motor_map[Immortals::Motor::Id::BackLeft  ].setTargetVelocityMs(compute_wheel_velocity(local_velocity, wheel_config.at(Immortals::Motor::Id::BackLeft)));
                }
                else if (robot_command.move_command().has_global_velocity())
                {
                    LOG_DEBUG("Received global velocity");
                }
                else
                {
                    LOG_ERROR("Received unknown move command");
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

            motor_map[Immortals::Motor::Id::Dribbler].setTargetVelocityRpm(robot_command.dribbler_speed());
        }
    }

    shutdown_spi();

    Immortals::Services::shutdown();

    return 0;
}
