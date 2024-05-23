#include "command.h"
#include "micro.h"
#include "motor.h"
#include "spi_hal.h"

std::unordered_map<Immortals::Motor::Id, Immortals::Motor> motor_map;

void motors_test()
{
    enable_drivers(true);

    // Rotate right
    for (auto &motor : motor_map)
    {
        motor.second.setTargetVelocityRpm(100);
    }

    time_sleep(1);

    // Rotate left
    for (auto &motor : motor_map)
    {
        motor.second.setTargetVelocityRpm(-100);
    }

    time_sleep(1);

    // Stop
    for (auto &motor : motor_map)
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

float compute_wheel_velocity(const Protos::Immortals::Command &command, const WheelConfig &wheel)
{
    // TODO: rotate motion
    const float vx = -command.motion().x();
    const float vy = command.motion().y();
    const float w  = command.target_angle().deg() - command.current_angle().deg();

    return vx * std::cos(wheel.angle) + vy * std::sin(wheel.angle) + w * wheel.distance;
}

int main(int argc, char *argv[])
{
    Immortals::Services::initialize();

    if (!init_spi())
    {
        LOG_CRITICAL("Failed to initialize SPI\n");
        return 1;
    }

    Immortals::Micro micro{};

    static constexpr float   wheel_radius   = 0.05f;
    static constexpr float   wheel_distance = 0.1f;
    static constexpr uint8_t pole_count     = 8;

    const std::unordered_map<Immortals::Motor::Id, WheelConfig> wheel_config = {
        {Immortals::Motor::Id::M1, {wheel_distance, 2.4958208f}},  //  180 - 37
        {Immortals::Motor::Id::M2, {wheel_distance, 0.6457718f}},  //  37
        {Immortals::Motor::Id::M3, {wheel_distance, -0.7853982f}}, // -45
        {Immortals::Motor::Id::M4, {wheel_distance, -2.3561945f}}, // -135
    };

    motor_map = {
        {Immortals::Motor::Id::MD, Immortals::Motor{Immortals::Motor::Id::MD, 1, 0.01f}},
        {Immortals::Motor::Id::M1, Immortals::Motor{Immortals::Motor::Id::M1, pole_count, wheel_radius}},
        {Immortals::Motor::Id::M2, Immortals::Motor{Immortals::Motor::Id::M2, pole_count, wheel_radius}},
        {Immortals::Motor::Id::M3, Immortals::Motor{Immortals::Motor::Id::M3, pole_count, wheel_radius}},
        {Immortals::Motor::Id::M4, Immortals::Motor{Immortals::Motor::Id::M4, pole_count, wheel_radius}},
    };

    for (auto &motor : motor_map)
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
            const Protos::Immortals::Command &robot_command = command.getCommand();

            Protos::Immortals::MicroCommand micro_command{};

            LOG_DEBUG("Received command\n");

            static bool wifi_led = false;
            wifi_led             = !wifi_led;

            micro_command.mutable_led()->set_wifi_activity(wifi_led);

            LOG_DEBUG("Received local velocity");

            motor_map[Immortals::Motor::Id::FrontLeft].setTargetVelocityMs(
                compute_wheel_velocity(robot_command, wheel_config.at(Immortals::Motor::Id::FrontLeft)));
            motor_map[Immortals::Motor::Id::FrontRight].setTargetVelocityMs(
                compute_wheel_velocity(robot_command, wheel_config.at(Immortals::Motor::Id::FrontRight)));
            motor_map[Immortals::Motor::Id::BackRight].setTargetVelocityMs(
                compute_wheel_velocity(robot_command, wheel_config.at(Immortals::Motor::Id::BackRight)));
            motor_map[Immortals::Motor::Id::BackLeft].setTargetVelocityMs(
                compute_wheel_velocity(robot_command, wheel_config.at(Immortals::Motor::Id::BackLeft)));

            micro_command.mutable_mikona()->set_charge(true);

            // TODO: convert from m/s to uS pulse width
            micro_command.mutable_mikona()->set_kick_a(robot_command.shoot());
            micro_command.mutable_mikona()->set_kick_b(robot_command.chip());

            motor_map[Immortals::Motor::Id::Dribbler].setTargetVelocityRpm(robot_command.dribbler());
        }
    }

    shutdown_spi();

    Immortals::Services::shutdown();

    return 0;
}
