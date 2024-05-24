#include "command.h"
#include "micro.h"
#include "motor.h"
#include "spi_hal.h"

std::unordered_map<Immortals::Daemon::Motor::Id, Immortals::Daemon::Motor> motor_map;

void motors_test()
{
    enable_drivers(true);

    // Rotate right
    for (auto &motor : motor_map)
    {
        motor.second.setTargetVelocityRpm(100);
    }

    std::this_thread::sleep_for(1s);

    // Rotate left
    for (auto &motor : motor_map)
    {
        motor.second.setTargetVelocityRpm(-100);
    }

    std::this_thread::sleep_for(1s);

    // Stop
    for (auto &motor : motor_map)
    {
        motor.second.setTargetVelocityRpm(0);
        motor.second.setMotionMode(Immortals::Daemon::Motor::MotionMode::Stopped);
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

int main()
{
    Immortals::Common::Services::initialize();

    if (!init_spi())
    {
        Immortals::Common::logCritical("Failed to initialize SPI");
        return 1;
    }

    Immortals::Daemon::Micro micro{};

    static constexpr float   wheel_radius   = 0.05f;
    static constexpr float   wheel_distance = 0.1f;
    static constexpr uint8_t pole_count     = 8;

    const std::unordered_map<Immortals::Daemon::Motor::Id, WheelConfig> wheel_config = {
        {Immortals::Daemon::Motor::Id::M1, {wheel_distance, 2.4958208f}},  //  180 - 37
        {Immortals::Daemon::Motor::Id::M2, {wheel_distance, 0.6457718f}},  //  37
        {Immortals::Daemon::Motor::Id::M3, {wheel_distance, -0.7853982f}}, // -45
        {Immortals::Daemon::Motor::Id::M4, {wheel_distance, -2.3561945f}}, // -135
    };

    motor_map = {
        {Immortals::Daemon::Motor::Id::MD, Immortals::Daemon::Motor{Immortals::Daemon::Motor::Id::MD, 1, 0.01f}},
        {Immortals::Daemon::Motor::Id::M1,
         Immortals::Daemon::Motor{Immortals::Daemon::Motor::Id::M1, pole_count, wheel_radius}},
        {Immortals::Daemon::Motor::Id::M2,
         Immortals::Daemon::Motor{Immortals::Daemon::Motor::Id::M2, pole_count, wheel_radius}},
        {Immortals::Daemon::Motor::Id::M3,
         Immortals::Daemon::Motor{Immortals::Daemon::Motor::Id::M3, pole_count, wheel_radius}},
        {Immortals::Daemon::Motor::Id::M4,
         Immortals::Daemon::Motor{Immortals::Daemon::Motor::Id::M4, pole_count, wheel_radius}},
    };

    for (auto &motor : motor_map)
    {
        motor.second.init();
        motor.second.setMotionMode(Immortals::Daemon::Motor::MotionMode::Velocity);
    }

    micro.requestStatus();

    Immortals::Daemon::Command command{};
    const unsigned             id = micro.getStatus().robot_id();
    Immortals::Common::logInfo("ID: {}", id);
    command.setRobotId(id);
    command.connect();

    while (true)
    {
        if (command.receive())
        {
            const Protos::Immortals::Command robot_command{};

            Protos::Immortals::MicroCommand micro_command{};

            Immortals::Common::logDebug("Received command");

            static bool wifi_led = false;
            wifi_led             = !wifi_led;

            micro_command.mutable_led()->set_wifi_activity(wifi_led);

            Immortals::Common::logDebug("Received local velocity");

            motor_map[Immortals::Daemon::Motor::Id::FrontLeft].setTargetVelocityMs(
                compute_wheel_velocity(robot_command, wheel_config.at(Immortals::Daemon::Motor::Id::FrontLeft)));
            motor_map[Immortals::Daemon::Motor::Id::FrontRight].setTargetVelocityMs(
                compute_wheel_velocity(robot_command, wheel_config.at(Immortals::Daemon::Motor::Id::FrontRight)));
            motor_map[Immortals::Daemon::Motor::Id::BackRight].setTargetVelocityMs(
                compute_wheel_velocity(robot_command, wheel_config.at(Immortals::Daemon::Motor::Id::BackRight)));
            motor_map[Immortals::Daemon::Motor::Id::BackLeft].setTargetVelocityMs(
                compute_wheel_velocity(robot_command, wheel_config.at(Immortals::Daemon::Motor::Id::BackLeft)));

            micro_command.mutable_mikona()->set_charge(true);

            // TODO: convert from m/s to uS pulse width
            micro_command.mutable_mikona()->set_kick_a(robot_command.shoot());
            micro_command.mutable_mikona()->set_kick_b(robot_command.chip());

            motor_map[Immortals::Daemon::Motor::Id::Dribbler].setTargetVelocityRpm(robot_command.dribbler());
        }
    }

    shutdown_spi();

    Immortals::Common::Services::shutdown();

    return 0;
}
