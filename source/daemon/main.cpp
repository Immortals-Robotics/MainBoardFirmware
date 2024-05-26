#include "command.h"
#include "micro.h"
#include "motor.h"
#include "spi_hal.h"

#include "icm-42688/device.h"

std::unordered_map<Immortals::Daemon::Motor::Id, Immortals::Daemon::Motor> g_motor_map;

void motorsTest()
{
    enable_drivers(true);

    // Rotate right
    for (auto &[id, motor] : g_motor_map)
    {
        motor.setTargetVelocityRpm(100);
    }

    std::this_thread::sleep_for(1s);

    // Rotate left
    for (auto &[id, motor] : g_motor_map)
    {
        motor.setTargetVelocityRpm(-100);
    }

    std::this_thread::sleep_for(1s);

    // Stop
    for (auto &[id, motor] : g_motor_map)
    {
        motor.setTargetVelocityRpm(0);
        motor.setMotionMode(Immortals::Daemon::Motor::MotionMode::Stopped);
    }
}

struct WheelConfig
{
    // distance from center of robot to wheel in meters
    float distance;
    // angle of wheel from robot's x (right) vector in radians
    float angle;
};

float computeWheelVelocity(const Protos::Immortals::Command &t_command, const WheelConfig &t_wheel)
{
    // TODO: rotate motion
    const float vx = -t_command.motion().x();
    const float vy = t_command.motion().y();
    const float w  = t_command.target_angle().deg() - t_command.current_angle().deg();

    return vx * std::cos(t_wheel.angle) + vy * std::sin(t_wheel.angle) + w * t_wheel.distance;
}

int main()
{
    Immortals::Common::Services::initialize({
        .t_config_path = std::filesystem::path{DATA_DIR} / "config.toml",
    });

    if (!init_spi())
    {
        Immortals::Common::logCritical("Failed to initialize SPI");
        return 1;
    }

    ICM42688::Device imu{0, ICM42688::Device::Connection::Spi};
    if (imu.begin() < 0)
    {
        Immortals::Common::logCritical("IMU initialization failed");
    }

    Immortals::Daemon::Micro micro{};

    static constexpr float   kWheelRadius   = 0.05f;
    static constexpr float   kWheelDistance = 0.1f;
    static constexpr uint8_t kPoleCount     = 8;

    const std::unordered_map<Immortals::Daemon::Motor::Id, WheelConfig> wheel_config = {
        {Immortals::Daemon::Motor::Id::M1, {kWheelDistance, 2.4958208f}},  //  180 - 37
        {Immortals::Daemon::Motor::Id::M2, {kWheelDistance, 0.6457718f}},  //  37
        {Immortals::Daemon::Motor::Id::M3, {kWheelDistance, -0.7853982f}}, // -45
        {Immortals::Daemon::Motor::Id::M4, {kWheelDistance, -2.3561945f}}, // -135
    };

    g_motor_map = {
        {Immortals::Daemon::Motor::Id::MD, Immortals::Daemon::Motor{Immortals::Daemon::Motor::Id::MD, 1, 0.01f}},
        {Immortals::Daemon::Motor::Id::M1,
         Immortals::Daemon::Motor{Immortals::Daemon::Motor::Id::M1, kPoleCount, kWheelRadius}},
        {Immortals::Daemon::Motor::Id::M2,
         Immortals::Daemon::Motor{Immortals::Daemon::Motor::Id::M2, kPoleCount, kWheelRadius}},
        {Immortals::Daemon::Motor::Id::M3,
         Immortals::Daemon::Motor{Immortals::Daemon::Motor::Id::M3, kPoleCount, kWheelRadius}},
        {Immortals::Daemon::Motor::Id::M4,
         Immortals::Daemon::Motor{Immortals::Daemon::Motor::Id::M4, kPoleCount, kWheelRadius}},
    };

    for (auto &[id, motor] : g_motor_map)
    {
        motor.init();
        motor.setMotionMode(Immortals::Daemon::Motor::MotionMode::Velocity);
    }

    micro.requestStatus();

    Immortals::Daemon::Command command{};
    const unsigned             id = micro.getStatus().robot_id();
    Immortals::Common::logInfo("ID: {}", id);
    command.setRobotId(id);
    command.connect();

    bool wifi_led = false;

    while (true)
    {
        if (command.receive())
        {
            const Protos::Immortals::Command robot_command = command.getCommand();

            Protos::Immortals::MicroCommand micro_command{};

            Immortals::Common::logDebug("Received command");

            wifi_led = !wifi_led;

            micro_command.mutable_led()->set_wifi_activity(wifi_led);

            Immortals::Common::logDebug("Received local velocity");

            g_motor_map[Immortals::Daemon::Motor::Id::FrontLeft].setTargetVelocityMs(
                computeWheelVelocity(robot_command, wheel_config.at(Immortals::Daemon::Motor::Id::FrontLeft)));
            g_motor_map[Immortals::Daemon::Motor::Id::FrontRight].setTargetVelocityMs(
                computeWheelVelocity(robot_command, wheel_config.at(Immortals::Daemon::Motor::Id::FrontRight)));
            g_motor_map[Immortals::Daemon::Motor::Id::BackRight].setTargetVelocityMs(
                computeWheelVelocity(robot_command, wheel_config.at(Immortals::Daemon::Motor::Id::BackRight)));
            g_motor_map[Immortals::Daemon::Motor::Id::BackLeft].setTargetVelocityMs(
                computeWheelVelocity(robot_command, wheel_config.at(Immortals::Daemon::Motor::Id::BackLeft)));

            micro_command.mutable_mikona()->set_charge(true);

            // TODO: convert from m/s to uS pulse width
            micro_command.mutable_mikona()->set_kick_a(robot_command.shoot());
            micro_command.mutable_mikona()->set_kick_b(robot_command.chip());

            g_motor_map[Immortals::Daemon::Motor::Id::Dribbler].setTargetVelocityRpm(robot_command.dribbler());
        }
    }

    shutdown_spi();

    Immortals::Common::Services::shutdown();

    return 0;
}
