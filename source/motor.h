#pragma once
#include <cstdint>

namespace Immortals
{
class Motor
{
public:
    enum class Id : uint8_t
    {
        Unknown = (uint8_t)(~0),

        M1 = 1,
        M2 = 2,
        M3 = 3,
        M4 = 4,
        MD = 0,

        FrontLeft  = M1,
        FrontRight = M2,
        BackRight  = M3,
        BackLeft   = M4,
        Dribbler   = MD,
    };

    enum class MotionMode : uint8_t
    {
        Unknown,
        Velocity,
        Position,
        Stopped,
    };

    Motor() = default;
    Motor(Id t_id);
    ~Motor() = default;

    bool init();

    void                     setMotionMode(MotionMode t_mode);
    [[nodiscard]] MotionMode getMotionMode() const;

    void              setTargetVelocity(int t_velocity);
    [[nodiscard]] int getTargetVelocity() const;

    [[nodiscard]] int getActualVelocity() const;

private:
    bool initController();
    bool initDriver();

    void calibrateAdc();

    void enableDriver(bool enable);

    int m_id = (uint8_t)Id::Unknown;
};
} // namespace Immortals
