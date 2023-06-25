#pragma once
#include <cstdint>

namespace Immortals
{
class Motor
{
public:
    enum class MotionMode : uint8_t
    {
        Unknown,
        Velocity,
        Position,
        Stopped,
    };

    Motor(uint8_t t_id);
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

    int m_id;
};
} // namespace Immortals
