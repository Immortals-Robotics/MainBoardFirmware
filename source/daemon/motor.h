#pragma once

namespace Immortals::Daemon
{
class Motor
{
public:
    enum class Id : uint8_t
    {
        Unknown = (uint8_t) (~0),

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
    Motor(Id t_id, uint8_t t_pole_count, float t_radius = 1.0f);
    ~Motor() = default;

    bool init();

    void                     setMotionMode(MotionMode t_mode);
    [[nodiscard]] MotionMode getMotionMode() const;

    void              setTargetVelocityRaw(int t_velocity);
    [[nodiscard]] int getTargetVelocityRaw() const;

    [[nodiscard]] int getActualVelocityRaw() const;

    void setTargetVelocityRpm(const int t_velocity)
    {
        setTargetVelocityRaw(t_velocity * m_pole_count);
    }

    [[nodiscard]] int getTargetVelocityRpm() const
    {
        return getTargetVelocityRaw() / m_pole_count;
    }

    [[nodiscard]] int getActualVelocityRpm() const
    {
        return getActualVelocityRaw() / m_pole_count;
    }

    void setTargetVelocityMs(float t_velocity)
    {
        setTargetVelocityRpm(t_velocity / m_radius);
    }

    [[nodiscard]] float getTargetVelocityMs() const
    {
        return getTargetVelocityRpm() * m_radius;
    }

    [[nodiscard]] float getActualVelocityMs() const
    {
        return getActualVelocityRpm() * m_radius;
    }

private:
    bool initController();
    bool initDriver();

    void calibrateAdc();

    void enableDriver(bool enable);

    int m_id = (uint8_t) Id::Unknown;

    // wheel radius in meters
    float m_radius;

    uint8_t m_pole_count = 0;
};
} // namespace Immortals::Daemon
