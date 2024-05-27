#pragma once

namespace Immortals::Daemon
{
class Motor
{
public:
    enum class Id : uint8_t
    {
        Unknown = std::numeric_limits<uint8_t>::max(),

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

    uint8_t idInt() const
    {
        return static_cast<uint8_t>(m_id);
    }

    Id m_id = Id::Unknown;

    // wheel radius in meters
    float m_radius;

    uint8_t m_pole_count = 0;
};
} // namespace Immortals::Daemon

#if FEATURE_LOGGING
template <>
struct fmt::formatter<Immortals::Daemon::Motor::Id> : fmt::formatter<std::string>
{
    static constexpr std::string_view idName(const Immortals::Daemon::Motor::Id t_id)
    {
        switch (t_id)
        {
        default:
        case Immortals::Daemon::Motor::Id::Unknown:
            return "Unknown";
        case Immortals::Daemon::Motor::Id::FrontLeft:
            return "FrontLeft";
        case Immortals::Daemon::Motor::Id::FrontRight:
            return "FrontRight";
        case Immortals::Daemon::Motor::Id::BackRight:
            return "BackRight";
        case Immortals::Daemon::Motor::Id::BackLeft:
            return "BackLeft";
        case Immortals::Daemon::Motor::Id::Dribbler:
            return "Dribbler";
        }
    }

    auto format(Immortals::Daemon::Motor::Id t_id, format_context &t_ctx) const
    {
        return fmt::format_to(t_ctx.out(), "{}", idName(t_id));
    }
};
#endif
