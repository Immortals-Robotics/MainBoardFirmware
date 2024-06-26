#include "motor.h"

extern "C"
{
#include "tmc/ic/TMC4671/TMC4671.h"
#include "tmc/ic/TMC4671/TMC4671_Variants.h"
#include "tmc/ic/TMC6200/TMC6200.h"
}

namespace Immortals::Daemon
{
Motor::Motor(const Id t_id, const uint8_t t_pole_count, const float t_radius)
    : m_id(t_id), m_radius(t_radius), m_pole_count(t_pole_count)
{}

bool Motor::init()
{
    enableDriver(false);

    if (!initController())
    {
        Common::logError("Failed to initialize motor controller {}", m_id);
        return false;
    }

    if (!initDriver())
    {
        Common::logError("Failed to initialize motor driver {}", m_id);
        return false;
    }

    calibrateAdc();

    enableDriver(true);

    return true;
}

bool Motor::initController()
{
    // check the device
    tmc4671_writeInt(idInt(), TMC4671_CHIPINFO_ADDR, 0);
    const int result = tmc4671_readInt(idInt(), TMC4671_CHIPINFO_DATA);
    if (result != 0x34363731) // ASCII "4671"
    {
        return false;
    }

    // Motor type &  PWM configuration
    tmc4671_setMotorType(idInt(), TMC4671_THREE_PHASE_BLDC);
    tmc4671_setPolePairs(idInt(), m_pole_count); // 8 for main motors, 1 for the dribbler
    tmc4671_writeInt(idInt(), TMC4671_PWM_POLARITIES, 0x00000000);
    tmc4671_writeInt(idInt(), TMC4671_PWM_MAXCNT, 0x00000F9F);
    tmc4671_writeInt(idInt(), TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
    tmc4671_writeInt(idInt(), TMC4671_PWM_SV_CHOP, 0x00000007);

    // ADC configuration
    tmc4671_writeInt(idInt(), TMC4671_ADC_I_SELECT, 0x24000100);
    tmc4671_writeInt(idInt(), TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
    tmc4671_writeInt(idInt(), TMC4671_dsADC_MCLK_A, 0x20000000);
    tmc4671_writeInt(idInt(), TMC4671_dsADC_MCLK_B, 0x20000000);
    tmc4671_writeInt(idInt(), TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
    tmc4671_writeInt(idInt(), TMC4671_ADC_I0_SCALE_OFFSET, 0xFF0083DA);
    tmc4671_writeInt(idInt(), TMC4671_ADC_I1_SCALE_OFFSET, 0xFF0082F9);

    // Digital hall settings
    tmc4671_writeInt(idInt(), TMC4671_HALL_MODE, 0x00001000);
    tmc4671_writeInt(idInt(), TMC4671_HALL_PHI_E_PHI_M_OFFSET, 0x55550000);

    // ABN encoder settings
    tmc4671_writeInt(idInt(), TMC4671_ABN_DECODER_MODE, 0x00001000);
    tmc4671_writeInt(idInt(), TMC4671_ABN_DECODER_PPR, 0x00004000);
    tmc4671_writeInt(idInt(), TMC4671_ABN_DECODER_COUNT, 0x00002246);
    tmc4671_writeInt(idInt(), TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0xA9E00000);

    // Limits
    // TODO: read from the config file
    tmc4671_writeInt(idInt(), TMC4671_PIDOUT_UQ_UD_LIMITS, 0x00005A81);
    tmc4671_writeInt(idInt(), TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000007D0);

    // PI settings
    // TODO: read from the config file
    tmc4671_setTorqueFluxPI(idInt(), 256, 256);
    tmc4671_setVelocityPI(idInt(), 200, 15);
    tmc4671_setPositionPI(idInt(), 25, 2);

#if 0
    // Init encoder (mode 0)
    tmc4671_writeInt(idInt(), TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
    tmc4671_writeInt(idInt(), TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
    tmc4671_writeInt(idInt(), TMC4671_PHI_E_SELECTION, 0x00000001);
    tmc4671_writeInt(idInt(), TMC4671_PHI_E_EXT, 0x00000000);
    tmc4671_writeInt(idInt(), TMC4671_UQ_UD_EXT, 0x00000FA0);
    time_sleep(1);
    tmc4671_writeInt(idInt(), TMC4671_ABN_DECODER_COUNT, 0x00000000);
#endif

    tmc4671_switchToMotionMode(idInt(), TMC4671_MOTION_MODE_STOPPED);

    // Selectors
    // TODO: read from the config file
    tmc4671_writeInt(idInt(), TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_HALL);
    tmc4671_writeInt(idInt(), TMC4671_VELOCITY_SELECTION, TMC4671_VELOCITY_PHI_E_ABN);
    tmc4671_writeInt(idInt(), TMC4671_POSITION_SELECTION, TMC4671_POSITION_PHI_E_ABN);

    return true;
}

bool Motor::initDriver()
{
    const int version = TMC6200_FIELD_READ(idInt(), TMC6200_IOIN_OUTPUT, TMC6200_VERSION_MASK, TMC6200_VERSION_SHIFT);
    if (version != 0x10)
    {
        return false;
    }

    [[maybe_unused]] int gstat = tmc6200_readInt(idInt(), TMC6200_GSTAT);

    TMC6200_FIELD_UPDATE(idInt(), TMC6200_DRV_CONF, TMC6200_DRVSTRENGTH_MASK, TMC6200_DRVSTRENGTH_SHIFT, 1);

    // Set current amplification to 10x
    TMC6200_FIELD_UPDATE(idInt(), TMC6200_GCONF, TMC6200_AMPLIFICATION_MASK, TMC6200_AMPLIFICATION_SHIFT, 1);
    // Set current amplification to 10x
    TMC6200_FIELD_UPDATE(idInt(), TMC6200_GCONF, TMC6200_AMPLIFICATION_MASK, TMC6200_AMPLIFICATION_SHIFT, 1);
    // set default PWM configuration for use with TMC4671
    TMC6200_FIELD_UPDATE(idInt(), TMC6200_GCONF, TMC6200_SINGLELINE_MASK, TMC6200_SINGLELINE_SHIFT, 0);

    return true;
}

// should be called during init when the motor is not moving
// and the driver is disabled
void Motor::calibrateAdc()
{
    // loop over and find the median value
    static constexpr int loop_count = 10;

    Common::MedianFilter<uint16_t, loop_count> i0_filter;
    Common::MedianFilter<uint16_t, loop_count> i1_filter;

    for (int i = 0; i < loop_count; ++i)
    {
        const uint16_t i0_raw = tmc4671_readFieldWithDependency(idInt(), TMC4671_ADC_RAW_DATA, TMC4671_ADC_RAW_ADDR,
                                                                ADC_RAW_ADDR_ADC_I1_RAW_ADC_I0_RAW,
                                                                TMC4671_ADC_I0_RAW_MASK, TMC4671_ADC_I0_RAW_SHIFT);

        const uint16_t i1_raw = tmc4671_readFieldWithDependency(idInt(), TMC4671_ADC_RAW_DATA, TMC4671_ADC_RAW_ADDR,
                                                                ADC_RAW_ADDR_ADC_I1_RAW_ADC_I0_RAW,
                                                                TMC4671_ADC_I1_RAW_MASK, TMC4671_ADC_I1_RAW_SHIFT);

        i0_filter.add(i0_raw);
        i1_filter.add(i1_raw);

        std::this_thread::sleep_for(10ms);
    }

    const uint16_t i0_filtered = i0_filter.current();
    const uint16_t i1_filtered = i1_filter.current();

    tmc4671_setAdcI0Offset(idInt(), i0_filtered);
    tmc4671_setAdcI1Offset(idInt(), i1_filtered);
}

void Motor::enableDriver(bool enable)
{
    TMC6200_FIELD_UPDATE(idInt(), TMC6200_GCONF, TMC6200_DISABLE_MASK, TMC6200_DISABLE_SHIFT, enable ? 0 : 1);
}

void Motor::setMotionMode(const MotionMode t_mode)
{
    uint32_t mode = 0;
    switch (t_mode)
    {
    case MotionMode::Stopped:
        mode = TMC4671_MOTION_MODE_STOPPED;
        break;
    case MotionMode::Velocity:
        mode = TMC4671_MOTION_MODE_VELOCITY;
        break;
    case MotionMode::Position:
        mode = TMC4671_MOTION_MODE_POSITION;
        break;
    case MotionMode::Unknown:
        Common::logWarning("Unknown motion mode");
    }

    tmc4671_switchToMotionMode(idInt(), mode);
}

Motor::MotionMode Motor::getMotionMode() const
{
    const uint32_t mode =
        TMC4671_FIELD_READ(idInt(), TMC4671_MODE_RAMP_MODE_MOTION, TMC4671_MODE_MOTION_MASK, TMC4671_MODE_MOTION_SHIFT);

    switch (mode)
    {
    case TMC4671_MOTION_MODE_STOPPED:
        return MotionMode::Stopped;
    case TMC4671_MOTION_MODE_VELOCITY:
        return MotionMode::Velocity;
    case TMC4671_MOTION_MODE_POSITION:
        return MotionMode::Position;
    default:
        return MotionMode::Unknown;
    }
}

void Motor::setTargetVelocityRaw(const int t_velocity)
{
    tmc4671_setTargetVelocity(idInt(), t_velocity);
}

int Motor::getTargetVelocityRaw() const
{
    return tmc4671_getTargetVelocity(idInt());
}

int Motor::getActualVelocityRaw() const
{
    return tmc4671_getActualVelocity(idInt());
}
} // namespace Immortals::Daemon
