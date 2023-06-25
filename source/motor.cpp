#include "motor.h"

extern "C"
{
#include "tmc/ic/TMC4671/TMC4671.h"
#include "tmc/ic/TMC6200/TMC6200.h"
}

namespace Immortals
{
Motor::Motor(const uint8_t t_id)
    : m_id(t_id)
{
}

bool Motor::init()
{
    if (!initController())
    {
        return false;
    }

    if (!initDriver())
    {
        return false;
    }

    return true;
}

bool Motor::initController()
{
    // check the device
    tmc4671_writeInt(m_id, TMC4671_CHIPINFO_ADDR, 0);
    const int result = tmc4671_readInt(m_id, TMC4671_CHIPINFO_DATA);
    if (result != 0x34363731) // ASCII "4671"
    {
        return false;
    }

    // Motor type &  PWM configuration
    tmc4671_setMotorType(m_id, TMC4671_THREE_PHASE_BLDC);
    tmc4671_setPolePairs(m_id, 8); // 8 for main motors, 1 for the dribbler
    tmc4671_writeInt(m_id, TMC4671_PWM_POLARITIES, 0x00000000);
    tmc4671_writeInt(m_id, TMC4671_PWM_MAXCNT, 0x00000F9F);
    tmc4671_writeInt(m_id, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
    tmc4671_writeInt(m_id, TMC4671_PWM_SV_CHOP, 0x00000007);

    // ADC configuration
    tmc4671_writeInt(m_id, TMC4671_ADC_I_SELECT, 0x18000100);
    tmc4671_writeInt(m_id, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
    tmc4671_writeInt(m_id, TMC4671_dsADC_MCLK_A, 0x20000000);
    tmc4671_writeInt(m_id, TMC4671_dsADC_MCLK_B, 0x20000000);
    tmc4671_writeInt(m_id, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
    tmc4671_writeInt(m_id, TMC4671_ADC_I0_SCALE_OFFSET, 0xFF007EF5);
    tmc4671_writeInt(m_id, TMC4671_ADC_I1_SCALE_OFFSET, 0xFF007F85);

    // Digital hall settings
    tmc4671_writeInt(m_id, TMC4671_HALL_MODE, 0x00001000);
    tmc4671_writeInt(m_id, TMC4671_HALL_PHI_E_PHI_M_OFFSET, 0x55550000);

    // ABN encoder settings
    tmc4671_writeInt(m_id, TMC4671_ABN_DECODER_MODE, 0x00001000);
    tmc4671_writeInt(m_id, TMC4671_ABN_DECODER_PPR, 0x00004000);
    tmc4671_writeInt(m_id, TMC4671_ABN_DECODER_COUNT, 0x00002246);
    tmc4671_writeInt(m_id, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0xA9E00000);

    // Limits
    // TODO: read from the config file
    tmc4671_writeInt(m_id, TMC4671_PIDOUT_UQ_UD_LIMITS, 0x00005A81);
    tmc4671_writeInt(m_id, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000007D0);

    // PI settings
    // TODO: read from the config file
    tmc4671_setTorqueFluxPI(m_id, 550, 11000);
    tmc4671_setVelocityPI(m_id, 200, 15);
    tmc4671_setPositionPI(m_id, 25, 2);

#if 0
    // Init encoder (mode 0)
    tmc4671_writeInt(m_id, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
    tmc4671_writeInt(m_id, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
    tmc4671_writeInt(m_id, TMC4671_PHI_E_SELECTION, 0x00000001);
    tmc4671_writeInt(m_id, TMC4671_PHI_E_EXT, 0x00000000);
    tmc4671_writeInt(m_id, TMC4671_UQ_UD_EXT, 0x00000FA0);
    time_sleep(1);
    tmc4671_writeInt(m_id, TMC4671_ABN_DECODER_COUNT, 0x00000000);
#endif

    // Selectors
    // TODO: read from the config file
    tmc4671_writeInt(m_id, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_HALL);
    tmc4671_writeInt(m_id, TMC4671_VELOCITY_SELECTION, TMC4671_VELOCITY_PHI_E_ABN);
    tmc4671_writeInt(m_id, TMC4671_POSITION_SELECTION, TMC4671_POSITION_PHI_E_ABN);
    tmc4671_writeInt(m_id, TMC4671_ADC_I_SELECT, 0x18000100);

    tmc4671_switchToMotionMode(m_id, TMC4671_MOTION_MODE_STOPPED);

    return true;
}

bool Motor::initDriver()
{
    const int version = TMC6200_FIELD_READ(m_id, TMC6200_IOIN_OUTPUT, TMC6200_VERSION_MASK, TMC6200_VERSION_SHIFT);
    if (version != 0x10)
    {
        return false;
    }

    int gstat = tmc6200_readInt(m_id, TMC6200_GSTAT);

    TMC6200_FIELD_UPDATE(m_id, TMC6200_DRV_CONF, TMC6200_DRVSTRENGTH_MASK, TMC6200_DRVSTRENGTH_SHIFT, 1);

    // set default PWM configuration for use with TMC4671
    tmc6200_writeInt(m_id, TMC6200_GCONF, 0x0);

    return true;
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
    }

    tmc4671_switchToMotionMode(m_id, mode);
}

Motor::MotionMode Motor::getMotionMode() const
{
    const uint32_t mode =
        TMC4671_FIELD_READ(m_id, TMC4671_MODE_RAMP_MODE_MOTION, TMC4671_MODE_MOTION_MASK, TMC4671_MODE_MOTION_SHIFT);

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

void Motor::setTargetVelocity(const int t_velocity)
{
    tmc4671_setTargetVelocity(m_id, t_velocity);
}

int Motor::getTargetVelocity() const
{
    return tmc4671_getTargetVelocity(m_id);
}

int Motor::getActualVelocity() const
{
    return tmc4671_getActualVelocity(m_id);
}
} // namespace Immortals
