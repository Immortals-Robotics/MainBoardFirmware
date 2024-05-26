#pragma once

namespace ICM42688
{
inline float Device::accX() const
{
    return m_acc[0];
}
inline float Device::accY() const
{
    return m_acc[1];
}
inline float Device::accZ() const
{
    return m_acc[2];
}

inline float Device::gyrX() const
{
    return m_gyr[0];
}
inline float Device::gyrY() const
{
    return m_gyr[1];
}
inline float Device::gyrZ() const
{
    return m_gyr[2];
}

inline float Device::temp() const
{
    return m_temp;
}

inline float Device::getGyroBiasX() const
{
    return m_gyr_b[0];
}

inline float Device::getGyroBiasY() const
{
    return m_gyr_b[1];
}

inline float Device::getGyroBiasZ() const
{
    return m_gyr_b[2];
}

inline void Device::setGyroBiasX(const float t_bias)
{
    m_gyr_b[0] = t_bias;
}

inline void Device::setGyroBiasY(const float t_bias)
{
    m_gyr_b[1] = t_bias;
}

inline void Device::setGyroBiasZ(const float t_bias)
{
    m_gyr_b[2] = t_bias;
}

inline float Device::getAccelBiasXMss() const
{
    return m_acc_b[0];
}

inline float Device::getAccelScaleFactorX() const
{
    return m_acc_s[0];
}

inline float Device::getAccelBiasYMss() const
{
    return m_acc_b[1];
}

inline float Device::getAccelScaleFactorY() const
{
    return m_acc_s[1];
}

inline float Device::getAccelBiasZMss() const
{
    return m_acc_b[2];
}

inline float Device::getAccelScaleFactorZ() const
{
    return m_acc_s[2];
}

inline void Device::setAccelCalX(float t_bias, float t_scale_factor)
{
    m_acc_b[0] = t_bias;
    m_acc_s[0] = t_scale_factor;
}

inline void Device::setAccelCalY(float t_bias, float t_scale_factor)
{
    m_acc_b[1] = t_bias;
    m_acc_s[1] = t_scale_factor;
}

inline void Device::setAccelCalZ(float t_bias, float t_scale_factor)
{
    m_acc_b[2] = t_bias;
    m_acc_s[2] = t_scale_factor;
}

inline int16_t Device::getAccelXCount() const
{
    return m_raw_meas[1];
}

inline int16_t Device::getAccelYCount() const
{
    return m_raw_meas[2];
}

inline int16_t Device::getAccelZCount() const
{
    return m_raw_meas[3];
}

inline int16_t Device::getGyroXCount() const
{
    return m_raw_meas[4];
}

inline int16_t Device::getGyroYCount() const
{
    return m_raw_meas[5];
}

inline int16_t Device::getGyroZCount() const
{
    return m_raw_meas[6];
}
} // namespace ICM42688
