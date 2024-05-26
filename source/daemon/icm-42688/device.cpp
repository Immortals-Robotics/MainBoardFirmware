#include "device.h"

#include <assert.h>
#include <cstring>
#include <thread>

#include "hal.h"

namespace ICM42688
{
int Device::begin()
{
    reset();

    if (whoAmI() != kWhoAmI)
    {
        return -3;
    }

    // turn on accel and gyro in Low Noise (LN) Mode
    if (writeRegister(Register::UB0_REG_PWR_MGMT0, 0x0F) < 0)
    {
        return -4;
    }

    // 16G is default -- do this to set up accel resolution scaling
    int ret = setAccelFS(AccelFullScale::gpm16);
    if (ret < 0)
        return ret;

    // 2000DPS is default -- do this to set up gyro resolution scaling
    ret = setGyroFS(dps2000);
    if (ret < 0)
        return ret;

    // // disable inner filters (Notch filter, Anti-alias filter, UI filter block)
    // if (setFilters(false, false) < 0) {
    //   return -7;
    // }

    // estimate gyro bias
    if (calibrateGyro() < 0)
    {
        return -8;
    }
    // successful init, return 1
    return 1;
}

int Device::setAccelFS(AccelFullScale t_fssel)
{
    setBank(0);

    // read current register value
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0)
        return -1;

    // only change FS_SEL in reg
    reg = ((uint8_t) t_fssel << 5) | (reg & 0x1F);

    if (writeRegister(Register::UB0_REG_ACCEL_CONFIG0, reg) < 0)
        return -2;

    m_accel_scale = static_cast<float>(1 << (4 - (uint8_t) t_fssel)) / 32768.0f;
    m_accel_fs    = t_fssel;

    return 1;
}

int Device::setGyroFS(GyroFullScale t_fssel)
{
    setBank(0);

    // read current register value
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_GYRO_CONFIG0, 1, &reg) < 0)
        return -1;

    // only change FS_SEL in reg
    reg = (t_fssel << 5) | (reg & 0x1F);

    if (writeRegister(Register::UB0_REG_GYRO_CONFIG0, reg) < 0)
        return -2;

    m_gyro_scale = (2000.0f / static_cast<float>(1 << t_fssel)) / 32768.0f;
    m_gyro_fs    = t_fssel;

    return 1;
}

int Device::setAccelODR(OutputDataRate t_odr)
{
    setBank(0);

    // read current register value
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0)
        return -1;

    // only change ODR in reg
    reg = (uint8_t) t_odr | (reg & 0xF0);

    if (writeRegister(Register::UB0_REG_ACCEL_CONFIG0, reg) < 0)
        return -2;

    return 1;
}

int Device::setGyroODR(OutputDataRate t_odr)
{
    setBank(0);

    // read current register value
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_GYRO_CONFIG0, 1, &reg) < 0)
        return -1;

    // only change ODR in reg
    reg = (uint8_t) t_odr | (reg & 0xF0);

    if (writeRegister(Register::UB0_REG_GYRO_CONFIG0, reg) < 0)
        return -2;

    return 1;
}

int Device::setFilters(bool t_gyro_filters, bool t_acc_filters)
{
    if (setBank(1) < 0)
        return -1;

    if (t_gyro_filters == true)
    {
        if (writeRegister(Register::UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_ENABLE | GYRO_AAF_ENABLE) < 0)
        {
            return -2;
        }
    }
    else
    {
        if (writeRegister(Register::UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE) < 0)
        {
            return -3;
        }
    }

    if (setBank(2) < 0)
        return -4;

    if (t_acc_filters == true)
    {
        if (writeRegister(Register::UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE) < 0)
        {
            return -5;
        }
    }
    else
    {
        if (writeRegister(Register::UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE) < 0)
        {
            return -6;
        }
    }
    if (setBank(0) < 0)
        return -7;
    return 1;
}

int Device::enableDataReadyInterrupt()
{
    // push-pull, pulsed, active HIGH interrupts
    if (writeRegister(Register::UB0_REG_INT_CONFIG, 0x18 | 0x03) < 0)
        return -1;

    // need to clear bit 4 to allow proper INT1 and INT2 operation
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_INT_CONFIG1, 1, &reg) < 0)
        return -2;
    reg &= ~0x10;
    if (writeRegister(Register::UB0_REG_INT_CONFIG1, reg) < 0)
        return -3;

    // route UI data ready interrupt to INT1
    if (writeRegister(Register::UB0_REG_INT_SOURCE0, 0x18) < 0)
        return -4;

    return 1;
}

int Device::disableDataReadyInterrupt()
{
    // set pin 4 to return to reset value
    uint8_t reg;
    if (readRegisters(Register::UB0_REG_INT_CONFIG1, 1, &reg) < 0)
        return -1;
    reg |= 0x10;
    if (writeRegister(Register::UB0_REG_INT_CONFIG1, reg) < 0)
        return -2;

    // return reg to reset value
    if (writeRegister(Register::UB0_REG_INT_SOURCE0, 0x10) < 0)
        return -3;

    return 1;
}

int Device::getAGT()
{
    // grab the data from the ICM42688
    if (readRegisters(Register::UB0_REG_TEMP_DATA1, 14, m_buffer) < 0)
        return -1;

    // combine bytes into 16 bit values
    for (size_t i = 0; i < 7; i++)
    {
        m_raw_meas[i] = ((int16_t) m_buffer[i * 2] << 8) | m_buffer[i * 2 + 1];
    }

    m_temp = (static_cast<float>(m_raw_meas[0]) / kTempDataRegScale) + kTempOffset;

    m_acc[0] = ((m_raw_meas[1] * m_accel_scale) - m_acc_b[0]) * m_acc_s[0];
    m_acc[1] = ((m_raw_meas[2] * m_accel_scale) - m_acc_b[1]) * m_acc_s[1];
    m_acc[2] = ((m_raw_meas[3] * m_accel_scale) - m_acc_b[2]) * m_acc_s[2];

    m_gyr[0] = (m_raw_meas[4] * m_gyro_scale) - m_gyr_b[0];
    m_gyr[1] = (m_raw_meas[5] * m_gyro_scale) - m_gyr_b[1];
    m_gyr[2] = (m_raw_meas[6] * m_gyro_scale) - m_gyr_b[2];

    return 1;
}

int Device::calibrateGyro()
{
    // set at a lower range (more resolution) since IMU not moving
    const GyroFullScale current_fssel = m_gyro_fs;
    if (setGyroFS(dps250) < 0)
        return -1;

    // take samples and find bias
    m_gyro_bd[0] = 0;
    m_gyro_bd[1] = 0;
    m_gyro_bd[2] = 0;
    for (size_t i = 0; i < kNumCalibSamples; i++)
    {
        getAGT();
        m_gyro_bd[0] += (gyrX() + m_gyr_b[0]) / kNumCalibSamples;
        m_gyro_bd[1] += (gyrY() + m_gyr_b[1]) / kNumCalibSamples;
        m_gyro_bd[2] += (gyrZ() + m_gyr_b[2]) / kNumCalibSamples;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    m_gyr_b[0] = m_gyro_bd[0];
    m_gyr_b[1] = m_gyro_bd[1];
    m_gyr_b[2] = m_gyro_bd[2];

    // recover the full scale setting
    if (setGyroFS(current_fssel) < 0)
        return -4;
    return 1;
}

int Device::calibrateAccel()
{
    // set at a lower range (more resolution) since IMU not moving
    const AccelFullScale current_fssel = m_accel_fs;
    if (setAccelFS(AccelFullScale::gpm2) < 0)
        return -1;

    // take samples and find min / max
    m_acc_bd[0] = 0;
    m_acc_bd[1] = 0;
    m_acc_bd[2] = 0;
    for (size_t i = 0; i < kNumCalibSamples; i++)
    {
        getAGT();
        m_acc_bd[0] += (accX() / m_acc_s[0] + m_acc_b[0]) / kNumCalibSamples;
        m_acc_bd[1] += (accY() / m_acc_s[1] + m_acc_b[1]) / kNumCalibSamples;
        m_acc_bd[2] += (accZ() / m_acc_s[2] + m_acc_b[2]) / kNumCalibSamples;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (m_acc_bd[0] > 0.9f)
    {
        m_acc_max[0] = m_acc_bd[0];
    }
    if (m_acc_bd[1] > 0.9f)
    {
        m_acc_max[1] = m_acc_bd[1];
    }
    if (m_acc_bd[2] > 0.9f)
    {
        m_acc_max[2] = m_acc_bd[2];
    }
    if (m_acc_bd[0] < -0.9f)
    {
        m_acc_min[0] = m_acc_bd[0];
    }
    if (m_acc_bd[1] < -0.9f)
    {
        m_acc_min[1] = m_acc_bd[1];
    }
    if (m_acc_bd[2] < -0.9f)
    {
        m_acc_min[2] = m_acc_bd[2];
    }

    // find bias and scale factor
    if ((abs(m_acc_min[0]) > 0.9f) && (abs(m_acc_max[0]) > 0.9f))
    {
        m_acc_b[0] = (m_acc_min[0] + m_acc_max[0]) / 2.0f;
        m_acc_s[0] = 1 / ((abs(m_acc_min[0]) + abs(m_acc_max[0])) / 2.0f);
    }
    if ((abs(m_acc_min[1]) > 0.9f) && (abs(m_acc_max[1]) > 0.9f))
    {
        m_acc_b[1] = (m_acc_min[1] + m_acc_max[1]) / 2.0f;
        m_acc_s[1] = 1 / ((abs(m_acc_min[1]) + abs(m_acc_max[1])) / 2.0f);
    }
    if ((abs(m_acc_min[2]) > 0.9f) && (abs(m_acc_max[2]) > 0.9f))
    {
        m_acc_b[2] = (m_acc_min[2] + m_acc_max[2]) / 2.0f;
        m_acc_s[2] = 1 / ((abs(m_acc_min[2]) + abs(m_acc_max[2])) / 2.0f);
    }

    // recover the full scale setting
    if (setAccelFS(current_fssel) < 0)
        return -4;
    return 1;
}

int Device::writeRegister(Register t_register, uint8_t t_data)
{
    /* write data to device */
#if ICM42688_FEATURE_SPI
    if (m_connection == Connection::Spi)
    {
        const uint8_t buffer[2] = {(uint8_t) t_register, t_data};
        icm42688SpiTransfer(m_idx, &buffer[0], nullptr, 2);
    }
#endif
#if ICM42688_FEATURE_I2C
    if (m_connection == Connection::I2c)
    {
        ewwmci(m_idx, (uint8_t) t_register); // write the register address
        icm42688I2cWrite(m_idx, t_data);               // write the data
    }
#endif

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    /* read back the register */
    readRegisters(t_register, 1, m_buffer);
    /* check the read back register against the written register */
    if (m_buffer[0] == t_data)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

int Device::readRegisters(Register t_register, uint8_t t_count, uint8_t *t_dest)
{
#if ICM42688_FEATURE_SPI
    if (m_connection == Connection::Spi)
    {
        assert(t_count < 16);

        uint8_t buffer[16] = {};
        buffer[0]          = (uint8_t) t_register | 0x80;
        icm42688SpiTransfer(m_idx, &buffer[0], &buffer[0], t_count + 1);
        memcpy(t_dest, buffer, t_count);

        return 1;
    }
#endif

#if ICM42688_FEATURE_I2C
    if (m_connection == Connection::I2c)
    {
        icm42688I2cWrite(m_idx, (uint8_t) t_register); // specify the starting register address
        m_i2c_num_bytes = t_count;
        for (uint8_t i = 0; i < t_count; i++)
        {
            t_dest[i] = icm42688I2cRead(m_idx);
        }

        return 1;
    }
#endif

    return -1;
}

int Device::setBank(uint8_t t_bank)
{
    // if we are already on this bank, bail
    if (m_bank == t_bank)
        return 1;

    m_bank = t_bank;

    return writeRegister(Register::REG_BANK_SEL, t_bank);
}

void Device::reset()
{
    setBank(0);

    writeRegister(Register::UB0_REG_DEVICE_CONFIG, 0x01);

    // wait for ICM42688 to come back up
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

uint8_t Device::whoAmI()
{
    setBank(0);

    // read the WHO AM I register
    if (readRegisters(Register::UB0_REG_WHO_AM_I, 1, m_buffer) < 0)
    {
        return std::numeric_limits<uint8_t>::max();
    }
    // return the register value
    return m_buffer[0];
}
} // namespace ICM42688
