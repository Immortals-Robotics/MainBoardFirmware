#include "fifo.h"

#include <assert.h>
#include <cstring>

#if ICM42688_FEATURE_FIFO
namespace ICM42688
{
/* configures and enables the FIFO buffer  */
int Fifo::enableFifo(bool t_accel, bool t_gyro, bool t_temp)
{
    if (writeRegister(Register::UB0_REG_FIFO_CONFIG1,
                      (t_accel * FIFO_ACCEL) | (t_gyro * FIFO_GYRO) | (t_temp * FIFO_TEMP_EN)) < 0)
    {
        return -2;
    }
    m_en_fifo_accel   = t_accel;
    m_en_fifo_gyro    = t_gyro;
    m_en_fifo_temp    = t_temp;
    m_fifo_frame_size = t_accel * 6 + t_gyro * 6 + t_temp * 2;
    return 1;
}

/* reads data from the ICM42688 FIFO and stores in buffer */
int Fifo::readFifo()
{
    // get the fifo size
    readRegisters(Register::UB0_REG_FIFO_COUNTH, 2, m_buffer);
    m_fifo_size = (((uint16_t) (m_buffer[0] & 0x0F)) << 8) + (((uint16_t) m_buffer[1]));
    // read and parse the buffer
    for (size_t i = 0; i < m_fifo_size / m_fifo_frame_size; i++)
    {
        // grab the data from the ICM42688
        if (readRegisters(Register::UB0_REG_FIFO_DATA, m_fifo_frame_size, m_buffer) < 0)
        {
            return -1;
        }
        if (m_en_fifo_accel)
        {
            // combine into 16 bit values
            int16_t raw_meas[3];
            raw_meas[0] = (((int16_t) m_buffer[0]) << 8) | m_buffer[1];
            raw_meas[1] = (((int16_t) m_buffer[2]) << 8) | m_buffer[3];
            raw_meas[2] = (((int16_t) m_buffer[4]) << 8) | m_buffer[5];
            // transform and convert to float values
            m_ax_fifo[i] = ((raw_meas[0] * m_accel_scale) - m_acc_b[0]) * m_acc_s[0];
            m_ay_fifo[i] = ((raw_meas[1] * m_accel_scale) - m_acc_b[1]) * m_acc_s[1];
            m_az_fifo[i] = ((raw_meas[2] * m_accel_scale) - m_acc_b[2]) * m_acc_s[2];
            m_a_size     = m_fifo_size / m_fifo_frame_size;
        }
        if (m_en_fifo_temp)
        {
            // combine into 16 bit values
            int16_t raw_meas = (((int16_t) m_buffer[0 + m_en_fifo_accel * 6]) << 8) | m_buffer[1 + m_en_fifo_accel * 6];
            // transform and convert to float values
            m_t_fifo[i] = (static_cast<float>(raw_meas) / kTempDataRegScale) + kTempOffset;
            m_t_size    = m_fifo_size / m_fifo_frame_size;
        }
        if (m_en_fifo_gyro)
        {
            // combine into 16 bit values
            int16_t raw_meas[3];
            raw_meas[0] = (((int16_t) m_buffer[0 + m_en_fifo_accel * 6 + m_en_fifo_temp * 2]) << 8) |
                         m_buffer[1 + m_en_fifo_accel * 6 + m_en_fifo_temp * 2];
            raw_meas[1] = (((int16_t) m_buffer[2 + m_en_fifo_accel * 6 + m_en_fifo_temp * 2]) << 8) |
                         m_buffer[3 + m_en_fifo_accel * 6 + m_en_fifo_temp * 2];
            raw_meas[2] = (((int16_t) m_buffer[4 + m_en_fifo_accel * 6 + m_en_fifo_temp * 2]) << 8) |
                         m_buffer[5 + m_en_fifo_accel * 6 + m_en_fifo_temp * 2];
            // transform and convert to float values
            m_gx_fifo[i] = (raw_meas[0] * m_gyro_scale) - m_gyr_b[0];
            m_gy_fifo[i] = (raw_meas[1] * m_gyro_scale) - m_gyr_b[1];
            m_gz_fifo[i] = (raw_meas[2] * m_gyro_scale) - m_gyr_b[2];
            m_g_size     = m_fifo_size / m_fifo_frame_size;
        }
    }
    return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void Fifo::getFifoAccelXMss(size_t *t_size, float *t_data)
{
    *t_size = m_a_size;
    memcpy(t_data, m_ax_fifo, m_a_size * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void Fifo::getFifoAccelYMss(size_t *t_size, float *t_data)
{
    *t_size = m_a_size;
    memcpy(t_data, m_ay_fifo, m_a_size * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void Fifo::getFifoAccelZMss(size_t *t_size, float *t_data)
{
    *t_size = m_a_size;
    memcpy(t_data, m_az_fifo, m_a_size * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, dps */
void Fifo::getFifoGyroX(size_t *t_size, float *t_data)
{
    *t_size = m_g_size;
    memcpy(t_data, m_gx_fifo, m_g_size * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, dps */
void Fifo::getFifoGyroY(size_t *t_size, float *t_data)
{
    *t_size = m_g_size;
    memcpy(t_data, m_gy_fifo, m_g_size * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, dps */
void Fifo::getFifoGyroZ(size_t *t_size, float *t_data)
{
    *t_size = m_g_size;
    memcpy(t_data, m_gz_fifo, m_g_size * sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void Fifo::getFifoTemperatureC(size_t *t_size, float *t_data)
{
    *t_size = m_t_size;
    memcpy(t_data, m_t_fifo, m_t_size * sizeof(float));
}
} // namespace ICM42688
#endif
