#pragma once
#include "device.h"

#if ICM42688_FEATURE_FIFO
namespace ICM42688
{
class Fifo : public Device
{
public:
    using Device::Device;

    int enableFifo(bool t_accel, bool t_gyro, bool t_temp);

    int readFifo();

    void getFifoAccelXMss(size_t *t_size, float *t_data);
    void getFifoAccelYMss(size_t *t_size, float *t_data);
    void getFifoAccelZMss(size_t *t_size, float *t_data);

    void getFifoGyroX(size_t *t_size, float *t_data);
    void getFifoGyroY(size_t *t_size, float *t_data);
    void getFifoGyroZ(size_t *t_size, float *t_data);

    void getFifoTemperatureC(size_t *t_size, float *t_data);

protected:
    // fifo
    bool m_en_fifo_accel = false;
    bool m_en_fifo_gyro  = false;
    bool m_en_fifo_temp  = false;

    size_t m_fifo_size       = 0;
    size_t m_fifo_frame_size = 0;

    float  m_ax_fifo[85] = {};
    float  m_ay_fifo[85] = {};
    float  m_az_fifo[85] = {};
    size_t m_a_size      = 0;

    float  m_gx_fifo[85] = {};
    float  m_gy_fifo[85] = {};
    float  m_gz_fifo[85] = {};
    size_t m_g_size      = 0;

    float  m_t_fifo[256] = {};
    size_t m_t_size      = 0;
};
} // namespace ICM42688
#endif
