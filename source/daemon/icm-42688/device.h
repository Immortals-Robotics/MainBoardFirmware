#pragma once
#include <cstdint>

#include "registers.h"

namespace ICM42688
{
class Device
{
public:
    enum GyroFullScale : uint8_t
    {
        dps2000   = 0x00,
        dps1000   = 0x01,
        dps500    = 0x02,
        dps250    = 0x03,
        dps125    = 0x04,
        dps62_5   = 0x05,
        dps31_25  = 0x06,
        dps15_625 = 0x07
    };

    enum class AccelFullScale : uint8_t
    {
        gpm16 = 0x00,
        gpm8  = 0x01,
        gpm4  = 0x02,
        gpm2  = 0x03
    };

    enum class OutputDataRate : uint8_t
    {
        odr32k    = 0x01, // LN mode only
        odr16k    = 0x02, // LN mode only
        odr8k     = 0x03, // LN mode only
        odr4k     = 0x04, // LN mode only
        odr2k     = 0x05, // LN mode only
        odr1k     = 0x06, // LN mode only
        odr200    = 0x07,
        odr100    = 0x08,
        odr50     = 0x09,
        odr25     = 0x0A,
        odr12_5   = 0x0B,
        odr6a25   = 0x0C, // LP mode only (accel only)
        odr3a125  = 0x0D, // LP mode only (accel only)
        odr1a5625 = 0x0E, // LP mode only (accel only)
        odr500    = 0x0F,
    };

    enum class Connection
    {
#if ICM42688_FEATURE_I2C
        I2c,
#endif
#if ICM42688_FEATURE_SPI
        Spi,
#endif
    };

    // Constructor for device with the selected id and connection method
    Device(const uint8_t t_idx, const Connection t_connection) : m_idx(t_idx), m_connection(t_connection)
    {}

    // Starts the communication with the device
    // returns < 0 if error
    int begin();

    // Sets the Full Scale (FS) range for the accelerometer
    // returns < 0 if error
    int setAccelFS(AccelFullScale t_fssel);

    // Sets the Full Scale (FS) range for the gyro
    // returns < 0 if error
    int setGyroFS(GyroFullScale t_fssel);

    // Set the Output Data Rate (ODR) for accelerometer
    // returns < 0 if error
    int setAccelODR(OutputDataRate t_odr);

    // Set the Output Data Rate (ODR) for gyro
    // returns < 0 if error
    int setGyroODR(OutputDataRate t_odr);

    int setFilters(bool t_gyro_filters, bool t_acc_filters);

    // Enables the data ready interrupt.
    // routes UI data ready interrupt to INT1
    // push-pull, pulsed, active HIGH interrupts
    // returns < 0 if error
    int enableDataReadyInterrupt();

    // Masks the data ready interrupt
    // returns < 0 if error
    int disableDataReadyInterrupt();

    // Transfers data from ICM 42688-p to microcontroller.
    // Must be called to access new measurements.
    // returns < 0 if error
    int getAGT();

    // Get accelerometer x-axis acceleration data in g's
    float accX() const;
    // Get accelerometer y-axis acceleration data in g's
    float accY() const;
    // Get accelerometer z-axis acceleration data in g's
    float accZ() const;

    // Get gyro x-axis angular velocity data in dps
    float gyrX() const;
    // Get gyro y-axis angular velocity data in dps
    float gyrY() const;
    // Get gyro z-axis angular velocity data in dps
    float gyrZ() const;

    // Get temperature of gyro die in Celsius
    float temp() const;

    // returns the accelerometer measurement in the x direction, raw 16-bit integer
    int16_t getAccelXCount() const;
    // returns the accelerometer measurement in the y direction, raw 16-bit integer
    int16_t getAccelYCount() const;
    // returns the accelerometer measurement in the z direction, raw 16-bit integer
    int16_t getAccelZCount() const;

    // returns the gyroscople measurement in the x direction, raw 16-bit integer
    int16_t getGyroXCount() const;
    // returns the gyroscople measurement in the y direction, raw 16-bit integer
    int16_t getGyroYCount() const;
    // returns the gyroscople measurement in the z direction, raw 16-bit integer
    int16_t getGyroZCount() const;

    // estimates the gyro biases
    int calibrateGyro();

    // returns the gyro bias in the X direction, dps
    float getGyroBiasX() const;
    // returns the gyro bias in the Y direction, dps
    float getGyroBiasY() const;
    // returns the gyro bias in the Z direction, dps
    float getGyroBiasZ() const;
    // sets the gyro bias in the X direction to bias, dps
    void setGyroBiasX(float t_bias);
    // sets the gyro bias in the Y direction to bias, dps
    void setGyroBiasY(float t_bias);
    // sets the gyro bias in the Z direction to bias, dps
    void setGyroBiasZ(float t_bias);

    // finds bias and scale factor calibration for the accelerometer,
    // this should be run for each axis in each direction (6 total) to find
    // the min and max values along each
    int calibrateAccel();

    // returns the accelerometer bias in the X direction, m/s/s
    float getAccelBiasXMss() const;
    // returns the accelerometer scale factor in the X direction
    float getAccelScaleFactorX() const;
    // returns the accelerometer bias in the Y direction, m/s/s
    float getAccelBiasYMss() const;
    // returns the accelerometer scale factor in the Y direction
    float getAccelScaleFactorY() const;
    // returns the accelerometer bias in the Z direction, m/s/s
    float getAccelBiasZMss() const;
    // returns the accelerometer scale factor in the Z direction
    float getAccelScaleFactorZ() const;

    // sets the accelerometer bias (m/s/s) and scale factor in the X direction
    void setAccelCalX(float t_bias, float t_scale_factor);
    // sets the accelerometer bias (m/s/s) and scale factor in the Y direction
    void setAccelCalY(float t_bias, float t_scale_factor);
    // sets the accelerometer bias (m/s/s) and scale factor in the Z direction
    void setAccelCalZ(float t_bias, float t_scale_factor);

protected:
    uint8_t    m_idx = 0;
    Connection m_connection;

#if ICM42688_FEATURE_I2C
    ///\brief I2C Communication
    size_t m_i2c_num_bytes = 0; // number of bytes received from I2C
#endif

    // temp, accel xyz, gyro xyz
    int16_t m_raw_meas[7];

    // buffer for reading from sensor
    uint8_t m_buffer[15] = {};

    // data buffer
    float m_temp   = 0.0f;
    float m_acc[3] = {};
    float m_gyr[3] = {};

    // Full scale resolution factors
    float m_accel_scale = 0.0f;
    float m_gyro_scale  = 0.0f;

    // Full scale selections
    AccelFullScale m_accel_fs;
    GyroFullScale  m_gyro_fs;

    // Accel calibration
    float m_acc_bd[3]  = {};
    float m_acc_b[3]   = {};
    float m_acc_s[3]   = {1.0f, 1.0f, 1.0f};
    float m_acc_max[3] = {};
    float m_acc_min[3] = {};

    // Gyro calibration
    float m_gyro_bd[3] = {};
    float m_gyr_b[3]   = {};

    // Constants
    static constexpr uint8_t kWhoAmI          = 0x47; ///< expected value in UB0_REG_WHO_AM_I reg
    static constexpr int     kNumCalibSamples = 1000; ///< for gyro/accel bias calib

    // Conversion formula to get temperature in Celsius (Sec 4.13)
    static constexpr float kTempDataRegScale = 132.48f;
    static constexpr float kTempOffset       = 25.0f;

    uint8_t m_bank = 0; ///< current user bank

#if ICM42688_FEATURE_FIFO
    const uint8_t FIFO_TEMP_EN = 0x04;
    const uint8_t FIFO_GYRO    = 0x02;
    const uint8_t FIFO_ACCEL   = 0x01;
#endif

    // BANK 1
    const uint8_t GYRO_NF_ENABLE   = 0x00;
    const uint8_t GYRO_NF_DISABLE  = 0x01;
    const uint8_t GYRO_AAF_ENABLE  = 0x00;
    const uint8_t GYRO_AAF_DISABLE = 0x02;

    // BANK 2
    const uint8_t ACCEL_AAF_ENABLE  = 0x00;
    const uint8_t ACCEL_AAF_DISABLE = 0x01;

    // writes a byte to ICM42688 register given a register address and data
    int writeRegister(Register t_register, uint8_t t_data);
    // reads registers from ICM42688 given a starting register address,
    // number of bytes, and a pointer to store data
    int readRegisters(Register t_register, uint8_t t_count, uint8_t *t_dest);
    int setBank(uint8_t t_bank);

    // Software reset of the device
    void reset();

    // Read the WHO_AM_I register
    uint8_t whoAmI();
};
} // namespace ICM42688

#include "device_inl.h"
