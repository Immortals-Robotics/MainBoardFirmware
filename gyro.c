#include "gyro.h"

#include <drv_i2cm.h>
#include <timing.h>
#include <time.h>

#include "robot_data_types.h"

uint8_t GYRO_ADDR = 0xD2; // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define I2C_READ    0x01
#define I2C_WRITE   0x00
#define SMPLRT_DIV 0x15
#define DLPF_FS 0x16
#define INT_CFG 0x17
#define PWR_MGM 0x3E

#ifdef YZ_READ
#define TO_READ 8 // 2 bytes for each axis x, y, z
#else
#define TO_READ 4 // 2 bytes for each axis x, y, z
#endif

extern struct drivers_t g_drivers;
extern struct robot_config_t g_robot_config;

static bool _write_i2c ( uint8_t registerAdd , uint8_t data )
{
    int retry;
    for ( retry = 10; retry; retry-- )
    {
        if (  !i2cm_putchar( g_drivers.gyro_i2c, true, GYRO_ADDR | I2C_WRITE ))  // Generate start and send device write address
           break;
        if ( retry ) i2cm_stop( g_drivers.gyro_i2c );
        for ( clock_t t = clock() + CLOCKS_PER_SEC / 1000; clock() < t; ) __nop();
    }
    if (  !retry                                                // Address acknowledged
          || i2cm_putchar( g_drivers.gyro_i2c, false, registerAdd )                        // Set address counter
          || i2cm_putchar( g_drivers.gyro_i2c, false, data )                  // Send data
       )
    {
       i2cm_stop( g_drivers.gyro_i2c );
       return false;
    }
    i2cm_stop( g_drivers.gyro_i2c );
    return true;
}

//initializes the gyroscope
bool init_gyro(void)
{
  /*****************************************
  * ITG 3200
  * power management set to:
  * clock select = internal oscillator
  *     no reset, no sleep mode
  *   no standby mode
  * sample rate to = 125Hz
  * parameter to +/- 2000 degrees/sec
  * low pass filter = 5Hz
  * no interrupt
  ******************************************/

  if ( i2cm_get_bus(g_drivers.gyro_i2c) != -1 )
  {
    if ( !_write_i2c(PWR_MGM,0x80)
         ||!_write_i2c(SMPLRT_DIV,0x0B)
         ||!_write_i2c(DLPF_FS,0x18)
         ||!_write_i2c(INT_CFG,0x01)
       )
    {
       i2cm_release_bus( g_drivers.gyro_i2c );
       return false;
    }


    /*uint8_t gyroConfigData[2];

    gyroConfigData[0] = PWR_MGM;
    gyroConfigData[1] = 0x00;
    i2cm_write(gyroI2C,GYRO_ADDR|I2C_WRITE,gyroConfigData,2);

    gyroConfigData[0] = SMPLRT_DIV;
    gyroConfigData[1] = 0x07; // EB, 50, 80, 7F, DE, 23, 20, FF
    i2cm_write(gyroI2C,GYRO_ADDR|I2C_WRITE,gyroConfigData,2);

    gyroConfigData[0] = DLPF_FS;
    gyroConfigData[1] = 0x1E; // +/- 2000 dgrs/sec, 1KHz, 1E, 19
    i2cm_write(gyroI2C,GYRO_ADDR|I2C_WRITE,gyroConfigData,2);

    gyroConfigData[0] = INT_CFG;
    gyroConfigData[1] = 0x00;
    i2cm_write(gyroI2C,GYRO_ADDR|I2C_WRITE,gyroConfigData,2);*/

    i2cm_release_bus( g_drivers.gyro_i2c );
    return true;
  }
  return false;
}

inline int tcmplnt16 ( int in )
{
    if ( in & 0x8000 )
    {
        in = - ( ( ~in + 1 ) & 0xFFFF );
    }
    return in;
}


bool get_gyro_data(struct gyro_data_t* const result)
{
  /**************************************
  Gyro ITG-3200 I2C
  registers:
  temp MSB = 1B, temp LSB = 1C
  x axis MSB = 1D, x axis LSB = 1E
  y axis MSB = 1F, y axis LSB = 20
  z axis MSB = 21, z axis LSB = 22
  *************************************/

  if ( i2cm_get_bus( g_drivers.gyro_i2c ) == -1 )
    return false;

  if (i2cm_putchar( g_drivers.gyro_i2c, true, GYRO_ADDR | I2C_WRITE )   // Generate start and send device write address
         || i2cm_putchar( g_drivers.gyro_i2c, false, 0x1B )                        // Set address counter to 27
     )
  {
     i2cm_stop( g_drivers.gyro_i2c );
     return false;
  }

  uint8_t buffer[TO_READ];
  if (i2cm_read( g_drivers.gyro_i2c, GYRO_ADDR | I2C_READ, buffer, TO_READ) != -1)
  {
     i2cm_release_bus( g_drivers.gyro_i2c );
     int tmp;
     /*tmp = (result->buff[0] << 8) | result->buff[1];
     tmp = tcmplnt16 ( tmp ) + 13200;
     result->temp = (float)(tmp) / 280.0;*/

     tmp = (buffer[2] << 8) | buffer[3];
     tmp = tcmplnt16 ( tmp );
     result->x =  (float)(tmp) / 14.375;
     result->x += g_robot_config.gyro_offset;
     #ifdef YZ_READ
     tmp = (buffer[4] << 8) | buffer[5];
     tmp = tcmplnt16 ( tmp );
     result->y = (float)(tmp) / 14.375;
     tmp = (buffer[6] << 8) | buffer[7];
     tmp = tcmplnt16 ( tmp );
     result->z = (float)(tmp) / 14.375;
     #endif
     return true;
  }
  else
  {
     i2cm_release_bus( g_drivers.gyro_i2c );
     return false;
  }
}

