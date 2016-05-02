#include <drv_i2cm.h>
#include <timing.h>
#include <time.h>

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


//#define YZ_READ

// offsets are chip specific.
int offx = 0;
#ifdef YZ_READ
int offy = 20;
int offz = 93;
#endif

i2cm_t* gyroI2C;

typedef struct
{
    uint8_t buff[TO_READ];
    float x;
    #ifdef YZ_READ
    float y;
    float z;
    #endif
    float temp;
}gyroData;

bool writeI2C ( uint8_t registerAdd , uint8_t data )
{
    int retry;
    for ( retry = 10; retry; retry-- )
    {
        if (  !i2cm_putchar( gyroI2C, true, GYRO_ADDR | I2C_WRITE ))  // Generate start and send device write address
           break;
        if ( retry ) i2cm_stop( gyroI2C );
        for ( clock_t t = clock() + CLOCKS_PER_SEC / 1000; clock() < t; ) __nop();
    }
    if (  !retry                                                // Address acknowledged
          || i2cm_putchar( gyroI2C, false, registerAdd )                        // Set address counter
          || i2cm_putchar( gyroI2C, false, data )                  // Send data
       )
    {
       i2cm_stop( gyroI2C );
       return false;
    }
    i2cm_stop( gyroI2C );
    return true;
}

//initializes the gyroscope
bool initGyro()
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

  gyroI2C = i2cm_open(DRV_I2CM_1);

  if ( gyroI2C == NULL )
  {
    return false;
  }

  if ( i2cm_get_bus(gyroI2C) != -1 )
  {
    if ( !writeI2C(PWR_MGM,0x80)
         ||!writeI2C(SMPLRT_DIV,0x0B)
         ||!writeI2C(DLPF_FS,0x18)
         ||!writeI2C(INT_CFG,0x01)
       )
    {
       i2cm_release_bus( gyroI2C );
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

    i2cm_release_bus( gyroI2C );
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


bool getGyroscopeData(gyroData * result)
{
  /**************************************
  Gyro ITG-3200 I2C
  registers:
  temp MSB = 1B, temp LSB = 1C
  x axis MSB = 1D, x axis LSB = 1E
  y axis MSB = 1F, y axis LSB = 20
  z axis MSB = 21, z axis LSB = 22
  *************************************/

  if ( i2cm_get_bus( gyroI2C ) != -1 )
  {
     if (   i2cm_putchar( gyroI2C, true, GYRO_ADDR | I2C_WRITE )   // Generate start and send device write address
            || i2cm_putchar( gyroI2C, false, 0x1B )                        // Set address counter to 27
        )
     {
        i2cm_stop( gyroI2C );
     }

     else if (i2cm_read( gyroI2C, GYRO_ADDR | I2C_READ, result->buff, TO_READ) != -1)
     {
        i2cm_release_bus( gyroI2C );
        int tmp;
        /*tmp = (result->buff[0] << 8) | result->buff[1];
        tmp = tcmplnt16 ( tmp ) + 13200;
        result->temp = (float)(tmp) / 280.0;*/

        tmp = (result->buff[2] << 8) | result->buff[3];
        tmp = tcmplnt16 ( tmp );
        result->x =  (float)(tmp) / 14.375;
        result->x += 10.07;
        #ifdef YZ_READ
        tmp = (result->buff[4] << 8) | result->buff[5];
        tmp = tcmplnt16 ( tmp );
        result->y = (float)(tmp) / 14.375;
        tmp = (result->buff[6] << 8) | result->buff[7];
        tmp = tcmplnt16 ( tmp );
        result->z = (float)(tmp) / 14.375;
        #endif
        return true;
     }
     i2cm_release_bus( gyroI2C );
  }
  return false;

  //uint8_t buff[TO_READ];

  /*i2cm_write(gyroI2C,GYRO_ADDR, &regAddress , 1 );

  i2cm_read(gyroI2C,GYRO_ADDR, result->buff,TO_READ); //read the gyro data from the ITG3200*/


}


/*//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device
  Wire.send(address);        //sends address to read from
  Wire.endTransmission(); //end transmission

  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  {
    buff[i] = Wire.receive(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}
*/
