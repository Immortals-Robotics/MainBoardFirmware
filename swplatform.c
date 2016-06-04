//
//	Software Platform Generated File
//	--------------------------------
//


#include "swplatform.h"

// Global variables to access Software Platform stacks
ioport_t * drv_motordir;
ioport_t * drv_servoio;
i2cm_t *   drv_i2cm_1;
ioport_t * drv_debugio;
ioport_t * drv_motorvel;
ioport_t * drv_kickio;
pwm8_t *   drv_pwm8_1;
pwmx_t *   drv_pwmx_4;
pwmx_t *   drv_pwmx_d;
spi_t *    drv_spi_2;
pwmx_t *   drv_pwmx_1;
pwmx_t *   drv_pwmx_2;
pwmx_t *   drv_pwmx_3;
m25px0_t * drv_m25px0_1;

 // Initialize all stacks in the Software Platform
void swplatform_init_stacks(void)
{
    drv_motordir = ioport_open(DRV_MOTORDIR);
    drv_servoio  = ioport_open(DRV_SERVOIO);
    drv_i2cm_1   = i2cm_open(DRV_I2CM_1);
    drv_debugio  = ioport_open(DRV_DEBUGIO);
    drv_motorvel = ioport_open(DRV_MOTORVEL);
    drv_kickio   = ioport_open(DRV_KICKIO);
    drv_pwm8_1   = pwm8_open(DRV_PWM8_1);
    drv_pwmx_4   = pwmx_open(DRV_PWMX_4);
    drv_pwmx_d   = pwmx_open(DRV_PWMX_D);
    drv_spi_2    = spi_open(DRV_SPI_2);
    drv_pwmx_1   = pwmx_open(DRV_PWMX_1);
    drv_pwmx_2   = pwmx_open(DRV_PWMX_2);
    drv_pwmx_3   = pwmx_open(DRV_PWMX_3);
    drv_m25px0_1 = m25px0_open(DRV_M25PX0_1);
}
