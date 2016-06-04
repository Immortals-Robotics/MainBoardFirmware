//
//	Software Platform Generated File
//	--------------------------------
//

#ifndef _SWPLATFORM_H
#define _SWPLATFORM_H

// Device ID's
#include "devices.h"

// Extra project headers
#include "ioport.h"
#include "generic_devices.h"

// Sofware Services
#include <interrupts.h>
#include <timers.h>
#include <timing.h>

// Top Level Stack Items
#include <drv_i2cm.h>
#include <drv_ioport.h>
#include <drv_m25px0.h>
#include <drv_pwm8.h>
#include <drv_pwmx.h>
#include <drv_spi.h>

// Lower Level Stack Items
#include <per_i2cm.h>
#include <per_ioport.h>
#include <per_pwm8.h>
#include <per_pwmx.h>
#include <per_spi.h>

// Global variables to access Software Platform stacks
extern ioport_t * drv_motordir;
extern ioport_t * drv_servoio;
extern i2cm_t *   drv_i2cm_1;
extern ioport_t * drv_debugio;
extern ioport_t * drv_motorvel;
extern ioport_t * drv_kickio;
extern pwm8_t *   drv_pwm8_1;
extern pwmx_t *   drv_pwmx_4;
extern pwmx_t *   drv_pwmx_d;
extern spi_t *    drv_spi_2;
extern pwmx_t *   drv_pwmx_1;
extern pwmx_t *   drv_pwmx_2;
extern pwmx_t *   drv_pwmx_3;
extern m25px0_t * drv_m25px0_1;

 // Initialize all stacks in the Software Platform
extern void swplatform_init_stacks(void);

#endif
