#ifndef ROBOT_DATA_TYPES
#define ROBOT_DATA_TYPES

#include <drv_i2cm.h>
#include <drv_spi.h>
#include <drv_pwm8.h>
#include <drv_pwmx.h>
#include <drv_ioport.h>

#include <stdint.h>

#include "pid.h"
#include "data_lite.h"
#include "gyro.h"

struct drivers_t
{
    ioport_t*  servo_port;
    ioport_t*  motorvel_port;
    ioport_t*  motordir_port;
    ioport_t*  kick_port;
    ioport_t*  debug_port;

    pwmx_t*    motor_pwm[4];
    pwmx_t*    motor_d_pwm;
    pwm8_t*    buzzer_pwm;

    spi_t*     flash_spi;
    spi_t*     nrf_spi;

    i2cm_t* gyro_i2c;
};

struct robot_config_t
{
    uint8_t robot_num;

    struct pid_config_t motor_pid_config;
    struct pid_config_t gyro_pid_config;

    float max_w_acc;
    float max_w_dec;
    float gyro_d;
    float gyro_offset;

	bool check_motor_fault;
};

struct robot_state_t
{
    struct pid_state_t motor_pid[4];
    struct pid_state_t gyro_pid;

    float motor_desired[4];
    float motor_current[4];
    float motor_pwm[4];

	float orientation;
    float angle_predict;
    float omega_desired;
    float omega_current;

    bool encoder_has_fault;
};

#endif

