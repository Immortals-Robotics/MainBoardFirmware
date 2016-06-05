#ifndef GLOBALVARS_H
#define GLOBALVARS_H

#include <math.h>
#include <stdlib.h>

#include <drv_ioport.h>
#include <drv_pwmx.h>
#include <drv_pwm8.h>
#include <interrupts.h>

#include "devices.h"

#include "nrf24l01.h"
#include "gyro.h"
#include "pid.h"
#include "helpers.h"

#define ANGLE_PREDICT_STEPS 124
#define LOOP_INT_NUMBER    1
#define GYRO_INT_NUMBER    11

#define LOOP_LED 2
#define RX_LED   4
#define ALL_LED  LOOP_LED + RX_LED
#define ON  1
#define OFF 0

#define ENCODER_FAULT_CHECK_TIME 3 //(second)
#define ENCODER_FAULT_CHECK_LOOP_COUNT ENCODER_FAULT_CHECK_TIME*1280

#define WAIT_FOR_GYRO_CALIBRATION_BUTTON  5000;

// Software Platform Variablses  ///////////////////////////////////////////////

struct drivers_t
{
    ioport_t * servo;
    ioport_t * motorvel;
    ioport_t * motordir;
    ioport_t * kick;
    ioport_t * debug;

    pwmx_t* motor_pwm[4];
    pwmx_t* motor_d_pwm;
    pwm8_t* buzzer;

    spi_t*    flash_spi;
};

struct drivers_t g_drivers;



// Types and Structs Variablses  ///////////////////////////////////////////////

struct robot_cmd_t
{
    float targetAngle;
    float angle;
    float tW;
    float vx;
    float vy;
    unsigned char direct;
    unsigned char chip;
    unsigned char buzzer;
    bool dribbler;
    bool discharge;
    bool booster;
    bool runPID;
};

struct robot_cmd_t g_robot_cmd;

// Robot Variablses  ///////////////////////////////////////////////

struct robot_config_t
{
	uint8_t robot_num;
    uint8_t own_rx_add[5];
};

struct robot_config_t g_robot_config;

struct robot_state_t
{
	int16_t no_cmd_counter;
	float angle_predict;

    uint8_t payload[11];

    int16_t feedback_step;
    float curr_vel[4];
    float des[4];
    float des_w;

    int zeroVelCount[4];
    int TimeToResetEncoderFaultValues;
    bool isEncoderHasFault;
    bool checkMotorFault;

    int TimeToEnterGyroCalibration;

    struct gyro_data_t gyro_data;
};

struct robot_state_t g_robot_state;

/*uint8_t robotNum = 1;
int16_t no_cmd_counter = -1;
uint8_t own_rx_add[5];

float angle_history[ANGLE_PREDICT_STEPS];
int8_t angle_history_index = 0;
float angle_predict = 0;

unsigned char payload[11];

int ackStep = 0;
float pwm[4];
float curr_vel[4];
float des[4];
float desW;
float oldDesW = 0;

int zeroVelCount[4];
int TimeToResetEncoderFaultValues;
bool isEncoderHasFault;
bool checkMotorFault;

int TimeToEnterGyroCalibration;

gyroData gdata;*/

// Const Variablses  ///////////////////////////////////////////////

const float max_w_acc = 0.73f;
const float max_w_dec = 1.074f;
float gyroD = 0.296;

#endif

