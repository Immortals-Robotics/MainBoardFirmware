#ifndef GLOBALVARS_H
#define GLOBALVARS_H

#include "devices.h"
#include <drv_ioport.h>
#include <drv_pwmx.h>
#include <drv_pwm8.h>
#include <math.h>
#include <stdlib.h>
#include "nrf.h"
#include "gyro.h"
#include <interrupts.h>
#include "pid.h"

#ifndef max
#define max(a,b) (a>b?a:b)
#endif
#ifndef min
#define min(a,b) (a<b?a:b)
#endif


#define anglePredictSteps 124
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

extern ioport_t * servo;
ioport_t * motorvel;
ioport_t * motordir;
ioport_t * kick;
ioport_t * debug;

pwmx_t* m0;
pwmx_t* m1;
pwmx_t* m2;
pwmx_t* m3;
pwmx_t* md;
pwm8_t* buzzer;

spi_t*    flash_spi;

// Types and Structs Variablses  ///////////////////////////////////////////////

typedef struct
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
}robotCMD;

typedef struct
{
    bool led0;
    bool led1;

}outputDebugPort;

// Robot Variablses  ///////////////////////////////////////////////

unsigned char robotNum=1;
robotCMD recievedCMD;
int noCMDCnounter = -1;
unsigned char own_rx_add[5];

bool push[4];
bool dip[8];
unsigned char ledState=0;

SPid plantPID[4];
SPid anglePID;

float angleHistory[anglePredictSteps];
int angleHistoryIndex = 0;
float anglePredict = 0;

unsigned char PID_Vals[4]; // P : I : IMAX : TORQUE

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

gyroData gdata;

// Const Variablses  ///////////////////////////////////////////////

const float max_w_acc = 0.73f;
const float max_w_dec = 1.074f;
float gyroD = 0.296;

#endif

