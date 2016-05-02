#include "devices.h"
#include <drv_ioport.h>
#include <drv_pwmx.h>
#include <drv_uart8.h>
#include <drv_m25px0.h>
#include "gyro.h"
#include <math.h>
#include <stdlib.h>

#include <interrupts.h>

#include "pid.h"

#ifndef max
#define max(a,b) (a>b?a:b)
#endif
#ifndef min
#define min(a,b) (a<b?a:b)
#endif

/*inline float max ( float a , float b )
{
    if ( a > b )
       return a;
    return b;
}

inline float min ( float a , float b )
{
    if ( a < b )
       return a;
    return b;
}*/

inline short sgn ( float a )
{
    if ( a > 0 )
       return 0;
    return 1;
}

ioport_t * motorvel;
ioport_t * motordir;
ioport_t * kick;
ioport_t * servo;
ioport_t * debug;
ioport_t * adc;

pwmx_t* m0;
pwmx_t* m1;
pwmx_t* m2;
pwmx_t* m3;
pwmx_t* md;

uart8_t* serial = NULL;

m25px0_t* flash_mem;

unsigned char robotNum=1;

uint8_t batteryV;

float des[4];
float desW;

typedef struct
{

    float targetAngle;
    float vx;
    float vy;
    unsigned char Direct_Power;
    unsigned char Chip_Power;
    float Dribbler_Power;
    unsigned char Servo_State;
    unsigned char rN; // Robot Number\
    unsigned char checkSum;
    bool Force;
    bool Ready;
    bool beep_Command ;
    bool dirW;
    bool dirVx;
    bool dirVy;
    bool Debug_Mode;

}robotCMD;

typedef struct
{
    bool led0;
    bool led1;
    bool buzzer;

}outputDebugPort;

robotCMD recievedCMD;

bool push[4];
bool dip[8];

unsigned char BatteryLevel;

SPid plantPID[4];
SPid anglePID;
short cycleCounter=0;

#define anglePredictSteps 44
float angleHistory[anglePredictSteps];
int angleHistoryIndex = 0;

int updateCounter=10;
unsigned char PID_Vals[4]; // P : I : IMAX : TORQUE

float max_w_acc = 3.73f;
float max_w_dec = 13.074f;

void recivePacket()
{
     int counter=0;
     unsigned char c;
     int getNums = 0;
     //int sum=0;
     //int aa;
     //int bb;

     unsigned char buf[140];

     short maxGets = uart8_read( serial  , (buf+cycleCounter)  , uart8_receive_buf_available( serial ));
     maxGets +=cycleCounter;
     cycleCounter =0;
     robotCMD tmpCMD;
     while(getNums<maxGets)
     {
          c=buf[getNums];
          getNums ++;
          if(c==0)
          {
             counter=1;
             //sum=0;
          }
          else if(counter==1)
          {
            //sum+=c;
            if((c%16)==robotNum)
            {
             /*plantPID[0].iState = 0;
             plantPID[1].iState = 0;
             plantPID[2].iState = 0;
             plantPID[3].iState = 0;*/
             ioport_set_value( debug , 0 , 2 );   // Terun On the Wireless LED
             c=c>>4;
             tmpCMD.dirW = c%2;
             c=c>>1;
             tmpCMD.dirVx=c%2;
             c=c>>1;
             tmpCMD.dirVy=c%2;
             counter++;
            }
            else
            {
                counter = 0;
            }
          }
          else if(counter==2)
          {
             tmpCMD.targetAngle=c-1;
             //sum+=c;
             counter++;
          }
          else if(counter==3)
          {
             tmpCMD.vx=c-1;
             //sum+=c;
             counter++;
          }
          else if(counter==4)
          {
             tmpCMD.vy=c-1;
             //sum+=c;
             counter++;
          }
          else if(counter==5)
          {
             //tmpCMD.Direct_Power=c-1;
             tmpCMD.Direct_Power=0;
             if(PID_Vals[0]!=c-1 && updateCounter<10)
                updateCounter=10;
             PID_Vals[0]=c-1;  //P
             //sum+=c;
             counter++;
          }
          else if(counter==6)
          {
             //tmpCMD.Chip_Power=c-1;
             tmpCMD.Chip_Power=0;
             if(PID_Vals[1]!=c-1 && updateCounter<10)
                updateCounter=10;
             PID_Vals[1]=c-1;  //I
             //sum+=c;
             counter++;
          }
          else if(counter==7)
          {
            //sum+=c;
            tmpCMD.Servo_State = c%8;
            c=c>>3;
            tmpCMD.Debug_Mode = c%2;
            c=c>>1;
            tmpCMD.Ready = c%2;
            c=c>>1;
            tmpCMD.Force = c%2;
            c=c>>1;
            tmpCMD.beep_Command = c%2;
            counter++;
          }
          else if(counter==8)
          {
            //sum+=c;
            //tmpCMD.Dribbler_Power=c-1;
            tmpCMD.Dribbler_Power=0;
            if(PID_Vals[2]!=c-1 && updateCounter<10)
                updateCounter=10;
            PID_Vals[2]=c-1;
            counter++;
          }
          else if(counter==9)
          {
            //sum-=c-1;
            if(PID_Vals[3]!=c-1 && updateCounter<10)
                updateCounter=10;
            PID_Vals[3]=c-1;
            counter++;

            if(updateCounter>0)
               updateCounter--;


            if(tmpCMD.Servo_State!=6)
               updateCounter=10;

            if(updateCounter==1)
            {
               float tmp[4];
               tmp[0]=PID_Vals[0]/10.0 + 5;
               tmp[1]=PID_Vals[1]/500.0;
               tmp[2]=PID_Vals[2]*4;
               tmp[3]=PID_Vals[3];
               //update_PID_Vals(&plantPID[0],tmp[0],tmp[1],0,tmp[2]);
               //update_PID_Vals(&plantPID[1],tmp[0],tmp[1],0,tmp[2]);
               //update_PID_Vals(&plantPID[2],tmp[0],tmp[1],0,tmp[2]);
               //update_PID_Vals(&plantPID[3],tmp[0],tmp[1],0,tmp[2]);
               update_PID_Vals(&anglePID,tmp[0],tmp[1],tmp[3],tmp[2]);
               int i = 0;
               for ( int j = 0 ; j < 260*500 ; j ++ )
               {
                if ( i < 250 )
                   ioport_set_value( debug , 0 , 1 );
                else
                    ioport_set_value( debug , 0 , 0 );

                    i = (i+1)%100;
               }

            }

                    recievedCMD = tmpCMD;

                    if(recievedCMD.dirW)
                       recievedCMD.targetAngle = -recievedCMD.targetAngle;

                    float predictingAngle = 0;
                    for ( int i = 0 ; i < anglePredictSteps ; i ++ )
                        predictingAngle += angleHistory[i];
                    recievedCMD.targetAngle += predictingAngle;
                    if ( recievedCMD.targetAngle > 180 )
                        recievedCMD.targetAngle -= 360;
                    if ( recievedCMD.targetAngle < -180 )
                        recievedCMD.targetAngle += 360;

                    if(recievedCMD.dirVx)
                       recievedCMD.vx = -recievedCMD.vx;

                    if(recievedCMD.dirVy)
                       recievedCMD.vy = -recievedCMD.vy;

                    predictingAngle *= -0.0174533;
                    float coss = cos ( predictingAngle );
                    float sinn = sin ( predictingAngle );

                    predictingAngle = recievedCMD.vx * coss - recievedCMD.vy * sinn;
                    recievedCMD.vy = recievedCMD.vx * sinn + recievedCMD.vy * coss;
                    recievedCMD.vx = predictingAngle;

                    des[0]=((recievedCMD.vy*0.8)-(recievedCMD.vx*0.6));//-(recievedCMD.w/6.66667));
                    des[1]=(-(recievedCMD.vy*0.8)-(recievedCMD.vx*0.6));//-(recievedCMD.w/6.66667));
                    des[2]=(-(recievedCMD.vy*0.707)+(recievedCMD.vx*0.707));//-(recievedCMD.w/6.66667));
                    des[3]=((recievedCMD.vy*0.707)+(recievedCMD.vx*0.707));//-(recievedCMD.w/6.66667));
                    /*des[0]=((recievedCMD.vy*0.8)-(recievedCMD.vx*0.6)-(recievedCMD.w/12.0));
                    des[1]=(-(recievedCMD.vy*0.8)-(recievedCMD.vx*0.6)-(recievedCMD.w/12.0));
                    des[2]=(-(recievedCMD.vy*0.707)+(recievedCMD.vx*0.707)-(recievedCMD.w/12.0));
                    des[3]=((recievedCMD.vy*0.707)+(recievedCMD.vx*0.707)-(recievedCMD.w/12.0));*/

                    des[0] *= -4.0f;
                    des[1] *= -4.0f;
                    des[2] *= -4.0f;
                    des[3] *= -4.0f;

                    ioport_set_value(kick,0,recievedCMD.Direct_Power);
                    ioport_set_value(kick,1,(recievedCMD.Ready<<7)|recievedCMD.Chip_Power);
                    pwmx_set_pulsewidth( md , recievedCMD.Dribbler_Power*4.0f );
                    ioport_set_value( debug , 0 , 0 );

          }

          if(maxGets - getNums < 10)
          {
              if(buf[getNums]==0)
              {
                  cycleCounter = maxGets - getNums ;
                  break;
              }

          }

     }

      for(int i=0;i<cycleCounter;i++)
      {
             buf[i]=buf[getNums+i];
      }
}

int getDipState(int n)
{
    unsigned int a =0;
    a = ioport_get_value(debug , 0);

    if(n==1)
    {
        unsigned char c = a>>12;
        return c;
    }
    else if(n==2)
    {
        a>>8;
        a = a%16;
        return a;
    }
    return 0;
}

int getRobotNum()
{
    unsigned char a =  getDipState(1);
    return  a;

}


float curr_vel[4];

/*inline float medianFilter ( float * data )
{
    if ( ( data[0] <= max ( data[1] , data[2] ) ) && ( data[0] >= min ( data[1] , data[2] ) ) ) return data[0];
    if ( ( data[1] <= max ( data[0] , data[2] ) ) && ( data[1] >= min ( data[0] , data[2] ) ) ) return data[1];
    if ( ( data[2] <= max ( data[1] , data[0] ) ) && ( data[2] >= min ( data[1] , data[0] ) ) ) return data[2];
    return data[0];
}*/

inline void CalculateVels ( void )
{
    /*static float vel_buffer[4][3]={{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
    static int index[4]={0,0,0,0};*/

    unsigned char tmp_dir = ioport_get_value ( motordir , 0 );

    curr_vel[0] = 4.0 * ioport_get_value ( motorvel , 0 );
    if ( tmp_dir & 16 )
       curr_vel[0] = -curr_vel[0];

    curr_vel[1] = 4.0 * ioport_get_value ( motorvel , 1 );
    if ( tmp_dir & 32 )
       curr_vel[1] = -curr_vel[1];

    curr_vel[2] = 4.0 * ioport_get_value ( motorvel , 2 );
    if ( tmp_dir & 64 )
       curr_vel[2] = -curr_vel[2];

    curr_vel[3] = 4.0 * ioport_get_value ( motorvel , 3 );
    if ( tmp_dir & 128 )
       curr_vel[3] = -curr_vel[3];

        /*if ( vel_buffer[i][index[i]] != curr_vel[i] )
        {
            if ( vel_buffer[i][index[i]] * curr_vel[i] < 0 )
               curr_vel[i] = 0;
            index[i] ++;
            if ( index[i] > 2 )
            index[i] = 0;
            vel_buffer[i][index[i]] = curr_vel[i];
        }

        curr_vel[i] = medianFilter ( vel_buffer[i] );*/
}

//#define main_k 20

float pwm[4];

inline void ControllLoop ( void )
{
    pwm[0] = ( UpdatePID( &plantPID[0] , des[0]+desW-curr_vel[0] , curr_vel[0] ) );//+ pwm[0] ) / 2.0;
    pwm[1] = ( UpdatePID( &plantPID[1] , des[1]+desW-curr_vel[1] , curr_vel[1] ) );//+ pwm[1] ) / 2.0;
    pwm[2] = ( UpdatePID( &plantPID[2] , des[2]+desW-curr_vel[2] , curr_vel[2] ) );//+ pwm[2] ) / 2.0;
    pwm[3] = ( UpdatePID( &plantPID[3] , des[3]+desW-curr_vel[3] , curr_vel[3] ) );//+ pwm[3] ) / 2.0;

    /*pwm[0] = P ( curr_vel[0] , des[0] , main_k );
    pwm[1] = P ( curr_vel[1] , des[1] , main_k );
    pwm[2] = P ( curr_vel[2] , des[2] , main_k );
    pwm[3] = P ( curr_vel[3] , des[3] , main_k );*/

    pwm[0] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[0] ) );
    pwm[1] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[1] ) );
    pwm[2] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[2] ) );
    pwm[3] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[3] ) );

    /*pwm[0] = pwm[0] + (curr_vel[0]*0.644);
    pwm[1] = pwm[1] + (curr_vel[1]*0.644);
    pwm[2] = pwm[2] + (curr_vel[2]*0.644);
    pwm[3] = pwm[3] + (curr_vel[3]*0.644);*/

    if ( pwm[0] > 1023 )
       pwm[0] = 1023;
    else if ( pwm[0] < -1023 )
         pwm[0] = -1023;

    if ( pwm[1] > 1023 )
       pwm[1] = 1023;
    else if ( pwm[1] < -1023 )
         pwm[1] = -1023;

    if ( pwm[2] > 1023 )
       pwm[2] = 1023;
    else if ( pwm[2] < -1023 )
         pwm[2] = -1023;

    if ( pwm[3] > 1023 )
       pwm[3] = 1023;
    else if ( pwm[3] < -1023 )
         pwm[3] = -1023;
    /*pwm[0] = min ( 1023 , max ( -1023 , pwm[0] ) );
    pwm[1] = min ( 1023 , max ( -1023 , pwm[1] ) );
    pwm[2] = min ( 1023 , max ( -1023 , pwm[2] ) );
    pwm[3] = min ( 1023 , max ( -1023 , pwm[3] ) );*/
    //pwm[1] = 800;
    pwmx_set_pulsewidth( m0 , abs(pwm[0]) );
    pwmx_set_pulsewidth( m1 , abs(pwm[1]) );
    pwmx_set_pulsewidth( m2 , abs(pwm[2]) );
    pwmx_set_pulsewidth( m3 , abs(pwm[3]) );

    ioport_set_value( motordir , 0 , sgn(pwm[0]) | (sgn(pwm[1])<<1) | (sgn(pwm[2])<<2) | (sgn(pwm[3])<<3) );
    //ioport_set_value( motordir , 0 , sgn(pwm[0]) + (sgn(pwm[1])*2) + (sgn(pwm[2])*4) + (sgn(pwm[3])*8) );
}

float dess = 0;

#define INTNUMBER    1
#define GYROINTNUMBER    11

gyroData gdata;

//float angle = 0;

int sendCounter = 0;

float oldDesW = 0;

__INTERRUPT_NATIVE void interrupt_handler(void)
{
    ioport_set_value( debug , 0 , 4 );
    /*int * ptr = (int*)interrupt_native_context(interrupt_get_current());
    if (start && !stop)
    {
        (*ptr)++;
    }*/

    CalculateVels();
    ControllLoop();

    //des[1] = 100;
    if ( uart8_receive_buf_available( serial ) >= 10 )
    {
       //uart8_putchar( serial , ioport_get_value ( motorvel , 1 ) );
       recivePacket();
    }

    if ( getGyroscopeData(&gdata) )
    {
        /*sendCounter = ( ++sendCounter ) % 10;
        if ( sendCounter == 0 )
        {
            batteryV = ioport_get_value ( adc , 0 );
               uart8_putchar(serial,0);
               for ( int i = 2 ; i < 4 ; i ++ )
               {
                   if ( gdata.buff[i] == 0 )
                      uart8_putchar(serial,gdata.buff[i]+1);
                   else
                      uart8_putchar(serial,gdata.buff[i]);
               }
        }*/

        if ( recievedCMD.Debug_Mode )
           if ( fabs ( recievedCMD.targetAngle ) < 0.1 )
              ioport_set_value(kick,1,(0x80)|recievedCMD.Chip_Power);

        angleHistory[angleHistoryIndex] = gdata.x / 642.5;
        angleHistoryIndex = ( angleHistoryIndex + 1 ) % anglePredictSteps;

        recievedCMD.targetAngle += gdata.x/642.5;
        if ( recievedCMD.targetAngle > 180 )
           recievedCMD.targetAngle -= 360;
        if ( recievedCMD.targetAngle < -180 )
           recievedCMD.targetAngle += 360;

      desW = sqrt ( 2.0f * max_w_dec * fabs( recievedCMD.targetAngle ) ) * sgn ( recievedCMD.targetAngle );

      if ( desW * oldDesW < 0 )
      {
            float tmp = oldDesW + max_w_dec * sgn ( desW );
            //float tmp = oldAns[state.vision_id].X + 20.0f * max_acc.X * sgn ( ans.X );

            if ( tmp * desW > 0 )
            {
                tmp = max_w_acc * sgn ( desW );
                if ( fabs ( tmp ) > fabs ( desW ) )
                    tmp = desW;
            }
            desW = tmp;
      }
      else
      {
            if ( fabs ( desW ) > fabs ( oldDesW ) + max_w_acc )
            {
                desW = ( fabs ( oldDesW ) + max_w_acc ) * sgn ( desW );
            }
      }
      if ( fabs ( desW ) > 180 )
      {
            desW = 180.0f * sgn ( desW );
      }

      oldDesW = desW;

      desW *= -0.19099;

        /*desW = recievedCMD.targetAngle;
        if ( desW > 180 )
           desW -= 360;
        if ( desW < -180 )
           desW += 360;
        desW = UpdatePID ( &anglePID , desW , 0 );
        desW += gdata.x/anglePID.dTerm; //Calculate d term here, cause we have W from gyro
        desW = max ( -120 , min ( 120 ,desW ) );*/
    }

    /*dess+=0.05;
    if ( dess >= 360 )
       dess -= 360;
    //des[1] = 800 * sin(3.14*dess/180.0);
    //des[1] = (int)(des[1]/100) * 100;*/

    ioport_set_value( debug , 0 , 0 );

    interrupt_acknowledge(INTNUMBER);
}

/*__INTERRUPT_NATIVE void gyro_interrupt_handler(void)
{
    ioport_set_value( debug , 0 , 2 );
    getGyroscopeData(&gdata);
    interrupt_acknowledge(GYROINTNUMBER);
    ioport_set_value( debug , 0 , 0 );
}*/

void main( void )
{
    des[0]=0;
    des[1]=0;
    des[2]=0;
    des[3]=0;
    desW = 0;
    recievedCMD.beep_Command = false;
    recievedCMD.Debug_Mode = false;
    recievedCMD.dirVx = false;
    recievedCMD.dirVy = false;
    recievedCMD.dirW = false;
    recievedCMD.Force = false;
    recievedCMD.Ready = false;
    recievedCMD.Servo_State = 0;
    recievedCMD.vx = 0;
    recievedCMD.vy = 0;
    recievedCMD.Dribbler_Power = 0 ;
    recievedCMD.Direct_Power = 0 ;
    recievedCMD.Chip_Power = 0 ;
    recievedCMD.targetAngle = 0;

    volatile int    tenuscs         = 0;
    interrupt_register_native( INTNUMBER, (void*)&tenuscs, interrupt_handler );
    interrupt_configure( INTNUMBER, EDGE_RISING );

    /*volatile int    gtenuscs         = 0;
    interrupt_register_native( GYROINTNUMBER, (void*)&gtenuscs, gyro_interrupt_handler );
    interrupt_configure( GYROINTNUMBER, LEVEL_HIGH );*/

    motorvel = ioport_open(DRV_MOTORVEL);
    motordir = ioport_open(DRV_MOTORDIR);
    kick = ioport_open(DRV_KICKIO);
    servo = ioport_open(DRV_SERVOIO);
    debug = ioport_open(DRV_DEBUGIO);
    adc = ioport_open(DRV_ADC);
    batteryV = 0x5D;

    robotNum = getRobotNum();

    int i = 0;

    if ( ioport_get_value(debug , 0) & 0x4 )
    {
        for ( int j = 0 ; j < 130*500 ; j ++ )
        {
            if ( i < 250 )
               ioport_set_value( debug , 0 , 1 );
            else
               ioport_set_value( debug , 0 , 0 );

               i = (i+1)%700;
        }
    }

    ioport_set_value( motordir , 0 , 0 );

    serial =
    uart8_open(DRV_UART8_1);

    flash_mem = m25px0_open(DRV_M25PX0_1);
    /*if ( serial )
    {
       uart8_set_baudrate( serial , 38400 );
       //uart8_rts( serial , 0 );
    }*/

    if ( !initGyro() )
    {
        if ( ioport_get_value(debug , 0) & 0x4 )
        {
                i = 0;
                for ( int j = 0 ; j < 130*500 ; j ++ )
                {
                    if ( i < 250 )
                       ioport_set_value( debug , 0 , 1 );
                    else
                       ioport_set_value( debug , 0 , 0 );

                       i = (i+1)%700;
                }
        }
    }

    if ( ioport_get_value(debug , 0) & 0x4 )
    {
        i = 0;
        for ( int j = 0 ; j < 130*500 ; j ++ )
        {
            if ( i < 250 )
               ioport_set_value( debug , 0 , 1 );
            else
               ioport_set_value( debug , 0 , 0 );

               i = (i+1)%500;
        }
    }

    m0 = pwmx_open(DRV_PWMX_1);
    m1 = pwmx_open(DRV_PWMX_2);
    m2 = pwmx_open(DRV_PWMX_3);
    m3 = pwmx_open(DRV_PWMX_4);
    md = pwmx_open(DRV_PWMX_D);

    /*pwmx_set_resolution_mode( m0 , PWMX_MODE_10BIT );
    pwmx_set_resolution_mode( m1 , PWMX_MODE_10BIT );
    pwmx_set_resolution_mode( m2 , PWMX_MODE_10BIT );
    pwmx_set_resolution_mode( m3 , PWMX_MODE_10BIT );
    pwmx_set_resolution_mode( md , PWMX_MODE_10BIT );

    pwmx_set_prescaler( m0 , 0 );
    pwmx_set_prescaler( m1 , 0 );
    pwmx_set_prescaler( m2 , 0 );
    pwmx_set_prescaler( m3 , 0 );
    pwmx_set_prescaler( md , 0 );*/

    pwmx_enable_controller( m0 );
    pwmx_enable_controller( m1 );
    pwmx_enable_controller( m2 );
    pwmx_enable_controller( m3 );
    pwmx_enable_controller( md );

    initSPid( &plantPID[0] );
    initSPid( &plantPID[1] );
    initSPid( &plantPID[2] );
    initSPid( &plantPID[3] );

    initSPid( &anglePID );
    anglePID.dGain = 10;
    anglePID.iGain = 0.1;
    anglePID.pGain = 10;
    anglePID.iMax = 150;
    anglePID.iMin = -150;

    for ( i = 0 ; i < anglePredictSteps ; i ++ )
        angleHistory[i] = 0;

    if ( ioport_get_value(debug , 0) & 0x4 )
    {
        i = 0;
        for ( int j = 0 ; j < 130*500 ; j ++ )
        {
            if ( i < 150 )
               ioport_set_value( debug , 0 , 1 );
            else
               ioport_set_value( debug , 0 , 0 );

               i = (i+1)%300;
        }
    }

    //start         = true;
    interrupt_acknowledge(INTNUMBER);
    interrupt_enable( INTNUMBER );

    /*interrupt_acknowledge(GYROINTNUMBER);
    interrupt_enable( GYROINTNUMBER );*/





    //while ( true )
    {
        //pwmx_set_dutycycle( m0 , (ioport_get_value( debug , 0 )/256)*100.0/256.0 );
        //if ( uart8_getchar( serial ) == -1 )
        /*if ( uart8_receive_buf_available( serial ) == 0 )
           ioport_set_value( debug , 0 , 0 );
        else
        {
            ioport_set_value( debug , 0 , ioport_get_value( debug , 0 ) / 16 );
            for ( int i = 0 ; i < 1000 ; i ++ )
                __nop();
        }*/
        //if ( uart8_receive_buf_available( serial ) >= 8 )
        {
            //uart8_putchar( serial , ioport_get_value ( motorvel , 1 ) );
            //uart8_putchar( serial , ioport_get_value ( motorvel , 1 ) );
            //uart8_putchar( serial , abs(plantPID[1].iState/2) );
            //uart8_putchar( serial , abs(plantPID[1].dState[0]-plantPID[1].dState[1]) );
            //uart8_putchar( serial , abs(plantPID[1].dt)/4 );
           //recivePacket();




        }

        //uart8_putchar( serial , 0 );

          /*  uart8_putchar( serial , 100 );
            uart8_putchar( serial , 100 );
            uart8_putchar( serial , 100 );
            uart8_putchar( serial , 100 );
            uart8_putchar( serial , 4 );
            uart8_putchar( serial , 2 );*/


        //ioport_set_value( debug , 0 , (2*sgn(curr_vel[0]))+(4*sgn(curr_vel[3])) );
        //ioport_set_value( debug , 0 , ioport_get_value( debug , 0 ) / 16 );
        //des[2]=(ioport_get_value( debug , 0 )/256)*1024.0/256.0;

        //ioport_set_value( debug , 0 , -2*uart8_putchar( serial , curr_vel[0]/4 ) );
    }
}

