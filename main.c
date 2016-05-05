#include "devices.h"
#include <drv_ioport.h>
#include <drv_pwmx.h>
#include <drv_pwm8.h>
#include <math.h>
#include <stdlib.h>

#include "nrf.h"
#include "gyro.h"

//#define FLASH_PROM

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
extern ioport_t * servo;
ioport_t * debug;

pwmx_t* m0;
pwmx_t* m1;
pwmx_t* m2;
pwmx_t* m3;
pwmx_t* md;

pwm8_t* buzzer;

unsigned char robotNum=1;

uint8_t batteryV;

float des[4];
float desW;

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

robotCMD recievedCMD;

bool push[4];
bool dip[8];

unsigned char BatteryLevel;

SPid plantPID[4];
SPid anglePID;
float gyroD = 0.296;
short cycleCounter=0;

#define anglePredictSteps 124
float angleHistory[anglePredictSteps];
int angleHistoryIndex = 0;
float anglePredict = 0;

unsigned char PID_Vals[4]; // P : I : IMAX : TORQUE

float max_w_acc = 0.73f;
float max_w_dec = 1.074f;

unsigned char payload[11];

int needAck = 0;
float pwm[4];

gyroData gdata;

void constructAck ( )
{
    //payload[0] = 74; //Vbat * 5
    payload[0] = gdata.buff[2];

    payload[1] = ioport_get_value ( motorvel , 0 ) * 2;
    payload[2] = ioport_get_value ( motorvel , 1 ) * 2;
    payload[3] = ioport_get_value ( motorvel , 2 ) * 2;
    payload[4] = ioport_get_value ( motorvel , 3 ) * 2;

    payload[5] = ioport_get_value(debug , 0)>>8;
    if ( ioport_get_value(debug , 0)&1 )
       payload[5] |= 1;
    payload[6] = ioport_get_value ( motordir , 0 );
    payload[6] |= ( ioport_get_value ( debug , 0 ) & 0xF0 ) >> 4;

    //payload[5] = gdata.buff[0];
    //payload[6] = gdata.buff[1];

    //payload[7] = 0;
    payload[7] = gdata.buff[3];

    payload[8] = (ioport_get_value ( debug , 0 )&2)>>1;

    payload[9] = abs(recievedCMD.angle);
    if ( recievedCMD.angle < 0 )
       payload[0] |= 128;

    /*payload[0] = fabs ( pwm[0]/4.0) ;
    payload[5] = fabs ( pwm[1]/4.0) ;
    payload[6] = fabs ( des[0]/4.0) ;
    payload[7] = fabs ( des[1]/4.0) ;
    payload[8] = fabs ( des[2]/4.0) ;
    payload[9] = fabs ( des[3]/4.0) ;*/


}

void sendAck()
{
    if ( needAck == 6 )
    {
        constructAck();
        needAck --;
    }
    else if ( needAck == 5 )
    {
        nrf24l01_set_as_tx();
        needAck --;
    }
    else if ( needAck == 4 )
    {
        nrf24l01_write_tx_payload(payload,10,false);
        needAck --;
    }
    else if ( needAck == 3 )
    {
        nrf24l01_transmit();
        needAck --;
    }
    else if ( needAck == 2 )
    {
        if ( nrf24l01_irq_pin_active() )
        {
            if ( nrf24l01_irq_tx_ds_active() )
            {
                needAck --;
                //needAck = 1;
                nrf24l01_irq_clear_tx_ds();
                nrf24l01_flush_tx();
            }
        }

        //needAck --;
    }
    else if ( needAck == 1 )
    {
        //nrf24l01_flush_tx();
        nrf24l01_set_as_rx();
        needAck --;
    }
    else
    {
        needAck --;
    }
}

float omniMatrix[4][3];

void recieveMatrix( int n )
{
    for ( int i = 0 ; i < 3 ; i ++ )
    {
           omniMatrix[n][i] =  65536 * (int)(payload[i*3]&63);
           omniMatrix[n][i] += 256   * (int)(payload[i*3+1]);
           omniMatrix[n][i] +=         (int)(payload[i*3+2]);
           omniMatrix[n][i] /= 4194303.0;

           if ( payload[i*3]&64 )
              omniMatrix[n][i] += 1.0;
           if ( payload[i*3]&128 )
              omniMatrix[n][i] = -omniMatrix[n][i];
    }

}

void recievePID()
{
    float newKp = payload[1];
    newKp /= 5.0;

    float newKi = payload[2];
    newKi /= 1000.0;
    newKi += 0.2;

    float newMaxI = payload[3];
    newMaxI *= 4.0;

    update_PID_Vals(&plantPID[0],newKp,newKi,0.0,newMaxI);
    update_PID_Vals(&plantPID[1],newKp,newKi,0.0,newMaxI);
    update_PID_Vals(&plantPID[2],newKp,newKi,0.0,newMaxI);
    update_PID_Vals(&plantPID[3],newKp,newKi,0.0,newMaxI);

    newKp = payload[4];
    newKp /= 10.0;

    newKi = payload[5];
    newKi /= 1000.0;
    newKi += 0.2;

    newMaxI = payload[6];
    newMaxI *= 4.0;

    update_PID_Vals(&anglePID,newKp,newKi,0.0,newMaxI);

    float newKd = payload[7];
    newKd /= 500.0;
    gyroD = newKd;

    GYRO_OFFSET = payload[8];
    GYRO_OFFSET += 256*((0x7F)&payload[9]);
    if ( (0x80)&payload[9] != 0 )
       GYRO_OFFSET = -GYRO_OFFSET;
    GYRO_OFFSET /= 2500.0;
}

void recivePacket()
{
    robotCMD tmpCMD;

    tmpCMD.vx = payload[1];
    tmpCMD.vy = payload[2];
    tmpCMD.angle = payload[3];
    tmpCMD.targetAngle = payload[4];

    if ( payload[9]&16 )
       tmpCMD.vx = -tmpCMD.vx;
    if ( payload[9]&32 )
       tmpCMD.vy = -tmpCMD.vy;
    if ( payload[9]&64 )
       tmpCMD.angle = -tmpCMD.angle;
    if ( payload[9]&128 )
       tmpCMD.targetAngle = -tmpCMD.targetAngle;

    tmpCMD.tW = (float)(payload[5])/25.0f;

    tmpCMD.direct = payload[6];
    tmpCMD.chip = payload[7];

    tmpCMD.buzzer = payload[8];

    tmpCMD.booster = (payload[9]&8)>0;
    tmpCMD.discharge = (payload[9]&4)>0;
    tmpCMD.dribbler = (payload[9]&2)>0;
    tmpCMD.runPID = (payload[9]&1)>0;

    float newAngle = tmpCMD.angle;
    if ( newAngle <= 180 )
    {
        newAngle += anglePredict;
    }
    else
    {
        newAngle = recievedCMD.angle;
    }

    recievedCMD = tmpCMD;
    recievedCMD.angle = newAngle;

            float angleRad = (90.0-recievedCMD.angle) * (0.0174528);
            float coss = cos ( angleRad );
            float sinn = sin ( angleRad );

            angleRad = recievedCMD.vx * coss - recievedCMD.vy * sinn;
            recievedCMD.vy = recievedCMD.vx * sinn + recievedCMD.vy * coss;
            recievedCMD.vx = angleRad;

            des[0]=((recievedCMD.vy*0.8387)-(recievedCMD.vx*0.5446));//-(recievedCMD.w/6.66667));
            des[1]=(-(recievedCMD.vy*0.8387)-(recievedCMD.vx*0.5446));//-(recievedCMD.w/6.66667));
            des[2]=(-(recievedCMD.vy*0.707)+(recievedCMD.vx*0.707));//-(recievedCMD.w/6.66667));
            des[3]=((recievedCMD.vy*0.707)+(recievedCMD.vx*0.707));//-(recievedCMD.w/6.66667));

            des[0] *= -4.0f;
            des[1] *= -4.0f;
            des[2] *= -4.0f;
            des[3] *= -4.0f;

            ioport_set_value(kick,0,recievedCMD.direct);
            ioport_set_value(kick,1,recievedCMD.chip);
            pwmx_set_pulsewidth( md , recievedCMD.dribbler?1023.0:0.0 );

            if ( recievedCMD.buzzer > 0 )
            {
                pwm8_set_frequency(buzzer,recievedCMD.buzzer*59);
                pwm8_set_dutycycle(buzzer,45);
            }
            else
            {
                pwm8_set_frequency(buzzer,2000);
                pwm8_set_dutycycle(buzzer,0);
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

    curr_vel[0] = 8.0 * ioport_get_value ( motorvel , 0 );
    if ( tmp_dir & 16 )
       curr_vel[0] = -curr_vel[0];

    curr_vel[1] = 8.0 * ioport_get_value ( motorvel , 1 );
    if ( tmp_dir & 32 )
       curr_vel[1] = -curr_vel[1];

    curr_vel[2] = 8.0 * ioport_get_value ( motorvel , 2 );
    if ( tmp_dir & 64 )
       curr_vel[2] = -curr_vel[2];

    curr_vel[3] = 8.0 * ioport_get_value ( motorvel , 3 );
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



inline void ControllLoop ( void )
{
    if ( recievedCMD.runPID == false )
    {
        pwmx_set_pulsewidth( m0 , 0 );
        pwmx_set_pulsewidth( m1 , 0 );
        pwmx_set_pulsewidth( m2 , 0 );
        pwmx_set_pulsewidth( m3 , 0 );
        return;
    }
    pwm[0] = ( UpdatePID( &plantPID[0] , des[0]+desW-curr_vel[0] , curr_vel[0] ) + pwm[0] ) / 2.0;
    pwm[1] = ( UpdatePID( &plantPID[1] , des[1]+desW-curr_vel[1] , curr_vel[1] ) + pwm[1] ) / 2.0;
    pwm[2] = ( UpdatePID( &plantPID[2] , des[2]+desW-curr_vel[2] , curr_vel[2] ) + pwm[2] ) / 2.0;
    pwm[3] = ( UpdatePID( &plantPID[3] , des[3]+desW-curr_vel[3] , curr_vel[3] ) + pwm[3] ) / 2.0;

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

//float angle = 0;

//int sendCounter = 0;

float oldDesW = 0;

int noCMDCnounter = -1;

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

    if ( noCMDCnounter >= 0 )
       noCMDCnounter ++;

    if ( ( needAck <= 0 ) && ( nrf24l01_irq_pin_active() ) )
    {
        if ( nrf24l01_irq_rx_dr_active() )
        {
            ioport_set_value( debug , 0 , 6 );
            nrf24l01_read_rx_payload ( payload , 10 );

            switch ( payload[0]&0x0F )
            {
                case 1:
                     recivePacket();
                     noCMDCnounter = 0;
                     break;

                case 2:
                     recievePID();
                     break;

                /*case 3:
                     recieveMatrix(0);
                     break;
                case 4:
                     recieveMatrix(1);
                     break;
                case 5:
                     recieveMatrix(2);
                     break;
                case 6:
                     recieveMatrix(3);
                     break;*/
            }

            /*switch ( (payload[0]&0xF0)/16 )
            {
                case 1:
                     needAck = 6;
                     break;
            }*/

            //recivePacket();

            if ( payload[0] > 15 )
            {
                needAck = 7;
            }

            nrf24l01_irq_clear_rx_dr();
            nrf24l01_flush_rx();
        }
    }

    if ( needAck > 0 )
    {
        ioport_set_value( debug , 0 , 6 );
        sendAck();
    }

    if ( noCMDCnounter >= 1200 )
    {
        recievedCMD.runPID = false;
        noCMDCnounter = 1200;
    }

    if ( getGyroscopeData(&gdata) )
    {

        anglePredict -= angleHistory[angleHistoryIndex];
        angleHistory[angleHistoryIndex] = gdata.x / 1285.0;
        anglePredict += angleHistory[angleHistoryIndex];

        recievedCMD.angle += angleHistory[angleHistoryIndex];
        if ( recievedCMD.angle > 180 )
           recievedCMD.angle -= 360;
        if ( recievedCMD.angle < -180 )
           recievedCMD.angle += 360;

        angleHistoryIndex = ( angleHistoryIndex + 1 ) % anglePredictSteps;

    }

      //desW = sqrt ( 21.4167f * max_w_dec * fabs( recievedCMD.targetAngle ) );
      //if ( recievedCMD.targetAngle < 0 )
      //   desW = -desW;

      desW = recievedCMD.angle-recievedCMD.targetAngle;
        if ( desW > 180 )
           desW -= 360;
        if ( desW < -180 )
           desW += 360;
        //desW /= 2.0;
        desW = UpdatePID ( &anglePID , desW , 0 );
        desW += gdata.x*gyroD; //Calculate d term here, cause we have W from gyro
        //desW = ( oldDesW + desW ) / 2.0;
        //desW = max ( -120 , min ( 120 ,desW ) );

      if ( desW * oldDesW < 0 )
      {
            float tmp = max_w_dec;
            if ( desW < 0 )
               tmp = -tmp;
            tmp += oldDesW;
            //float tmp = oldAns[state.vision_id].X + 20.0f * max_acc.X * sgn ( ans.X );

            if ( tmp * desW > 0 )
            {
                tmp = max_w_acc;
                if ( desW < 0 )
                   tmp = -tmp;
                if ( fabs ( tmp ) > fabs ( desW ) )
                    tmp = desW;
            }
            desW = tmp;
      }
      else
      {
            if ( fabs ( desW ) > fabs ( oldDesW ) + max_w_acc )
            {
                if ( desW < 0 )
                   desW = -( fabs ( oldDesW ) + max_w_acc );
                else
                    desW = ( fabs ( oldDesW ) + max_w_acc );
            }
      }
      desW = max ( -120 , min ( 120 ,desW ) );
      /*if ( fabs ( desW ) > 180 )
      {
            desW = 180.0f * sgn ( desW );
      }*/
      //desW = max ( -180 , min ( 180 ,desW ) );


      oldDesW = desW;

      //desW *= 0.6666667;

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
    recievedCMD.angle = 0.0;
    recievedCMD.booster = false;
    recievedCMD.buzzer = 0;
    recievedCMD.chip = 0;
    recievedCMD.direct = 0;
    recievedCMD.discharge = false;
    recievedCMD.dribbler = false;
    recievedCMD.runPID = true;
    recievedCMD.targetAngle = 0;
    recievedCMD.tW = 1;
    recievedCMD.vx = 0 ;
    recievedCMD.vy = 0 ;

    volatile int    tenuscs         = 0;
    interrupt_register_native( INTNUMBER, (void*)&tenuscs, interrupt_handler );
    interrupt_configure( INTNUMBER, EDGE_RISING );

    /*volatile int    gtenuscs         = 0;
    interrupt_register_native( GYROINTNUMBER, (void*)&gtenuscs, gyro_interrupt_handler );
    interrupt_configure( GYROINTNUMBER, LEVEL_HIGH );*/

    motorvel = ioport_open(DRV_MOTORVEL);
    motordir = ioport_open(DRV_MOTORDIR);
    kick = ioport_open(DRV_KICKIO);
    debug = ioport_open(DRV_DEBUGIO);
    batteryV = 0x5D;

    robotNum = getRobotNum();

    buzzer = pwm8_open(DRV_PWM8_1);
    pwm8_enable_controller(buzzer);

    if ( ioport_get_value(debug , 0) & 0x4 )
    {
        pwm8_set_frequency(buzzer,2000);
        pwm8_set_pulsewidth(buzzer,100);
        delay_ms ( 200 );
        pwm8_set_pulsewidth(buzzer,0);
    }

    ioport_set_value( motordir , 0 , 0 );

    init_spi();
    servo = ioport_open(DRV_SERVOIO);
    nrf24l01_initialize_debug(true,10,false);
    nrf24l01_rx_active_to_standby();
    nrf24l01_flush_rx();
    nrf24l01_set_rf_ch(55);
    unsigned char own_rx_add[5];
    own_rx_add[0]=110;
    own_rx_add[1]=110;
    own_rx_add[2]=robotNum;
    own_rx_add[3]=110;
    own_rx_add[4]=110;
    nrf24l01_set_rx_addr(own_rx_add,5,0);
    own_rx_add[2]=30;
    nrf24l01_set_tx_addr(own_rx_add,5,0);

    if ( nrf24l01_get_status() == 0 )
    {
        if ( ioport_get_value(debug , 0) & 0x4 )
        {
            pwm8_set_frequency(buzzer,1000);
            pwm8_set_pulsewidth(buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(buzzer,0);
        }
    }

    if ( !initGyro() )
    {
        if ( ioport_get_value(debug , 0) & 0x4 )
        {
            pwm8_set_frequency(buzzer,1000);
            pwm8_set_pulsewidth(buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(buzzer,0);
        }
    }

    if ( ioport_get_value(debug , 0) & 0x4 )
    {
        pwm8_set_frequency(buzzer,1000);
        pwm8_set_pulsewidth(buzzer,100);
        delay_ms ( 200 );
        pwm8_set_pulsewidth(buzzer,0);
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
    anglePID.dGain = 0;
    anglePID.iGain = 0.23;
    anglePID.pGain = 5;
    anglePID.iMax = 16;
    anglePID.iMin = -16;

    for ( int i = 0 ; i < anglePredictSteps ; i ++ )
        angleHistory[i] = 0;

    bool found_ch = false;
    char rf_ch = 2;

    if ( ioport_get_value(debug , 0) & 0x4 )
    {
        own_rx_add[2]=25;
        nrf24l01_set_rx_addr(own_rx_add,5,0);

        while ( !found_ch )
        {
        nrf24l01_rx_active_to_standby();
        for ( unsigned char test_ch = 125 ; ( test_ch > 0 ) && ( !found_ch ) ; test_ch -= 5 )
        {
            nrf24l01_set_rf_ch(test_ch);
            nrf24l01_rx_standby_to_active();
            delay_ms(20);
            nrf24l01_rx_active_to_standby();
            if ( nrf24l01_irq_pin_active() )
            {
               if ( nrf24l01_irq_rx_dr_active() )
               {
                found_ch = true;
                rf_ch = test_ch;
                nrf24l01_irq_clear_rx_dr();
                nrf24l01_flush_rx();
               }
            }
        }
        if ( !found_ch )
        {
            pwm8_set_frequency(buzzer,1000);
            pwm8_set_pulsewidth(buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(buzzer,0);
        }
        }

        own_rx_add[2]=robotNum;
        nrf24l01_set_rx_addr(own_rx_add,5,0);
        nrf24l01_set_rf_ch(rf_ch);
        nrf24l01_rx_standby_to_active();
        delay_ms(1);
    }

    if ( ioport_get_value(debug , 0) & 0x4 )
    {
        pwm8_set_frequency(buzzer,5000);
        pwm8_set_pulsewidth(buzzer,100);
        delay_ms ( 200 );
        pwm8_set_pulsewidth(buzzer,0);
    }

    //nrf24l01_set_rf_ch(110);
    //nrf24l01_rx_standby_to_active();

    //start         = true;
    interrupt_acknowledge(INTNUMBER);
    interrupt_enable( INTNUMBER );

    /*interrupt_acknowledge(GYROINTNUMBER);
    interrupt_enable( GYROINTNUMBER );*/
}

