#include "GlobalVars.h"

inline short sgn ( float a )
{
    if ( a > 0 )
       return 0;
    return 1;
}

void setTheBit(unsigned char * var , unsigned char bitPosition , bool Value)
{
    //TODO : To be implemented as template function
}

bool getTheBit(unsigned char var , unsigned char bitPosition)
{
   unsigned char num = pow(2,bitPosition);
   return (var&num);
}

int getSign(int sign)
{
   if(sign >= 0)
     return 1;

   return -1;
}

uint32_t AbsMotorSpeed(uint8_t motorNumber)
{
    if(motorNumber > 3)
      return 0; ///TODO: Handle Error

    return ioport_get_value ( motorvel , motorNumber );
}

float MotorSpeed(uint8_t motorNumber)
{
    if(motorNumber > 3)
      return 0; ///TODO: Handle Error

    uint8_t signVar = ioport_get_value ( motordir , 0 );
    return ioport_get_value ( motorvel , motorNumber );
    //return getSign(getTheBit(signVar,motorNumber+4))*ioport_get_value ( motorvel , motorNumber );
}

void constructAckPacket ( )
{
    payload[0] = gdata.buff[2];
    payload[1] = AbsMotorSpeed(0) * 2;
    payload[2] = AbsMotorSpeed(1) * 2;
    payload[3] = AbsMotorSpeed(2) * 2;
    payload[4] = AbsMotorSpeed(3) * 2;

    payload[5] = ioport_get_value(debug , 0)>>8;
    if ( ioport_get_value(debug , 0)&1 )
       payload[5] |= 1;
    payload[6] = ioport_get_value ( motordir , 0 );
    payload[6] |= ( ioport_get_value ( debug , 0 ) & 0xF0 ) >> 4;

    payload[7] = gdata.buff[3];

    payload[8] = (ioport_get_value ( debug , 0 )&2)>>1;

    payload[9] = abs(recievedCMD.angle);
    if ( recievedCMD.angle < 0 )
       payload[0] |= 128;
}

bool isAckProcessCompleted()
{
    return (ackStep<=0);
}

void sendAckProcess()
{
    switch(ackStep)
    {
        case 6:
             constructAckPacket();
             break;
        case 5:
             nrf24l01_set_as_tx();
             break;
        case 4:
             nrf24l01_write_tx_payload(payload,10,false);
             break;
        case 3:
             nrf24l01_transmit();
             break;
        case 2:
             if ( nrf24l01_irq_pin_active() )
             {
                if ( nrf24l01_irq_tx_ds_active() )
                {
                    nrf24l01_irq_clear_tx_ds();
                    nrf24l01_flush_tx();
                }
             }
             break;
        case 1:
             nrf24l01_set_as_rx();
             break;
    }
    ackStep--;
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
    tmpCMD.vx = payload[1] * getSign(getTheBit(payload[9],4));
    tmpCMD.vy = payload[2] * getSign(getTheBit(payload[9],5));
    tmpCMD.angle = payload[3] * getSign(getTheBit(payload[9],6));
    tmpCMD.targetAngle = payload[4] * getSign(getTheBit(payload[9],6));
    tmpCMD.tW = (float)(payload[5])/25.0f;
    tmpCMD.direct = payload[6];
    tmpCMD.chip = payload[7];
    tmpCMD.buzzer = payload[8];
    tmpCMD.booster = getTheBit(payload[9],3);
    tmpCMD.discharge = getTheBit(payload[9],2);
    tmpCMD.dribbler = getTheBit(payload[9],1);
    tmpCMD.runPID = getTheBit(payload[9],0);

    if(tmpCMD.vx > 10 || tmpCMD.vy > 10)
       checkMotorFault = true;

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

int getPushState()
{
    unsigned int a =0;
    a = ioport_get_value(debug , 0);
    unsigned char c = (a>>4)%256;
    return c;
}

void SetLed(unsigned char ledNumber , bool state)
{
    if(state)
    {
       ledState |= ledNumber;
    }
    else
    {
        if(ledState & ledNumber)
           ledState -= (ledState & ledNumber);
    }

    ioport_set_value( debug , 0 , ledState );
}

/*inline void CalculateVels ( void )
{
    curr_vel[0] = 4.0 * MotorSpeed(0);
    curr_vel[1] = 4.0 * MotorSpeed(1);
    curr_vel[2] = 4.0 * MotorSpeed(2);
    curr_vel[3] = 4.0 * MotorSpeed(3);
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

inline void ControllLoop ( void )
{
    if ( recievedCMD.runPID == false || isEncoderHasFault )
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

    pwm[0] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[0] ) );
    pwm[1] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[1] ) );
    pwm[2] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[2] ) );
    pwm[3] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[3] ) );

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

    pwmx_set_pulsewidth( m0 , abs(pwm[0]) );
    pwmx_set_pulsewidth( m1 , abs(pwm[1]) );
    pwmx_set_pulsewidth( m2 , abs(pwm[2]) );
    pwmx_set_pulsewidth( m3 , abs(pwm[3]) );
    ioport_set_value( motordir , 0 , sgn(pwm[0]) | (sgn(pwm[1])<<1) | (sgn(pwm[2])<<2) | (sgn(pwm[3])<<3) );
}

void CheckNoCommandState()
{
    if ( noCMDCnounter >= 0 )
       noCMDCnounter ++;

    if ( noCMDCnounter >= 1200 )
    {
        recievedCMD.runPID = false;
        noCMDCnounter = 1200;
    }
}

void NRFProcess()
{
    if ( isAckProcessCompleted() && ( nrf24l01_irq_pin_active() ) )
    {
        if ( nrf24l01_irq_rx_dr_active() )
        {
            SetLed(RX_LED,ON);
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
            }

            if ( payload[0] > 15 )
            {
                ackStep = 7;
            }

            nrf24l01_irq_clear_rx_dr();
            nrf24l01_flush_rx();
        }
    }

    if ( !isAckProcessCompleted())
    {
        SetLed(RX_LED,ON);
        sendAckProcess();
    }
}

void GyroProcess()
{
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

      desW = recievedCMD.angle-recievedCMD.targetAngle;
        if ( desW > 180 )
           desW -= 360;
        if ( desW < -180 )
           desW += 360;
        desW = UpdatePID ( &anglePID , desW , 0 );
        desW += gdata.x*gyroD; //Calculate d term here, cause we have W from gyro

      if ( desW * oldDesW < 0 )
      {
            float tmp = max_w_dec;
            if ( desW < 0 )
               tmp = -tmp;
            tmp += oldDesW;

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
      oldDesW = desW;
}

void CheckEncoderFaultProcess()
{
    //TODO : Choose the if Fault action -> never run the motors or check it again

    if(!checkMotorFault)
       return;

    if(isEncoderHasFault)
       return;

    TimeToResetEncoderFaultValues--;
    if(TimeToResetEncoderFaultValues<=0)
    {
        TimeToResetEncoderFaultValues = ENCODER_FAULT_CHECK_LOOP_COUNT;
        for(int i=0;i<4;i++)
        {
            if(zeroVelCount[i] > (ENCODER_FAULT_CHECK_LOOP_COUNT/2))
            {
                isEncoderHasFault = true;
                zeroVelCount[0]=0;
                zeroVelCount[1]=0;
                zeroVelCount[2]=0;
                zeroVelCount[3]=0;
                pwm8_set_frequency(buzzer,2000);
                pwm8_set_pulsewidth(buzzer,100);
                return;
            }
        }
        if(isEncoderHasFault)
           pwm8_set_pulsewidth(buzzer,0);
        isEncoderHasFault = false;
        zeroVelCount[0]=0;
        zeroVelCount[1]=0;
        zeroVelCount[2]=0;
        zeroVelCount[3]=0;
        return;
    }

    for(int i=0;i<4;i++)
    {
        if(fabs(curr_vel[i]) <= 10 && abs(pwm[0]) > 30) //Encoder Fault //TODO: opitimize the numbers
           zeroVelCount[i]++;

        if(fabs(pwm[i] - curr_vel[i]) > 900 ) //Motor Fault //TODO: opitimize the numbers
           zeroVelCount[i]++;
    }
}

__INTERRUPT_NATIVE void interrupt_handler(void)
{
    SetLed(LOOP_LED,ON);
    CalculateVels();
    ControllLoop();
    NRFProcess();
    CheckNoCommandState();
    GyroProcess();
    CheckEncoderFaultProcess();
    SetLed(ALL_LED,OFF);
    interrupt_acknowledge(LOOP_INT_NUMBER);
}

void InitDefaultValues()
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
    checkMotorFault = false;

    for(int i=0;i<4;i++)
    {
        zeroVelCount[i]=0;
        isEncoderHasFault = false;
    }
    TimeToResetEncoderFaultValues = ENCODER_FAULT_CHECK_LOOP_COUNT;
}

void InitPrepherals()
{
    volatile int    tenuscs         = 0;
    interrupt_register_native( LOOP_INT_NUMBER, (void*)&tenuscs, interrupt_handler );
    interrupt_configure( LOOP_INT_NUMBER, EDGE_RISING );
    motorvel = ioport_open(DRV_MOTORVEL);
    motordir = ioport_open(DRV_MOTORDIR);
    kick = ioport_open(DRV_KICKIO);
    debug = ioport_open(DRV_DEBUGIO);
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
    flash_init(flash_spi);
}

void InitNRF()
{
    nrf24l01_initialize_debug(true,10,false);
    nrf24l01_rx_active_to_standby();
    nrf24l01_flush_rx();
    nrf24l01_set_rf_ch(55);
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
}

void InitGyro()
{
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
}

void InitPID()
{
    m0 = pwmx_open(DRV_PWMX_1);
    m1 = pwmx_open(DRV_PWMX_2);
    m2 = pwmx_open(DRV_PWMX_3);
    m3 = pwmx_open(DRV_PWMX_4);
    md = pwmx_open(DRV_PWMX_D);


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
}

void NRFChannelSearch(bool isBlocking)
{
    bool found_ch = false;
    char rf_ch = 2;
    int notFoundCounter=0;

    if ( ioport_get_value(debug , 0) & 0x4 )
    {
        own_rx_add[2]=25;
        nrf24l01_set_rx_addr(own_rx_add,5,0);

        while ( !found_ch )
        {
            notFoundCounter++;
            if(!isBlocking && notFoundCounter > 10)
            {
                return;
            }
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
}

void Beep(int OnDelay , int offDelay , int count)
{
    pwm8_set_frequency(buzzer,400);
    for(int i=0;i<count;i++)
    {
         pwm8_set_pulsewidth(buzzer,100);
         delay_ms ( OnDelay );
         pwm8_set_pulsewidth(buzzer,0);
         delay_ms ( offDelay );
    }
}

void Gyro_Calibration_Process()
{
    char data[6]={0};
    TimeToEnterGyroCalibration = WAIT_FOR_GYRO_CALIBRATION_BUTTON;
    while(TimeToEnterGyroCalibration > 0)
    {
        if(getTheBit(getPushState(),0)== 0)
        {
            pwm8_set_frequency(buzzer,1000);
            pwm8_set_pulsewidth(buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(buzzer,0);
            delay_ms ( 200 );
            pwm8_set_frequency(buzzer,800);
            pwm8_set_pulsewidth(buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(buzzer,0);
            delay_ms ( 200 );
            pwm8_set_frequency(buzzer,1200);
            pwm8_set_pulsewidth(buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(buzzer,0);
            TimeToEnterGyroCalibration = 0;
            recievedCMD.runPID = false;
            delay_ms(500);
            float gyro_offset_tmp = 0;
            for(int i=0;i<100;i++)
            {
                getGyroscopeData(&gdata);
                gyro_offset_tmp += gdata.x/100.0;
                delay_ms(50);
            }
            GYRO_OFFSET = -(gyro_offset_tmp);
            if(GYRO_OFFSET < 0)
              data[0]=1;
            data[1] = (int)(fabs(GYRO_OFFSET*1000.0))/100;
            data[2] = (int)(fabs(GYRO_OFFSET*1000.0))%100;
            flash_sector_erase(flash_spi,1,true);
            flash_write(flash_spi,1,data,3,true);
            TimeToEnterGyroCalibration=0;
            pwm8_set_frequency(buzzer,1000);
            pwm8_set_pulsewidth(buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(buzzer,0);
            isEncoderHasFault = false;
            TimeToResetEncoderFaultValues = ENCODER_FAULT_CHECK_LOOP_COUNT;
            zeroVelCount[0]=0;
            zeroVelCount[1]=0;
            zeroVelCount[2]=0;
            zeroVelCount[3]=0;
            desW=0;
            des[0]=0;
            des[1]=0;
            des[2]=0;
            des[3]=0;
            recievedCMD.angle=0;
            recievedCMD.targetAngle=0;
            for(int t=0;t<anglePredictSteps;t++)
            {
               angleHistory[t]=0;
            }
            recievedCMD.runPID = true;
            break;
        }
        delay_ms(1);
        TimeToEnterGyroCalibration--;
    }
}

void Get_Gyro_Offset_From_Flash()
{
    unsigned char data[6]={255};
    float res;
    flash_read(flash_spi,1,data,5);
    if(data[0]==255 || data[1]==255 || data[2]==255)
      return;
    res = data[1];
    res = res*100 + data[2];
    res = res / 1000.0;
    if(data[0]==1)
      res = -res;
    if(fabs(res)< 6)
       GYRO_OFFSET = res;
}

void main( void )
{

    InitDefaultValues();
    InitPrepherals();
    InitNRF();
    InitGyro();
    InitPID();
    if(robotNum != 15)
    {
       NRFChannelSearch(false);
    }

    if(robotNum == 15)
    {
        Gyro_Calibration_Process();
    }
    Get_Gyro_Offset_From_Flash();
    Beep(100,500,(int)GYRO_OFFSET);
    interrupt_acknowledge(LOOP_INT_NUMBER);
    interrupt_enable( LOOP_INT_NUMBER );


}

