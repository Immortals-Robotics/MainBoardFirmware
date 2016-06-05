#include "GlobalVars.h"

void construct_feedback_packet ( )
{
    /*payload[0] = gdata.buff[2];
    payload[1] = curr_vel[0];
    payload[2] = curr_vel[1];
    payload[3] = curr_vel[2];
    payload[4] = curr_vel[3];

    payload[5] = ioport_get_value(debug , 0)>>8;
    if ( ioport_get_value(debug , 0)&1 )
       payload[5] |= 1;
    payload[6] = ioport_get_value ( motordir , 0 );
    payload[6] |= ( ioport_get_value ( debug , 0 ) & 0xF0 ) >> 4;

    payload[7] = gdata.buff[3];

    payload[8] = (ioport_get_value ( debug , 0 )&2)>>1;

    payload[9] = abs(recievedCMD.angle);
    if ( recievedCMD.angle < 0 )
       payload[0] |= 128;*/
}

bool is_feedback_process_completed()
{
    return (g_robot_state.feedback_step<=0);
}

void send_feedback_process()
{
    switch(g_robot_state.feedback_step)
    {
        case 6:
             construct_feedback_packet();
             break;
        case 5:
             nrf24l01_set_as_tx();
             break;
        case 4:
             nrf24l01_write_tx_payload(g_robot_state.payload,10,false);
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
             nrf24l01_set_as_rx(true);
             break;
    }
    --g_robot_state.feedback_step;
}

void recieve_pid()
{
    /*float newKp = payload[1];
    newKp /= 5.0;

    float newKi = payload[2];
    newKi /= 1000.0;
    newKi += 0.2;

    float newMaxI = payload[3];
    newMaxI *= 4.0;

    motor_pid_config.p_gain = newKp;
    motor_pid_config.i_gain = newKi;
    motor_pid_config.i_max = newMaxI;

    newKp = payload[4];
    newKp /= 10.0;

    newKi = payload[5];
    newKi /= 1000.0;
    newKi += 0.2;

    newMaxI = payload[6];
    newMaxI *= 4.0;

    gyro_pid_config.p_gain = newKp;
    gyro_pid_config.i_gain = newKi;
    gyro_pid_config.i_max = newMaxI;

    float newKd = payload[7];
    newKd /= 500.0;
    gyroD = newKd;

    GYRO_OFFSET = payload[8];
    GYRO_OFFSET += 256*((0x7F)&payload[9]);
    if ( (0x80)&payload[9] != 0 )
       GYRO_OFFSET = -GYRO_OFFSET;
    GYRO_OFFSET /= 2500.0;*/
}

void recive_command()
{
	/*recievedCMD.vx = payload[1];
    recievedCMD.vy = payload[2];
    if ( payload[3] <= 180 )
        recievedCMD.angle = payload[3] + anglePredict;
    recievedCMD.targetAngle = payload[4];

    if ( payload[9]&16 )
       recievedCMD.vx = -recievedCMD.vx;
    if ( payload[9]&32 )
       recievedCMD.vy = -recievedCMD.vy;
    if ( payload[9]&64 )
       recievedCMD.angle = -recievedCMD.angle;
    if ( payload[9]&128 )
       recievedCMD.targetAngle = -recievedCMD.targetAngle;

    recievedCMD.tW = (float)(payload[5])/25.0f;

    recievedCMD.direct = payload[6];
    recievedCMD.chip = payload[7];

    recievedCMD.buzzer = payload[8];

    recievedCMD.booster = (payload[9]&8)>0;
    recievedCMD.discharge = (payload[9]&4)>0;
    recievedCMD.dribbler = (payload[9]&2)>0;
    recievedCMD.runPID = (payload[9]&1)>0;

    const float angleRad = (90.0-recievedCMD.angle) * (0.0174528);
    const float coss = cos ( angleRad );
    const float sinn = sin ( angleRad );

    const float local_vx = recievedCMD.vx * coss - recievedCMD.vy * sinn;
    const float local_vy = recievedCMD.vx * sinn + recievedCMD.vy * coss;
    recievedCMD.vy = local_vy;
    recievedCMD.vx = local_vx;

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
    pwmx_set_pulsewidth( motor_pwm_d , recievedCMD.dribbler?1023.0:0.0 );

    if ( recievedCMD.buzzer > 0 )
    {
        pwm8_set_frequency(buzzer,recievedCMD.buzzer*59);
        pwm8_set_dutycycle(buzzer,45);
    }
    else
    {
        pwm8_set_frequency(buzzer,2000);
        pwm8_set_dutycycle(buzzer,0);
    }*/
}

uint8_t get_swicth(const uint8_t n)
{
	const uint32_t a = ioport_get_value(g_drivers.debug , 0);

    if (n == 1)
        return (a >> 12) & 0x0F;
    else if (n == 2)
        return (a >> 8) & 0x0F;
    else
        return 0;
}

uint8_t get_robot_num()
{
    return get_swicth(1);
}

uint8_t get_button()
{
    const uint32_t a = ioport_get_value(g_drivers.debug , 0);
    return (a >> 4) & 0x0F;
}

void set_led(const uint8_t led_mask, bool state)
{
    static uint8_t ledState = 0;
    set_bit_mask_u8(&ledState, led_mask, state);
    ioport_set_value( g_drivers.debug , 0 , ledState );
}

void calculate_motor_vels ( void )
{
    const unsigned char tmp_dir = (unsigned char)ioport_get_value ( g_drivers.motordir , 0 );

    g_robot_state.curr_vel[0] = 8.0f * ioport_get_value ( g_drivers.motorvel , 0 );
    if ( tmp_dir & 16 )
       g_robot_state.curr_vel[0] = -g_robot_state.curr_vel[0];

    g_robot_state.curr_vel[1] = 8.0f * ioport_get_value ( g_drivers.motorvel , 1 );
    if ( tmp_dir & 32 )
       g_robot_state.curr_vel[1] = -g_robot_state.curr_vel[1];

    g_robot_state.curr_vel[2] = 8.0f * ioport_get_value ( g_drivers.motorvel , 2 );
    if ( tmp_dir & 64 )
       g_robot_state.curr_vel[2] = -g_robot_state.curr_vel[2];

    g_robot_state.curr_vel[3] = 8.0f * ioport_get_value ( g_drivers.motorvel , 3 );
    if ( tmp_dir & 128 )
       g_robot_state.curr_vel[3] = -g_robot_state.curr_vel[3];
}

void control_loop ( void )
{
	if ( g_robot_cmd.runPID == false )
    {
        pwmx_set_pulsewidth( g_drivers.motor_pwm[0] , 0 );
        pwmx_set_pulsewidth( g_drivers.motor_pwm[1] , 0 );
        pwmx_set_pulsewidth( g_drivers.motor_pwm[2] , 0 );
        pwmx_set_pulsewidth( g_drivers.motor_pwm[3] , 0 );
        return;
    }

    float pwm[4];
    pwm[0] = update_pid( &motor_pid[0] , g_robot_state.des[0]+g_robot_state.des_w-g_robot_state.curr_vel[0], &motor_pid_config);
    pwm[1] = update_pid( &motor_pid[1] , g_robot_state.des[1]+g_robot_state.des_w-g_robot_state.curr_vel[1], &motor_pid_config);
    pwm[2] = update_pid( &motor_pid[2] , g_robot_state.des[2]+g_robot_state.des_w-g_robot_state.curr_vel[2], &motor_pid_config);
    pwm[3] = update_pid( &motor_pid[3] , g_robot_state.des[3]+g_robot_state.des_w-g_robot_state.curr_vel[3], &motor_pid_config);

    /*pwm[0] = P ( curr_vel[0] , des[0] , main_k );
    pwm[1] = P ( curr_vel[1] , des[1] , main_k );
    pwm[2] = P ( curr_vel[2] , des[2] , main_k );
    pwm[3] = P ( curr_vel[3] , des[3] , main_k );*/

    /*pwm[0] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[0] ) );
    pwm[1] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[1] ) );
    pwm[2] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[2] ) );
    pwm[3] *= 1400.0 / ( 1400.0 - fabs ( curr_vel[3] ) );*/

    /*pwm[0] = pwm[0] + (curr_vel[0]*0.644);
    pwm[1] = pwm[1] + (curr_vel[1]*0.644);
    pwm[2] = pwm[2] + (curr_vel[2]*0.644);
    pwm[3] = pwm[3] + (curr_vel[3]*0.644);*/

    /*if ( pwm[0] > 1023 )
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
         pwm[3] = -1023;*/

    /*pwm[0] = min ( 1023 , max ( -1023 , pwm[0] ) );
    pwm[1] = min ( 1023 , max ( -1023 , pwm[1] ) );
    pwm[2] = min ( 1023 , max ( -1023 , pwm[2] ) );
    pwm[3] = min ( 1023 , max ( -1023 , pwm[3] ) );*/
    //pwm[1] = 800;
    pwmx_set_pulsewidth( g_drivers.motor_pwm[0] , min_u_short((unsigned short)abs(pwm[0]), 1023) );
    pwmx_set_pulsewidth( g_drivers.motor_pwm[1] , min_u_short((unsigned short)abs(pwm[1]), 1023) );
    pwmx_set_pulsewidth( g_drivers.motor_pwm[2] , min_u_short((unsigned short)abs(pwm[2]), 1023) );
    pwmx_set_pulsewidth( g_drivers.motor_pwm[3] , min_u_short((unsigned short)abs(pwm[3]), 1023) );

    ioport_set_value( g_drivers.motordir , 0 , sgn_01_inv_f(pwm[0]) |
                                     (sgn_01_inv_f(pwm[1])<<1) |
                                     (sgn_01_inv_f(pwm[2])<<2) |
                                     (sgn_01_inv_f(pwm[3])<<3) );
}

void check_no_command_state()
{
    if ( g_robot_state.no_cmd_counter >= 0 )
       g_robot_state.no_cmd_counter ++;

    if ( g_robot_state.no_cmd_counter >= 1200 )
    {
        g_robot_cmd.runPID = false;
        g_robot_state.no_cmd_counter = 1200;
    }
}

void nrf_process()
{
    if ( is_feedback_process_completed() && ( nrf24l01_irq_pin_active() ) )
    {
        if ( nrf24l01_irq_rx_dr_active() )
        {
            set_led(RX_LED,ON);
            nrf24l01_read_rx_payload ( g_robot_state.payload , 10 );

            switch ( g_robot_state.payload[0]&0x0F )
            {
                case 1:
                     recive_command();
                     g_robot_state.no_cmd_counter = 0;
                     break;

                case 2:
                     recieve_pid();
                     break;
            }

            if ( g_robot_state.payload[0] > 15 )
            {
                g_robot_state.feedback_step = 7;
            }

            nrf24l01_irq_clear_rx_dr();
            nrf24l01_flush_rx();
        }
    }

    if ( !is_feedback_process_completed())
    {
        set_led(RX_LED,ON);
        send_feedback_process();
    }
}

void gyro_process()
{
	static float angle_history[ANGLE_PREDICT_STEPS];
    static int16_t angle_history_index = 0;
	static float old_des_w = 0;

	/*for ( int i = 0 ; i < anglePredictSteps ; i ++ )
        angleHistory[i] = 0;*/

	if ( get_gyro_data(&g_robot_state.gyro_data) )
    {
		const float yaw = g_robot_state.gyro_data.x / 1285.0f;
        g_robot_state.angle_predict -= angle_history[angle_history_index];
        angle_history[angle_history_index] = yaw;
        g_robot_state.angle_predict += yaw;

        g_robot_cmd.angle += yaw;
        if ( g_robot_cmd.angle > 180 )
           g_robot_cmd.angle -= 360;
        if ( g_robot_cmd.angle < -180 )
           g_robot_cmd.angle += 360;

        angle_history_index = ( angle_history_index + 1 ) % angle_history_index;
    }

      float d_angle = g_robot_cmd.angle-g_robot_cmd.targetAngle;
        if ( d_angle > 180 )
           d_angle -= 360;
        if ( d_angle < -180 )
           d_angle += 360;
        g_robot_state.des_w = update_pid ( &gyro_pid , d_angle , &gyro_pid_config );
        g_robot_state.des_w += g_robot_state.gyro_data.x*gyroD; //Calculate d term here, cause we have W from gyro

      if ( g_robot_state.des_w * old_des_w < 0 )
      {
            float tmp = max_w_dec;
            if ( g_robot_state.des_w < 0 )
               tmp = -tmp;
            tmp += old_des_w;

            if ( tmp * g_robot_state.des_w > 0 )
            {
                tmp = max_w_acc;
                if ( g_robot_state.des_w < 0 )
                   tmp = -tmp;
                if ( fabs ( tmp ) > fabs ( g_robot_state.des_w ) )
                    tmp = g_robot_state.des_w;
            }
            g_robot_state.des_w = tmp;
      }
      else
      {
            if ( fabs ( g_robot_state.des_w ) > fabs ( old_des_w ) + max_w_acc )
            {
                if ( g_robot_state.des_w < 0 )
                   g_robot_state.des_w = -( fabs ( old_des_w ) + max_w_acc );
                else
                    g_robot_state.des_w = ( fabs ( old_des_w ) + max_w_acc );
            }
      }
      g_robot_state.des_w = max_float ( -120 , min_float ( 120 ,g_robot_state.des_w ) );
      old_des_w = g_robot_state.des_w;
}

void check_encoder_fault_process()
{
    //TODO : Choose the if Fault action -> never run the motors or check it again

    if(!g_robot_state.checkMotorFault)
       return;

    if(g_robot_state.isEncoderHasFault)
       return;

    g_robot_state.TimeToResetEncoderFaultValues--;
    if(g_robot_state.TimeToResetEncoderFaultValues<=0)
    {
        g_robot_state.TimeToResetEncoderFaultValues = ENCODER_FAULT_CHECK_LOOP_COUNT;
        for(int i=0;i<4;i++)
        {
            if(g_robot_state.zeroVelCount[i] > (ENCODER_FAULT_CHECK_LOOP_COUNT/2))
            {
                g_robot_state.isEncoderHasFault = true;
                g_robot_state.zeroVelCount[0]=0;
                g_robot_state.zeroVelCount[1]=0;
                g_robot_state.zeroVelCount[2]=0;
                g_robot_state.zeroVelCount[3]=0;
                pwm8_set_frequency(g_drivers.buzzer,2000);
                pwm8_set_pulsewidth(g_drivers.buzzer,100);
                return;
            }
        }
        if(g_robot_state.isEncoderHasFault)
           pwm8_set_pulsewidth(g_drivers.buzzer,0);
        g_robot_state.isEncoderHasFault = false;
        g_robot_state.zeroVelCount[0]=0;
        g_robot_state.zeroVelCount[1]=0;
        g_robot_state.zeroVelCount[2]=0;
        g_robot_state.zeroVelCount[3]=0;
        return;
    }

    /*for(int i=0;i<4;i++)
    {
        if(fabs(curr_vel[i]) <= 10 && abs(pwm[0]) > 30) //Encoder Fault //TODO: opitimize the numbers
           zeroVelCount[i]++;

        if(fabs(pwm[i] - curr_vel[i]) > 900 ) //Motor Fault //TODO: opitimize the numbers
           zeroVelCount[i]++;
    }*/
}

__INTERRUPT_NATIVE void interrupt_handler(void)
{
    set_led(LOOP_LED,ON);
    calculate_motor_vels();
    control_loop();
    nrf_process();
    check_no_command_state();
    gyro_process();
    check_encoder_fault_process();
    set_led(ALL_LED,OFF);
    interrupt_acknowledge(LOOP_INT_NUMBER);
}

void init_default_values()
{
    g_robot_state.des[0]=0;
    g_robot_state.des[1]=0;
    g_robot_state.des[2]=0;
    g_robot_state.des[3]=0;
    g_robot_state.des_w = 0;

	g_robot_state.no_cmd_counter = -1;
    g_robot_state.angle_predict = 0;
    g_robot_state.feedback_step = 0;


    g_robot_cmd.angle = 0.0;
    g_robot_cmd.booster = false;
    g_robot_cmd.buzzer = 0;
    g_robot_cmd.chip = 0;
    g_robot_cmd.direct = 0;
    g_robot_cmd.discharge = false;
    g_robot_cmd.dribbler = false;
    g_robot_cmd.runPID = true;
    g_robot_cmd.targetAngle = 0;
    g_robot_cmd.tW = 1;
    g_robot_cmd.vx = 0 ;
    g_robot_cmd.vy = 0 ;

    g_robot_state.checkMotorFault = false;
	g_robot_state.isEncoderHasFault = false;
	g_robot_state.TimeToResetEncoderFaultValues = ENCODER_FAULT_CHECK_LOOP_COUNT;

    for(int i=0;i<4;i++)
    {
        g_robot_state.zeroVelCount[i]=0;
    }

}

void init_prepherals()
{
    volatile int    tenuscs         = 0;
    interrupt_register_native( LOOP_INT_NUMBER, (void*)&tenuscs, interrupt_handler );
    interrupt_configure( LOOP_INT_NUMBER, EDGE_RISING );
    g_drivers.motorvel = ioport_open(DRV_MOTORVEL);
    g_drivers.motordir = ioport_open(DRV_MOTORDIR);
    g_drivers.kick = ioport_open(DRV_KICKIO);
    g_drivers.debug = ioport_open(DRV_DEBUGIO);
    g_robot_config.robot_num = get_robot_num();
    g_drivers.buzzer = pwm8_open(DRV_PWM8_1);
    pwm8_enable_controller(g_drivers.buzzer);

    if ( ioport_get_value(g_drivers.debug , 0) & 0x4 )
    {
        pwm8_set_frequency(g_drivers.buzzer,2000);
        pwm8_set_pulsewidth(g_drivers.buzzer,100);
        delay_ms ( 200 );
        pwm8_set_pulsewidth(g_drivers.buzzer,0);
    }

    ioport_set_value( g_drivers.motordir , 0 , 0 );
    init_spi();
    g_drivers.servo = ioport_open(DRV_SERVOIO);
    flash_init(g_drivers.flash_spi);
}

void init_nrf()
{
    nrf24l01_initialize_debug(true,10,false);
    nrf24l01_rx_active_to_standby();
    nrf24l01_flush_rx();
    nrf24l01_set_rf_ch(55);
    g_robot_config.own_rx_add[0]=110;
    g_robot_config.own_rx_add[1]=110;
    g_robot_config.own_rx_add[2]=g_robot_config.robot_num;
    g_robot_config.own_rx_add[3]=110;
    g_robot_config.own_rx_add[4]=110;
    nrf24l01_set_rx_addr(g_robot_config.own_rx_add,5,0);
    g_robot_config.own_rx_add[2]=30;
    nrf24l01_set_tx_addr(g_robot_config.own_rx_add,5);

    if ( nrf24l01_get_status() == 0 )
    {
        if ( ioport_get_value(g_drivers.debug , 0) & 0x4 )
        {
            pwm8_set_frequency(g_drivers.buzzer,1000);
            pwm8_set_pulsewidth(g_drivers.buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(g_drivers.buzzer,0);
        }
    }
}

void init_gyro_m()
{
    if ( !init_gyro() )
    {
        if ( ioport_get_value(g_drivers.debug , 0) & 0x4 )
        {
            pwm8_set_frequency(g_drivers.buzzer,1000);
            pwm8_set_pulsewidth(g_drivers.buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(g_drivers.buzzer,0);
        }
    }
    if ( ioport_get_value(g_drivers.debug , 0) & 0x4 )
    {
        pwm8_set_frequency(g_drivers.buzzer,1000);
        pwm8_set_pulsewidth(g_drivers.buzzer,100);
        delay_ms ( 200 );
        pwm8_set_pulsewidth(g_drivers.buzzer,0);
    }
}

void init_pid()
{
    g_drivers.motor_pwm[0] = pwmx_open(DRV_PWMX_1);
    g_drivers.motor_pwm[1] = pwmx_open(DRV_PWMX_2);
    g_drivers.motor_pwm[2] = pwmx_open(DRV_PWMX_3);
    g_drivers.motor_pwm[3] = pwmx_open(DRV_PWMX_4);
    g_drivers.motor_d_pwm = pwmx_open(DRV_PWMX_D);

    pwmx_enable_controller( g_drivers.motor_pwm[0] );
    pwmx_enable_controller( g_drivers.motor_pwm[1] );
    pwmx_enable_controller( g_drivers.motor_pwm[2] );
    pwmx_enable_controller( g_drivers.motor_pwm[3] );
    pwmx_enable_controller( g_drivers.motor_d_pwm );


    init_pid_state( &motor_pid[0] );
    init_pid_state( &motor_pid[1] );
    init_pid_state( &motor_pid[2] );
    init_pid_state( &motor_pid[3] );

    init_pid_state( &gyro_pid );

    motor_pid_config.d_gain = 0.0f;
    motor_pid_config.i_gain = 0.23f;
    motor_pid_config.p_gain = 25.0f;
    motor_pid_config.i_max = 328.0f;

    gyro_pid_config.d_gain = 0;
    gyro_pid_config.i_gain = 0.23;
    gyro_pid_config.p_gain = 5;
    gyro_pid_config.i_max = 16;


}

void nrf_channel_search(bool blocking)
{
    bool found_ch = false;
    char rf_ch = 2;
    int notFoundCounter=0;

    if ( ioport_get_value(g_drivers.debug , 0) & 0x4 )
    {
        g_robot_config.own_rx_add[2]=25;
        nrf24l01_set_rx_addr(g_robot_config.own_rx_add,5,0);

        while ( !found_ch )
        {
            notFoundCounter++;
            if(!blocking && notFoundCounter > 10)
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
                pwm8_set_frequency(g_drivers.buzzer,1000);
                pwm8_set_pulsewidth(g_drivers.buzzer,100);
                delay_ms ( 200 );
                pwm8_set_pulsewidth(g_drivers.buzzer,0);
            }
        }

        g_robot_config.own_rx_add[2]=g_robot_config.robot_num;
        nrf24l01_set_rx_addr(g_robot_config.own_rx_add,5,0);
        nrf24l01_set_rf_ch(rf_ch);
        nrf24l01_rx_standby_to_active();
        delay_ms(1);
    }

    if ( ioport_get_value(g_drivers.debug , 0) & 0x4 )
    {
        pwm8_set_frequency(g_drivers.buzzer,5000);
        pwm8_set_pulsewidth(g_drivers.buzzer,100);
        delay_ms ( 200 );
        pwm8_set_pulsewidth(g_drivers.buzzer,0);
    }
}

void beep(const int on_delay , const int off_delay , const int count)
{
    pwm8_set_frequency(g_drivers.buzzer,400);
    for(int i=0;i<count;i++)
    {
         pwm8_set_pulsewidth(g_drivers.buzzer,100);
         delay_ms ( on_delay );
         pwm8_set_pulsewidth(g_drivers.buzzer,0);
         delay_ms ( off_delay );
    }
}

void gyro_calibration_process()
{
    char data[6]={0};
    g_robot_state.TimeToEnterGyroCalibration = WAIT_FOR_GYRO_CALIBRATION_BUTTON;
    while(g_robot_state.TimeToEnterGyroCalibration > 0)
    {
        if(get_bit_u8(get_button(),0)== 0)
        {
            pwm8_set_frequency(g_drivers.buzzer,1000);
            pwm8_set_pulsewidth(g_drivers.buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(g_drivers.buzzer,0);
            delay_ms ( 200 );
            pwm8_set_frequency(g_drivers.buzzer,800);
            pwm8_set_pulsewidth(g_drivers.buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(g_drivers.buzzer,0);
            delay_ms ( 200 );
            pwm8_set_frequency(g_drivers.buzzer,1200);
            pwm8_set_pulsewidth(g_drivers.buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(g_drivers.buzzer,0);
            g_robot_state.TimeToEnterGyroCalibration = 0;
            g_robot_cmd.runPID = false;
            delay_ms(500);
            float gyro_offset_tmp = 0;
            for(int i=0;i<100;i++)
            {
                get_gyro_data(&g_robot_state.gyro_data);
                gyro_offset_tmp += g_robot_state.gyro_data.x/100.0;
                delay_ms(50);
            }
            GYRO_OFFSET = -(gyro_offset_tmp);
            if(GYRO_OFFSET < 0)
              data[0]=1;
            data[1] = (int)(fabs(GYRO_OFFSET*1000.0))/100;
            data[2] = (int)(fabs(GYRO_OFFSET*1000.0))%100;
            flash_sector_erase(g_drivers.flash_spi,1,true);
            flash_write(g_drivers.flash_spi,1,data,3,true);
            g_robot_state.TimeToEnterGyroCalibration=0;
            pwm8_set_frequency(g_drivers.buzzer,1000);
            pwm8_set_pulsewidth(g_drivers.buzzer,100);
            delay_ms ( 200 );
            pwm8_set_pulsewidth(g_drivers.buzzer,0);
            g_robot_state.isEncoderHasFault = false;
            g_robot_state.TimeToResetEncoderFaultValues = ENCODER_FAULT_CHECK_LOOP_COUNT;
            g_robot_state.zeroVelCount[0]=0;
            g_robot_state.zeroVelCount[1]=0;
            g_robot_state.zeroVelCount[2]=0;
            g_robot_state.zeroVelCount[3]=0;
			g_robot_state.des[0]=0;
    		g_robot_state.des[1]=0;
    		g_robot_state.des[2]=0;
    		g_robot_state.des[3]=0;
    		g_robot_state.des_w = 0;
            g_robot_cmd.angle=0;
            g_robot_cmd.targetAngle=0;
            g_robot_cmd.runPID = true;
            break;
        }
        delay_ms(1);
        g_robot_state.TimeToEnterGyroCalibration--;
    }
}

void get_gyro_offset_from_flash()
{
    unsigned char data[6]={255};
    float res;
    flash_read(g_drivers.flash_spi,1,data,5);
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
    init_default_values();
    init_prepherals();
    init_nrf();
    init_gyro_m();
    init_pid();
    if(g_robot_config.robot_num != 15)
    {
       nrf_channel_search(false);
    }

    if(g_robot_config.robot_num == 15)
    {
        gyro_calibration_process();
    }
    get_gyro_offset_from_flash();
    beep(100,500,(int)GYRO_OFFSET);
    interrupt_acknowledge(LOOP_INT_NUMBER);
    interrupt_enable( LOOP_INT_NUMBER );


}

