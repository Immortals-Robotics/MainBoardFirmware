#include <math.h>
#include <stdlib.h>

#include <drv_ioport.h>
#include <drv_pwmx.h>
#include <drv_pwm8.h>
#include <interrupts.h>
#include <timing.h>

#include "devices.h"

#include "nrf24l01.h"
#include "gyro.h"
#include "pid.h"
#include "flash.h"
#include "helpers.h"
#include "writer.h"
#include "reader.h"
#include "debug_io.h"

#define ANGLE_PREDICT_STEPS 124
#define LOOP_INT_NUMBER    1
#define GYRO_INT_NUMBER    11
#define NRF_RX_PAYLOAD_SIZE 10
#define NRF_TX_PAYLOAD_SIZE 10

#include "robot_data_types.h"

struct drivers_t g_drivers;
struct robot_command_msg_t g_robot_cmd;
struct robot_config_t g_robot_config;
struct robot_state_t g_robot_state;

void construct_feedback_packet()
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
    return g_robot_state.feedback_step <= 0;
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
             //nrf24l01_write_tx_payload(g_robot_state.payload,10,false);
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


void calculate_motor_vels ( void )
{
    const uint32_t tmp_dir = ioport_get_value(g_drivers.motordir_port, 0);

    for (uint8_t i = 0; i < 4; i++)
    {
        g_robot_state.motor_current[i] = 8.0f * ioport_get_value(g_drivers.motorvel_port, i);
        if (get_bit_u32(tmp_dir, 4 + i))
            g_robot_state.motor_current[i] = -g_robot_state.motor_current[i];
    }
}

void control_loop ( void )
{
    if ( g_robot_cmd.halt == 1 )
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            pwmx_set_pulsewidth(g_drivers.motor_pwm[i], 0);
        }
        return;
    }

    uint32_t sign_bits = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        const float error = g_robot_state.motor_desired[i] + g_robot_state.omega + g_robot_state.motor_current[i];
        g_robot_state.motor_pwm[i] = update_pid(&g_robot_state.motor_pid[i], error, &g_robot_config.motor_pid_config);
        g_robot_state.motor_pwm[i] *= 1400.0f / (1400.0f - fabs(g_robot_state.motor_current[i]));
        g_robot_state.motor_pwm[i] = min_float(1023.0f, max_float(-1023.0f, g_robot_state.motor_pwm[i]));

        pwmx_set_pulsewidth(g_drivers.motor_pwm[i], (uint16_t)abs(g_robot_state.motor_pwm[i]));
        sign_bits |= sgn_01_inv_f(g_robot_state.motor_pwm[i]) << i;
    }

    ioport_set_value(g_drivers.motordir_port, 0, sign_bits);
}

void check_no_command_state()
{
    if ( g_robot_state.no_cmd_counter >= 0 )
       g_robot_state.no_cmd_counter ++;

    if ( g_robot_state.no_cmd_counter >= 1200 )
    {
        g_robot_cmd.halt = 1;
        g_robot_state.no_cmd_counter = 1200;
    }
}

void nrf_process()
{
    if ( is_feedback_process_completed() && ( nrf24l01_irq_pin_active() ) )
    {
        if ( nrf24l01_irq_rx_dr_active() )
        {
            set_led(RX_LED);
            uint8_t payload[NRF_RX_PAYLOAD_SIZE+1];
            nrf24l01_read_rx_payload(payload, NRF_RX_PAYLOAD_SIZE);

            switch ( payload[0]&0x0F )
            {
                case 1:
                     recive_command();
                     g_robot_state.no_cmd_counter = 0;
                     break;

                case 2:
                     recieve_pid();
                     break;
            }

            if ( payload[0] > 15 )
            {
                g_robot_state.feedback_step = 7;
            }

            nrf24l01_irq_clear_rx_dr();
            nrf24l01_flush_rx();
        }
    }

    if ( !is_feedback_process_completed())
    {
        set_led(RX_LED);
        send_feedback_process();
    }
}

void gyro_process()
{
    static float angle_history[ANGLE_PREDICT_STEPS] = {0};
    static uint16_t angle_history_index = 0;
    static float old_omega = 0;

    if ( get_gyro_data(&g_robot_state.gyro_data) )
    {
        const float yaw = g_robot_state.gyro_data.x / 1285.0f;
        g_robot_state.angle_predict -= angle_history[angle_history_index];
        angle_history[angle_history_index] = yaw;
        g_robot_state.angle_predict += yaw;

        g_robot_cmd.orientation.f32 += yaw;
        if (g_robot_cmd.orientation.f32 > 180.0f)
           g_robot_cmd.orientation.f32 -= 360.0f;
        if (g_robot_cmd.orientation.f32 < -180.0f)
           g_robot_cmd.orientation.f32 += 360.0f;

        angle_history_index = (angle_history_index + 1) % ANGLE_PREDICT_STEPS;
    }

    float d_angle = g_robot_cmd.orientation.f32-g_robot_cmd.target_orientation.f32;
    if (d_angle > 180.0f)
        d_angle -= 360.0f;
    if (d_angle < -180.0f)
        d_angle += 360.0f;
    g_robot_state.omega = update_pid(&g_robot_state.gyro_pid, d_angle, &g_robot_config.gyro_pid_config);
    g_robot_state.omega += g_robot_state.gyro_data.x*g_robot_config.gyro_d; //Calculate d term here, cause we have omega from gyro

    if ( g_robot_state.omega * old_omega < 0 )
    {
        float tmp = g_robot_config.max_w_dec;
        if ( g_robot_state.omega < 0 )
           tmp = -tmp;
        tmp += old_omega;

        if ( tmp * g_robot_state.omega > 0 )
        {
            tmp = g_robot_config.max_w_acc;
            if ( g_robot_state.omega < 0 )
               tmp = -tmp;
            if ( fabs ( tmp ) > fabs ( g_robot_state.omega ) )
                tmp = g_robot_state.omega;
        }
        g_robot_state.omega = tmp;
    }
    else
    {
        if ( fabs ( g_robot_state.omega ) > fabs ( old_omega ) + g_robot_config.max_w_acc )
        {
            if ( g_robot_state.omega < 0 )
               g_robot_state.omega = -( fabs ( old_omega ) + g_robot_config.max_w_acc );
            else
                g_robot_state.omega = ( fabs ( old_omega ) + g_robot_config.max_w_acc );
        }
    }
    g_robot_state.omega = max_float ( -120 , min_float ( 120 ,g_robot_state.omega ) );
    old_omega = g_robot_state.omega;
}

void check_encoder_fault_process()
{
    #define ENCODER_FAULT_CHECK_TIME_SEC 3
    #define ENCODER_FAULT_CHECK_LOOP_COUNT (ENCODER_FAULT_CHECK_TIME_SEC*1280)

    static uint16_t ticks_to_recalculate = ENCODER_FAULT_CHECK_LOOP_COUNT;
    static uint16_t zero_vel_count[4] = {0};

    //TODO : Choose the if Fault action -> never run the motors or check it again

    if (!g_robot_state.check_motor_fault ||
        g_robot_state.encoder_has_fault)
       return;

    ticks_to_recalculate--;
    if (ticks_to_recalculate <= 0)
    {
        ticks_to_recalculate = ENCODER_FAULT_CHECK_LOOP_COUNT;
        for (uint8_t i = 0; i < 4; i++)
        {
            if (zero_vel_count[i] > (ENCODER_FAULT_CHECK_LOOP_COUNT / 2))
            {
                g_robot_state.encoder_has_fault = true;
                zero_vel_count[0]=0;
                zero_vel_count[1]=0;
                zero_vel_count[2]=0;
                zero_vel_count[3]=0;
                set_buzzer(2000);
                return;
            }
        }

        if(g_robot_state.encoder_has_fault)
           clear_buzzer();
        g_robot_state.encoder_has_fault = false;
        zero_vel_count[0]=0;
        zero_vel_count[1]=0;
        zero_vel_count[2]=0;
        zero_vel_count[3]=0;
    }
    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            if(fabs(g_robot_state.motor_current[i]) <= 10 && fabs(g_robot_state.motor_pwm[i]) > 30) //Encoder Fault //TODO: opitimize the numbers
               zero_vel_count[i]++;

            if(fabs(g_robot_state.motor_pwm[i] - g_robot_state.motor_current[i]) > 900 ) //Motor Fault //TODO: opitimize the numbers
               zero_vel_count[i]++;
        }
    }
}

__INTERRUPT_NATIVE void interrupt_handler(void)
{
    set_led(LOOP_LED);
    calculate_motor_vels();
    control_loop();
    nrf_process();
    check_no_command_state();
    gyro_process();
    check_encoder_fault_process();
    clear_led(ALL_LED);
    interrupt_acknowledge(LOOP_INT_NUMBER);
}

void init_default_values()
{
    g_robot_state.gyro_pid.i_state = 0.0f;
    g_robot_state.gyro_pid.prev_error = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        g_robot_state.motor_pid[i].i_state = 0.0f;
        g_robot_state.motor_pid[i].prev_error = 0.0f;
        g_robot_state.motor_desired[i] = 0.0f;
        g_robot_state.motor_current[i] = 0.0f;
        g_robot_state.motor_pwm[i] = 0.0f;
    }

    g_robot_state.angle_predict = 0.0f;
    g_robot_state.omega = 0.0f;
    g_robot_state.gyro_data.x = 0.0f;
    g_robot_state.no_cmd_counter = -1;
    g_robot_state.feedback_step = 0;

    g_robot_state.encoder_has_fault = false;
    g_robot_state.check_motor_fault = false;

    g_robot_config.robot_num = 0;
    g_robot_config.motor_pid_config.d_gain = 0.0f;
    g_robot_config.motor_pid_config.i_gain = 0.23f;
    g_robot_config.motor_pid_config.p_gain = 25.0f;
    g_robot_config.motor_pid_config.i_max = 328.0f;

    g_robot_config.gyro_pid_config.d_gain = 0.0f;
    g_robot_config.gyro_pid_config.i_gain = 0.23f;
    g_robot_config.gyro_pid_config.p_gain = 5.0f;
    g_robot_config.gyro_pid_config.i_max = 16.0f;

    g_robot_config.gyro_d = 0.296f;
    g_robot_config.max_w_acc = 0.73f;
    g_robot_config.max_w_dec = 1.074f;
    g_robot_config.gyro_offset = 0.0f;
}

void init_prepherals()
{
    g_drivers.servo_port    = ioport_open(DRV_SERVOIO);
    g_drivers.motorvel_port = ioport_open(DRV_MOTORVEL);
    g_drivers.motordir_port = ioport_open(DRV_MOTORDIR);
    g_drivers.kick_port     = ioport_open(DRV_KICKIO);
    g_drivers.debug_port    = ioport_open(DRV_DEBUGIO);
    g_drivers.motor_pwm[0]  = pwmx_open(DRV_PWMX_1);
    g_drivers.motor_pwm[1]  = pwmx_open(DRV_PWMX_2);
    g_drivers.motor_pwm[2]  = pwmx_open(DRV_PWMX_3);
    g_drivers.motor_pwm[3]  = pwmx_open(DRV_PWMX_4);
    g_drivers.motor_d_pwm   = pwmx_open(DRV_PWMX_D);
    g_drivers.buzzer_pwm    = pwm8_open(DRV_PWM8_1);

    g_drivers.flash_spi     = spi_open(DRV_SPI_1);
    g_drivers.nrf_spi       = spi_open(DRV_SPI_2);
    g_drivers.gyro_i2c      = i2cm_open(DRV_I2CM_1);

    pwmx_enable_controller(g_drivers.motor_pwm[0]);
    pwmx_enable_controller(g_drivers.motor_pwm[1]);
    pwmx_enable_controller(g_drivers.motor_pwm[2]);
    pwmx_enable_controller(g_drivers.motor_pwm[3]);
    pwmx_enable_controller(g_drivers.motor_d_pwm);
    pwm8_enable_controller(g_drivers.buzzer_pwm);

    if (get_fpga_delay_boot_state())
    {
        beep(2000, 200);
    }

    flash_init(g_drivers.flash_spi);

    g_robot_config.robot_num = get_robot_num();
}

void init_nrf()
{
    nrf24l01_initialize_debug(true, NRF_RX_PAYLOAD_SIZE, false);
    nrf24l01_rx_active_to_standby();
    nrf24l01_flush_rx();
    nrf24l01_set_rf_ch(55);
	uint8_t own_rx_add[5] = {110, 110, g_robot_config.robot_num, 110, 110};
    nrf24l01_set_rx_addr(own_rx_add, 5, 0);
    own_rx_add[2] = 30;
    nrf24l01_set_tx_addr(own_rx_add, 5);

    if (nrf24l01_get_status() == 0)
    {
        if (get_fpga_delay_boot_state())
        {
            beep(1000, 200);
        }
    }
}

void init_gyro_m()
{
    if (!init_gyro())
    {
        if (get_fpga_delay_boot_state())
        {
            beep(1000, 200);
        }
    }
    if (get_fpga_delay_boot_state())
    {
        beep(1000, 200);
    }
}

void nrf_channel_search(bool blocking)
{
    if (get_fpga_delay_boot_state())
    {
		bool found_ch = false;
    	uint8_t rf_ch = 2;
    	uint8_t not_found_counter = 0;

        uint8_t own_rx_add[5] = {110, 110, 25, 110, 110};
        nrf24l01_set_rx_addr(own_rx_add, 5, 0);

        while (!found_ch)
        {
            not_found_counter++;
            if (!blocking && not_found_counter > 10)
            {
                return;
            }

            nrf24l01_rx_active_to_standby();
            for (uint8_t test_ch = 125; test_ch > 0; test_ch -= 5)
            {
                nrf24l01_set_rf_ch(test_ch);
                nrf24l01_rx_standby_to_active();
                delay_ms(20);
                nrf24l01_rx_active_to_standby();
                if (nrf24l01_irq_pin_active())
                {
                   if (nrf24l01_irq_rx_dr_active())
                   {
                       found_ch = true;
                       rf_ch = test_ch;
                       nrf24l01_irq_clear_rx_dr();
                       nrf24l01_flush_rx();
                       break;
                   }
                }
            }
            if ( !found_ch )
            {
                beep(1000, 200);
            }
        }

        own_rx_add[2] = g_robot_config.robot_num;
        nrf24l01_set_rx_addr(own_rx_add, 5, 0);
        nrf24l01_set_rf_ch(rf_ch);
        nrf24l01_rx_standby_to_active();
        delay_ms(1);
    }

    if (get_fpga_delay_boot_state())
    {
        beep(5000, 200);
    }
}

void gyro_calibration_process()
{
    const uint16_t WAIT_FOR_GYRO_CALIBRATION_BUTTON_TICKS = 5000;
    uint16_t remaining_ticks = WAIT_FOR_GYRO_CALIBRATION_BUTTON_TICKS;

    while (remaining_ticks > 0)
    {
        if (get_button_bit(0) == 0)
        {
            remaining_ticks = 0;

            // wait and beep
            beep(1000, 200);
            delay_ms(200);
            beep(800, 200);
            delay_ms(200);
            beep(1200, 200);
            g_robot_cmd.halt = 1;
            delay_ms(500);

            // compute the offset
            union float_32_u_t gyro_offset_tmp;
            gyro_offset_tmp.f32 = 0.0f;
            for(uint8_t i = 0; i < 100; i++)
            {
                get_gyro_data(&g_robot_state.gyro_data);
                gyro_offset_tmp.f32 += g_robot_state.gyro_data.x / 100.0f;
                delay_ms(50);
            }
            g_robot_config.gyro_offset = -gyro_offset_tmp.f32;

            // store the offset to flash memory
            uint8_t data[4];
            size_t length = 0;
            write_float(data, &length, gyro_offset_tmp);
            flash_sector_erase(g_drivers.flash_spi, 1, true);
            flash_write(g_drivers.flash_spi, 1, data, length, true);

            beep(1000, 200);
            break;
        }
        delay_ms(1);
        remaining_ticks--;
    }
}

void get_gyro_offset_from_flash()
{
    uint8_t data[4] = {255, 255, 255, 255};

    flash_read(g_drivers.flash_spi, 1, data, 4);
    // check if data is valid
    if (data[0] == 255 || data[1] == 255 || data[2] == 255 || data[3] == 255)
      return;

    union float_32_u_t res;
    size_t pos = 0;
    read_float(data, &pos, &res);
    // sanity check
    if (fabs(res.f32) < 6.0f)
       g_robot_config.gyro_offset = res.f32;
}

void init_interrupts()
{
    volatile int tenuscs = 0;
    interrupt_register_native(LOOP_INT_NUMBER, (void*)&tenuscs, interrupt_handler);
    interrupt_configure(LOOP_INT_NUMBER, EDGE_RISING);

    interrupt_acknowledge(LOOP_INT_NUMBER);
    interrupt_enable(LOOP_INT_NUMBER);
}

void main( void )
{
    init_default_values();
    init_prepherals();
    init_nrf();
    init_gyro_m();

    if(g_robot_config.robot_num != 15)
    {
       nrf_channel_search(false);
    }

    if(g_robot_config.robot_num == 15)
    {
        gyro_calibration_process();
    }

    get_gyro_offset_from_flash();

    init_interrupts();
}

