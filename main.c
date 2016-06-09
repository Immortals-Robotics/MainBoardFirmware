#include <math.h>
#include <stdlib.h>

#include <drv_ioport.h>
#include <drv_pwmx.h>
#include <drv_pwm8.h>
#include <interrupts.h>
#include <timing.h>

#include "devices.h"

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

#include "robot_data_types.h"

#include "wireless.h"

struct drivers_t g_drivers;
struct robot_command_msg_t g_robot_cmd;
struct robot_config_t g_robot_config;
struct robot_state_t g_robot_state;

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
        const float error = g_robot_state.motor_desired[i] + g_robot_state.omega_desired - g_robot_state.motor_current[i];
        g_robot_state.motor_pwm[i] = update_pid(&g_robot_state.motor_pid[i], error, &g_robot_config.motor_pid_config);
        g_robot_state.motor_pwm[i] *= 1400.0f / (1400.0f - fabs(g_robot_state.motor_current[i]));
        g_robot_state.motor_pwm[i] = min_float(1023.0f, max_float(-1023.0f, g_robot_state.motor_pwm[i]));

        pwmx_set_pulsewidth(g_drivers.motor_pwm[i], (uint16_t)abs(g_robot_state.motor_pwm[i]));
        sign_bits |= sgn_01_inv_f(g_robot_state.motor_pwm[i]) << i;
    }

    ioport_set_value(g_drivers.motordir_port, 0, sign_bits);
}

void gyro_process()
{
    static float angle_history[ANGLE_PREDICT_STEPS] = {0};
    static uint16_t angle_history_index = 0;
    static float old_omega_desired = 0;

	g_robot_state.omega_current = get_gyro_omega();

	const float d_omega = g_robot_state.omega_current / 1285.0f;

    g_robot_state.angle_predict -= angle_history[angle_history_index];
    angle_history[angle_history_index] = d_omega;
    g_robot_state.angle_predict += d_omega;

    g_robot_state.orientation += d_omega;
    if (g_robot_state.orientation > 180.0f)
       g_robot_state.orientation -= 360.0f;
    if (g_robot_state.orientation < -180.0f)
       g_robot_state.orientation += 360.0f;

    angle_history_index = (angle_history_index + 1) % ANGLE_PREDICT_STEPS;

    float d_angle = g_robot_state.orientation - g_robot_cmd.target_orientation.f32;
    if (d_angle > 180.0f)
        d_angle -= 360.0f;
    if (d_angle < -180.0f)
        d_angle += 360.0f;
    g_robot_state.omega_desired = update_pid(&g_robot_state.gyro_pid, d_angle, &g_robot_config.gyro_pid_config);
    g_robot_state.omega_desired += g_robot_state.omega_current * g_robot_config.gyro_d; //Calculate d term here, cause we have omega from gyro

    if ( g_robot_state.omega_desired * old_omega_desired < 0 )
    {
        float tmp = g_robot_config.max_w_dec;
        if ( g_robot_state.omega_desired < 0 )
           tmp = -tmp;
        tmp += old_omega_desired;

        if ( tmp * g_robot_state.omega_desired > 0 )
        {
            tmp = g_robot_config.max_w_acc;
            if ( g_robot_state.omega_desired < 0 )
               tmp = -tmp;
            if ( fabs ( tmp ) > fabs ( g_robot_state.omega_desired ) )
                tmp = g_robot_state.omega_desired;
        }
        g_robot_state.omega_desired = tmp;
    }
    else
    {
        if ( fabs ( g_robot_state.omega_desired ) > fabs ( old_omega_desired ) + g_robot_config.max_w_acc )
        {
            if ( g_robot_state.omega_desired < 0 )
               g_robot_state.omega_desired = -( fabs ( old_omega_desired ) + g_robot_config.max_w_acc );
            else
                g_robot_state.omega_desired = ( fabs ( old_omega_desired ) + g_robot_config.max_w_acc );
        }
    }
    g_robot_state.omega_desired = max_float ( -120 , min_float ( 120 ,g_robot_state.omega_desired ) );
    old_omega_desired = g_robot_state.omega_desired;
}

void check_encoder_fault_process()
{
    #define ENCODER_FAULT_CHECK_TIME_SEC 3
    #define ENCODER_FAULT_CHECK_LOOP_COUNT (ENCODER_FAULT_CHECK_TIME_SEC*1280)

    static uint16_t ticks_to_recalculate = ENCODER_FAULT_CHECK_LOOP_COUNT;
    static uint16_t zero_vel_count[4] = {0};

    //TODO : Choose the if Fault action -> never run the motors or check it again

    if (!g_robot_config.check_motor_fault ||
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
    if (get_no_command_limit_reached())
    	g_robot_cmd.halt = true;
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

	g_robot_state.orientation = 0.0f;
    g_robot_state.angle_predict = 0.0f;
    g_robot_state.omega_desired = 0.0f;
    g_robot_state.omega_current = 0.0f;

    g_robot_state.encoder_has_fault = false;
    g_robot_config.check_motor_fault = false;

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
                gyro_offset_tmp.f32 += get_gyro_omega() / 100.0f;
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

