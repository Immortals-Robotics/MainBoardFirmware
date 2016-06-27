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

#define ANGLE_PREDICT_STEPS 73
#define LOOP_INT_NUMBER    1
#define GYRO_INT_NUMBER    14
#define NRF_INT_NUMBER     15

#include "robot_data_types.h"

#include "wireless.h"

struct drivers_t g_drivers;
struct robot_command_msg_t g_robot_cmd;
struct robot_config_t g_robot_config;
struct robot_state_t g_robot_state;

__noinline void read_on_board_config_from_flash(void);
__noinline void write_on_board_config_to_flash(void);

void calculate_motor_vels ( void )
{
    const uint32_t tmp_dir = ioport_get_value(g_drivers.motordir_port, 0);

    for (uint8_t i = 0; i < 4; i++)
    {
        g_robot_state.motor_current[i] = 4.0f * ioport_get_value(g_drivers.motorvel_port, i);
        if (get_bit_u32(tmp_dir, 4 + i))
            g_robot_state.motor_current[i] = -g_robot_state.motor_current[i];
    }
}

void control_loop ( void )
{
    if ( g_robot_cmd.halt == true )
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

	const float gyro_dt = 1.0f / 650.0f;

    g_robot_state.omega_current = get_gyro_omega();

    const float d_omega = g_robot_state.omega_current * gyro_dt;

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
    #define ENCODER_FAULT_CHECK_TIME_MS 3000

	static uint64_t last_check_time_ms = 0;

    static uint16_t zero_vel_count[4] = {0, 0, 0, 0};

    //TODO : Choose the if Fault action -> never run the motors or check it again

    if (!g_robot_config.check_motor_fault ||
        g_robot_state.encoder_has_fault)
       return;


    if (elapsed_time_ms(last_check_time_ms) > ENCODER_FAULT_CHECK_TIME_MS)
    {
        last_check_time_ms = clock_ms();

        for (uint8_t i = 0; i < 4; i++)
        {
            if (zero_vel_count[i] > 1000)
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

	check_encoder_fault_process();
	if (get_no_command_limit_reached())
        g_robot_cmd.halt = true;

	clear_led(LOOP_LED);
    interrupt_acknowledge(LOOP_INT_NUMBER);
}

__INTERRUPT_NATIVE void gyro_interrupt_handler(void)
{
    gyro_process();

    interrupt_acknowledge(GYRO_INT_NUMBER);
}

__INTERRUPT_NATIVE void nrf_interrupt_handler(void)
{
    set_led(RX_LED);
	nrf_process();

    clear_led(RX_LED);
    interrupt_acknowledge(NRF_INT_NUMBER);
}

__noinline void init_default_values()
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

    g_robot_cmd.halt = false;

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

	g_robot_config.check_motor_fault = false;
    g_robot_config.use_encoders = false;

	g_robot_config.nrf_channel_rx = 55;
    g_robot_config.nrf_channel_tx = 40;
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

void diag_process()
{
    #define WAIT_FOR_DIAG_BUTTON_MS 10000
    uint64_t start_time_ms = clock_ms();

    while (elapsed_time_ms(start_time_ms) < WAIT_FOR_DIAG_BUTTON_MS)
    {
        if (get_button_bit(0) == 0)
        {
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
            write_on_board_config_to_flash();

            beep(1000, 200);
            break;
        }
		else if (get_button_bit(1) == 0)
        {
            // wait and beep
            beep(1000, 200);
            delay_ms(200);

			init_default_values();
            write_on_board_config_to_flash();

            beep(1000, 200);
            break;
        }
		else if (get_button_bit(2) == 0)
        {
            nrf_channel_search();

            write_on_board_config_to_flash();

        	beep(5000, 200);
            break;
        }
    }
}

__noinline void read_on_board_config_from_flash()
{
    uint8_t data[MAX_ON_BOARD_SIZE + 1];

    flash_read(g_drivers.flash_spi, 1, data, MAX_ON_BOARD_SIZE + 1);

	const uint8_t length = data[0];
    uint8_t *buffer = data + 1;

    // check if data is valid
    if (length > MAX_ON_BOARD_SIZE)
      return;

	struct robot_on_board_config_t on_board_config;
    uint8_t result = read_robot_on_board_config_fixed(buffer, length, &on_board_config);

	if (result != PARSE_RESULT_SUCCESS)
    	return;

	// TODO
    g_robot_config.motor_pid_config.p_gain = on_board_config.control_config.motor_kp.f32;
    g_robot_config.motor_pid_config.i_gain = on_board_config.control_config.motor_ki.f32;
    g_robot_config.motor_pid_config.d_gain = on_board_config.control_config.motor_kd.f32;
    g_robot_config.motor_pid_config.i_max = on_board_config.control_config.motor_i_limit.f32;

	g_robot_config.gyro_pid_config.p_gain = on_board_config.control_config.gyro_kp.f32;
    g_robot_config.gyro_pid_config.i_gain = on_board_config.control_config.gyro_ki.f32;
    g_robot_config.gyro_d = on_board_config.control_config.gyro_kd.f32;
    g_robot_config.gyro_pid_config.i_max = on_board_config.control_config.gyro_i_limit.f32;

	g_robot_config.max_w_acc = on_board_config.control_config.max_w_acc.f32;
    g_robot_config.max_w_dec = on_board_config.control_config.max_w_dec.f32;

	g_robot_config.gyro_offset = on_board_config.gyro_offset.f32;
    g_robot_config.use_encoders = on_board_config.use_encoders;

	g_robot_config.nrf_channel_rx = on_board_config.nrf_channel_rx;
    g_robot_config.nrf_channel_tx = on_board_config.nrf_channel_tx;
}

__noinline void write_on_board_config_to_flash()
{
	struct robot_on_board_config_t on_board_config;

	on_board_config.control_config.motor_kp.f32 = g_robot_config.motor_pid_config.p_gain;
    on_board_config.control_config.motor_ki.f32 = g_robot_config.motor_pid_config.i_gain;
    on_board_config.control_config.motor_kd.f32 = g_robot_config.motor_pid_config.d_gain;
    on_board_config.control_config.motor_i_limit.f32 = g_robot_config.motor_pid_config.i_max;

	on_board_config.control_config.gyro_kp.f32 = g_robot_config.gyro_pid_config.p_gain;
    on_board_config.control_config.gyro_ki.f32 = g_robot_config.gyro_pid_config.i_gain;
    on_board_config.control_config.gyro_kd.f32 = g_robot_config.gyro_d;
    on_board_config.control_config.gyro_i_limit.f32 = g_robot_config.gyro_pid_config.i_max;

	on_board_config.control_config.max_w_acc.f32 = g_robot_config.max_w_acc;
    on_board_config.control_config.max_w_dec.f32 = g_robot_config.max_w_dec;

	on_board_config.gyro_offset.f32 = g_robot_config.gyro_offset;
    on_board_config.use_encoders = g_robot_config.use_encoders;

	on_board_config.nrf_channel_rx = g_robot_config.nrf_channel_rx;
    on_board_config.nrf_channel_tx = g_robot_config.nrf_channel_tx;

	uint8_t data[MAX_ON_BOARD_SIZE + 1];
	data[0] = write_robot_on_board_config_fixed(data + 1, &on_board_config);

	flash_sector_erase(g_drivers.flash_spi, 1, true);
	flash_write(g_drivers.flash_spi, 1, data, MAX_ON_BOARD_SIZE + 1, true);
}

void main( void )
{
    init_default_values();
    init_prepherals();

	read_on_board_config_from_flash();

	init_gyro_m();
	init_nrf();

	if(g_robot_config.robot_num == 15)
        diag_process();

	interrupt_register_native(LOOP_INT_NUMBER, NULL, interrupt_handler);
    interrupt_configure(LOOP_INT_NUMBER, EDGE_RISING);
	interrupt_acknowledge(LOOP_INT_NUMBER);
    interrupt_enable(LOOP_INT_NUMBER);

	interrupt_register_native(GYRO_INT_NUMBER, NULL, gyro_interrupt_handler);
    interrupt_configure(GYRO_INT_NUMBER, EDGE_RISING);
    interrupt_acknowledge(GYRO_INT_NUMBER);
    interrupt_enable(GYRO_INT_NUMBER);

	interrupt_register_native(NRF_INT_NUMBER, NULL, nrf_interrupt_handler);
    interrupt_configure(NRF_INT_NUMBER, LEVEL_HIGH);
	interrupt_acknowledge(NRF_INT_NUMBER);
    interrupt_enable(NRF_INT_NUMBER);
}

