#include "wireless.h"

#include <stdint.h>
#include <math.h>
#include <timing.h>
#include <drv_ioport.h>

#include "nrf24l01.h"
#include "debug_io.h"
#include "helpers.h"
#include "robot_data_types.h"
#include "defines.h"
#include "data_lite.h"
#include "reader.h"
#include "writer.h"

#define NRF_RX_PAYLOAD_SIZE MAX_PAYLOAD_SIZE
#define NRF_TX_PAYLOAD_SIZE MAX_PAYLOAD_SIZE
#define NO_CMD_COUNTER_LIMIT 1200
#define FEEDBACK_SEND_STEPS 7

static uint8_t feedback_step = 0;
static int16_t no_cmd_counter = -1;

static uint8_t payload[MAX_PAYLOAD_SIZE];

extern struct robot_command_msg_t g_robot_cmd;
extern struct robot_config_t g_robot_config;
extern struct robot_state_t g_robot_state;
extern struct drivers_t g_drivers;

static void construct_feedback_packet(void)
{
    struct robot_feedback_msg_t feedback_msg;
    feedback_msg.battery_voltage.f32 = 14.8f;
    feedback_msg.capacitor_voltage.f32 = 250.0f;
    feedback_msg.omega.f32 = g_robot_state.omega_current;
    feedback_msg.orientation.f32 = g_robot_state.orientation;

    feedback_msg.motor_velocity.x.f32 = g_robot_state.motor_current[0];
    feedback_msg.motor_velocity.y.f32 = g_robot_state.motor_current[1];
    feedback_msg.motor_velocity.z.f32 = g_robot_state.motor_current[2];
    feedback_msg.motor_velocity.w.f32 = g_robot_state.motor_current[3];

    feedback_msg.motor_target.x.f32 = g_robot_state.motor_desired[0] + g_robot_state.omega_desired;
    feedback_msg.motor_target.y.f32 = g_robot_state.motor_desired[1] + g_robot_state.omega_desired;
    feedback_msg.motor_target.z.f32 = g_robot_state.motor_desired[2] + g_robot_state.omega_desired;
    feedback_msg.motor_target.w.f32 = g_robot_state.motor_desired[3] + g_robot_state.omega_desired;

	// TODO: per-motor fault bits
	// TODO: button bits

	feedback_msg.fault = g_robot_state.encoder_has_fault;

    feedback_msg.ball_detected = get_button_bit(0);
    feedback_msg.booster_enabled = get_swicth_bit(0);

    // TODO: dribbler conected

	struct robot_wrapper_msg_t wrapper_msg;
    wrapper_msg.length = (uint8_t)write_robot_feedback_fixed(wrapper_msg.data, &feedback_msg, g_robot_cmd.feedback_request);

	write_robot_wrapper_fixed(payload, &wrapper_msg);
}

static bool is_feedback_process_completed(void)
{
    return feedback_step <= 0;
}

static void send_feedback_process()
{
    switch(feedback_step)
    {
        case 6:
             construct_feedback_packet();
             break;
        case 5:
             nrf24l01_set_as_tx();
             break;
        case 4:
             nrf24l01_write_tx_payload(payload, NRF_TX_PAYLOAD_SIZE, false);
             break;
        case 3:
             nrf24l01_transmit();
             break;
        case 2:
             if ( nrf24l01_irq_pin_active() )
             {
                nrf24l01_irq_clear_all();
                nrf24l01_flush_tx();
             }
             else
             	feedback_step++;
             break;
        case 1:
             nrf24l01_set_as_rx(true);
             break;
    }
    feedback_step--;
}

static void process_received_command(const struct robot_command_msg_t* const command)
{
	if (command->has_orientation)
        g_robot_state.orientation = command->orientation.f32 + g_robot_state.angle_predict;

	if (command->halt == false)
    {
    	const float orient_rad = (90.0f - g_robot_state.orientation) * 0.0174528f;
        const float coss = cos ( orient_rad );
        const float sinn = sin ( orient_rad );

        const float local_vx = command->velocity.x.f32 * coss - command->velocity.y.f32 * sinn;
        const float local_vy = command->velocity.x.f32 * sinn + command->velocity.y.f32 * coss;

        g_robot_state.motor_desired[0] =  (local_vy * 0.8387f) - (local_vx * 0.5446f);
        g_robot_state.motor_desired[1] = -(local_vy * 0.8387f) - (local_vx * 0.5446f);
        g_robot_state.motor_desired[2] = -(local_vy * 0.7070f) + (local_vx * 0.7070f);
        g_robot_state.motor_desired[3] =  (local_vy * 0.7070f) + (local_vx * 0.7070f);

        g_robot_state.motor_desired[0] *= -4.0f;
        g_robot_state.motor_desired[1] *= -4.0f;
        g_robot_state.motor_desired[2] *= -4.0f;
        g_robot_state.motor_desired[3] *= -4.0f;
    }

	ioport_set_value(g_drivers.kick_port, (uint8_t)command->shoot_type, (uint32_t)command->shoot_power.f32);
	pwmx_set_pulsewidth(g_drivers.motor_d_pwm, (uint16_t)command->dribbler.f32);

    if (command->beep > 0)
		set_buzzer(command->beep * 60);
    else
    	clear_buzzer();

	g_robot_cmd = *command;
    feedback_step = FEEDBACK_SEND_STEPS;

	no_cmd_counter = 0;
}

static void process_received_control_config(const struct robot_control_config_msg_t* const config)
{
    g_robot_config.motor_pid_config.p_gain = config->motor_kp.f32;
    g_robot_config.motor_pid_config.i_gain = config->motor_ki.f32;
    g_robot_config.motor_pid_config.d_gain = config->motor_kd.f32;
    g_robot_config.motor_pid_config.i_max = config->motor_i_limit.f32;

    g_robot_config.gyro_pid_config.p_gain = config->gyro_kp.f32;
    g_robot_config.gyro_pid_config.i_gain = config->gyro_ki.f32;
    g_robot_config.gyro_d = config->gyro_kd.f32;
    g_robot_config.gyro_pid_config.i_max = config->gyro_i_limit.f32;

    g_robot_config.max_w_acc = config->max_w_acc.f32;
    g_robot_config.max_w_dec = config->max_w_dec.f32;
}

static void update_no_command_state(void)
{
    if ( no_cmd_counter >= 0 )
       no_cmd_counter ++;

	no_cmd_counter = min_s16(no_cmd_counter, NO_CMD_COUNTER_LIMIT);
}

bool get_no_command_limit_reached(void)
{
    return no_cmd_counter >= NO_CMD_COUNTER_LIMIT ? true : false;
}

void nrf_process(void)
{
    if ( is_feedback_process_completed() && ( nrf24l01_irq_pin_active() ) )
    {
        if ( nrf24l01_irq_rx_dr_active() )
        {
            set_led(RX_LED);
            nrf24l01_read_rx_payload(payload, NRF_RX_PAYLOAD_SIZE);

			struct robot_wrapper_msg_t wrapper_msg;
            if (read_robot_wrapper_fixed(payload, NRF_RX_PAYLOAD_SIZE, &wrapper_msg) == PARSE_RESULT_SUCCESS)
            {
                const uint8_t msg_type = MESSAGE_TYPE(wrapper_msg.data[0]);
                if (msg_type == TYPE_COMMAND)
                {
                    struct robot_command_msg_t tmp_command;
                    if (read_robot_command_fixed(wrapper_msg.data, wrapper_msg.length, &tmp_command) == PARSE_RESULT_SUCCESS)
                    {
                        process_received_command(&tmp_command);
                    }
                }
                else if (msg_type == TYPE_CONFIG_CONTROL)
                {
                    struct robot_control_config_msg_t tmp_control_config;
                    if (read_robot_control_config_fixed(wrapper_msg.data, wrapper_msg.length, &tmp_control_config) == PARSE_RESULT_SUCCESS)
                    {
                        process_received_control_config(&tmp_control_config);
                    }
                }
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

	update_no_command_state();
}

void init_nrf(void)
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
