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

#define NO_CMD_TIME_LIMIT UINT64_MAX // in ms
static uint64_t last_cmd_time_ms = 0;
static int8_t pending_feedback = -1;

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

    // TODO: button bits
    
	// WARNING: this line is platform dependent
	feedback_msg.motor_fault = *((struct bits8_t*)(&g_robot_state.motor_fault));

    feedback_msg.fault = g_robot_state.motor_fault > 0;

    feedback_msg.ball_detected = get_button_bit(0);
    feedback_msg.booster_enabled = get_swicth_bit(0);

    // TODO: dribbler conected

    struct robot_wrapper_msg_t wrapper_msg;
    wrapper_msg.length = (uint8_t)write_robot_feedback_fixed(wrapper_msg.data, &feedback_msg, pending_feedback);

    write_robot_wrapper_fixed(payload, &wrapper_msg);
}

static void send_feedback()
{
    nrf24l01_set_rf_ch(g_robot_config.nrf_channel_tx);
    nrf24l01_set_as_tx();
    construct_feedback_packet();
    nrf24l01_write_tx_payload(payload, NRF_TX_PAYLOAD_SIZE, true);

    pending_feedback = -1;
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

    // TODO: send feedback
    pending_feedback = (int8_t) g_robot_cmd.feedback_request;

    last_cmd_time_ms = clock_ms();
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

bool get_no_command_limit_reached(void)
{
    const uint64_t elapsed_since_last_cmd_ms = elapsed_time_ms(last_cmd_time_ms);
    return elapsed_since_last_cmd_ms >= NO_CMD_TIME_LIMIT ? true : false;
}

void nrf_process(void)
{
    if ( nrf24l01_irq_rx_dr_active() )
    {
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

        nrf24l01_flush_rx();
        nrf24l01_irq_clear_rx_dr();
    }
    else if ( nrf24l01_irq_tx_ds_active() )
    {
        nrf24l01_flush_tx();
        nrf24l01_set_rf_ch(g_robot_config.nrf_channel_rx);
        nrf24l01_set_as_rx(true);
        nrf24l01_irq_clear_tx_ds();
    }
    else
    {
        nrf24l01_irq_clear_all();
    }

    if (pending_feedback >= 0)
    {
        send_feedback();
    }
}

void init_nrf(void)
{
    nrf24l01_initialize_debug(true, NRF_RX_PAYLOAD_SIZE, false);
    nrf24l01_rx_active_to_standby();
    nrf24l01_flush_rx();
    nrf24l01_set_rf_ch(g_robot_config.nrf_channel_rx);
    uint8_t own_rx_add[5] = {110, 110, g_robot_config.robot_num, 110, 110};
    nrf24l01_set_rx_addr(own_rx_add, 5, 0);
    own_rx_add[2] = 30;
    nrf24l01_set_tx_addr(own_rx_add, 5);

    nrf24l01_rx_standby_to_active();
    delay_ms(1);

    if (nrf24l01_get_status() == 0)
    {
        if (get_fpga_delay_boot_state())
        {
            beep(1000, 200);
        }
    }
}

void nrf_channel_search()
{
    if (get_fpga_delay_boot_state())
    {
        bool found_ch = false;

        uint8_t own_rx_add[5] = {110, 110, 25, 110, 110};
        nrf24l01_set_rx_addr(own_rx_add, 5, 0);

        while (!found_ch)
        {
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
                       nrf24l01_read_rx_payload(payload, NRF_RX_PAYLOAD_SIZE);
                       if (payload[0] != test_ch)
                          continue;

                       found_ch = true;
                       g_robot_config.nrf_channel_rx = payload[0];
                       g_robot_config.nrf_channel_tx = payload[1];
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
        nrf24l01_set_rf_ch(g_robot_config.nrf_channel_rx);
        nrf24l01_rx_standby_to_active();
        delay_ms(1);
    }
}
