#include <cstdio>

#include <pigpiod_if2.h>

#include "spi_hal.h"

extern "C" {
#include "tmc/ic/TMC4671/TMC4671.h"
#include "tmc/ic/TMC6200/TMC6200.h"
}

#include "immortals/micro.pb.h"
#include <google/protobuf/util/delimited_message_util.h>

bool init_4671(const uint8_t motor_id)
{
    // check the device
    tmc4671_writeInt(motor_id, TMC4671_CHIPINFO_ADDR, 0);
    const int result = tmc4671_readInt(motor_id, TMC4671_CHIPINFO_DATA);
    if (result != 0x34363731) // ASCII "4671"
    {
        return false;
    }

    // Motor type &  PWM configuration
    tmc4671_setMotorType(motor_id, TMC4671_THREE_PHASE_BLDC);
    tmc4671_setPolePairs(motor_id, 8); // 8 for main motors, 1 for the dribbler
    tmc4671_writeInt(motor_id, TMC4671_PWM_POLARITIES, 0x00000000);
    tmc4671_writeInt(motor_id, TMC4671_PWM_MAXCNT, 0x00000F9F);
    tmc4671_writeInt(motor_id, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
    tmc4671_writeInt(motor_id, TMC4671_PWM_SV_CHOP, 0x00000007);

    // ADC configuration
    tmc4671_writeInt(motor_id, TMC4671_ADC_I_SELECT, 0x18000100);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCLK_A, 0x20000000);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCLK_B, 0x20000000);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
    tmc4671_writeInt(motor_id, TMC4671_ADC_I0_SCALE_OFFSET, 0xFF007EF5);
    tmc4671_writeInt(motor_id, TMC4671_ADC_I1_SCALE_OFFSET, 0xFF007F85);

    // Digital hall settings
    tmc4671_writeInt(motor_id, TMC4671_HALL_MODE, 0x00001000);
    tmc4671_writeInt(motor_id, TMC4671_HALL_PHI_E_PHI_M_OFFSET, 0x55550000);

    // ABN encoder settings
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_MODE, 0x00001000);
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_PPR, 0x00004000);
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_COUNT, 0x00002246);
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0xA9E00000);

    // Limits
    // TODO: read from the config file
    tmc4671_writeInt(motor_id, TMC4671_PIDOUT_UQ_UD_LIMITS, 0x00005A81);
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000007D0);

    // PI settings
    // TODO: read from the config file
    tmc4671_setTorqueFluxPI(motor_id, 550, 11000);
    tmc4671_setVelocityPI(motor_id, 200, 15);
    tmc4671_setPositionPI(motor_id, 0, 0);

#if 0
    // Init encoder (mode 0)
    tmc4671_writeInt(motor_id, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
    tmc4671_writeInt(motor_id, TMC4671_PHI_E_SELECTION, 0x00000001);
    tmc4671_writeInt(motor_id, TMC4671_PHI_E_EXT, 0x00000000);
    tmc4671_writeInt(motor_id, TMC4671_UQ_UD_EXT, 0x00000FA0);
    time_sleep(1);
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_COUNT, 0x00000000);
#endif

    // Selectors
    // TODO: read from the config file
    tmc4671_writeInt(motor_id, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_HALL);
    tmc4671_writeInt(motor_id, TMC4671_VELOCITY_SELECTION, TMC4671_VELOCITY_PHI_E_ABN);
    tmc4671_writeInt(motor_id, TMC4671_POSITION_SELECTION, TMC4671_POSITION_PHI_E_ABN);
    tmc4671_writeInt(motor_id, TMC4671_ADC_I_SELECT, 0x18000100);

    tmc4671_switchToMotionMode(motor_id, TMC4671_MOTION_MODE_STOPPED);

    return true;
}

bool init_6200(const uint8_t motor_id)
{
    const int version = TMC6200_FIELD_READ(motor_id, TMC6200_IOIN_OUTPUT, TMC6200_VERSION_MASK, TMC6200_VERSION_SHIFT);
    if (version != 0x10)
    {
        return false;
    }

    int gstat = tmc6200_readInt(motor_id, TMC6200_GSTAT);

    TMC6200_FIELD_UPDATE(motor_id, TMC6200_DRV_CONF, TMC6200_DRVSTRENGTH_MASK, TMC6200_DRVSTRENGTH_SHIFT, 1);

    // set default PWM configuration for use with TMC4671
    tmc6200_writeInt(motor_id, TMC6200_GCONF, 0x0);

    return true;
}

void open_loop_test(const uint8_t motor_id)
{
    // Motor type &  PWM configuration
    tmc4671_writeInt(motor_id, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030008);
    tmc4671_writeInt(motor_id, TMC4671_PWM_POLARITIES, 0x00000000);
    tmc4671_writeInt(motor_id, TMC4671_PWM_MAXCNT, 0x00000F9F);
    tmc4671_writeInt(motor_id, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
    tmc4671_writeInt(motor_id, TMC4671_PWM_SV_CHOP, 0x00000007);

    // ADC configuration
    tmc4671_writeInt(motor_id, TMC4671_ADC_I_SELECT, 0x18000100);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCLK_A, 0x20000000);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCLK_B, 0x20000000);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
    tmc4671_writeInt(motor_id, TMC4671_ADC_I0_SCALE_OFFSET, 0xFF007EF5);
    tmc4671_writeInt(motor_id, TMC4671_ADC_I1_SCALE_OFFSET, 0xFF007F85);

    // Open loop settings
    tmc4671_writeInt(motor_id, TMC4671_OPENLOOP_MODE, 0x00000000);
    tmc4671_writeInt(motor_id, TMC4671_OPENLOOP_ACCELERATION, 0x0000003C);
    tmc4671_writeInt(motor_id, TMC4671_OPENLOOP_VELOCITY_TARGET, 0xFFFFFFFB);

    // Feedback selection
    tmc4671_writeInt(motor_id, TMC4671_PHI_E_SELECTION, 0x00000002);
    tmc4671_writeInt(motor_id, TMC4671_UQ_UD_EXT, 0x00000FA0);

    // ===== Open loop test drive =====

    // Switch to open loop velocity mode
    tmc4671_writeInt(0, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);

    // Rotate right
    tmc4671_writeInt(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000003C);
    time_sleep(2);

    // Rotate left
    tmc4671_writeInt(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0xFFFFFFC4);
    time_sleep(4);

    // Stop
    tmc4671_writeInt(0, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x00000000);
    time_sleep(2);
    tmc4671_writeInt(0, TMC4671_UQ_UD_EXT, 0x00000000);
}

void hall_test(const uint8_t motor_id)
{
    // Motor type &  PWM configuration
    tmc4671_writeInt(motor_id, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030008);
    tmc4671_writeInt(motor_id, TMC4671_PWM_POLARITIES, 0x00000000);
    tmc4671_writeInt(motor_id, TMC4671_PWM_MAXCNT, 0x00000F9F);
    tmc4671_writeInt(motor_id, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
    tmc4671_writeInt(motor_id, TMC4671_PWM_SV_CHOP, 0x00000007);

    // ADC configuration
    tmc4671_writeInt(motor_id, TMC4671_ADC_I_SELECT, 0x18000100);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCLK_A, 0x20000000);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCLK_B, 0x20000000);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
    tmc4671_writeInt(motor_id, TMC4671_ADC_I0_SCALE_OFFSET, 0xFF007EF5);
    tmc4671_writeInt(motor_id, TMC4671_ADC_I1_SCALE_OFFSET, 0xFF007F85);

    // Digital hall settings
    tmc4671_writeInt(motor_id, TMC4671_HALL_MODE, 0x00001000);
    tmc4671_writeInt(motor_id, TMC4671_HALL_PHI_E_PHI_M_OFFSET, 0x55550000);

    // Feedback selection
    tmc4671_writeInt(motor_id, TMC4671_PHI_E_SELECTION, 0x00000005);
    tmc4671_writeInt(motor_id, TMC4671_VELOCITY_SELECTION, 0x0000000C);

    // Limits
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000003E8);

    // PI settings
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100);
    tmc4671_writeInt(motor_id, TMC4671_PID_FLUX_P_FLUX_I, 0x01000100);

    // ===== Digital hall test drive =====

    // Switch to torque mode
    tmc4671_writeInt(motor_id, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000001);

    // Rotate right
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_FLUX_TARGET, 0x03E80000);
    time_sleep(3);

    // Rotate left
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_FLUX_TARGET, 0xFC180000);
    time_sleep(3);

    // Stop
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_FLUX_TARGET, 0x00000000);
}

void encoder_test(const uint8_t motor_id)
{
    // Motor type &  PWM configuration
    tmc4671_writeInt(motor_id, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030008);
    tmc4671_writeInt(motor_id, TMC4671_PWM_POLARITIES, 0x00000000);
    tmc4671_writeInt(motor_id, TMC4671_PWM_MAXCNT, 0x00000F9F);
    tmc4671_writeInt(motor_id, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
    tmc4671_writeInt(motor_id, TMC4671_PWM_SV_CHOP, 0x00000007);

    // ADC configuration
    tmc4671_writeInt(motor_id, TMC4671_ADC_I_SELECT, 0x18000100);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCLK_A, 0x20000000);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MCLK_B, 0x20000000);
    tmc4671_writeInt(motor_id, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E);
    tmc4671_writeInt(motor_id, TMC4671_ADC_I0_SCALE_OFFSET, 0xFF007EF5);
    tmc4671_writeInt(motor_id, TMC4671_ADC_I1_SCALE_OFFSET, 0xFF007F85);

    // ABN encoder settings
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_MODE, 0x00001000);
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_PPR, 0x00004000);
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_COUNT, 0x00002246);
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0xA9E00000);

    // Limits
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x000003E8);

    // PI settings
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100);
    tmc4671_writeInt(motor_id, TMC4671_PID_FLUX_P_FLUX_I, 0x01000100);

    // ===== ABN encoder test drive =====

    // Init encoder (mode 0)
    tmc4671_writeInt(motor_id, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
    tmc4671_writeInt(motor_id, TMC4671_PHI_E_SELECTION, 0x00000001);
    tmc4671_writeInt(motor_id, TMC4671_PHI_E_EXT, 0x00000000);
    tmc4671_writeInt(motor_id, TMC4671_UQ_UD_EXT, 0x00000FA0);
    time_sleep(1);
    tmc4671_writeInt(motor_id, TMC4671_ABN_DECODER_COUNT, 0x00000000);

    // Feedback selection
    tmc4671_writeInt(motor_id, TMC4671_PHI_E_SELECTION, 0x00000003);
    tmc4671_writeInt(motor_id, TMC4671_VELOCITY_SELECTION, 0x00000009);

    // Switch to torque mode
    tmc4671_writeInt(motor_id, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000001);

    // Rotate right
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_FLUX_TARGET, 0x03E80000);
    time_sleep(3);

    // Rotate left
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_FLUX_TARGET, 0xFC180000);
    time_sleep(3);

    // Stop
    tmc4671_writeInt(motor_id, TMC4671_PID_TORQUE_FLUX_TARGET, 0x00000000);
}

void velocity_test(const uint8_t motor_id)
{
    // Switch to velocity mode
    tmc4671_switchToMotionMode(motor_id, TMC4671_MOTION_MODE_VELOCITY);

    // Rotate right
    tmc4671_setTargetVelocity(motor_id, 8000);
    time_sleep(3);

    // Rotate left
    tmc4671_setTargetVelocity(motor_id, -8000);
    time_sleep(3);

    // Stop
    tmc4671_setTargetVelocity(motor_id, 0);
    tmc4671_switchToMotionMode(motor_id, TMC4671_MOTION_MODE_STOPPED);
}

void motors_test(const uint8_t motor_id)
{
    init_spi();

    init_4671(motor_id);

    init_6200(motor_id);

    enable_driver(true);

    velocity_test(motor_id);

    shutdown_spi();
}

void micro_test()
{
    const int pi = pigpio_start(0, 0); /* Connect to local Pi. */

    if (pi < 0)
    {
        printf("Can't connect to pigpio daemon\n");
        return;
    }

    static constexpr size_t buffer_size = 128;

    char rx_buf[buffer_size] = {};
    char tx_buf[buffer_size] = {};

    int h = spi_open(pi, 1, 4 * 1000 * 1000, 
        PI_SPI_FLAGS_AUX_SPI(1) | 
                PI_SPI_FLAGS_MODE(0));

    if (h < 0)
        return;

    while (true)
    {
        Immortals::Protos::MicroCommand command{};

        Immortals::Protos::MikonaCommand* const mikona = command.mutable_mikona();
        mikona->set_charge(true);
        mikona->set_discharge(false);
        mikona->set_kick_a(0);
        mikona->set_kick_b(0);

        Immortals::Protos::LEDCommand *const led = command.mutable_led();
        led->set_wifi_connected(true);
        led->set_wifi_acitivity(false);
        led->set_fault(false);

        command.set_buzzer(false);

        command.SerializeToArray(tx_buf, buffer_size);

        google::protobuf::io::ArrayOutputStream output_stream{tx_buf, buffer_size};
        google::protobuf::util::SerializeDelimitedToZeroCopyStream(command, &output_stream);

        spi_xfer(pi, h, tx_buf, rx_buf, buffer_size);

        Immortals::Protos::MicroStatus status{};
        google::protobuf::io::ArrayInputStream input_stream{rx_buf, buffer_size};
        google::protobuf::util::ParseDelimitedFromZeroCopyStream(&status, &input_stream, nullptr);
        printf("ir: %d\n", status.balldetected());
    }

    spi_close(pi, h);

    pigpio_stop(pi); /* Disconnect from local Pi. */
}

int main(int argc, char* argv[])
{
    motors_test(3);
    //micro_test();

    return 0;
}
