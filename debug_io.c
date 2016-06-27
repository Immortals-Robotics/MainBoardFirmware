#include "debug_io.h"

#include <drv_ioport.h>
#include <drv_pwm8.h>
#include <timing.h>
#include "helpers.h"
#include "robot_data_types.h"

extern struct drivers_t g_drivers;

__noinline bool get_fpga_delay_boot_state(void)
{
    return ioport_get_value(g_drivers.debug_port, 0) & 0x4;
}

__noinline uint8_t get_swicth(void)
{
    const uint32_t a = ioport_get_value(g_drivers.debug_port, 0);
    return (a >> 8) & 0x0F;
}

__noinline bool get_swicth_bit(const uint8_t bit)
{
    const uint8_t switch_value = get_swicth();
    return get_bit_u8(switch_value, bit);
}

__noinline uint8_t get_robot_num(void)
{
    const uint32_t a = ioport_get_value(g_drivers.debug_port, 0);
    return (a >> 12) & 0x0F;
}

__noinline uint8_t get_button(void)
{
    const uint32_t a = ioport_get_value(g_drivers.debug_port, 0);
    return (a >> 4) & 0x0F;
}

__noinline bool get_button_bit(const uint8_t bit)
{
    const uint8_t button_value = get_button();
    return get_bit_u8(button_value, bit);
}

static uint8_t ledState = 0;
__noinline void set_led(const uint8_t led_mask)
{
    set_bit_mask_u8(&ledState, led_mask);
    ioport_set_value(g_drivers.debug_port, 0, ledState);
}

__noinline void clear_led(const uint8_t led_mask)
{
    clear_bit_mask_u8(&ledState, led_mask);
    ioport_set_value(g_drivers.debug_port, 0, ledState);
}

__noinline void set_buzzer(const uint16_t freq)
{
    pwm8_set_frequency(g_drivers.buzzer_pwm, freq);
    pwm8_set_pulsewidth(g_drivers.buzzer_pwm, 128);
}

__noinline void clear_buzzer(void)
{
    pwm8_set_pulsewidth(g_drivers.buzzer_pwm, 0);
}

__noinline void beep(const uint16_t freq, const uint32_t delay)
{
    set_buzzer(freq);
    delay_ms(delay);
    clear_buzzer();
}

__noinline void beep_multi(const uint32_t on_delay, const uint32_t off_delay, const uint8_t count)
{
    for(int i = 0; i < count; i++)
    {
        set_buzzer(400);
        delay_ms(on_delay);
        clear_buzzer();
        delay_ms(off_delay);
    }
}

