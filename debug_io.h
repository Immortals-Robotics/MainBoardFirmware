#ifndef DEBUG_IO_H
#define DEBUG_IO_H

#include <stdint.h>
#include <stdbool.h>

#define LOOP_LED 4
#define RX_LED 2
#define ALL_LED LOOP_LED+RX_LED
#define ON  1
#define OFF 0

__noinline bool get_fpga_delay_boot_state(void);

__noinline uint8_t get_swicth(void);
__noinline bool get_swicth_bit(const uint8_t bit);
__noinline uint8_t get_robot_num(void);

__noinline uint8_t get_button(void);
__noinline bool get_button_bit(const uint8_t bit);

__noinline void set_led(const uint8_t led_mask);
__noinline void clear_led(const uint8_t led_mask);
__noinline void set_buzzer(const uint16_t freq);
__noinline void clear_buzzer(void);
__noinline void beep(const uint16_t freq, const uint32_t delay);
__noinline void beep_multi(const uint32_t on_delay, const uint32_t off_delay, const uint8_t count);

#endif

