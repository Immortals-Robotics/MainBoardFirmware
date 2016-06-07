#ifndef DEBUG_IO_H
#define DEBUG_IO_H

#include <stdint.h>
#include <stdbool.h>

#define LOOP_LED 2
#define RX_LED 4
#define ALL_LED LOOP_LED+RX_LED
#define ON  1
#define OFF 0

bool get_fpga_delay_boot_state(void);

uint8_t get_swicth(void);
bool get_swicth_bit(const uint8_t bit);
uint8_t get_robot_num(void);

uint8_t get_button(void);
bool get_button_bit(const uint8_t bit);

void set_led(const uint8_t led_mask);
void clear_led(const uint8_t led_mask);
void set_buzzer(const uint16_t freq);
void clear_buzzer(void);
void beep(const uint16_t freq, const uint32_t delay);
void beep_multi(const uint32_t on_delay, const uint32_t off_delay, const uint8_t count);

#endif

