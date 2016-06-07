#ifndef DATA_LITE_H
#define DATA_LITE_H

#include <stdint.h>

#include "half.h"

struct vector2f_t
{
	union  float_32_u_t x;
	union  float_32_u_t y;
};

struct vector3f_t
{
	union  float_32_u_t x;
	union  float_32_u_t y;
	union  float_32_u_t z;
};

struct vector4f_t
{
	union  float_32_u_t x;
	union  float_32_u_t y;
	union  float_32_u_t z;
	union  float_32_u_t w;
};

struct bits8_t
{
	uint8_t bit0 : 1;
	uint8_t bit1 : 1;
	uint8_t bit2 : 1;
	uint8_t bit3 : 1;
	uint8_t bit4 : 1;
	uint8_t bit5 : 1;
	uint8_t bit6 : 1;
	uint8_t bit7 : 1;
};

struct bits16_t
{
	uint8_t bit0 : 1;
	uint8_t bit1 : 1;
	uint8_t bit2 : 1;
	uint8_t bit3 : 1;
	uint8_t bit4 : 1;
	uint8_t bit5 : 1;
	uint8_t bit6 : 1;
	uint8_t bit7 : 1;
	uint8_t bit8 : 1;
	uint8_t bit9 : 1;
	uint8_t bit10 : 1;
	uint8_t bit11 : 1;
	uint8_t bit12 : 1;
	uint8_t bit13 : 1;
	uint8_t bit14 : 1;
	uint8_t bit15 : 1;
};

struct bits32_t
{
	uint8_t bit0  : 1;
	uint8_t bit1  : 1;
	uint8_t bit2  : 1;
	uint8_t bit3  : 1;
	uint8_t bit4  : 1;
	uint8_t bit5  : 1;
	uint8_t bit6  : 1;
	uint8_t bit7  : 1;
	uint8_t bit8  : 1;
	uint8_t bit9  : 1;
	uint8_t bit10 : 1;
	uint8_t bit11 : 1;
	uint8_t bit12 : 1;
	uint8_t bit13 : 1;
	uint8_t bit14 : 1;
	uint8_t bit15 : 1;
	uint8_t bit16 : 1;
	uint8_t bit17 : 1;
	uint8_t bit18 : 1;
	uint8_t bit19 : 1;
	uint8_t bit20 : 1;
	uint8_t bit21 : 1;
	uint8_t bit22 : 1;
	uint8_t bit23 : 1;
	uint8_t bit24 : 1;
	uint8_t bit25 : 1;
	uint8_t bit26 : 1;
	uint8_t bit27 : 1;
	uint8_t bit28 : 1;
	uint8_t bit29 : 1;
	uint8_t bit30 : 1;
	uint8_t bit31 : 1;
};

enum team_color_e
{
	TEAM_COLOR_BLUE = 0,
	TEAM_COLOR_YELLOW = 1
};

enum feedback_request_e
{
	FEEDBACK_TYPE_DEBUG = 0,
	FEEDBACK_TYPE_INFO = 1,
	FEEDBACK_TYPE_FATAL = 2,
	FEEDBACK_TYPE_CUSTOM = 3
};

enum shoot_type_e
{
	SHOOT_TYPE_DIRECT = 0,
	SHOOT_TYPE_CHIP = 1
};

struct robot_command_msg_t
{
	// commands
	struct  vector2f_t   velocity;
	union  float_32_u_t    omega;
	union  float_32_u_t    target_orientation;

	union  float_32_u_t    orientation;

	union  float_32_u_t    shoot_power;
	union  float_32_u_t    dribbler;
	union  float_32_u_t    servo;

	// debug
	uint8_t      beep;

	enum shoot_type_e shoot_type;
	enum feedback_request_e feedback;
	uint8_t            halt : 1;
	uint8_t            has_orientation : 1;
};

struct robot_config_msg_t
{
	union float_32_u_t      kp;
	union float_32_u_t      ki;
	union float_32_u_t      kd;
	union float_32_u_t      i_limit;

	union float_32_u_t      head_offset;

	struct vector3f_t   direct_coeffs;
	struct vector3f_t   chip_coeffs;
};

struct robot_matrix_msg_t
{
	struct vector3f_t matrix[4];
};

struct robot_feedback_msg_t
{
	union float_32_u_t     battery_voltage;
	union float_32_u_t     capacitor_voltage;

	union float_32_u_t     omega;
	union float_32_u_t     orientation;

	struct vector4f_t  motor_velocity;
	struct vector4f_t  motor_target;

	struct bits8_t  motor_fault;
	struct bits8_t button_status;

	uint8_t      fault : 1;
	uint8_t      ball_detected : 1;
	uint8_t      booster_enabled : 1;
	uint8_t      dribbler_connected : 1;
};

struct robot_feedback_custom_t
{
	uint32_t    length;
	void*     debug_dump;
};

#endif
