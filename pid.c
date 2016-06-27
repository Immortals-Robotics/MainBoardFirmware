#include "pid.h"

#include "helpers.h"

#define CONTROL_LOOP_DT 1.556f

float update_pid(struct pid_state_t* const state, const float error, const struct pid_config_t* const config)
{
    const float p_term = config->p_gain * error;

    state->i_state += error * CONTROL_LOOP_DT;
    state->i_state = min_float(config->i_max, max_float(-config->i_max, state->i_state));

    const float i_term = config->i_gain * state->i_state;

    const float d_term = config->d_gain * (error - state->prev_error) / CONTROL_LOOP_DT;

    state->prev_error = error;

    return p_term + i_term - d_term;
}
