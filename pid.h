#ifndef PID_H
#define PID_H

struct pid_config_t
{
    float i_max;      //Maximum and minimum allowable integrator state

    float i_gain;     //integral     gain
    float p_gain;     //proportional gain
    float d_gain;     //derivative   gain
};

struct pid_state_t
{
	float prev_error; //Last error
    float i_state;    //Integrator state
};

float update_pid(struct pid_state_t* const state, const float error, const struct pid_config_t* const config);

#endif

