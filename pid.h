

struct pid_config
{
    float i_max;      //Maximum and minimum allowable integrator state

    float i_gain;           //integral     gain
    float p_gain;           //proportional gain
    float d_gain;           //derivative   gain
};

struct pid_state
{
    float d_state;        //Last position input
    float i_state;           //Integrator state
};

void init_pid_state ( struct pid_state* pid )
{
    pid->d_state = 0;
    pid->i_state = 0;
}

float update_pid ( struct pid_state* state , float error, struct pid_config* config)
{
    const float p_term = config->p_gain * error;

    state->i_state += error;
    if ( state->i_state > config->i_max )   state->i_state = config->i_max;
    else if ( state->i_state < -config->i_max )   state->i_state = -config->i_max;

    const float i_term = config->i_gain * state->i_state;

    return p_term + i_term ;//- dTerm;
}

struct pid_config motor_pid_config;
struct pid_config gyro_pid_config;

struct pid_state motor_pid[4];
struct pid_state gyro_pid;
