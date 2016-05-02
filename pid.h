
inline float P ( float current , float desired , float k )
{
    //k *= 1100 / ( 1100 - abs ( current ) );
    return k * ( desired - current );
}

typedef struct
{
    float dState[5];           //Last position input
    float dt;
    float iState;           //Integrator state
    float iMax , iMin;      //Maximum and minimum allowable integrator state

    float iGain ,           //integral     gain
          pGain ,           //proportional gain
          dGain ;           //derivative   gain

    float pTerm , dTerm , iTerm;

}SPid;

void initSPid ( SPid * pid )
{
    pid->dState[0] = 0;
    pid->dState[1] = 0;
    pid->dState[2] = 0;
    pid->dState[3] = 0;
    pid->dState[4] = 0;

    pid->iState = 0;

    pid->iMax = 996;
    pid->iMin = -996;

    pid->iGain = 0.358;
    pid->pGain = 16.4;
    pid->dGain = 0;
}

void update_PID_Vals(SPid * pid , float p , float i , float d , float imax )
{
    pid->iMax = imax;
    pid->iMin = -imax;
    pid->iGain = i;
    pid->pGain = p;
    pid->dGain = d;

}

inline float UpdatePID ( SPid * pid , float error , float position )
{
    pid->pTerm = pid->pGain * error;

    pid->iState += error;
    if ( pid->iState > pid->iMax )   pid->iState = pid->iMax;
    else if ( pid->iState < pid->iMin )   pid->iState = pid->iMin;

    pid->iTerm = pid->iGain * pid->iState;

    //pid->dState[4] = pid->dState[3];
    //pid->dState[3] = pid->dState[2];
    //pid->dState[2] = pid->dState[1];
    //pid->dTerm = pid->dGain * (position - pid->dState[1] );
    //pid->dState[1] = pid->dState[0];
    //pid->dState[0] = position;

    /*float y_avg = ( pid->dState[0] + pid->dState[1] + pid->dState[2] + pid->dState[3] + pid->dState[4] ) / 5.0;
    float x_avg = ( 0.0 + 1.0 + 2.0 + 3.0 + 4.0 ) / 5.0;
    pid->dt = 0;
    pid->dt += (0.0-x_avg)*( pid->dState[4]-y_avg);
    pid->dt += (1.0-x_avg)*( pid->dState[3]-y_avg);
    pid->dt += (2.0-x_avg)*( pid->dState[2]-y_avg);
    pid->dt += (3.0-x_avg)*( pid->dState[1]-y_avg);
    pid->dt += (4.0-x_avg)*( pid->dState[0]-y_avg);
    pid->dt /= ((0.0-x_avg)*(0.0-x_avg)) + ((1.0-x_avg)*(1.0-x_avg)) + ((2.0-x_avg)*(2.0-x_avg)) + ((3.0-x_avg)*(3.0-x_avg)) + ((4.0-x_avg)*(4.0-x_avg));

    //pid->dt *= 20.0;

    dTerm = pid->dGain* pid->dt;

    //pid->dt=y_avg;*/



    return pid->pTerm + pid->iTerm ;//- pid->dTerm;
}
