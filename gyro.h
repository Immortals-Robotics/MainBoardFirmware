#ifndef GYRO_H
#define GYRO_H

#include <stdbool.h>

struct gyro_data_t
{
    float x;
    #ifdef YZ_READ
    float y;
    float z;
    #endif
    float temp;
};

//initializes the gyroscope
bool init_gyro(void);
bool get_gyro_data(struct gyro_data_t* const result);

#endif

