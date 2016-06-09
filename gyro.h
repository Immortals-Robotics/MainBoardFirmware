#ifndef GYRO_H
#define GYRO_H

#include <stdbool.h>

//initializes the gyroscope
bool init_gyro(void);
float get_gyro_omega(void);

#endif

