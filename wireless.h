#ifndef WIRELESS_H
#define WIRELESS_H

#include <stdbool.h>

void nrf_process(void);
void init_nrf(void);
void nrf_channel_search(bool blocking);
bool get_no_command_limit_reached(void);

#endif

