#ifndef HELPERS_H
#define HELPERS_H

signed char max_s_char ( signed char a , signed char b ) { return a > b ? a : b; }
signed char min_s_char ( signed char a , signed char b ) { return a < b ? a : b; }
unsigned char max_u_char ( unsigned char a , unsigned char b ) { return a > b ? a : b; }
unsigned char min_u_char ( unsigned char a , unsigned char b ) { return a < b ? a : b; }

signed short max_s_short ( signed short a , signed short b ) { return a > b ? a : b; }
signed short min_s_short ( signed short a , signed short b ) { return a < b ? a : b; }
unsigned short max_u_short ( unsigned short a , unsigned short b ) { return a > b ? a : b; }
unsigned short min_u_short ( unsigned short a , unsigned short b ) { return a < b ? a : b; }

signed int max_s_int ( signed int a , signed int b ) { return a > b ? a : b; }
signed int min_s_int ( signed int a , signed int b ) { return a < b ? a : b; }
unsigned int max_u_int ( unsigned int a , unsigned int b ) { return a > b ? a : b; }
unsigned int min_u_int ( unsigned int a , unsigned int b ) { return a < b ? a : b; }

float max_float ( float a , float b ) { return a > b ? a : b; }
float min_float ( float a , float b ) { return a < b ? a : b; }

short sgn_01_inv ( float a ) { return a > 0 ? 0 : 1; }

#endif
