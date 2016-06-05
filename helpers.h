#ifndef HELPERS_H
#define HELPERS_H

void set_bit_pos_u8(uint8_t* const var, const uint8_t bit_position , const bool value)
{
    if (value)
    	(*var) |= 1 << bit_position;
    else
    	(*var) &= ~(1 << bit_position);
}

void set_bit_mask_u8(uint8_t* const var, const uint8_t bit_mask , const bool value)
{
    if (value)
    	(*var) |= bit_mask;
    else
    	(*var) &= ~(bit_mask);
}

bool get_bit_u8(const uint8_t var, const uint8_t bit_position) { return (var >> bit_position) & 0x01; /*just because i like hex numbers more :D*/ }

int8_t getSign(int var) { return var >= 0 ? 1 : -1; }


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

short sgn_01_inv_f ( float a ) { return a > 0 ? 0 : 1; }

#endif
