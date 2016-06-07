#ifndef HELPERS_H
#define HELPERS_H

#include <stdint.h>
#include <stdbool.h>

inline void set_bit_pos_u8 (uint8_t*  const var, const uint8_t bit_position) { (*var) |= 1 << bit_position; }
inline void set_bit_pos_u16(uint16_t* const var, const uint8_t bit_position) { (*var) |= 1 << bit_position; }
inline void set_bit_pos_u32(uint32_t* const var, const uint8_t bit_position) { (*var) |= 1 << bit_position; }

inline void clear_bit_pos_u8 (uint8_t*  const var, const uint8_t bit_position) { (*var) &= ~(1 << bit_position); }
inline void clear_bit_pos_u16(uint16_t* const var, const uint8_t bit_position) { (*var) &= ~(1 << bit_position); }
inline void clear_bit_pos_u32(uint32_t* const var, const uint8_t bit_position) { (*var) &= ~(1 << bit_position); }

inline void set_bit_mask_u8 (uint8_t*  const var, const uint8_t  bit_mask) { (*var) |= bit_mask; }
inline void set_bit_mask_u16(uint16_t* const var, const uint16_t bit_mask) { (*var) |= bit_mask; }
inline void set_bit_mask_u32(uint32_t* const var, const uint32_t bit_mask) { (*var) |= bit_mask; }

inline void clear_bit_mask_u8 (uint8_t*  const var, const uint8_t  bit_mask) { (*var) &= ~(bit_mask); }
inline void clear_bit_mask_u16(uint16_t* const var, const uint16_t bit_mask) { (*var) &= ~(bit_mask); }
inline void clear_bit_mask_u32(uint32_t* const var, const uint32_t bit_mask) { (*var) &= ~(bit_mask); }

inline bool get_bit_u8 (const uint8_t  var, const uint8_t bit_position) { return var & (1 << bit_position); }
inline bool get_bit_u16(const uint16_t var, const uint8_t bit_position) { return var & (1 << bit_position); }
inline bool get_bit_u32(const uint32_t var, const uint8_t bit_position) { return var & (1 << bit_position); }

inline bool get_bit_mask_u8 (const uint8_t  var, const uint8_t  bit_mask) { return var & bit_mask; }
inline bool get_bit_mask_u16(const uint16_t var, const uint16_t bit_mask) { return var & bit_mask; }
inline bool get_bit_mask_u32(const uint32_t var, const uint32_t bit_mask) { return var & bit_mask; }

inline int8_t  max_s8(const int8_t  a, const int8_t  b) { return a > b ? a : b; }
inline int8_t  min_s8(const int8_t  a, const int8_t  b) { return a < b ? a : b; }
inline uint8_t max_u8(const uint8_t a, const uint8_t b) { return a > b ? a : b; }
inline uint8_t min_u8(const uint8_t a, const uint8_t b) { return a < b ? a : b; }

inline int16_t  max_s16(const int16_t  a, const int16_t  b) { return a > b ? a : b; }
inline int16_t  min_s16(const int16_t  a, const int16_t  b) { return a < b ? a : b; }
inline uint16_t max_u16(const uint16_t a, const uint16_t b) { return a > b ? a : b; }
inline uint16_t min_u16(const uint16_t a, const uint16_t b) { return a < b ? a : b; }

inline int32_t  max_s32(const int32_t  a, const int32_t  b) { return a > b ? a : b; }
inline int32_t  min_s32(const int32_t  a, const int32_t  b) { return a < b ? a : b; }
inline uint32_t max_u32(const uint32_t a, const uint32_t b) { return a > b ? a : b; }
inline uint32_t min_u32(const uint32_t a, const uint32_t b) { return a < b ? a : b; }

inline float max_float(const float a, const float b) { return a > b ? a : b; }
inline float min_float(const float a, const float b) { return a < b ? a : b; }

inline int8_t sgn_01_inv_f(const float a) { return a >= 0 ? 0 : 1; }
inline int8_t sgn_32(const int32_t a) { return a >= 0 ? 1 : -1; }

#endif
