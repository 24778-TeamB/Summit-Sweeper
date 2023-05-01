#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdbool.h>

#define BIT0                   0x0001
#define BIT1                   0x0002
#define BIT2                   0x0004
#define BIT3                   0x0008
#define BIT4                   0x0010
#define BIT5                   0x0020
#define BIT6                   0x0040
#define BIT7                   0x0080
#define BIT8                   0x0100
#define BIT9                   0x0200
#define BITA                   0x0400
#define BITB                   0x0800
#define BITC                   0x1000
#define BITD                   0x2000
#define BITE                   0x4000
#define BITF                   0x8000

#define SET_BIT(BITFIELD, N)   (BITFIELD |= ((uint64_t)0x1 << N))
#define CLEAR_BIT(BITFIELD, N) (BITFIELD &= ~((uint64_t)0x1 << N))
#define CHECK_BIT(BITFIELD, N) ((BITFIELD >> N) & 0x1)

// The banks are considered groups of 16 bits
#define BANK0 0x00 // Bits 15:0
#define BANK1 0x10 // Bits 31:16
#define BANK2 0x20 // Bits 47:32
#define BANK3 0x30 // Bits 63:48

// BITS are not constricted to 16 bits only
// If you want to use 32 bits at a time, then only BANK0 and BANK2 should be
// used for BANK For 64 bits at a time, only BANK0 should be used for BANK
#define SET_BITS(BITFIELD, BITS, BANK) (BITFIELD |= ((uint64_t)(BITS) << BANK))
#define CLEAR_BITS(BITFIELD, BITS, BANK)                                       \
    (BITFIELD &= ~((uint64_t)(BITS) << BANK))
#define CHECK_BITS(BITFIELD, BITS, BANK)                                       \
    ((BITFIELD >> BANK) &                                                      \
     (uint64_t)(BITS)) // Checks if any of the given bits are set
#define STRICT_CHECK_BITS(BITFIELD, BITS, BANK)                                \
    (((BITFIELD >> BANK) & (uint64_t)(BITS)) ==                                \
     (uint64_t)(BITS)) // Checks if all of the given bits are set
#define GEN_BITMASK(LOW, HIGH)                  ((-1L << LOW) & ~((-1L << HIGH) << 1)                         //Generates a bit mask given the range from low to high

bool isInteger(const char *);
bool isFloat(const char *);

#endif
