//-----------------------------------------------------------------------------
// Copyright (C) 2008-2014 bla <blapost@gmail.com>
// Copyright (C) Proxmark3 contributors. See AUTHORS.md for details.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// See LICENSE.txt for the text of the license.
//-----------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

struct Crypto1State {uint32_t odd, even;};
void crypto1_init(struct Crypto1State *state, uint64_t key);
void crypto1_deinit(struct Crypto1State *);
void crypto1_get_lfsr(struct Crypto1State *, uint64_t *);
uint8_t crypto1_bit(struct Crypto1State *, uint8_t, int);
uint8_t crypto1_byte(struct Crypto1State *, uint8_t, int);
uint32_t crypto1_word(struct Crypto1State *, uint32_t, int);
uint32_t prng_successor(uint32_t x, uint32_t n);

#define LF_POLY_ODD (0x29CE5C)
#define LF_POLY_EVEN (0x870804)
#define BIT(x, n) ((x) >> (n) & 1)
#define BEBIT(x, n) BIT(x, (n) ^ 24)
static inline int filter(uint32_t const x) {
    uint32_t f;

    f  = 0xf22c0 >> (x       & 0xf) & 16;
    f |= 0x6c9c0 >> (x >>  4 & 0xf) &  8;
    f |= 0x3c8b0 >> (x >>  8 & 0xf) &  4;
    f |= 0x1e458 >> (x >> 12 & 0xf) &  2;
    f |= 0x0d938 >> (x >> 16 & 0xf) &  1;
    return BIT(0xEC57E80A, f);
}
