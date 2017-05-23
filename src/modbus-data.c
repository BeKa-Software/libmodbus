/*
 * Copyright © 2010-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdlib.h>
#ifndef _MSC_VER
#include <stdint.h>
#else
#include "stdint.h"
#endif
#include <string.h>
#include <assert.h>

#include "modbus.h"
#include <byteswap.h>


#if defined(__GNUC__)
#  define GCC_VERSION (__GNUC__ * 100 + __GNUC_MINOR__ * 10)
#  if GCC_VERSION >= 430
// Since GCC >= 4.30, GCC provides __builtin_bswapXX() alternatives so we switch to them
#    undef bswap_32
#    define bswap_32 __builtin_bswap32
#  endif
#endif

#if defined(__GNUC__)
#  define GCC_VERSION (__GNUC__ * 100 + __GNUC_MINOR__ * 10)
#  if GCC_VERSION >= 430
// Since GCC >= 4.30, GCC provides __builtin_bswapXX() alternatives so we switch to them
#    undef bswap_64
#    define bswap_64 __builtin_bswap64
#  endif
#endif

#if !defined(bswap_32)

#if !defined(bswap_16)
#   warning "Fallback on C functions for bswap_16"
static inline uint16_t bswap_16(uint16_t x)
{
    return (x >> 8) | (x << 8);
}
#endif

#   warning "Fallback on C functions for bswap_32"
static inline uint32_t bswap_32(uint32_t x)
{
    return (bswap_16(x & 0xffff) << 16) | (bswap_16(x >> 16));
}
#endif

/* Sets many bits from a single byte value (all 8 bits of the byte value are
   set) */
void modbus_set_bits_from_byte(uint8_t *dest, int index, const uint8_t value)
{
    int i;

    for (i=0; i < 8; i++) {
        dest[index+i] = (value & (1 << i)) ? 1 : 0;
    }
}

/* Sets many bits from a table of bytes (only the bits between index and
   index + nb_bits are set) */
void modbus_set_bits_from_bytes(uint8_t *dest, int index, unsigned int nb_bits,
                                const uint8_t *tab_byte)
{
    unsigned int i;
    int shift = 0;

    for (i = index; i < index + nb_bits; i++) {
        dest[i] = tab_byte[(i - index) / 8] & (1 << shift) ? 1 : 0;
        /* gcc doesn't like: shift = (++shift) % 8; */
        shift++;
        shift %= 8;
    }
}

/* Gets the byte value from many bits.
   To obtain a full byte, set nb_bits to 8. */
uint8_t modbus_get_byte_from_bits(const uint8_t *src, int index,
                                  unsigned int nb_bits)
{
    unsigned int i;
    uint8_t value = 0;

    if (nb_bits > 8) {
        /* Assert is ignored if NDEBUG is set */
        assert(nb_bits < 8);
        nb_bits = 8;
    }

    for (i=0; i < nb_bits; i++) {
        value |= (src[index+i] << i);
    }

    return value;
}

/* Get a float from 4 bytes in Modbus format (ABCD) */
float modbus_get_float(const uint16_t *src)
{
    float f;
    uint32_t i;

    // The following instruction may cause SIGILL (signal 4) or signal 7 when the pointer address is not be 2-byte aligned
    // i = (((uint32_t)src[1]) << 16) + src[0];
    // Thus we have to use the following workaround:
    uint16_t left, right;
    memcpy(&left, src + 1, sizeof(uint16_t));
    memcpy(&right, src, sizeof(uint16_t));
    i = (((uint32_t)left) << 16) + right;

    memcpy(&f, &i, sizeof(float));

    return f;
}

/* Get a float from 4 bytes in inversed Modbus format (DCBA) */
float modbus_get_float_dcba(const uint16_t *src)
{
    float f;
    uint32_t i;
    // The following instruction may cause SIGILL (signal 4) or signal 7 when the pointer address is not be 2-byte aligned
    // i = (((uint32_t)src[1]) << 16) + src[0];
    // Thus we have to use the following workaround:
    uint16_t left, right;
    memcpy(&left, src + 1, sizeof(uint16_t));
    memcpy(&right, src, sizeof(uint16_t));
    i = (((uint32_t)left) << 16) + right;

    i = bswap_32(i);
    memcpy(&f, &i, sizeof(float));

    return f;
}

/* Set a float to 4 bytes in Modbus format (ABCD) */
void modbus_set_float(float f, uint16_t *dest)
{
    uint32_t i;

    memcpy(&i, &f, sizeof(uint32_t));
    // The following instructions may cause SIGILL (signal 4) or signal 7 when the pointer address is not be 2-byte aligned
    // dest[0] = (uint16_t)i;
    // dest[1] = (uint16_t)(i >> 16);
    // Thus we have to use the following workaround:
    uint16_t first, second;
    first = (uint16_t)i;
    second = (uint16_t)(i >> 16);
    memcpy(dest, &first, sizeof(uint16_t));
    memcpy(dest + 1, &second, sizeof(uint16_t));
}

/* Set a float to 4 bytes in inversed Modbus format (DCBA) */
void modbus_set_float_dcba(float f, uint16_t *dest)
{
    uint32_t i;

    memcpy(&i, &f, sizeof(uint32_t));
    i = bswap_32(i);
    // The following instructions may cause SIGILL (signal 4) or signal 7 when the pointer address is not be 2-byte aligned
    // dest[0] = (uint16_t)i;
    // dest[1] = (uint16_t)(i >> 16);
    // Thus we have to use the following workaround:
    uint16_t first, second;
    first = (uint16_t)i;
    second = (uint16_t)(i >> 16);
    memcpy(dest, &first, sizeof(uint16_t));
    memcpy(dest + 1, &second, sizeof(uint16_t));
}

double modbus_get_double(const uint16_t *src)
{
    double f;
    uint64_t i;

    // The following instructions may cause SIGILL (signal 4) or signal 7 when the pointer address is not be 4-byte aligned
    // i = (((uint64_t)src[3]) << 48) +  (((uint64_t)src[2]) << 32) + (((uint64_t)src[1]) << 16) + src[0];
    // Thus we have to use the following workaround:
    uint16_t v1, v2, v3, v4;
    memcpy(&v1, src + 3, sizeof(uint16_t));
    memcpy(&v2, src + 2, sizeof(uint16_t));
    memcpy(&v3, src + 1, sizeof(uint16_t));
    memcpy(&v4, src, sizeof(uint16_t));
    i = (((uint64_t)v1) << 48) + (((uint64_t)v2) << 32) + (((uint64_t)v3) << 16) + v4;

    memcpy(&f, &i, sizeof(uint64_t));

    return f;
}

double modbus_get_double_dcba(const uint16_t *src)
{
    double f;
    uint32_t i;

    // The following instructions may cause SIGILL (signal 4) or signal 7 when the pointer address is not be 4-byte aligned
    // i = bswap_64(((uint64_t)src[3]) << 48) +  (((uint64_t)src[2]) << 32) + (((uint64_t)src[1]) << 16) + src[0];
    // Thus we have to use the following workaround:
    uint16_t v1, v2, v3, v4;
    memcpy(&v1, src + 3, sizeof(uint16_t));
    memcpy(&v2, src + 2, sizeof(uint16_t));
    memcpy(&v3, src + 1, sizeof(uint16_t));
    memcpy(&v4, src, sizeof(uint16_t));
    i = bswap_64(((uint64_t)v1) << 48) +  (((uint64_t)v2) << 32) + (((uint64_t)v3) << 16) + v4;

    memcpy(&f, &i, sizeof(double));

    return f;
}

void modbus_set_double(double f, uint16_t *dest)
{
    uint64_t i;

    memcpy(&i, &f, sizeof(uint64_t));
    // The following instructions may cause SIGILL (signal 4) or signal 7 when the pointer address is not be 2-byte aligned
    // dest[0] = (uint16_t)i;
    // dest[1] = (uint16_t)(i >> 16);
    // dest[2] = (uint16_t)(i >> 32);
    // dest[3] = (uint16_t)(i >> 48);
    // Thus we have to use the following workaround:
    uint16_t v1, v2, v3, v4;
    v1 = (uint16_t)i;
    v2 = (uint16_t)(i >> 16);
    v3 = (uint16_t)(i >> 32);
    v4 = (uint16_t)(i >> 48);
    memcpy(dest, &v1, sizeof(uint16_t));
    memcpy(dest + 1, &v2, sizeof(uint16_t));
    memcpy(dest + 2, &v3, sizeof(uint16_t));
    memcpy(dest + 3, &v4, sizeof(uint16_t));

    return;
    memcpy(dest, &f, sizeof(double));
}

void modbus_set_double_dcba(double f, uint16_t *dest)
{
    uint64_t i;

    memcpy(&i, &f, sizeof(uint64_t));
    i = bswap_64(i);
    // The following instructions may cause SIGILL (signal 4) or signal 7 when the pointer address is not be 2-byte aligned
    // dest[0] = (uint16_t)i & 0xFF;
    // dest[1] = (uint16_t)(i >> 16) & 0xFF;
    // dest[2] = (uint16_t)(i >> 32) & 0xFF;
    // dest[3] = (uint16_t)(i >> 48) & 0xFF;
    // Thus we have to use the following workaround:
    uint16_t v1, v2, v3, v4;
    v1 = (uint16_t)i & 0xFF;
    v2 = (uint16_t)(i >> 16) & 0xFF;
    v3 = (uint16_t)(i >> 32) & 0xFF;
    v4 = (uint16_t)(i >> 48) & 0xFF;
    memcpy(dest, &v1, sizeof(uint16_t));
    memcpy(dest + 1, &v2, sizeof(uint16_t));
    memcpy(dest + 2, &v3, sizeof(uint16_t));
    memcpy(dest + 3, &v4, sizeof(uint16_t));
}
