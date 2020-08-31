/* Copyright 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef METAL__COMPILER_H
#define METAL__COMPILER_H

#define __METAL_DECLARE_VTABLE(type) extern const struct type type;

#define __METAL_DEFINE_VTABLE(type) const struct type type

#define __METAL_GET_FIELD(reg, mask)                                           \
    (((reg) & (mask)) / ((mask) & ~((mask) << 1)))

/* Set field with mask for a given value */
#define __METAL_SET_FIELD(reg, mask, val)                                      \
    (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define __METAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define __METAL_MAX(a, b) ((a) > (b) ? (a) : (b))

void _metal_trap(int ecode);

#define restrict __restrict__
#define metal_align(n) __attribute__((aligned(n)))
#define metal_weak __attribute__((weak))

#define METAL_PACKED_BEGIN
#define METAL_PACKED_END __attribute__((__packed__))

#endif
