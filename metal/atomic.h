/* Copyright 2019 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef METAL__ATOMIC_H
#define METAL__ATOMIC_H

#include <stdint.h>

#include <metal/compiler.h>

typedef volatile int32_t metal_atomic_t;

#define METAL_ATOMIC_DECLARE(name)                                             \
    __attribute((section(".data.atomics"))) metal_atomic_t name

#define _METAL_STORE_AMO_ACCESS_FAULT 7

/* This macro stores the memory address in mtval like a normal store/amo access
 * fault, triggers a trap, and then if execution returns, returns 0 as an
 * arbitrary choice */
#define _METAL_TRAP_AMO_ACCESS(addr)                                           \
    __asm__("csrw mtval, %[atomic]" ::[atomic] "r"(a));                        \
    _metal_trap(_METAL_STORE_AMO_ACCESS_FAULT);                                \
    return 0;

/*!
 * @brief Check if the platform supports atomic operations
 *
 * @return 1 if atomic operations are supported, 0 if not
 */
__inline__ int32_t metal_atomic_available(void) {
#ifdef __riscv_atomic
    return 1;
#else
    return 0;
#endif
}

/*!
 * @brief Atomically increment a metal_atomic_t and return its old value
 *
 * If atomics are not supported on the platform, this function will trap with
 * a Store/AMO access fault.
 *
 * @param a The pointer to the value to increment
 * @param increment the amount to increment the value
 *
 * @return The previous value of the metal_atomic_t
 */
__inline__ int32_t metal_atomic_add(metal_atomic_t *a, int32_t increment) {
#ifdef __riscv_atomic
    int32_t old;
    __asm__ volatile("amoadd.w %[old], %[increment], (%[atomic])"
                     : [old] "=r"(old)
                     : [increment] "r"(increment), [atomic] "r"(a)
                     : "memory");
    return old;
#else
    _METAL_TRAP_AMO_ACCESS(a);
#endif
}

/*!
 * @brief Atomically bitwise-AND a metal_atomic_t and return its old value
 *
 * If atomics are not supported on the platform, this function will trap with
 * a Store/AMO access fault.
 *
 * @param a The pointer to the value to bitwise-AND
 * @param mask the bitmask to AND
 *
 * @return The previous value of the metal_atomic_t
 */
__inline__ int32_t metal_atomic_and(metal_atomic_t *a, int32_t mask) {
#ifdef __riscv_atomic
    int32_t old;
    __asm__ volatile("amoand.w %[old], %[mask], (%[atomic])"
                     : [old] "=r"(old)
                     : [mask] "r"(mask), [atomic] "r"(a)
                     : "memory");
    return old;
#else
    _METAL_TRAP_AMO_ACCESS(a);
#endif
}

/*!
 * @brief Atomically bitwise-OR a metal_atomic_t and return its old value
 *
 * If atomics are not supported on the platform, this function will trap with
 * a Store/AMO access fault.
 *
 * @param a The pointer to the value to bitwise-OR
 * @param mask the bitmask to OR
 *
 * @return The previous value of the metal_atomic_t
 */
__inline__ int32_t metal_atomic_or(metal_atomic_t *a, int32_t mask) {
#ifdef __riscv_atomic
    int32_t old;
    __asm__ volatile("amoor.w %[old], %[mask], (%[atomic])"
                     : [old] "=r"(old)
                     : [mask] "r"(mask), [atomic] "r"(a)
                     : "memory");
    return old;
#else
    _METAL_TRAP_AMO_ACCESS(a);
#endif
}

/*!
 * @brief Atomically swap a metal_atomic_t and return its old value
 *
 * If atomics are not supported on the platform, this function will trap with
 * a Store/AMO access fault.
 *
 * @param a The pointer to the value to swap
 * @param new_value the value to store in the metal_atomic_t
 *
 * @return The previous value of the metal_atomic_t
 */
__inline__ int32_t metal_atomic_swap(metal_atomic_t *a, int32_t new_value) {
#ifdef __riscv_atomic
    int32_t old;
    __asm__ volatile("amoswap.w %[old], %[newval], (%[atomic])"
                     : [old] "=r"(old)
                     : [newval] "r"(new_value), [atomic] "r"(a)
                     : "memory");
    return old;
#else
    _METAL_TRAP_AMO_ACCESS(a);
#endif
}

/*!
 * @brief Atomically bitwise-XOR a metal_atomic_t and return its old value
 *
 * If atomics are not supported on the platform, this function will trap with
 * a Store/AMO access fault.
 *
 * @param a The pointer to the value to bitwise-XOR
 * @param mask the bitmask to XOR
 *
 * @return The previous value of the metal_atomic_t
 */
__inline__ int32_t metal_atomic_xor(metal_atomic_t *a, int32_t mask) {
#ifdef __riscv_atomic
    int32_t old;
    __asm__ volatile("amoxor.w %[old], %[mask], (%[atomic])"
                     : [old] "=r"(old)
                     : [mask] "r"(mask), [atomic] "r"(a)
                     : "memory");
    return old;
#else
    _METAL_TRAP_AMO_ACCESS(a);
#endif
}

/*!
 * @brief Atomically set the value of a memory location to the greater of
 * its current value or a value to compare it with.
 *
 * If atomics are not supported on the platform, this function will trap with
 * a Store/AMO access fault.
 *
 * @param a The pointer to the value to swap
 * @param compare the value to compare with the value in memory
 *
 * @return The previous value of the metal_atomic_t
 */
__inline__ int32_t metal_atomic_max(metal_atomic_t *a, int32_t compare) {
#ifdef __riscv_atomic
    int32_t old;
    __asm__ volatile("amomax.w %[old], %[compare], (%[atomic])"
                     : [old] "=r"(old)
                     : [compare] "r"(compare), [atomic] "r"(a)
                     : "memory");
    return old;
#else
    _METAL_TRAP_AMO_ACCESS(a);
#endif
}

/*!
 * @brief Atomically set the value of a memory location to the (unsigned)
 * greater of its current value or a value to compare it with.
 *
 * If atomics are not supported on the platform, this function will trap with
 * a Store/AMO access fault.
 *
 * @param a The pointer to the value to swap
 * @param compare the value to compare with the value in memory
 *
 * @return The previous value of the metal_atomic_t
 */
__inline__ uint32_t metal_atomic_max_u(metal_atomic_t *a, uint32_t compare) {
#ifdef __riscv_atomic
    int32_t old;
    __asm__ volatile("amomaxu.w %[old], %[compare], (%[atomic])"
                     : [old] "=r"(old)
                     : [compare] "r"(compare), [atomic] "r"(a)
                     : "memory");
    return old;
#else
    _METAL_TRAP_AMO_ACCESS(a);
#endif
}

/*!
 * @brief Atomically set the value of a memory location to the lesser of
 * its current value or a value to compare it with.
 *
 * If atomics are not supported on the platform, this function will trap with
 * a Store/AMO access fault.
 *
 * @param a The pointer to the value to swap
 * @param compare the value to compare with the value in memory
 *
 * @return The previous value of the metal_atomic_t
 */
__inline__ int32_t metal_atomic_min(metal_atomic_t *a, int32_t compare) {
#ifdef __riscv_atomic
    int32_t old;
    __asm__ volatile("amomin.w %[old], %[compare], (%[atomic])"
                     : [old] "=r"(old)
                     : [compare] "r"(compare), [atomic] "r"(a)
                     : "memory");
    return old;
#else
    _METAL_TRAP_AMO_ACCESS(a);
#endif
}

/*!
 * @brief Atomically set the value of a memory location to the (unsigned) lesser
 * of its current value or a value to compare it with.
 *
 * If atomics are not supported on the platform, this function will trap with
 * a Store/AMO access fault.
 *
 * @param a The pointer to the value to swap
 * @param compare the value to compare with the value in memory
 *
 * @return The previous value of the metal_atomic_t
 */
__inline__ uint32_t metal_atomic_min_u(metal_atomic_t *a, uint32_t compare) {
#ifdef __riscv_atomic
    int32_t old;
    __asm__ volatile("amominu.w %[old], %[compare], (%[atomic])"
                     : [old] "=r"(old)
                     : [compare] "r"(compare), [atomic] "r"(a)
                     : "memory");
    return old;
#else
    _METAL_TRAP_AMO_ACCESS(a);
#endif
}

typedef int atomic_flag;
typedef char atomic_char;
typedef unsigned char atomic_uchar;
typedef short atomic_short;
typedef unsigned short atomic_ushort;
typedef int atomic_int;
typedef unsigned int atomic_uint;
typedef atomic_uint atomic_uintptr_t;
typedef long atomic_long;
typedef unsigned long atomic_ulong;
typedef long long atomic_llong;
typedef unsigned long long atomic_ullong;

#define ATOMIC_FLAG_INIT	0
#define ATOMIC_VAR_INIT(VAL)	(VAL)

typedef enum {
	memory_order_relaxed,
	memory_order_consume,
	memory_order_acquire,
	memory_order_release,
	memory_order_acq_rel,
	memory_order_seq_cst,
} memory_order;

#define atomic_flag_test_and_set(FLAG)					\
	__sync_lock_test_and_set((FLAG), 1)
#define atomic_flag_test_and_set_explicit(FLAG, MO)			\
	atomic_flag_test_and_set(FLAG)
#define atomic_flag_clear(FLAG)						\
	__sync_lock_release((FLAG))
#define atomic_flag_clear_explicit(FLAG, MO)				\
	atomic_flag_clear(FLAG)
#define atomic_init(OBJ, VAL)						\
	do { *(OBJ) = (VAL); } while (0)
#define atomic_is_lock_free(OBJ)					\
	(sizeof(*(OBJ)) <= sizeof(long))
#define atomic_store(OBJ, VAL)						\
	do { *(OBJ) = (VAL); __sync_synchronize(); } while (0)
#define atomic_store_explicit(OBJ, VAL, MO)				\
	atomic_store((OBJ), (VAL))
#define atomic_load(OBJ)						\
	({ __sync_synchronize(); *(OBJ); })
#define atomic_load_explicit(OBJ, MO)					\
	atomic_load(OBJ)
#define atomic_exchange(OBJ, DES)					\
	({								\
		typeof(OBJ) obj = (OBJ);				\
		typeof(*obj) des = (DES);				\
		typeof(*obj) expval;					\
		typeof(*obj) oldval = atomic_load(obj);			\
		do {							\
			expval = oldval;				\
			oldval = __sync_val_compare_and_swap(		\
				obj, expval, des);			\
		} while (oldval != expval);				\
		oldval;							\
	})
#define atomic_exchange_explicit(OBJ, DES, MO)				\
	atomic_exchange((OBJ), (DES))
#define atomic_compare_exchange_strong(OBJ, EXP, DES)			\
	({								\
		typeof(OBJ) obj = (OBJ);				\
		typeof(EXP) exp = (EXP);				\
		typeof(*obj) expval = *exp;				\
		typeof(*obj) oldval = __sync_val_compare_and_swap(	\
			obj, expval, (DES));				\
		*exp = oldval;						\
		oldval == expval;					\
	})
#define atomic_compare_exchange_strong_explicit(OBJ, EXP, DES, MO)	\
	atomic_compare_exchange_strong((OBJ), (EXP), (DES))
#define atomic_compare_exchange_weak(OBJ, EXP, DES)			\
	atomic_compare_exchange_strong((OBJ), (EXP), (DES))
#define atomic_compare_exchange_weak_explicit(OBJ, EXP, DES, MO)	\
	atomic_compare_exchange_weak((OBJ), (EXP), (DES))
#define atomic_fetch_add(OBJ, VAL)					\
	__sync_fetch_and_add((OBJ), (VAL))
#define atomic_fetch_add_explicit(OBJ, VAL, MO)				\
	atomic_fetch_add((OBJ), (VAL))
#define atomic_fetch_sub(OBJ, VAL)					\
	__sync_fetch_and_sub((OBJ), (VAL))
#define atomic_fetch_sub_explicit(OBJ, VAL, MO)				\
	atomic_fetch_sub((OBJ), (VAL))
#define atomic_fetch_or(OBJ, VAL)					\
	__sync_fetch_and_or((OBJ), (VAL))
#define atomic_fetch_or_explicit(OBJ, VAL, MO)				\
	atomic_fetch_or((OBJ), (VAL))
#define atomic_fetch_xor(OBJ, VAL)					\
	__sync_fetch_and_xor((OBJ), (VAL))
#define atomic_fetch_xor_explicit(OBJ, VAL, MO)				\
	atomic_fetch_xor((OBJ), (VAL))
#define atomic_fetch_and(OBJ, VAL)					\
	__sync_fetch_and_and((OBJ), (VAL))
#define atomic_fetch_and_explicit(OBJ, VAL, MO)				\
	atomic_fetch_and((OBJ), (VAL))
#define atomic_thread_fence(MO)						\
	__sync_synchronize()
#define atomic_signal_fence(MO)						\
	__sync_synchronize()

#endif /* METAL__ATOMIC_H */
