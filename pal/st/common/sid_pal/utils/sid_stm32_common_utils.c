/**
  ******************************************************************************
  * @file    sid_stm32_common_utils.c
  * @brief   Utility functions applicable to all ST-based Sidewalk apps
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <stddef.h>
#include <stdint.h>

#include <sid_stm32_common_defs.h>
#include <sid_stm32_common_utils.h>

#include <cmsis_compiler.h>

/* Private macro -------------------------------------------------------------*/

/* Architecture detection for GCC/ARMCC/IAR */
#if (!defined(__ARM_ARCH_6M__) || (__ARM_ARCH_6M__ == 0)) && (!defined(__ARM_ARCH_6S_M__) || (__ARM_ARCH_6S_M__ == 0)) && (!defined(__TARGET_ARCH_6S_M) || (__TARGET_ARCH_6S_M == 0))
#  define USE_ONLY_ALIGNED_ACCESS              (0)
#else
#  define USE_ONLY_ALIGNED_ACCESS              (1)
#endif /* ARMv6-M */

#define QUICK_MOD_4(_x_)                       ((uint32_t)(_x_) & (uint32_t)0x03u)           /*!< Fast equivalent to (x % 4) operation */
#define ALIGNMENT_INCONGRUENCE_CHECK(_a_, _b_) (((uint32_t)(_a_) ^ (uint32_t)(_b_)) & 0x03u) /*!< A quick check to ensure that two addresses are either aligned to the word boundary or unaligned by the same amount of bytes */

#if defined(__ICCARM__)
    /**
     * IAR Embedded Workbench
     * IAR does not expose a specific pragma to disable Loop Idiom Recognition. We use "optimize=no_unroll" which often disrupts the pattern matcher.
     */
#  define SID_NO_BUILTIN_OPTIMIZATION  _Pragma("optimize=no_unroll")

#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    /**
     * ARM Compiler 6 (Clang-based)
     * "no_builtin" tells Clang not to generate calls to standard library functions (like memcpy/memset) inside this function.
     */
#  define SID_NO_BUILTIN_OPTIMIZATION __attribute__((no_builtin))

#elif defined(__GNUC__)
    /**
     * GNU GCC
     * "no-tree-loop-distribute-patterns" disables the specific pass that converts loops into memset/memcpy calls.
     */
#  define SID_NO_BUILTIN_OPTIMIZATION __attribute__((optimize("no-tree-loop-distribute-patterns")))

#else
#  error "Your compiler is not supported by this implementation"
#endif

/* Private typedef -----------------------------------------------------------*/

/* Helper type to force unaligned-safe access */
typedef __PACKED_STRUCT {
    uint32_t v;
} unaligned_u32_t;

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED SID_NO_BUILTIN_OPTIMIZATION void SID_STM32_UTIL_fast_memcpy(void * _dst, const void * _src, const uint32_t size)
{
    register       uint8_t * _dst8          = (      uint8_t *)_dst;
    register const uint8_t * _src8          = (const uint8_t *)_src;
    register       uint32_t  remaining_size = size;

#if (USE_ONLY_ALIGNED_ACCESS == 0)
    /* Platform supports hardware unaligned access, more optimizations are possible */

    /* If size >= 4, use word copies */
    if (remaining_size >= sizeof(uint32_t))
    {
        /* 1. Blind (un)aligned write. This aligns the logical flow to the destination 4-byte boundary */
        *(uint32_t *)_dst = *(const uint32_t *)_src;

        /* Calculate offset to the next 4-byte aligned address, e.g. Address 0x...1 -> Need to advance 3 bytes. If the first write was aligned, need to advance by 4 bytes */
        const register uint32_t offset = (sizeof(uint32_t) - QUICK_MOD_4(_dst));

        /* Advance pointers and decrement remaining size */
        _dst8          += offset;
        _src8          += offset;
        remaining_size -= offset;

        /* 2. Main Aligned Loop */
        if (QUICK_MOD_4(_src8) == 0u)
        {
            /* Both DST and SRC are now aligned. The compiler is free to use LDRD, STRD, LDM, STM instructions. This creates burst transactions on the AHB bus */
            register       uint32_t * _dst32 = (      uint32_t *)(      void *)_dst8;
            register const uint32_t * _src32 = (const uint32_t *)(const void *)_src8;

            while (remaining_size >= sizeof(uint32_t))
            {
                *_dst32 = *_src32;
                ++_dst32;
                ++_src32;
                remaining_size -= sizeof(uint32_t);
            }

            /* Update byte pointers for the tail */
            _dst8 = (      uint8_t *)(      void *)_dst32;
            _src8 = (const uint8_t *)(const void *)_src32;
        }
        else
        {
            /* DST is aligned, but SRC is unaligned (Must prevent LDM/LDRD reads). Using the packed-struct cast to force single LDR instructions */
            register       uint32_t * _dst32 = (uint32_t *)(void *)_dst8;
            register const unaligned_u32_t * _src32_un = (const unaligned_u32_t *)(const void *)_src8;

            while (remaining_size >= sizeof(uint32_t))
            {
                *_dst32 = _src32_un->v;
                ++_dst32;
                ++_src32_un;
                remaining_size -= sizeof(uint32_t);
            }

            /* Update byte pointers for the tail */
            _dst8 = (      uint8_t *)(      void *)_dst32;
            _src8 = (const uint8_t *)(const void *)_src32_un;
        }
    }
#else
    /* Platform does not support unaligned access, strict access alignment shall be respected */

    /* We can only optimize if pointers are congruent (same (mis)alignment) */
    if ((remaining_size >= sizeof(uint32_t)) && (ALIGNMENT_INCONGRUENCE_CHECK(_dst, _src) == FALSE))
    {
        /* 1. Prefix Loop (Align to 4 bytes) */
        /* We strip bytes until _dst hits a word boundary */
        while (QUICK_MOD_4(_dst8) != 0u)
        {
#  if defined(__ICCARM__)
            /* Using cast to volatile to prevent IAR from replacing this loop with a call to the built-in memcpy */
            *(volatile uint8_t *)_dst8 = *_src8;
#  else
            *_dst8 = *_src8;
#  endif /* __ICCARM__ */
            ++_dst8;
            ++_src8;
            --remaining_size;
        }

        /* 2. Main Aligned Loop */
        /* Both DST and SRC are now aligned. Safe for M0+. */
        register       uint32_t * _dst32 = (      uint32_t *)(      void *)_dst8;
        register const uint32_t * _src32 = (const uint32_t *)(const void *)_src8;

        while (remaining_size >= sizeof(uint32_t))
        {
            *_dst32 = *_src32;
            ++_dst32;
            ++_src32;
            remaining_size -= sizeof(uint32_t);
        }

        /* Update byte pointers for the tail */
        _dst8 = (      uint8_t *)(      void *)_dst32;
        _src8 = (const uint8_t *)(const void *)_src32;
    }
    /* If Incongruent, we fall through to byte-copy (safest/only option on M0) */

    /* If optimization was skipped (M0+ Incongruent), remaining_size could be large. Go byte-by-byte here */
    while (remaining_size >= sizeof(uint32_t))
    {
#  if defined(__ICCARM__)
        /* Using cast to volatile to prevent IAR from replacing this loop with a call to the built-in memcpy */
        *(volatile uint8_t *)_dst8 = *_src8;
#  else
        *_dst8 = *_src8;
#  endif /* __ICCARM__ */
        ++_dst8;
        ++_src8;
        --remaining_size;
    }
#endif /* USE_ONLY_ALIGNED_ACCESS */

    /**
     * Handle the remaining unaligned bytes. Using a Switch Fallthrough:
     * 1. It prevents recursion (Compiler won't optimize this to memcpy).
     * 2. It is FASTER than a loop for small counts (0-3).
    */
    switch (remaining_size)
    {
        case 3u:
            *_dst8 = *_src8;
            ++_dst8;
            ++_src8;
            SID_STM32_JUSTIFY_FALLTHROUGH();

        case 2u:
            *_dst8 = *_src8;
            ++_dst8;
            ++_src8;
            SID_STM32_JUSTIFY_FALLTHROUGH();

        case 1u:
            *_dst8 = *_src8;
            SID_STM32_JUSTIFY_FALLTHROUGH();

        default:
            break;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED SID_NO_BUILTIN_OPTIMIZATION void SID_STM32_UTIL_fast_memset(void * _dst, const uint8_t c, const uint32_t size)
{
    register uint8_t * _dst8          = (      uint8_t *)_dst;
    register uint32_t  remaining_size = size;
    register uint32_t  c_word;

    /* Expand the byte 'cc' into a 32-bit word 'cccccccc' */
    c_word = (uint32_t)c * 0x01010101u; /* Using multiplication instead of bitwise operations since it's singe-cycle operation on Cortex-M cores */

    /* If size >= 4, use word copies */
    if (remaining_size >= sizeof(uint32_t))
    {
#if (USE_ONLY_ALIGNED_ACCESS == 0)
        /* Platform supports hardware unaligned access, more optimizations are possible */

        /* 1. Blind (un)aligned write. This aligns the logical flow to the destination 4-byte boundary */
        *(uint32_t *)_dst = c_word;

        /* Calculate offset to the next 4-byte aligned address, e.g. Address 0x...1 -> Need to advance 3 bytes. If the first write was aligned, need to advance by 4 bytes */
        const register uint32_t offset = (sizeof(uint32_t) - QUICK_MOD_4(_dst));

        /* Advance pointers and decrement remaining size */
        _dst8          += offset;
        remaining_size -= offset;
#else
        /* Platform does not support unaligned access, strict access alignment shall be respected */

        /* 1. Prefix Loop (Align to 4 bytes) */
        /* We strip bytes until _dst hits a word boundary */
        while (QUICK_MOD_4(_dst8) != 0u)
        {
#  if defined(__ICCARM__)
            /* Using cast to volatile to prevent IAR from replacing this loop with a call to the built-in memset */
            *(volatile uint8_t *)_dst8 = c;
#  else
            *_dst8 = c;
#  endif /* __ICCARM__ */
            ++_dst8;
            --remaining_size;
        }
#endif /* USE_ONLY_ALIGNED_ACCESS */

        /* 2. Main Aligned Loop */
        /* DST is now aligned. The compiler is free to use STRD, STM instructions. This creates burst transactions on the AHB bus */
        register uint32_t * _dst32 = (uint32_t *)(void *)_dst8;

        while (remaining_size >= sizeof(uint32_t))
        {
            *_dst32 = c_word;
            ++_dst32;
            remaining_size -= sizeof(uint32_t);
        }

        /* Update byte pointer for the tail */
        _dst8 = (uint8_t *)(void *)_dst32;
    }

    /**
     * Handle the remaining unaligned bytes. Using a Switch Fallthrough:
     * 1. It prevents recursion (Compiler won't optimize this to memset).
     * 2. It is FASTER than a loop for small counts (0-3).
    */
    switch (remaining_size)
    {
        case 3u:
            *_dst8 = c;
            ++_dst8;
            SID_STM32_JUSTIFY_FALLTHROUGH();

        case 2u:
            *_dst8 = c;
            ++_dst8;
            SID_STM32_JUSTIFY_FALLTHROUGH();

        case 1u:
            *_dst8 = c;
            SID_STM32_JUSTIFY_FALLTHROUGH();

        default:
            break;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t SID_STM32_UTIL_fast_memcmp(const void * const _a, const void * const _b, const uint32_t size)
{
    register const uint8_t * _a8            = (const uint8_t *)_a;
    register const uint8_t * _b8            = (const uint8_t *)_b;
    register       uint32_t  remaining_size = size;

#if (USE_ONLY_ALIGNED_ACCESS == 0)
    /* Platform supports hardware unaligned access, more optimizations are possible */

    /* If size >= 4, use word copies */
    if (remaining_size >= sizeof(uint32_t))
    {
        /* 1. Blind (un)aligned comparison. This aligns the logical flow to the _a 4-byte boundary */
        if (*(const uint32_t *)_a != *(const uint32_t *)_b)
        {
            return (uint32_t)(uintptr_t)_a;
        }

        /* Calculate offset to the next 4-byte aligned address, e.g. Address 0x...1 -> Need to advance 3 bytes. If the first write was aligned, need to advance by 4 bytes */
        const register uint32_t offset = (sizeof(uint32_t) - QUICK_MOD_4(_a));

        /* Advance pointers and decrement remaining size */
        _a8            += offset;
        _b8            += offset;
        remaining_size -= offset;

        /* 2. Main Aligned Loop */
        if (QUICK_MOD_4(_b8) == 0u)
        {
            /* Both _a and _b are now aligned. The compiler is free to use LDRD, LDM instructions. This creates burst transactions on the AHB bus */
            register const uint32_t * _a32 = (const uint32_t *)(const void *)_a8;
            register const uint32_t * _b32 = (const uint32_t *)(const void *)_b8;

            while (remaining_size >= sizeof(uint32_t))
            {
                if (*_a32 != *_b32)
                {
                    return (uint32_t)(uintptr_t)(const void *)_a32;
                }
                ++_a32;
                ++_b32;
                remaining_size -= sizeof(uint32_t);
            }

            /* Update byte pointers for the tail */
            _a8 = (const uint8_t *)(const void *)_a32;
            _b8 = (const uint8_t *)(const void *)_b32;
        }
        else
        {
            /* DST is aligned, but SRC is unaligned (Must prevent LDM/LDRD reads). Using the packed-struct cast to force single LDR instructions */
            register const uint32_t *        _a32 = (uint32_t *)(void *)_a8;
            register const unaligned_u32_t * _b32_un = (const unaligned_u32_t *)(const void *)_b8;

            while (remaining_size >= sizeof(uint32_t))
            {
                if (*_a32 != _b32_un->v)
                {
                    return (uint32_t)(uintptr_t)(const void *)_a32;
                }
                ++_a32;
                ++_b32_un;
                remaining_size -= sizeof(uint32_t);
            }

            /* Update byte pointers for the tail */
            _a8 = (const uint8_t *)(const void *)_a32;
            _b8 = (const uint8_t *)(const void *)_b32_un;
        }
    }
#else
    /* Platform does not support unaligned access, strict access alignment shall be respected */

    /* We can only optimize if pointers are congruent (same (mis)alignment) */
    if ((remaining_size >= sizeof(uint32_t)) && (ALIGNMENT_INCONGRUENCE_CHECK(_a, _b) == FALSE))
    {
        /* 1. Prefix Loop (Align to 4 bytes) */
        /* We strip bytes until _a hits a word boundary */
        while (QUICK_MOD_4(_a8) != 0u)
        {
            if (*_a8 != *_b8)
            {
                return (uint32_t)(uintptr_t)(const void *)_a8;
            }
            ++_a8;
            ++_b8;
            --remaining_size;
        }

        /* 2. Main Aligned Loop */
        /* Both _a and _b are now aligned. Safe for M0+. */
        register const uint32_t * _a32 = (const uint32_t *)(const void *)_a8;
        register const uint32_t * _b32 = (const uint32_t *)(const void *)_b8;

        while (remaining_size >= sizeof(uint32_t))
        {
            if (*_a32 != *_b32)
            {
                return (uint32_t)(uintptr_t)(const void *)_a32;
            }
            ++_a32;
            ++_b32;
            remaining_size -= sizeof(uint32_t);
        }

        /* Update byte pointers for the tail */
        _a8 = (const uint8_t *)(const void *)_a32;
        _b8 = (const uint8_t *)(const void *)_b32;
    }
    /* If Incongruent, we fall through to byte-by-byte comparison (safest/only option on M0) */

    /* If optimization was skipped (M0+ Incongruent), remaining_size could be large. Go byte-by-byte here */
    while (remaining_size >= sizeof(uint32_t))
    {
#  if defined(__ICCARM__)
        /* Using cast to volatile to prevent IAR from replacing this loop with a call to the built-in memcpy */
        if(*(volatile uint8_t *)_a8 != *_b8)
        {
            return (uint32_t)(uintptr_t)(const void *)_a8;
        }
#  else
        if (*_a8 != *_b8)
        {
            return (uint32_t)(uintptr_t)(const void *)_a8;
        }
#  endif /* __ICCARM__ */
        ++_a8;
        ++_b8;
        --remaining_size;
    }
#endif /* USE_ONLY_ALIGNED_ACCESS */

    /**
     * Handle the remaining unaligned bytes. Using a Switch Fallthrough:
     * 1. It prevents recursion (Compiler won't optimize this to memcpy).
     * 2. It is FASTER than a loop for small counts (0-3).
    */
    switch (remaining_size)
    {
        case 3u:
            if (*_a8 != *_b8)
            {
                return (uint32_t)(uintptr_t)(const void *)_a8;
            }
            ++_a8;
            ++_b8;
            SID_STM32_JUSTIFY_FALLTHROUGH();

        case 2u:
            if (*_a8 != *_b8)
            {
                return (uint32_t)(uintptr_t)(const void *)_a8;
            }
            ++_a8;
            ++_b8;
            SID_STM32_JUSTIFY_FALLTHROUGH();

        case 1u:
            if (*_a8 != *_b8)
            {
                return (uint32_t)(uintptr_t)(const void *)_a8;
            }
            SID_STM32_JUSTIFY_FALLTHROUGH();

        default:
            break;
    }

    /* Full match if this point is reached */
    return 0u;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(void *, memcpy(void * _dst, const void * _src, const size_t size))
{
    /**
     * IMPORTANT NOTE: SID_STM32_UTIL_fast_memcpy may access certain addresses twice, which makes it incompatible with hardware FIFO buffers.
     * While memcpy on STM32 platform is mainly used for memory-to-memory copies, you need to ensure that no hardware FIFO is accessed with
     * performance-optimized function if you decide to replace the standard memcpy implementation with it. If you need to use memcpy with a
     * hardware FIFO or other location that expects only sequential reads or writes, use the SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION()
     * helper macro to call the original memcpy implementation.
     */
    SID_STM32_UTIL_fast_memcpy(_dst, _src, (uint32_t)size);
    return _dst;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(void *, memset(void * _dst, const int c, const size_t size))
{
    /**
     * IMPORTANT NOTE: SID_STM32_UTIL_fast_memset may access certain addresses twice, which makes it incompatible with hardware FIFO buffers.
     * While memset on STM32 platform is mainly used for initializing RAM objects, you need to ensure that no hardware FIFO is accessed with
     * performance-optimized function if you decide to replace the standard memset implementation with it. If you need to use memset with a
     * hardware FIFO or other location that expects only sequential writes, use the SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION() helper
     * macro to call the original memset implementation.
     */
    SID_STM32_UTIL_fast_memset(_dst, (uint8_t)c, (uint32_t)size);
    return _dst;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(int, memcmp(const void * const _a, const void * const _b, const size_t size))
{
    /**
     * IMPORTANT NOTE: SID_STM32_UTIL_fast_memcmp may access certain addresses twice, which makes it incompatible with hardware FIFO buffers.
     * While memcmp on STM32 platform is mainly used for RAM and flash comparisons, you need to ensure that no hardware FIFO is accessed with
     * performance-optimized function if you decide to replace the standard memcmp implementation with it. If you need to use memcmp with a
     * hardware FIFO or other location that expects only sequential writes, use the SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION() helper
     * macro to call the original memset implementation.
     */

    /* Run fast comparison to determine approximate location of the mismatch */
    register const void * _a_mismatch_loc = (void *)SID_STM32_UTIL_fast_memcmp(_a, _b, (uint32_t)size);

    /* Check for match */
    if (_a_mismatch_loc == NULL)
    {
        /* Full match */
        return 0;
    }

    /* Handle mismatch. We need to find the specific byte difference to satisfy the standard since SID_STM32_UTIL_fast_memcmp simply returns a non-zero value on mismatch */
    register const uint8_t * p1 = (const uint8_t *)_a_mismatch_loc;

    /* Calculate where p2 is based on the offset from _a */
    register uintptr_t       offset = (uintptr_t)p1 - (uintptr_t)_a;
    register const uint8_t * p2     = (const uint8_t *)_b + offset;

    /* We know the mismatch is here, or in the next 3 bytes (if it was a word fail). Since SID_STM32_UTIL_fast_memcmp() returned a mismatch, this loop IS GUARANTEED to terminate */
    while (1)
    {
        if (*p1 != *p2)
        {
            /* Standard requirement: return difference of unsigned chars */
            return (*p1 < *p2) ? -1 : 1;
        }
        p1++;
        p2++;
    }
}
