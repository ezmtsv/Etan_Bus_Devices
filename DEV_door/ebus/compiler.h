#ifndef _COMPILER_H_
#define _COMPILER_H_
#ifdef __cplusplus
extern "C"
{
#endif

#define _STATIC_INLINE   static inline __attribute__((always_inline))
#define _ASM __asm__ volatile
#define _RAMFUNC __attribute__((section(".ramfunc")))
#define _ALIGNED32 __attribute__((aligned(4)))
#define _PACKED __attribute__((packed))
#define _NORETURN __attribute__((noreturn))
#define _MAYUNUSED __attribute__((unused))
#define _STATIC_ASSERT(val, msg) _Static_assert(val, msg)

#if __BYTE_ORDER == __LITTLE_ENDIAN
    #define _CPUBE16(val) __builtin_bswap16(val)
    #define _BE16CPU(val) __builtin_bswap16(val)
    #define _CPUBE32(val) __builtin_bswap32(val)
    #define _BE32CPU(val) __builtin_bswap32(val)
#else
    #define _CPUBE16(val) val
    #define _BE16CPU(val) val
    #define _CPUBE32(val) val
    #define _BE32CPU(val) val
#endif

#ifndef CAT
#define __CAT(x,y) x ## y
#define CAT(x,y) __CAT(x,y)
#endif

#define _CAT3(x,y,z) x ## y ## z
#define CAT3(x,y,z) _CAT3(x,y,z)




#ifdef __cplusplus
}
#endif
#endif //_COMPILER_H_
