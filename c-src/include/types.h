#ifndef TYPES_H
#define TYPES_H
#include <stdint.h>

// Might (likely) be used as the type for complex samples.
// Right now there is a dangerous mix of using (u)int16_t and uint32_t to handle complex samples
// which is far from ideal (and also violates aliasing rules without -fno-strict-aliasing).
// Putting them in a struct should simplify things
typedef struct __attribute__((packed)) {
#ifdef CPLX_FMT_IQSWAP
    int16_t im;
    int16_t re;
#else
    int16_t re;
    int16_t im;
#endif
 } int16reim_t;

#endif /* TYPES_H */