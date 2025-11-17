#ifndef TYPES_H
#define TYPES_H
#include <stdint.h>
#include <cfg.h>

// Might (likely) be used as the type for complex samples.
// Right now there is a dangerous mix of using (u)int16_t and uint32_t to handle complex samples
// which is far from ideal (and also violates aliasing rules without -fno-strict-aliasing).
// Putting them in a struct should simplify things
typedef struct __attribute__((packed)) {
    int16_t im;
    int16_t re;
 } int16imre_t;

typedef struct __attribute__((packed)) {
    int16_t re;
    int16_t im;
 }int16reim_t;

 typedef int16imre_t radarcube_t[NUM_TX_ANTENNAS][NUM_DOPPLER_CHIRPS][NUM_RX_ANTENNAS][NUM_RANGEBINS];
 typedef uint16_t detmatrix_t[NUM_RANGEBINS][NUM_DOPPLER_CHIRPS];

 struct detected_point{
    uint16_t range;
    uint16_t doppler;
};

#endif /* TYPES_H */