#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <hwa.h>
#include <stdio.h>
#include <cfg.h>
#include <drivers/hwa.h>
#include <types.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

#define SQUARE_I16(x) (((int32_t)x) * ((int32_t)x))

static SemaphoreP_Object hwaDoneSem;
extern HWA_Handle gHwaHandle[1];

static void hwa_cb(uint32_t intrIdx, uint32_t paramSet, void *arg){
    SemaphoreP_post(&hwaDoneSem);
}


static inline uint16_t log2mag(int16imre_t val){
    return (uint16_t) (log2(sqrt(SQUARE_I16(val.re) + SQUARE_I16(val.im))));
}


static inline void sum_doppler_result(detmatrix_t detmatrix, int rbin){
    int16imre_t *hwaout = (int16imre_t*)(hwa_getaddr(gHwaHandle[0]) + 0x4000);
    // Calculates the sum of log2 magnitudes for the doppler fft output 
    // and inserts it into the [range][doppler] detection matrix 
    for(int db = 0; db < NUM_DOPPLER_CHIRPS; ++db){
        for(int tx = 0; tx < NUM_TX_ANTENNAS; ++tx){
            for(int rx = 0; rx < NUM_RX_ANTENNAS; ++rx){
                detmatrix[rbin][db] += log2mag(*(hwaout + (tx * NUM_RX_ANTENNAS * NUM_DOPPLER_CHIRPS) + (rx * NUM_DOPPLER_CHIRPS) + db));
            }
        }
    }

}

static void move_doppler_to_hwa(radarcube_t data, int rbin){
    int16imre_t *hwain = (int16imre_t*)(hwa_getaddr(gHwaHandle[0]));
    // Copies the data for a given rangebin to the HWA input memory (assuming M0 in this case)
    // The copied data will be in the form of [TX][RX][DC] which will also be the form for the HWA output.
    for(int tx = 0; tx < NUM_TX_ANTENNAS; ++tx){
        for(int rx = 0; rx < NUM_RX_ANTENNAS; ++rx){
            for(int dc = 0; dc < NUM_DOPPLER_CHIRPS; ++dc){
                *(hwain + (tx * NUM_RX_ANTENNAS * NUM_DOPPLER_CHIRPS) + (rx * NUM_DOPPLER_CHIRPS) + dc) = data[tx][dc][rx][rbin];
            }
        }
    }
}


void dp_run_doppler(radarcube_t data, detmatrix_t out){
    SemaphoreP_constructBinary(&hwaDoneSem, 0);

    hwa_doppler_init(gHwaHandle[0], &hwa_cb);

    for(int i = 0; i < NUM_RANGEBINS; ++i){
        // TODO: replace all of this with EDMA
        // First, move the data for a given rangebin to the HWA input and run it
        // this will change the order to [TX][RX][DC]
        move_doppler_to_hwa(data, i);
        HWA_reset(gHwaHandle[0]);

        hwa_run(gHwaHandle[0]);

        SemaphoreP_pend(&hwaDoneSem, SystemP_WAIT_FOREVER);

        // Once we have the result in HWA output, calculate a sum of log2 magnitudes across the virtual receivers
        // and insert it into the [range][doppler] detection matrix
        // this can then be input to CFAR
        sum_doppler_result(out, i);
    }
  
}