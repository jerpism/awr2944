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


// Use this when HWA is not doing the log2 post processing
static inline void sum_doppler_result_log2(detmatrix_t detmatrix, int rbin){
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


// Use this when HWA has done the log2 postprocessing
// Should be in Q11 assuming the manual isn't lying
static inline void sum_doppler_result(detmatrix_t detmatrix, int rbin){
    uint16_t *hwaout = (uint16_t*)(hwa_getaddr(gHwaHandle[0]) + 0x4000);
    for(int db = 0; db < NUM_DOPPLER_CHIRPS; ++db){
        for(int tx = 0; tx < NUM_TX_ANTENNAS; ++tx){
            for(int rx = 0; rx < NUM_RX_ANTENNAS; ++rx){
                detmatrix[rbin][db] += *(hwaout + (tx * NUM_RX_ANTENNAS * NUM_DOPPLER_CHIRPS) + (rx * NUM_DOPPLER_CHIRPS) + db);
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

    memset(out, 0, sizeof(detmatrix_t));

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


int dp_run_cfar(detmatrix_t detmatrix){
    int ret = 0;
    uint16_t *hwain = (uint16_t*)(hwa_getaddr(gHwaHandle[0]));
    DSSHWACCRegs *pregs = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
    uint16_t *dmatrix = &detmatrix[0][0];

    // First move our detmatrix to HWA input
    // currently it will contain 128 * 32 = 4096 data points 
    // which coincidentally is the max(+1) for CFARPEAKCNT so assume that limit won't be a problem for now
    for(int i = 0; i < NUM_DOPPLER_CHIRPS * NUM_RANGEBINS; ++i){
        *(hwain + i) =   *(dmatrix + i);
    }

    hwa_cfar_init(gHwaHandle[0], &hwa_cb);
    hwa_run(gHwaHandle[0]);
    SemaphoreP_pend(&hwaDoneSem, SystemP_WAIT_FOREVER);

    // 12 bit register
    ret = pregs->CFAR_PEAKCNT & 0x00000fff;

    return ret;
}


int dp_run_anglecfar(int16imre_t *angles, size_t len){
    int ret = 0;
    int16imre_t *hwain = (int16imre_t*)(hwa_getaddr(gHwaHandle[0]));
    uint32_t *hwaout = (uint32_t*)(hwa_getaddr(gHwaHandle[0]) + 0x1c000);

    DSSHWACCRegs *pregs = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;

    for(int i = 0; i < len; ++i){
        for(int j = 0; j < NUM_TX_ANTENNAS * NUM_RX_ANTENNAS; ++j){
            *(hwain + (i * NUM_TX_ANTENNAS * NUM_RX_ANTENNAS) + j) = *(angles + (i * NUM_TX_ANTENNAS * NUM_RX_ANTENNAS) + j);
        }
    }
    printf("len is %zu\r\n", len);

    hwa_anglecfar_init(gHwaHandle[0], &hwa_cb, len);
    HWA_reset(gHwaHandle[0]);
    hwa_run(gHwaHandle[0]);
    SemaphoreP_pend(&hwaDoneSem, SystemP_WAIT_FOREVER);

    ret = pregs->CFAR_PEAKCNT & 0x00000fff;

    return ret;
}


void dp_run_anglefft(radarcube_t data, struct detected_point *points, size_t len, int16imre_t *out){
    // First run a 2D FFT (doppler) on the detected ranges
    int16imre_t *hwain = (int16imre_t*)(hwa_getaddr(gHwaHandle[0]));
    int16imre_t *hwaout = (int16imre_t*)(hwa_getaddr(gHwaHandle[0]) + 0x4000);


    // yes this reimplements a lot of the doppler functionality 
    // but all of this will be replaced with EDMA soon enough hopefully
    for(int i = 0; i < len; ++i){
        hwa_doppler_init(gHwaHandle[0], &hwa_cb);

        move_doppler_to_hwa(data, points[i].range);

        HWA_reset(gHwaHandle[0]);
        hwa_run(gHwaHandle[0]);

        SemaphoreP_pend(&hwaDoneSem, SystemP_WAIT_FOREVER);

        // For now assume that the doppler bin will be 0 since we're measuring stationary objects
        // Essentially rearranges [TX][RC][DC] into [DC][TX][RX]
        for(int j = 0; j < NUM_TX_ANTENNAS; ++j){
            for(int k = 0; k < NUM_RX_ANTENNAS; ++k){
               *(hwain + (j * NUM_RX_ANTENNAS) + k) = *(hwaout + (j * NUM_RX_ANTENNAS * NUM_DOPPLER_CHIRPS) + (k * NUM_DOPPLER_CHIRPS));
            }
        }

        hwa_angle_init(gHwaHandle[0], &hwa_cb);

        HWA_reset(gHwaHandle[0]);
        hwa_run(gHwaHandle[0]);
        SemaphoreP_pend(&hwaDoneSem, SystemP_WAIT_FOREVER);

        for(int j = 0; j < NUM_RX_ANTENNAS * NUM_TX_ANTENNAS; ++j){
            *(out + (i * NUM_RX_ANTENNAS * NUM_TX_ANTENNAS) + j) = *(hwaout + j);
        }
    }

}