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
 

#define P_FA 0.09f

extern HWA_Handle gHwaHandle[1];

// TODO: REMOVE THIS ONCE IT'S UNUSED
void uart_dump_samples(void *buff, size_t n);


// This will be padded to 8 bytes
// so there's an extra 3 bytes to play around with if required
// also possible to turn this into a bitfield since we realistically won't have
// more than 128 range/doppler bins
struct detected_point{
    uint8_t rx;
    uint8_t chirp;
    uint8_t rbin;
    uint8_t dbin;
    uint8_t abin;
};


// range detections for [rx][rangebin]
// the counter for each rangebin is incremented if the CFAR result for a chirp detects something at it 
static uint8_t range_detected[4][128];
static uint16_t absbuff[128];
static uint32_t test[128];

// Input/output MUST be 4 byte aligned
void calc_doppler_fft(HWA_Handle hwahandle, void *in, void *out){

}


void calc_abs_vals(int16imre_t *in, uint16_t *out, size_t n){
    for(size_t i = 0; i < n; i++){
        out[i] = (uint16_t)sqrt(SQUARE_I16(in[i].re) + SQUARE_I16(in[i].im));
    }
}


void reflect_pad(const float* input, float* padded, int n, int pad) {
    for (int i = 0; i < pad; ++i)
        padded[i] = input[pad - i];
    for (int i = 0; i < n; ++i)
        padded[pad + i] = input[i];
    for (int i = 0; i < pad; ++i)
        padded[pad + n + i] = input[n - 2 - i];
}


void convolve_1d(const float* a, int n, const float* b, int m, float* output) {
    int radius = m / 2;
    int padded_len = n + 2 * radius;
    float* a_padded = (float*)malloc(sizeof(float) * padded_len);
    
    reflect_pad(a, a_padded, n, radius);

    for (int i = 0; i < n; ++i) {
        output[i] = 0;
        for (int j = 0; j < m; ++j) {
            output[i] += a_padded[i + j] * b[j];
        }
    }

    free(a_padded);
}


// This is only really set up to deal with n up to 128
void dp_cfar(uint8_t rx, uint8_t chirp, int16imre_t *data, uint8_t n) {
    float p_fa = P_FA;
    float threshold[128];
    int guard_len = 0;
    int train_len = 10;
    int k_len = 1 + 2*guard_len + 2*train_len;
    float cfar_kernel[k_len];
    float noise_level[128];

    float signal[128];

    if(n > 128){
        DebugP_logError("n is %u, more than 128\r\n",n);
        return;
    }


    calc_abs_vals(data, absbuff, n);


    for(int i = 0; i < n; ++i){
        signal[i] = ((float)(absbuff[i]));
    }

    for (int i = 0; i < k_len; ++i)
        cfar_kernel[i] = 1.0 / (2 * train_len);
    for (int i = train_len; i < train_len + 2*guard_len + 1; ++i)
        cfar_kernel[i] = 0;

    float a = train_len * (pow(p_fa, -1.0 / train_len) - 1.0);
    convolve_1d(signal, n, cfar_kernel, k_len, noise_level);
    for (int i = 0; i < n; ++i) {
        threshold[i] = (noise_level[i] + 1) * (a - 1);
        if (signal[i] > threshold[i]){
            range_detected[rx][i]++;
        }
    }

}


// data is a pointer to the already processed range data in DSS_L3
// rx is the number of receivers enabled
// chirps is the number of chirps (and thus dopper bins) 
// rbins is the number of rangebins
void process_data(int16imre_t *data, uint8_t rx_cnt, uint16_t chirps, uint8_t rbins){
    // Reset detections
    memset(range_detected, 0, sizeof(range_detected));

    // First, calculate the CFAR result for ranges
    // NOTE: it might faster to swap this around to i = chirp and j = rx
    // so we can go through data without jumping around (might be better for cache?)
    // It could also be advantageous if we want to "vote" on detections
    // e.g. check if the same chirp's rbin was detected on all 4 or a majority to count it
    for(uint8_t i = 0; i < rx_cnt; ++i){
        for(uint8_t j = 0; j < chirps; ++j){
            int16imre_t *dp = data + (i * rbins) + (j * NUM_RX_ANTENNAS * rbins);
            dp_cfar(i, j, dp, rbins);
        }
    }

    DebugP_log("Done with cfar\r\n");

    // Next, figure out which rangebins we should calculate the doppler for
    // TODO: 20 is used as a placeholder value for threshold here, this can and should be changed
    // TODO: add handling for 4 rx here
    int16imre_t *hwain = (int16imre_t*)hwa_getaddr(gHwaHandle[0]);
    uint8_t threshold = 10;
    uint8_t cnt = 0;

    // TODO: actually make this work with multiple rx
  /*  for(int rx = 0; rx < rx_cnt; ++rx){
    for(int i = 0; i < rbins; ++i){
        if(range_detected[rx][i] < threshold){
            continue;
        }

        // Copy the data for detected stuff to hwa input
        for(int j = 0; j < chirps; ++j){
            *(hwain + (cnt * chirps) + j) =  *(data + i + (j * 1 * chirps));
        }
        cnt++;
    }
    }
    printf("Doppler count is %u\r\n",cnt);*/
    for(int i = 0; i < 1; ++i){
        for(int j = 0; j < rbins; ++j){
            if(range_detected[i][j] > 63)
                printf("Rx:%d,rbin:%d\t %d\r\n",i,j,range_detected[i][j]);
        }  
    }

  


    while(1)__asm__("wfi");
}