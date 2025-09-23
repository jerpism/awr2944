#ifndef MMW_H
#define MMW_H

#include <stdint.h>
#include <ti/control/mmwave/mmwave.h>

/* Decode the error code in err and print it out along with the message s */
void mmw_printerr(const char *s, int32_t err);

/* Open the given handle
 * returns 0 on success, <0 otherwise 
 * err contains the specific error code */
int32_t mmw_open(MMWave_Handle, int32_t *err);

/* Configure the MMWave device
 * NOTE: Currently only profiles[0] will be configured 
 * profiles[1]-[MMWAVE_MAX_PROFILE] will be set to NULL 
 * returns 0 on success, <0 otherwise
 * err contains the specific error code */
int32_t mmw_config(MMWave_Handle handle, MMWave_ProfileHandle profiles[static MMWAVE_MAX_PROFILE], int32_t *err);

/* Start the MMWave device 
 * the device will start outputting ADC data after this has been called */
int32_t mmw_start(MMWave_Handle handle, int32_t *err);

/* Initialize the mmwave device
 * returns a valid handle on success, NULL otherwise 
 * err contains the specific error code */
MMWave_Handle mmw_init(int32_t *err);

/* Creates a profile for a given handle 
 * NOTE: the profile is configured according to a hardcoded configuration
 * returns a profile handle on success, NULL otherwise
 * err contains the specific error code */
MMWave_ProfileHandle mmw_create_profile(MMWave_Handle handle, int32_t *err);

/* Adds a chirp to a given profile 
 * NOTE: chirp configuration is also hardcoded 
 * returns a chirp handle on success, NULL othewrise
 * err contains the specific error code */
MMWave_ChirpHandle mmw_add_chirp(MMWave_ProfileHandle profile, int32_t *err);

int32_t mmw_add_chirps(MMWave_ProfileHandle profile, int32_t *err);


#endif /* MMW_H */
