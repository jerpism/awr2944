#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H
#include <stdint.h>
#include <types.h>

void calc_abs_vals(int16_t *in, uint16_t *out, uint32_t n);
void process_data(int16reim_t *data, uint8_t rx_cnt, uint16_t chirps, uint8_t rbins);

#endif /* DATAPROCESSING_H */
