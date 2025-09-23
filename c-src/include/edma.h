#ifndef EDMA_H
#define EDMA_H
#include <stdint.h>

void edma_test(void *args);
void edma_configure(EDMA_Handle handle, void *cb, void *dst, void *src, uint16_t acnt, uint16_t bcnt, uint16_t ccnt);
void edma_configure_hwa_l3(EDMA_Handle handle, void *cb, void *dst, void *src, uint16_t acnt, uint16_t bcnt, uint16_t ccnt);

void edma_write();

#endif /* EDMA_H */
