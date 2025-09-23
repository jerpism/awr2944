#ifndef HWA_H
#define HWA_H
#include <stdint.h>

uint32_t hwa_getaddr(HWA_Handle handle);
void hwa_run(HWA_Handle handle);
void hwa_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR);
void hwa_print_samples(HWA_Handle handle, uint16_t addr, size_t n, bool sign);


#endif /* HWA_H */