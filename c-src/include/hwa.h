#ifndef HWA_H
#define HWA_H
#include <stdint.h>
#include <drivers/hwa.h>


uint32_t hwa_getaddr(HWA_Handle handle);
void hwa_run(HWA_Handle handle);
void hwa_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR);
void hwa_cfar_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR);


#endif /* HWA_H */