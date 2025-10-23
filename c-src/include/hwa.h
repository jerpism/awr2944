#ifndef HWA_H
#define HWA_H
#include <stdint.h>
#include <drivers/hwa.h>


struct cfar_cfg{
    int num_noise_l;
    int num_noise_r;
    int num_guard;
    int avg_div_fact;
    int thresh_divd;
    int thresh_divs;
};

void hwa_cfg_cfar(HWA_Handle handle, struct cfar_cfg cfg);

uint32_t hwa_getaddr(HWA_Handle handle);
void hwa_run(HWA_Handle handle);
void hwa_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR);
void hwa_cfar_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR);
void hwa_anglecfar_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR, size_t ranges);
void hwa_doppler_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR);
void hwa_angle_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR);


#endif /* HWA_H */