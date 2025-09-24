#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <drivers/hwa.h>
#include "ti_drivers_config.h"
#include <cfg.h>

static HWA_CommonConfig HwaCommonConfig[1] =
{
    {
        .configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG,
		.numLoops = 1,
		.paramStartIdx = 0,
		.paramStopIdx = 0,
    },
};


static HWA_ParamConfig HwaParamConfig[1] =
{
    {
		.triggerMode = HWA_TRIG_MODE_DMA,
        .triggerSrc = 0,
		.accelMode = HWA_ACCELMODE_FFT,
		.source =
        {
            .srcAddr = 0,
            .srcAcnt = (CFG_PROFILE_NUMADCSAMPLES - 1),
            .srcAIdx = sizeof(uint16_t),
            .srcBcnt = (NUM_RX_ANTENNAS - 1),
            .srcBIdx = CFG_PROFILE_NUMADCSAMPLES * sizeof(uint16_t),
            .srcAcircShift = 0,
            .srcAcircShiftWrap = 0,
            .srcCircShiftWrap3 = HWA_FEATURE_BIT_DISABLE,
            .srcRealComplex = HWA_SAMPLES_FORMAT_REAL,
            .srcWidth = HWA_SAMPLES_WIDTH_16BIT,
            .srcSign = HWA_SAMPLES_UNSIGNED,
            .srcConjugate = HWA_FEATURE_BIT_DISABLE,
            .srcScale = 8,
            .srcIQSwap = HWA_FEATURE_BIT_DISABLE,
        },
		.dest =
        {
            .dstAddr = 0x4000,
            .dstAcnt = (CFG_PROFILE_NUMADCSAMPLES - 1) / 2,
            .dstAIdx = sizeof(int16_t) * 2,
            .dstBIdx = sizeof(int16_t) * CFG_PROFILE_NUMADCSAMPLES,
            .dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX,
            .dstWidth = HWA_SAMPLES_WIDTH_16BIT,
            .dstSign = HWA_SAMPLES_SIGNED,
            .dstConjugate = HWA_FEATURE_BIT_DISABLE,
            .dstScale = 8,
            .dstSkipInit = 0,
            .dstIQswap = HWA_FEATURE_BIT_DISABLE,
        },
        .accelModeArgs =
        {
            .fftMode =
            {
                .mode2X = HWA_FEATURE_BIT_DISABLE,
                .fftEn = HWA_FEATURE_BIT_ENABLE,
                .fftSize = 8,   // size is 2^fftSize
                .butterflyScaling = 0,
                .fftSize3xEn = HWA_FEATURE_BIT_DISABLE,
                .windowEn = HWA_FEATURE_BIT_DISABLE,
                .dcEstProfileSelect = HWA_DCEST_PROFILE_SELECT_PROFILE0,
                .preProcCfg =
                {
                    .dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                    .dcSubEnable = HWA_FEATURE_BIT_DISABLE,
                    .interfStat.resetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                    .interfLocalize =
                    {
                        .thresholdEnable = HWA_FEATURE_BIT_DISABLE,
                    },
                    .interfMitigation =
                    {
                        .enable = HWA_FEATURE_BIT_DISABLE,
                    },
                    .complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE,
                },
            },
        },
    },
};
/* HWA RAM atrributes */
HWA_RAMAttrs HwaRamCfg[HWA_NUM_RAMS] =
{
    {CSL_DSS_HWA_WINDOW_RAM_U_BASE, CSL_DSS_HWA_WINDOW_RAM_U_SIZE}
};


uint32_t hwa_getaddr(HWA_Handle handle){
    HWA_MemInfo meminfo;
    HWA_getHWAMemInfo(handle, &meminfo);

    return meminfo.baseAddress;
}


void hwa_init(HWA_Handle handle,  HWA_ParamDone_IntHandlerFuncPTR cb){
    DSSHWACCPARAMRegs *pparam = (DSSHWACCPARAMRegs*)gHwaObjectPtr[0]->hwAttrs->paramBaseAddr;
    HWA_configCommon(handle, &HwaCommonConfig[0]);
    HWA_configParamSet(handle, 0, &HwaParamConfig[0], NULL);

    HWA_InterruptConfig intrcfg;
    memset(&intrcfg, 0, sizeof(HWA_InterruptConfig));
    intrcfg.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    intrcfg.cpu.callbackFn = cb;
    HWA_enableParamSetInterrupt(handle, 0, &intrcfg);
    HWA_enable(handle, 1U);
    HWA_reset(handle);
}


void hwa_run(HWA_Handle handle){
    HWA_reset(handle);
    HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
}