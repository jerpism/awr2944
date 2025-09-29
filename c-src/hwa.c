#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <drivers/hwa.h>
#include "ti_drivers_config.h"
#include <cfg.h>
#include <types.h>

// TODO: these are here just temporarily to easily tune CFAR and they should be moved to cfg.h or something
// How many cells to use for noise averaging for CUT
// actual value used is val * 2
#define CFAR_NUM_NOISE_LEFT     3
#define CFAR_NUM_NOISE_RIGHT    3
// Guard cells (pretty self-explanatory)
#define CFAR_NUM_GUARD_CELLS    3

// Div factor is 2^CFAR_AVG_DIV_FACTOR
#define CFAR_AVG_DIV_FACTOR     0

#define CFAR_AVG_MODE           (HWA_NOISE_AVG_MODE_CFAR_CA)

// For OS mode so these won't be used with CA
#define CFAR_OS_KVALUE          0
#define CFAR_OS_EDGE_KSCALE_EN  0

// We're inputting complex samples so have the HWA perform a magnitude operation for us
#define CFAR_OPER_MODE      (HWA_CFAR_OPER_MODE_MAG_INPUT_REAL)

// Documentation is a bit confusing on this but this should output noise avg values on I channel (same for all?) and CUT on Q
#define CFAR_OUTPUT_MODE    (HWA_CFAR_OUTPUT_MODE_I_PEAK_IDX_Q_CUT)

// No idea what these really do as of now
#define CFAR_ADV_OUT_MODE   (HWA_FEATURE_BIT_DISABLE)
#define CFAR_PEAK_GROUP_EN  (HWA_FEATURE_BIT_DISABLE)
#define CFAR_CYCLIC_MODE_EN (HWA_FEATURE_BIT_DISABLE)



static HWA_CommonConfig HwaCommonConfig[1] =
{
    {
        .configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG,
		.numLoops = 1,
		.paramStartIdx = 0,
		.paramStopIdx = 0,
    },
};


static HWA_ParamConfig HwaParamConfig[] =
{
    {
		.triggerMode = HWA_TRIG_MODE_SOFTWARE,
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
                .fftEn = HWA_FEATURE_BIT_ENABLE,
                .fftSize = 8,   // size is 2^fftSize
                .butterflyScaling = 0,
                .fftSize3xEn = HWA_FEATURE_BIT_DISABLE,
                .windowEn = HWA_FEATURE_BIT_DISABLE,
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
    {
	    .triggerMode = HWA_TRIG_MODE_SOFTWARE,
        .triggerSrc = 0,
		.accelMode = HWA_ACCELMODE_FFT,
		.source =
        {  // TODO: get these from macros in some clever way
            .srcAddr = 0,
            .srcAcnt = 32 - 1,
            .srcAIdx = 4,
            .srcBcnt = 64,
            .srcBIdx = 32 * 4,
            .srcAcircShift = 0,
            .srcAcircShiftWrap = 0,
            .srcCircShiftWrap3 = HWA_FEATURE_BIT_DISABLE,
            .srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX,
            .srcWidth = HWA_SAMPLES_WIDTH_16BIT,
            .srcSign = HWA_SAMPLES_SIGNED,
            .srcConjugate = HWA_FEATURE_BIT_DISABLE,
            .srcScale = 8,
            .srcIQSwap = HWA_FEATURE_BIT_DISABLE,
        },
		.dest =
        {
            .dstAddr = 0x8000,
            .dstAcnt = 128 -1,
            .dstAIdx = 4,
            .dstBIdx = 128 * 4,
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
                .fftEn = HWA_FEATURE_BIT_ENABLE,
                .fftSize = 7,   // size is 2^fftSize
                .butterflyScaling = 0,
                .fftSize3xEn = HWA_FEATURE_BIT_DISABLE,
                .windowEn = HWA_FEATURE_BIT_DISABLE,
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

static HWA_ParamConfig cfarCfg = {
    .triggerMode = HWA_TRIG_MODE_SOFTWARE,
    .triggerSrc = 0,
    .accelMode = HWA_ACCELMODE_CFAR,
    .accelModeArgs = {
        .cfarMode = {
            .numNoiseSamplesLeft = CFAR_NUM_NOISE_LEFT,
            .numNoiseSamplesRight = CFAR_NUM_NOISE_RIGHT,
            .numGuardCells = CFAR_NUM_GUARD_CELLS,
            .nAvgDivFactor = CFAR_AVG_DIV_FACTOR,
            .nAvgMode = CFAR_AVG_MODE,
            .cfarOsKvalue = CFAR_OS_KVALUE,
            .cfarOsEdgeKScaleEn = CFAR_OS_EDGE_KSCALE_EN,
            .operMode = CFAR_OPER_MODE,
            .outputMode = CFAR_OUTPUT_MODE,
            .cfarAdvOutMode = CFAR_ADV_OUT_MODE,
            .peakGroupEn = CFAR_PEAK_GROUP_EN,
            .cyclicModeEn = CFAR_CYCLIC_MODE_EN,

        },
    },
    .source = {
        .srcAddr = 0,
        .srcAcnt = 127,
        .srcAIdx = sizeof(int16reim_t),
        .srcBcnt = 1,
        .srcBIdx = 128 * sizeof(int16reim_t),
        // These probably don't get applied
        .srcCcnt = 1,
        .srcCIdx = 0,
        .srcAcircShift = 0,
        .srcAcircShiftWrap = 0,
        .srcCircShiftWrap3 = HWA_FEATURE_BIT_DISABLE,
        // Do these apply before or after preprocessing?
        .srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX,
        .srcWidth = HWA_SAMPLES_WIDTH_16BIT,
        .srcSign = HWA_SAMPLES_SIGNED,
        .srcConjugate = HWA_FEATURE_BIT_DISABLE,
        .srcScale = 8,
        .srcIQSwap = HWA_FEATURE_BIT_DISABLE,
    },
    .dest = {
        .dstAddr = 0x4000,
        .dstAcnt = 127,
        .dstAIdx = 4,
        .dstBIdx = 128 * 4, // ignored in detected peaks mode
        // Does this even get applied with cfar? maybe required since the output is in I/Q form?
        .dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX, 
        .dstWidth = HWA_SAMPLES_WIDTH_32BIT, // output is 24 bit so need to make this larger
        .dstSign = HWA_SAMPLES_UNSIGNED,
        .dstConjugate = HWA_FEATURE_BIT_DISABLE,
        .dstScale = 8,
        .dstSkipInit = 0,
        .dstIQswap = HWA_FEATURE_BIT_DISABLE,
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


void hwa_cfar(HWA_Handle handle){
    HWA_configParamSet(handle, 0, &cfarCfg, NULL);
    HWA_enable(handle, 1);
    HWA_reset(handle);
    HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
}


void hwa_run(HWA_Handle handle){
    HWA_reset(handle);
    HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
}