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
#define CFAR_NUM_NOISE_LEFT     4
#define CFAR_NUM_NOISE_RIGHT    4
// Guard cells (pretty self-explanatory)
#define CFAR_NUM_GUARD_CELLS    1

// Div factor is 2^CFAR_AVG_DIV_FACTOR
#define CFAR_AVG_DIV_FACTOR     3

#define CFAR_AVG_MODE           (HWA_NOISE_AVG_MODE_CFAR_CA)

// For OS mode so these won't be used with CA
#define CFAR_OS_KVALUE          0
#define CFAR_OS_EDGE_KSCALE_EN  0

#define CFAR_OPER_MODE      (HWA_CFAR_OPER_MODE_LOG_INPUT_REAL)

#define CFAR_OUTPUT_MODE    (HWA_CFAR_OUTPUT_MODE_I_PEAK_IDX_Q_CUT)

// No idea what these really do as of now
#define CFAR_ADV_OUT_MODE   (HWA_FEATURE_BIT_DISABLE)
#define CFAR_PEAK_GROUP_EN  (HWA_FEATURE_BIT_DISABLE)
#define CFAR_CYCLIC_MODE_EN (HWA_FEATURE_BIT_DISABLE)

#define CFAR_THRESHOLD (0)



static HWA_CommonConfig HwaCommonConfig[1] =
{
    {
        .configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG,
		.numLoops = 1,
		.paramStartIdx = 0,
		.paramStopIdx = 0,
    },
};



static HWA_ParamConfig rangeCfg = {
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
                    .fftSize = 8, // size is 2^fftSize
                    .butterflyScaling = 0,
                    .fftSize3xEn = HWA_FEATURE_BIT_DISABLE,
                    .windowEn = HWA_FEATURE_BIT_DISABLE,
                    .preProcCfg =
                        {
                            .dcEstResetMode =
                                HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                            .dcSubEnable = HWA_FEATURE_BIT_DISABLE,
                            .interfStat.resetMode =
                                HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                            .interfLocalize =
                                {
                                    .thresholdEnable = HWA_FEATURE_BIT_DISABLE,
                                },
                            .interfMitigation =
                                {
                                    .enable = HWA_FEATURE_BIT_DISABLE,
                                },
                            .complexMultiply.cmultMode =
                                HWA_COMPLEX_MULTIPLY_MODE_DISABLE,
                        },
                },
        },
};


static HWA_ParamConfig dopplerCfg = {
    .triggerMode = HWA_TRIG_MODE_SOFTWARE,
    .triggerSrc = 0,
    .accelMode = HWA_ACCELMODE_FFT,
    .source =
        {
            .srcAddr = 0,
            .srcAcnt = (NUM_DOPPLER_CHIRPS - 1),
            .srcAIdx = sizeof(int16imre_t),
            .srcBcnt = (NUM_RX_ANTENNAS * NUM_TX_ANTENNAS) - 1,
            .srcBIdx = NUM_DOPPLER_CHIRPS * sizeof(int16imre_t),
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
            .dstAddr = 0x4000,
            .dstAcnt = NUM_DOPPLER_CHIRPS * NUM_TX_ANTENNAS * NUM_RX_ANTENNAS - 1,
            .dstAIdx = sizeof(int16imre_t),
            .dstBIdx = NUM_DOPPLER_CHIRPS * sizeof(int16imre_t),
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
                    .fftSize = 5, // size is 2^fftSize
                    .butterflyScaling = 0,
                    .fftSize3xEn = HWA_FEATURE_BIT_DISABLE,
                    .windowEn = HWA_FEATURE_BIT_DISABLE,
                    .preProcCfg =
                        {
                            .dcEstResetMode =
                                HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                            .dcSubEnable = HWA_FEATURE_BIT_DISABLE,
                            .interfStat.resetMode =
                                HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                            .interfLocalize =
                                {
                                    .thresholdEnable = HWA_FEATURE_BIT_DISABLE,
                                },
                            .interfMitigation =
                                {
                                    .enable = HWA_FEATURE_BIT_DISABLE,
                                },
                            .complexMultiply.cmultMode =
                                HWA_COMPLEX_MULTIPLY_MODE_DISABLE,
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
        // Data being fed in is log2 magnitude as uint16
        .srcAcnt = NUM_DOPPLER_CHIRPS - 1,
        .srcAIdx = sizeof(uint16_t),
        .srcBcnt = NUM_RANGEBINS -1,
        .srcBIdx = NUM_DOPPLER_CHIRPS * sizeof(uint16_t),
        // These don't seem to get applied during CFAR
        .srcCcnt = 1,
        .srcCIdx = 0,
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
    .dest = {
        .dstAddr = 0x10000,
        .dstAcnt = NUM_DOPPLER_CHIRPS * NUM_RANGEBINS - 1,
        .dstAIdx = sizeof(uint32_t),
        .dstBIdx = 0, // ignored in detected peaks mode
        .dstRealComplex = HWA_SAMPLES_FORMAT_REAL, // For now we just care about detections so suppress Q channel 
        .dstWidth = HWA_SAMPLES_WIDTH_32BIT, // output is 24 bit so this should be bigger (can maybe get away with 16 using scaling?)
        .dstSign = HWA_SAMPLES_UNSIGNED,
        .dstConjugate = HWA_FEATURE_BIT_DISABLE,
        // Essentially shifts the result 8 bits to the right
        // so we don't have to do it later when masking
        // TODO: might need to remove this when we care about Q channel CUT/Noise data
        .dstScale = 8,  
        .dstSkipInit = 0,
        .dstIQswap = HWA_FEATURE_BIT_DISABLE,
    },

};


static HWA_ParamConfig angleCfg = {
    .triggerMode = HWA_TRIG_MODE_SOFTWARE,
    .triggerSrc = 0,
    .accelMode = HWA_ACCELMODE_FFT,
    .source =
        {
            .srcAddr = 0,
            .srcAcnt = (NUM_RX_ANTENNAS * NUM_TX_ANTENNAS)-1,
            .srcAIdx = sizeof(int16imre_t),
            .srcBcnt = 0,
            .srcBIdx = NUM_RX_ANTENNAS * NUM_TX_ANTENNAS * sizeof(int16imre_t),
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
            .dstAddr = 0x4000,
            .dstAcnt = NUM_RX_ANTENNAS * NUM_TX_ANTENNAS - 1,
            .dstAIdx = sizeof(int16imre_t),
            .dstBIdx = NUM_RX_ANTENNAS * NUM_TX_ANTENNAS * sizeof(int16imre_t),
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
                    .fftSize = 4, // size is 2^fftSize
                    .butterflyScaling = 0,
                    .fftSize3xEn = HWA_FEATURE_BIT_DISABLE,
                    .windowEn = HWA_FEATURE_BIT_DISABLE,
                    .preProcCfg =
                        {
                            .dcEstResetMode =
                                HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                            .dcSubEnable = HWA_FEATURE_BIT_DISABLE,
                            .interfStat.resetMode =
                                HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                            .interfLocalize =
                                {
                                    .thresholdEnable = HWA_FEATURE_BIT_DISABLE,
                                },
                            .interfMitigation =
                                {
                                    .enable = HWA_FEATURE_BIT_DISABLE,
                                },
                            .complexMultiply.cmultMode =
                                HWA_COMPLEX_MULTIPLY_MODE_DISABLE,
                        },
                },
        },
};

static HWA_ParamConfig anglecfarCfg = {
    .triggerMode = HWA_TRIG_MODE_SOFTWARE,
    .triggerSrc = 0,
    .accelMode = HWA_ACCELMODE_CFAR,
    .accelModeArgs = {
        .cfarMode = {
            .numNoiseSamplesLeft = 2,
            .numNoiseSamplesRight = 2,
            .numGuardCells = 0,
            .nAvgDivFactor = 2,
            .nAvgMode = CFAR_AVG_MODE,
            .cfarOsKvalue = CFAR_OS_KVALUE,
            .cfarOsEdgeKScaleEn = CFAR_OS_EDGE_KSCALE_EN,
            .operMode = HWA_CFAR_OPER_MODE_MAG_INPUT_COMPLEX,
            .outputMode = CFAR_OUTPUT_MODE,
            .cfarAdvOutMode = CFAR_ADV_OUT_MODE,
            .peakGroupEn = CFAR_PEAK_GROUP_EN,
            .cyclicModeEn = CFAR_CYCLIC_MODE_EN,

        },
    },
    .source = {
        .srcAddr = 0,
        .srcAcnt = NUM_TX_ANTENNAS * NUM_RX_ANTENNAS - 1,
        .srcAIdx = sizeof(int16imre_t),
        .srcBcnt = 0,
        .srcBIdx = NUM_TX_ANTENNAS * NUM_RX_ANTENNAS * sizeof(int16imre_t),
        // These don't seem to get applied during CFAR
        .srcCcnt = 1,
        .srcCIdx = 0,
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
    .dest = {
        .dstAddr = 0x1C000,
        .dstAcnt = NUM_TX_ANTENNAS * NUM_RX_ANTENNAS -1,
        .dstAIdx = sizeof(uint32_t),
        .dstBIdx = 0, // ignored in detected peaks mode
        .dstRealComplex = HWA_SAMPLES_FORMAT_REAL, // For now we just care about detections so suppress Q channel 
        .dstWidth = HWA_SAMPLES_WIDTH_32BIT, // output is 24 bit so this should be bigger (can maybe get away with 16 using scaling?)
        .dstSign = HWA_SAMPLES_UNSIGNED,
        .dstConjugate = HWA_FEATURE_BIT_DISABLE,
        // Essentially shifts the result 8 bits to the right
        // so we don't have to do it later when masking
        // TODO: might need to remove this when we care about Q channel CUT/Noise data
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
    HWA_configCommon(handle, &HwaCommonConfig[0]);
    HWA_configParamSet(handle, 0, &rangeCfg, NULL);

    HWA_InterruptConfig intrcfg;
    memset(&intrcfg, 0, sizeof(HWA_InterruptConfig));
    intrcfg.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    intrcfg.cpu.callbackFn = cb;
    HWA_enableParamSetInterrupt(handle, 0, &intrcfg);
    HWA_enable(handle, 1U);
    HWA_reset(handle);
}


void hwa_cfar_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR cb){
    DSSHWACCPARAMRegs *pparam = (DSSHWACCPARAMRegs*)gHwaObjectPtr[0]->hwAttrs->paramBaseAddr;
    DSSHWACCRegs *pregs = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
    HWA_configCommon(handle, &HwaCommonConfig[0]);
    HWA_configParamSet(handle, 0, &cfarCfg, NULL);
    pregs->CFAR_THRESH = CFAR_THRESHOLD;

   // int32_t ret = HWA_configCFARThresholdScale(CFAR_THRESHOLD);
    //DebugP_log("cfar threshold: %d\r\n",ret);
    HWA_InterruptConfig intrcfg;
    memset(&intrcfg, 0, sizeof(HWA_InterruptConfig));
    intrcfg.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    intrcfg.cpu.callbackFn = cb;
    HWA_enableParamSetInterrupt(handle, 0, &intrcfg);
    HWA_enable(handle, 1);
    HWA_reset(handle);
//    HWA_enable(handle, 1);
//    HWA_reset(handle);
//    HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
}


void hwa_anglecfar_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR cb, size_t ranges){
    DSSHWACCPARAMRegs *pparam = (DSSHWACCPARAMRegs*)gHwaObjectPtr[0]->hwAttrs->paramBaseAddr;
    DSSHWACCRegs *pregs = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
    anglecfarCfg.source.srcBcnt = ranges-1;
    HWA_configCommon(handle, &HwaCommonConfig[0]);
    HWA_configParamSet(handle, 0, &anglecfarCfg, NULL);
    pregs->CFAR_THRESH = 0;

    HWA_InterruptConfig intrcfg;
    memset(&intrcfg, 0, sizeof(HWA_InterruptConfig));
    intrcfg.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    intrcfg.cpu.callbackFn = cb;
    HWA_enableParamSetInterrupt(handle, 0, &intrcfg);
    HWA_enable(handle, 1);
    HWA_reset(handle);
}


void hwa_doppler_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR cb){
    DSSHWACCPARAMRegs *pparam = (DSSHWACCPARAMRegs*)gHwaObjectPtr[0]->hwAttrs->paramBaseAddr;
    DSSHWACCRegs *pregs = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
    HWA_configCommon(handle, &HwaCommonConfig[0]);
    HWA_configParamSet(handle, 0, &dopplerCfg, NULL);
    HWA_InterruptConfig intrcfg;
    memset(&intrcfg, 0, sizeof(HWA_InterruptConfig));
    intrcfg.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    intrcfg.cpu.callbackFn = cb;
    HWA_enableParamSetInterrupt(handle, 0, &intrcfg);
    HWA_enable(handle, 1);
    HWA_reset(handle);
}


void hwa_angle_init(HWA_Handle handle, HWA_ParamDone_IntHandlerFuncPTR cb){
    HWA_configCommon(handle, &HwaCommonConfig[0]);
    HWA_configParamSet(handle,0, &angleCfg, NULL);
    
    HWA_InterruptConfig intrcfg;
    memset(&intrcfg, 0, sizeof(HWA_InterruptConfig));
    intrcfg.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    intrcfg.cpu.callbackFn = cb;
    HWA_enableParamSetInterrupt(handle, 0, &intrcfg);

    HWA_enable(handle, 1);
    HWA_reset(handle);
}


void hwa_run(HWA_Handle handle){
   // HWA_reset(handle);
    HWA_setSoftwareTrigger(handle, HWA_TRIG_MODE_SOFTWARE);
}