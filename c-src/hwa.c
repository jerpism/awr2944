#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <drivers/hwa.h>
#include "ti_drivers_config.h"
#include <cfg.h>
#include <hwa.h>
#include <types.h>

// TODO: these are here just temporarily to easily tune CFAR and they should be moved to cfg.h or something
// How many cells to use for noise averaging for CUT
// actual value used is val * 2
#define CFAR_NUM_NOISE_LEFT     2
#define CFAR_NUM_NOISE_RIGHT    2
// Guard cells (pretty self-explanatory)
#define CFAR_NUM_GUARD_CELLS    1

// Div factor is 2^CFAR_AVG_DIV_FACTOR
#define CFAR_AVG_DIV_FACTOR     2

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


void hwa_cfg_cfar(HWA_Handle handle, struct cfar_cfg cfg){
    DSSHWACCRegs *pregs = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;

    // TODO: This isn't very pretty and really I should just pass in an array 
    // and figure it out from that but already did it this way.
    // This also needs a *LOT* more error checking depending on the mode set and so on
    // but for now simply don't pass in invalid arguments
    if(cfg.num_noise_l != -1 && cfg.num_noise_l < 64 && cfg.num_noise_l != 1){
        cfarCfg.accelModeArgs.cfarMode.numNoiseSamplesLeft = cfg.num_noise_l;
    }

    if(cfg.num_noise_r != -1 && cfg.num_noise_r < 64 && cfg.num_noise_r != 1){
        cfarCfg.accelModeArgs.cfarMode.numNoiseSamplesRight = cfg.num_noise_r;
    }

    // is the guard number actually limited to 0-63 as well?
    if(cfg.num_guard != -1 && cfg.num_guard < 64){
        cfarCfg.accelModeArgs.cfarMode.numGuardCells = cfg.num_guard;
    }

    // I'm pretty sure this might have had a limit of 7 but can't find it now
    if(cfg.avg_div_fact != -1 && cfg.avg_div_fact < 16){
        cfarCfg.accelModeArgs.cfarMode.nAvgDivFactor = cfg.avg_div_fact;
    }

    // Assume we're going to be operating in log mode for now
    // and if a too large value is input just truncate it to 7 bits
    if(cfg.thresh_divd != -1){
        pregs->CFAR_THRESH = ((cfg.thresh_divd & 0x7F) << 7U);
    }

    if(cfg.thresh_divs != -1){
        pregs->CFAR_THRESH |= cfg.thresh_divs & 0x7FF;
    }
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


    HWA_InterruptConfig intrcfg;
    memset(&intrcfg, 0, sizeof(HWA_InterruptConfig));
    intrcfg.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    intrcfg.cpu.callbackFn = cb;
    HWA_enableParamSetInterrupt(handle, 0, &intrcfg);
    HWA_enable(handle, 1);
    HWA_reset(handle);
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