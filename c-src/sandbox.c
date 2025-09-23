/*  This file should be used to test things separately from the full project.
    Simply #include <sandbox.h> and start sandbox_main as the task  */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <drivers/hwa.h>
#include "drivers/edma/v0/edma.h"
#include "drivers/hwa/v1/hwa.h"
#include "ti_drivers_config.h"
#include <cfg.h>
#include "FreeRTOS.h"
#include "task.h"
#include <drivers/edma.h>
#include <drivers/hwa.h>
#include <drivers/uart.h>

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>

#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


#define HWA_DATA 256
#define HWA_DATA_B 512
uint8_t gTestDst[HWA_DATA_B * 2] __attribute__((section(".bss.dss_l3")));

static Edma_IntrObject gIntrObjHwaL3;

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
		.triggerMode = HWA_TRIG_MODE_SOFTWARE,
        .triggerSrc = 0,
		.accelMode = HWA_ACCELMODE_FFT,
		.source =
        {
            .srcAddr = 0,
            .srcAcnt = HWA_DATA-1,
            .srcAIdx = sizeof(uint16_t),
            .srcBcnt = 0,
            .srcBIdx = HWA_DATA_B,
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
            .dstAcnt = HWA_DATA-1,
            .dstAIdx = sizeof(int16_t) * 2,
            .dstBIdx = HWA_DATA_B * 2,
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
static HWA_RAMAttrs HwaRamCfg[HWA_NUM_RAMS] =
{
    {CSL_DSS_HWA_WINDOW_RAM_U_BASE, CSL_DSS_HWA_WINDOW_RAM_U_SIZE}
};

void test_cb(){
    HWA_reset(gHwaHandle[0]);
    EDMA_enableTransferRegion(EDMA_getBaseAddr(gEdmaHandle[0]), 0, 0, EDMA_TRIG_MODE_MANUAL);
}

void sb_edma_configure(EDMA_Handle handle, void *cb, void *dst, void *src, uint16_t acnt, uint16_t bcnt, uint16_t ccnt){
    uint32_t base = 0;
    uint32_t region = 0;
    uint32_t ch = 0;
    uint32_t tcc = 0;
    uint32_t param = 0;
    int32_t ret = 0;
    uint8_t *srcp = (uint8_t*)src;
    uint8_t *dstp = (uint8_t*)dst;
    EDMACCPaRAMEntry edmaparam;

    DebugP_log("EDMA: Configuring hwa to l3\r\n");
    /* This channel is used for HWAOUT->DSS_L3 transfering of data */
    base = EDMA_getBaseAddr(handle);
    DebugP_assert(base != 0);

    region = EDMA_getRegionId(handle);
    DebugP_assert(region < SOC_EDMA_NUM_REGIONS);

    //ch = EDMA_RESOURCE_ALLOC_ANY;
    //ch = 1;
    ret = EDMA_allocDmaChannel(handle, &ch);
    DebugP_assert(ret == 0);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocTcc(handle, &tcc);
    DebugP_assert(ret == 0);

    param = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocParam(handle, &param);

    DebugP_log("EDMA: base: %#x, region: %u, ch: %u, tcc: %u, param: %u\r\n",base,region,ch,tcc,param);

    EDMA_configureChannelRegion(base, region, EDMA_CHANNEL_TYPE_DMA, ch, tcc , param, 1);
    EDMA_ccPaRAMEntry_init(&edmaparam);
    edmaparam.srcAddr       = (uint32_t) SOC_virtToPhy(srcp);
    edmaparam.destAddr      = (uint32_t) SOC_virtToPhy(dstp);
    edmaparam.aCnt          = (uint16_t) acnt;     
    edmaparam.bCnt          = (uint16_t) bcnt;    
    edmaparam.cCnt          = (uint16_t) ccnt;
    edmaparam.bCntReload    = (uint16_t) bcnt;
    edmaparam.srcBIdx       = (int16_t)  EDMA_PARAM_BIDX(acnt);
    edmaparam.destBIdx      = (int16_t)  EDMA_PARAM_BIDX(acnt);
    edmaparam.srcCIdx       = acnt;
    edmaparam.destCIdx      = acnt;
    edmaparam.linkAddr      = 0xFFFFU; // Change this when we want to constantly move frames
    edmaparam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(acnt);
    edmaparam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(acnt);
    // TODO: figure out what exactly are the required options here 
    // seems to get stuck in something without the interrupts enabled
    edmaparam.opt |= (EDMA_OPT_TCINTEN_MASK | ((((uint32_t)tcc)<< EDMA_OPT_TCC_SHIFT)& EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(base, param, &edmaparam);

    gIntrObjHwaL3.tccNum = tcc;
    gIntrObjHwaL3.cbFxn = cb;
    gIntrObjHwaL3.appData = (void*)0;
    EDMA_registerIntr(gEdmaHandle[0], &gIntrObjHwaL3);
    EDMA_enableEvtIntrRegion(base, region, ch);
//    EDMA_enableTransferRegion(base, region, ch, EDMA_TRIG_MODE_EVENT);

}

void sb_hwa_init(HWA_Handle handle,  HWA_Done_IntHandlerFuncPTR cb){
    DSSHWACCPARAMRegs *pparam = (DSSHWACCPARAMRegs*)gHwaObjectPtr[0]->hwAttrs->paramBaseAddr;
    HWA_configCommon(handle, &HwaCommonConfig[0]);
    HWA_configParamSet(handle, 0, &HwaParamConfig[0], NULL);
  //  if(cb != NULL){
  //      HWA_enableDoneInterrupt(handle, 0,  cb, NULL);
   // }
    // set HWA2DMA_TRIGDST to channel 3 (index 2)
   // pparam->HEADER |= (2U << 11);
    // and enable DMA trigger on completion
    //pparam->HEADER |= (1U << 10);
    HWA_InterruptConfig intrcfg;
    memset(&intrcfg, 0, sizeof(HWA_InterruptConfig));
    intrcfg.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_CPU_INTR1;
    intrcfg.dma.dstChannel = 0;
    intrcfg.cpu.callbackFn = &test_cb;
    HWA_enableParamSetInterrupt(handle, 0, &intrcfg);

    HWA_enable(handle, 1U);
    HWA_reset(handle);
}

void edma_cb(){
    printf("EDMA callback\r\n");
}

void sandbox_main(void *args){
    DebugP_log("Starting sandbox\r\n");
    Drivers_open();
    Board_driversOpen();
    DebugP_log("Init HWA\r\n");
    sb_hwa_init(gHwaHandle[0], NULL);
    
    HWA_MemInfo meminfo;
    HWA_getHWAMemInfo(gHwaHandle[0], &meminfo);
    uint32_t hwain = meminfo.baseAddress;
    uint32_t hwaout = meminfo.baseAddress + HwaParamConfig[0].dest.dstAddr;
    DebugP_log("HWA input is %#x and output %#x \r\n",hwain, hwaout);

    srand(1337);
    DebugP_log("Populating src\r\n");
    for(size_t i = 0; i < HWA_DATA_B * 4; ++i){
        ((uint8_t*)hwain)[i] = (uint8_t)(rand() % 255);
    }

    DebugP_log("Init EDMA\r\n");
    sb_edma_configure(gEdmaHandle[0], &edma_cb, &gTestDst, (void*)hwaout, 512, 4, 1);

    DebugP_log("Running HWA 1\r\n");
    HWA_reset(gHwaHandle[0]);
    HWA_setSoftwareTrigger(gHwaHandle[0], HWA_TRIG_MODE_SOFTWARE);
    
    DebugP_log("Running HWA 2\r\n");
    HWA_setSoftwareTrigger(gHwaHandle[0], HWA_TRIG_MODE_SOFTWARE);

    DebugP_log("Running HWA 3\r\n");
    HWA_setSoftwareTrigger(gHwaHandle[0], HWA_TRIG_MODE_SOFTWARE);

    DebugP_log("Running HWA 4\r\n");
    HWA_setSoftwareTrigger(gHwaHandle[0], HWA_TRIG_MODE_SOFTWARE);

    DebugP_log("Done\r\n");
    while(1)__asm__("wfi");

    vTaskDelete(NULL);
}