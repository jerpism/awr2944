#include <stdio.h>
#include <stdlib.h>

#include <drivers/edma.h>
#include <drivers/hwa.h>
#include <drivers/uart.h>

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>

#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <edma.h>
#include <hwa.h>


static Edma_IntrObject gIntrObjAdcHwa;
static Edma_IntrObject gIntrObjHwaL3;


static uint8_t gTestBuff[2048] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
static uint8_t gTestDst[4096] __attribute__((section(".bss.dss_l3")));


void edma_write(){
    // Write 1 to EDMA_TPCC_ESR to trigger a transfer
    // TODO: probably replace this with the proper function?
    volatile uint32_t * const addr = (uint32_t*)(EDMA_getBaseAddr(gEdmaHandle[0])+0x1010);
    *addr = 0b1;
}


void edma_configure_hwa_l3(EDMA_Handle handle, void *cb, void *dst, void *src, uint16_t acnt, uint16_t bcnt, uint16_t ccnt){
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

    //&ch = EDMA_RESOURCE_ALLOC_ANY
    ch = 3;
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
    // Since we're always reading what's at the HWA output srcBIdx should be left as 0
    edmaparam.srcBIdx       = 0;
    edmaparam.destBIdx      = (int16_t)  EDMA_PARAM_BIDX(acnt);
    edmaparam.srcCIdx       = acnt;
    edmaparam.destCIdx      = acnt;
    edmaparam.linkAddr      = EDMA_TPCC_OPT(param); 
    edmaparam.srcBIdxExt    = 0;
    edmaparam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(acnt);
    // We only want to get an interrupt when a whole frame has been moved to L3
    edmaparam.opt |= (EDMA_OPT_TCINTEN_MASK | ((((uint32_t)tcc)<< EDMA_OPT_TCC_SHIFT)& EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(base, param, &edmaparam);

    gIntrObjHwaL3.tccNum = tcc;
    gIntrObjHwaL3.cbFxn = cb;
    gIntrObjHwaL3.appData = (void*)0;
    EDMA_registerIntr(gEdmaHandle[0], &gIntrObjHwaL3);
    EDMA_enableEvtIntrRegion(base, region, ch);
    EDMA_enableTransferRegion(base, region, ch, EDMA_TRIG_MODE_EVENT);

}


// TODO: use a struct here for configuring things
// or actually just rename this to something and remove the bcnt/ccnt if we're not using them
void edma_configure(EDMA_Handle handle, void *cb, void *dst, void *src, uint16_t acnt, uint16_t bcnt, uint16_t ccnt){
    uint32_t base = 0;
    uint32_t region = 0;
    uint32_t ch = 0;
    uint32_t tcc = 0;
    uint32_t param = 0;
    int32_t ret = 0;
    uint8_t *srcp = (uint8_t*)src;
    uint8_t *dstp = (uint8_t*)dst;
    EDMACCPaRAMEntry edmaparam;

    uint32_t ch1 = 0;
    uint32_t tcc1 = 0;
    uint32_t param1 = 0;
    EDMACCPaRAMEntry edmaparam1;


    /* This first channel is used to handle the ADCBUF -> HWA input transfer of samples */
    base = EDMA_getBaseAddr(handle);
    DebugP_assert(base != 0);

    region = EDMA_getRegionId(handle);
    DebugP_assert(region < SOC_EDMA_NUM_REGIONS);

    ch = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocDmaChannel(handle, &ch);
    DebugP_assert(ret == 0);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocTcc(handle, &tcc);
    DebugP_assert(ret == 0);

    param = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocParam(handle, &param);

    // First PaRAM entry, this is to handle ADCBUF -> HWA input
    EDMA_configureChannelRegion(base, region, EDMA_CHANNEL_TYPE_DMA, ch, tcc , param, 0);
    EDMA_ccPaRAMEntry_init(&edmaparam);
    edmaparam.srcAddr       = (uint32_t) SOC_virtToPhy(srcp);
    edmaparam.destAddr      = (uint32_t) SOC_virtToPhy(dstp);
    edmaparam.aCnt          = (uint16_t) acnt;     
    edmaparam.bCnt          = (uint16_t) bcnt;    
    edmaparam.cCnt          = (uint16_t) ccnt;
    edmaparam.bCntReload    = 0U;
    edmaparam.srcBIdx       = 0U;
    edmaparam.destBIdx      = 0U;
    edmaparam.srcCIdx       = 0U;
    edmaparam.destCIdx      = 0U;
    edmaparam.linkAddr      = EDMA_TPCC_OPT(param); // Link to itself
    edmaparam.srcBIdxExt    = 0U;
    edmaparam.destBIdxExt   = 0U;
    // TODO: figure out what exactly are the required options here 
    // seems to get stuck in something without the interrupts enabled
    edmaparam.opt |= (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((((uint32_t)tcc)<< EDMA_OPT_TCC_SHIFT)& EDMA_OPT_TCC_MASK));
    // Maybe this can be used to link to itself(?)
    //EDMA_linkChannel(base, param, param);
    EDMA_setPaRAM(base, param, &edmaparam);



    /* 2nd channel used to trigger HWA.
     * By chaining a second transfer right after the initial one
     * we can trigger the HWA automatically once all the input samples
     * have been transferred over to the HWA input memory.
     * This is achieved by transferring a value to DMA2HWA_TRIGGER from a read-only register
     * that contains a bit corresponding to the DMA channel we are using as a trigger source 
     * which in this case is likely always going to be the 0th channel. */
    ch1 = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocDmaChannel(handle, &ch1);
    DebugP_assert(ret == 0);

    tcc1 = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocTcc(handle, &tcc1);
    DebugP_assert(ret == 0);

    HWA_SrcDMAConfig hwadma;
    HWA_getDMAconfig(gHwaHandle[0], 0,  &hwadma);
    param1 = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocParam(handle, &param1);
    EDMA_configureChannelRegion(base, region, EDMA_CHANNEL_TYPE_DMA, ch1, tcc1, param1, 0);
    EDMA_ccPaRAMEntry_init(&edmaparam1);
    edmaparam1.srcAddr       = (uint32_t) hwadma.srcAddr;
    edmaparam1.destAddr      = (uint32_t) hwadma.destAddr;
    edmaparam1.aCnt          = (uint16_t) hwadma.aCnt;     
    edmaparam1.bCnt          = (uint16_t) hwadma.bCnt;    
    edmaparam1.cCnt          = (uint16_t) hwadma.cCnt;
    edmaparam1.bCntReload    = 0U;
    edmaparam1.srcBIdx       = 0U;
    edmaparam1.destBIdx      = 0U;
    edmaparam1.srcCIdx       = 0U;
    edmaparam1.destCIdx      = 0U;
    edmaparam1.linkAddr      = EDMA_TPCC_OPT(param1); // Link to itself
    edmaparam1.srcBIdxExt    = 0U;
    edmaparam1.destBIdxExt   = 0U;
    // TODO: figure out what exactly are the required options here 
    // seems to get stuck in something without the interrupts enabled
    edmaparam1.opt |= (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | ((((uint32_t)tcc1)<< EDMA_OPT_TCC_SHIFT)& EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(base, param1, &edmaparam1);
    // Same thing here, does linking to itself with this work?
   // EDMA_linkChannel(base, param1, param1);

    // TODO: figure out these too but having TCCHEN and ITCCHEN seems to work
    uint32_t chainoptions = (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK);
    EDMA_chainChannel(base, param, ch1, chainoptions);


    gIntrObjAdcHwa.tccNum = tcc;
    gIntrObjAdcHwa.cbFxn = cb;
    gIntrObjAdcHwa.appData = (void*)0;
    ret = EDMA_registerIntr(handle, &gIntrObjAdcHwa);
    DebugP_assert(ret == 0);
  //  EDMA_enableEvtIntrRegion(base, region, ch);
    EDMA_enableTransferRegion(base, region, ch, EDMA_TRIG_MODE_EVENT);
    DebugP_log("Edma initialized\r\n");
}