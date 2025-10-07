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
#include <cfg.h>



static Edma_IntrObject gIntrObjAdcHwa;
static Edma_IntrObject gIntrObjHwaL3;


static uint32_t gbaseaddr;
static uint32_t ghwal3param;
static EDMACCPaRAMEntry edmaparam;

static EDMACCPaRAMEntry edmaparams[4];

static uint32_t params[4];





// This is terrible but for some reason the bcnt and dstbidx aren't being reloaded properly
// which means after the first frame only one single chirp is written to the end of the sample buffer
// simply self linking or linking to another set *should* work but both result in the same issue.
// Forcefully reloading a param set seems to reset those internal counters as intended but
// this is a slow, inelegant solution that will probably be the cause of some unforeseen consequences
void edma_reset_hwal3_param(){
    for(int i = 0; i < 4; ++i){
        EDMA_setPaRAM(gbaseaddr, params[i], &edmaparams[i]);
    }
}

// The HWA to L3 EDMA path will write data to L3 in accordance with TI's radar cube DATA_FORMAT_1
// which means complex samples in order of [numTXPatterns][numDopplerChirps][numRX][numRangeBins].
// This is achieved by rotating numTX (4) sets of EDMA params where the offset is set by idx(tx) * doppler *  numrx * rbins * sizeof(int16reim_t)
void edma_configure_hwa_l3(EDMA_Handle handle, void *cb, void *dst, void *src, uint16_t acnt, uint16_t bcnt, uint16_t ccnt){
    uint32_t base = 0;
    uint32_t region = 0;
    uint32_t tcc = 0;
    int32_t ret = 0;
    uint32_t param;

    uint32_t ch[4];

    base = EDMA_getBaseAddr(handle);
    gbaseaddr = base;
    region = EDMA_getRegionId(handle);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocTcc(handle, &tcc);

    // start with channel 3
    for(int i = 0; i < 4; ++i){
        ch[i] = 3+i;
        ret = EDMA_allocDmaChannel(handle, &ch[i]);
        DebugP_assert(ret == 0);
    }

    for(int i = 0; i < 4; ++i){
        params[i] = EDMA_RESOURCE_ALLOC_ANY;
        ret = EDMA_allocParam(handle,  &params[i]);
        DebugP_assert(ret == 0);
    }

    for(int i = 0; i < 4; ++i){
        EDMA_configureChannelRegion(base, region, EDMA_CHANNEL_TYPE_DMA, ch[i], tcc , params[i], 0);
    }   

    for(int i = 0; i < 4; ++i){
        EDMA_ccPaRAMEntry_init(&edmaparams[i]);  
        edmaparams[i].destAddr      = (uint32_t) (SOC_virtToPhy(dst) + (i * NUM_DOPPLER_CHIRPS * NUM_RX_ANTENNAS * NUM_RANGEBINS * CPLX_SAMPLE_SIZE));

        edmaparams[i].srcAddr       = (uint32_t) SOC_virtToPhy(src);
        edmaparams[i].aCnt          = (uint16_t) acnt;
        edmaparams[i].bCnt          = (uint16_t) bcnt;
        edmaparams[i].cCnt          = (uint16_t) ccnt;
        edmaparams[i].bCntReload    = (uint16_t) bcnt;

        // Always reading from HWA 
        edmaparams[i].srcBIdx       = 0;
        edmaparams[i].srcBIdxExt    = 0;
        edmaparams[i].srcCIdx       = acnt;

        edmaparams[i].destBIdx      = (int16_t)EDMA_PARAM_BIDX(acnt);
        edmaparams[i].destBIdxExt   = (int8_t)EDMA_PARAM_BIDX_EXT(acnt);
        edmaparams[i].destCIdx      = acnt;

        edmaparams[i].linkAddr      = 0xffff;//params[i];

        edmaparams[i].opt           = (EDMA_OPT_TCINTEN_MASK) | ((((uint32_t)tcc)<< EDMA_OPT_TCC_SHIFT)& EDMA_OPT_TCC_MASK);


        EDMA_setPaRAM(base, params[i], &edmaparams[i]);

    }





    DebugP_assert(ret == 0);

    for(int i = 0; i < 4; ++i){
        EDMA_enableTransferRegion(base, region, ch[i], EDMA_TRIG_MODE_EVENT);
    }




    gIntrObjHwaL3.tccNum = tcc;
    gIntrObjHwaL3.cbFxn = cb;
    gIntrObjHwaL3.appData = (void*)0;
    EDMA_registerIntr(gEdmaHandle[0], &gIntrObjHwaL3);
    EDMA_enableEvtIntrRegion(base, region, 6);
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

    ch = 1;
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
    ch1 = 2;
    ret = EDMA_allocDmaChannel(handle, &ch1);
    DebugP_assert(ret == 0);

    tcc1 = EDMA_RESOURCE_ALLOC_ANY;
    ret = EDMA_allocTcc(handle, &tcc1);
    DebugP_assert(ret == 0);

    HWA_SrcDMAConfig hwadma;
    HWA_getDMAconfig(handle, 0,  &hwadma);
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


    gIntrObjAdcHwa.tccNum = tcc;
    gIntrObjAdcHwa.cbFxn = cb;
    gIntrObjAdcHwa.appData = (void*)0;
    ret = EDMA_registerIntr(handle, &gIntrObjAdcHwa);
    DebugP_assert(ret == 0);
    EDMA_enableEvtIntrRegion(base, region, ch);
    EDMA_enableTransferRegion(base, region, ch, EDMA_TRIG_MODE_EVENT);
    DebugP_log("Edma initialized\r\n");
}