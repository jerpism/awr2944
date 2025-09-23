#include <drivers/adcbuf.h>
#include <kernel/dpl/DebugP.h>
#include <cfg.h>

#define ADCBUF_CHANCONF_DATASIZE (CFG_PROFILE_NUMADCSAMPLES * sizeof(uint16_t))


ADCBuf_Handle adcbuf_init(){
    int32_t ret = 0;
    ADCBuf_Params ADCBufParams;
    ADCBuf_Handle handle;
    DebugP_log("Initializing ADC...\r\n");

    ADCBuf_Params_init(&ADCBufParams);
    ADCBufParams.chirpThresholdPing = CFG_ADCBUF_PINGPONG_THRESHOLD;
    ADCBufParams.chirpThresholdPong = CFG_ADCBUF_PINGPONG_THRESHOLD;
    ADCBufParams.continousMode = 0;
    ADCBufParams.source = ADCBUF_SOURCE_DFE;

    handle = ADCBuf_open(0, &ADCBufParams);

    DebugP_assert(handle != NULL);

    DebugP_log("Got ADC handle\r\n");

    // These should apparently be 4 byte aligned
    ADCBuf_dataFormat datafmt __attribute__((aligned(4))) = {0};
    ADCBuf_RxChanConf chanconf __attribute__((aligned(4))) = {0};
    datafmt.adcOutFormat = 1;       // real
    datafmt.sampleInterleave = 0;
    datafmt.channelInterleave = 1;  // apparently non-interleaved might be the only valid option for AWR2544
    chanconf.channel = 0;
    chanconf.offset = 0;

    uint8_t mask = 0b0001;
    uint8_t enabledch = 0;
    for(int i = 0; i < 4; ++i){
        if((CFG_OPEN_CHCONF_RXEN_BMASK & mask) == 0){
            mask = mask << 1U;
            continue;
        }
        chanconf.channel = i;
        chanconf.offset = enabledch * ADCBUF_CHANCONF_DATASIZE;
        enabledch++;
        mask = mask << 1U;

        ret = ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_ENABLE, &chanconf);
        if(ret != 0){ 
            DebugP_logError("Failed to conf channel %d\r\n",i); 
            return NULL; 
    }

    }

    ret = ADCBuf_control(handle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, &datafmt);
    if(ret != 0){ 
        DebugP_logError("Failed to conf data fmt\r\n"); 
        return NULL;  
    }

    DebugP_log("ADC configured!\r\n");
    return handle;
}