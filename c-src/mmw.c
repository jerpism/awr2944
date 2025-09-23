#include <ti/common/syscommon.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwavelink/include/rl_sensor.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"

/* Header file for configuration values */
#include <cfg.h>

int32_t mmw_callback(uint8_t devIdx, uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload){
    DebugP_log("MMWave callback called\r\n");
    DebugP_log("devidx: %d, msgId: %u, sbId: %u, sbLen: %u\r\n", devIdx, msgId, sbId, sbLen);
    return 0;
}


void mmw_printerr(const char *s, int32_t err){
    int16_t mmwErr;
    int16_t subErr;
    MMWave_ErrorLevel errLvl;

    MMWave_decodeError(err, &errLvl, &mmwErr, &subErr);
    DebugP_log("ERROR: %s: ", s);
    DebugP_log("Error level %s, mmWaveErr: %hd, subSysErr: %hd\r\n", (errLvl == MMWave_ErrorLevel_ERROR ? "Error" : "Warning"), mmwErr, subErr);
}


int32_t mmw_open(MMWave_Handle handle, int32_t *err){
    int32_t ret = 0;
    MMWave_OpenCfg openCfg;
    memset(&openCfg, 0, sizeof(MMWave_OpenCfg));

    // these are from the oob demo
    // probably corresponds to 76-81 GHz
    openCfg.freqLimitLow = 760U;
    openCfg.freqLimitHigh = 810U;

    // disable these because nothing to handle them
    openCfg.disableFrameStartAsyncEvent = true;
    openCfg.disableFrameStopAsyncEvent = true;

    // don't run in lower power mode
    openCfg.lowPowerMode.lpAdcMode = 0;

    // ADC output configuration
    // 16 bit output
    openCfg.adcOutCfg.fmt.b2AdcBits |= 0b10;

    // this is either real or complex, docs can't really seem to agree
    openCfg.adcOutCfg.fmt.b2AdcOutFmt |= 0b00;

    // how many bits to reduce adc output by
    // has to be 0 for 16 bits
    openCfg.adcOutCfg.fmt.b8FullScaleReducFctr |= 0x0;

    // select LVDS as the interface
    openCfg.dataPathCfg.intfSel = 1;

    // only get out ADC data and suppress pkt1
    openCfg.dataPathCfg.transferFmtPkt0 = 1;
    openCfg.dataPathCfg.transferFmtPkt1 = 0;

    // size of cq samples just set to 16 bit I guess
    openCfg.dataPathCfg.cqConfig = 0b10;

    openCfg.dataPathCfg.cq0TransSize = 0;
    openCfg.dataPathCfg.cq1TransSize = 0;
    openCfg.dataPathCfg.cq2TransSize = 0;

    // SDR clock, this has to be DDR for CSI2
    openCfg.dataPathClkCfg.laneClkCfg = 0;

    // 300Mbps data rate
    openCfg.dataPathClkCfg.dataRate = 0b100;

    // clock speed for that
    openCfg.hsiClkCfg.hsiClk = 0xB;

    // only enable 1 lane
    openCfg.laneEnCfg.laneEn = 0b1;

    openCfg.chCfg.rxChannelEn = CFG_OPEN_CHCONF_RXEN_BMASK;
    openCfg.chCfg.txChannelEn = 0b1;

    openCfg.chCfg.cascading = 0;
    openCfg.chCfg.cascadingPinoutCfg = 0;

    // hopefully not getting complex output but this gives I first
    openCfg.iqSwapSel = 0;
    openCfg.chInterleave = 1; // counterintuitively this means we don't interleave

    openCfg.defaultAsyncEventHandler = MMWave_DefaultAsyncEventHandler_MSS;

    openCfg.useCustomCalibration = 0;
    openCfg.customCalibrationEnableMask = 0;
    openCfg.calibMonTimeUnit = 1;

    ret = MMWave_open(handle, &openCfg, NULL, err);

    return ret;
}


/* Pulls values from cfg.h */
MMWave_ProfileHandle mmw_create_profile(MMWave_Handle handle, int32_t *err){
    rlProfileCfg_t profileCfg;
    memset(&profileCfg, 0, sizeof(profileCfg));
    profileCfg.profileId = CFG_PROFILE_PROFILEID;
    profileCfg.pfCalLutUpdate |= 0b00;
    profileCfg.startFreqConst = CFG_PROFILE_STARTFREQCONST;
    profileCfg.idleTimeConst = CFG_PROFILE_IDLETIMECONST;         
    profileCfg.adcStartTimeConst = CFG_PROFILE_ADCSTARTTIMECONST;     
    profileCfg.rampEndTime = CFG_PROFILE_RAMPENDTIME;	    
    profileCfg.txStartTime = 0;
    profileCfg.numAdcSamples = CFG_PROFILE_NUMADCSAMPLES;
    profileCfg.digOutSampleRate = CFG_PROFILE_DIGOUTSAMPLERATE;
    profileCfg.rxGain = CFG_PROFILE_RXGAIN;
    profileCfg.freqSlopeConst = CFG_PROFILE_FREQSLOPECONST;

    return MMWave_addProfile(handle, &profileCfg, err);
}


MMWave_ChirpHandle mmw_add_chirp(MMWave_ProfileHandle profile, int32_t *err){
    rlChirpCfg_t chirpCfg;
    memset(&chirpCfg, 0, sizeof(chirpCfg));
    chirpCfg.chirpEndIdx = 0;
    chirpCfg.chirpStartIdx = 0;
    chirpCfg.profileId = 0;
    chirpCfg.startFreqVar = 0;
    chirpCfg.freqSlopeVar = 0;
    chirpCfg.idleTimeVar = 0;
    chirpCfg.adcStartTimeVar = 0;
    chirpCfg.txEnable |= 0b0001;

    return MMWave_addChirp(profile, &chirpCfg, err);
}

int32_t mmw_add_chirps(MMWave_ProfileHandle profile, int32_t *err){
    rlChirpCfg_t chirpCfg;
    memset(&chirpCfg, 0, sizeof(chirpCfg));

    chirpCfg.profileId = 0;
    chirpCfg.startFreqVar = 0;
    chirpCfg.freqSlopeVar = 0;
    chirpCfg.idleTimeVar = 0;
    chirpCfg.adcStartTimeVar = 0;
    uint8_t txmask = 0b0001;
    for(int i = 0; i < 4; ++i){
        chirpCfg.chirpEndIdx = i;
        chirpCfg.chirpStartIdx = i;
        chirpCfg.txEnable = 0b0001;
        MMWave_ChirpHandle chirp = MMWave_addChirp(profile, &chirpCfg,err);
        if(chirp == NULL){
            return -1;
        }
        txmask = (txmask << 1U);
    }
    return 0;
}

/* For now only supports configuring the first profile */
int32_t mmw_config(MMWave_Handle handle, MMWave_ProfileHandle profiles[static MMWAVE_MAX_PROFILE], int32_t *err){
    int32_t ret = 0;
    MMWave_CtrlCfg ctrlCfg;

    memset(&ctrlCfg, 0, sizeof(ctrlCfg));
    ctrlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
    ctrlCfg.numOfPhaseShiftChirps[0] = 0;
    ctrlCfg.u.frameCfg[0].profileHandle[0] = profiles[0];

    for(int i = 1; i < MMWAVE_MAX_PROFILE; ++i){
        ctrlCfg.u.frameCfg[0].profileHandle[i] = NULL;
    }

    ctrlCfg.u.frameCfg[0].frameCfg.chirpStartIdx = 0;
    ctrlCfg.u.frameCfg[0].frameCfg.chirpEndIdx = 3;
    ctrlCfg.u.frameCfg[0].frameCfg.framePeriodicity = 2000000;
    ctrlCfg.u.frameCfg[0].frameCfg.numFrames = 1;
    ctrlCfg.u.frameCfg[0].frameCfg.triggerSelect = 1;
    ctrlCfg.u.frameCfg[0].frameCfg.numLoops = 128 / 4;

    ret = MMWave_config(handle, &ctrlCfg, err);

    return ret;
}


int32_t mmw_start(MMWave_Handle handle, int32_t *err){
    int32_t ret = 0;
    MMWave_CalibrationCfg calibCfg;
    memset(&calibCfg, 0, sizeof(calibCfg));
    calibCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
    calibCfg.u.chirpCalibrationCfg.enableCalibration = 0;
    calibCfg.u.chirpCalibrationCfg.enablePeriodicity = 0;
    calibCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 0;
    calibCfg.u.chirpCalibrationCfg.reportEn = 1;

    ret = MMWave_start(handle, &calibCfg, err);

    return ret;
}


MMWave_Handle mmw_init(int32_t *err){
    MMWave_InitCfg initCfg;
    memset(&initCfg, 0, sizeof(initCfg));

    initCfg.domain = MMWave_Domain_MSS;
    initCfg.eventFxn = mmw_callback;
    initCfg.linkCRCCfg.crcBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    initCfg.linkCRCCfg.useCRCDriver = 1;
    initCfg.linkCRCCfg.crcChannel = CRC_CHANNEL_1;
    initCfg.cfgMode = MMWave_ConfigurationMode_FULL;

    return MMWave_init(&initCfg, err);
}