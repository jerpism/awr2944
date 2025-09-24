/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* TI Header files */
#include <stdlib.h>


#include "FreeRTOS.h"
#include "task.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_board_config.h"

#include <drivers/uart.h>
#include <drivers/adcbuf.h>
#include <drivers/hwa.h>

#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/DebugP.h>

#include <ti/common/syscommon.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/control/mmwavelink/include/rl_sensor.h>

/* Project header files */
// TODO: might need to rename these as to not conflict with SDK includes
#include <mmw.h>
#include <adcbuf.h>
#include <edma.h>
#include <cfg.h>
#include <gpio.h>
#include <hwa.h>
#include <network.h>

//#include <sandbox.h>

/* Task related macros */
#define EXEC_TASK_PRI   (configMAX_PRIORITIES-1)     // must be higher than INIT_TASK_PRI
#define MAIN_TASK_PRI   (configMAX_PRIORITIES-3)
#define INIT_TASK_PRI   (configMAX_PRIORITIES-2)
#define EXEC_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define MAIN_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define DPC_TASK_SIZE   (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define INIT_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))

/* Project related macros */
#define SAMPLE_SIZE         (sizeof(uint16_t))
#define CHIRP_DATASIZE      (NUM_RX_ANTENNAS * CFG_PROFILE_NUMADCSAMPLES * SAMPLE_SIZE)
#define CHIRPS_PER_FRAME    128
#define FRAME_DATASIZE      (CHIRP_DATASIZE * CHIRPS_PER_FRAME)
#define UDP_BYTES_PER_PKT   1024
#define UDP_PKT_CNT         (FRAME_DATASIZE / UDP_BYTES_PER_PKT)


/* Task related global variables */
StackType_t gInitTaskStack[INIT_TASK_SIZE] __attribute__((aligned(32)));
StackType_t gMainTaskStack[MAIN_TASK_SIZE] __attribute__((aligned(32)));
StackType_t gExecTaskStack[EXEC_TASK_SIZE] __attribute__((aligned(32)));
StackType_t gDpcTaskStack[DPC_TASK_SIZE] __attribute__((aligned(32)));

StaticTask_t gInitTaskObj;
StaticTask_t gMainTaskObj;
StaticTask_t gExecTaskObj;
StaticTask_t gDpcTaskObj;

TaskHandle_t gInitTask;
TaskHandle_t gMainTask;
TaskHandle_t gExecTask;
TaskHandle_t gDpcTask;


/* == Function Declarations == */
/* ISRs */
void btn_isr(void*);
void chirp_isr(void*);

/* Tasks */
static void init_task(void*);
static void exec_task(void*);
static void main_task(void*);

/* Other functions */
static inline void fail(void);

#ifdef SANDBOX
#else
extern void edma_test(void*);
/* == Global Variables == */
/* Handles */
MMWave_Handle gMmwHandle = NULL;
ADCBuf_Handle gADCBufHandle = NULL;
MMWave_ProfileHandle gMmwProfiles[MMWAVE_MAX_PROFILE];

SemaphoreP_Object gAdcSampledSem;
SemaphoreP_Object gBtnPressedSem;
SemaphoreP_Object gEdmaDoneSem;
SemaphoreP_Object gHwaDoneSem;
SemaphoreP_Object gFrameDoneSem;

/* Rest of them */
volatile bool gState = 0; /* Tracks the current (intended) state of the RSS */
static uint32_t gPushButtonBaseAddr = GPIO_PUSH_BUTTON_BASE_ADDR;

static uint8_t gSampleBuff[FRAME_DATASIZE] __attribute__((section(".bss.dss_l3")));


static inline void fail(void){
    DebugP_log("Failed\r\n");
    DebugP_assertNoLog(0);
    while(1) __asm__ volatile("wfi");
}


void edma_callback(Edma_IntrHandle handle, void *args){
}


void hwa_callback(uint32_t intrIdx, uint32_t paramSet, void *arg){
    HWA_reset(gHwaHandle[0]);

    // TODO: don't blindly guess channel number here but that's an issue for future me
    // and it should always be 3 anyways
    EDMA_setEvtRegion(EDMA_getBaseAddr(gEdmaHandle[0]), 0, 3);

}


static void frame_done(Edma_IntrHandle handle, void *args){
        SemaphoreP_post(&gFrameDoneSem);
}


static void exec_task(void *args){
    int32_t err;
    while(1){
        MMWave_execute(gMmwHandle, &err);
    }
}


static void main_task(void *args){
    int32_t err = 0;
    int32_t ret = 0;

    // TODO: grab this from sysconfig somehow but for now assume bank 2 will be output
    void *hwaout = (void*)(hwa_getaddr(gHwaHandle[0])+0x4000);

    HwiP_enable();

    while(1){
        DebugP_log("Ready to take a picture\r\n");
        SemaphoreP_pend(&gBtnPressedSem, SystemP_WAIT_FOREVER);

        DebugP_log("Taking a picture\r\n");
        mmw_start(gMmwHandle, &err);

        SemaphoreP_pend(&gFrameDoneSem, SystemP_WAIT_FOREVER);
        MMWave_stop(gMmwHandle, &err);
        CacheP_wbInv(&gSampleBuff, FRAME_DATASIZE, CacheP_TYPE_ALL);
        printf("Frame should be at 0x%p now\r\n",&gSampleBuff);

        DebugP_log("Sending it out over UDP now\r\n");
        for(size_t i = 0; i < UDP_PKT_CNT; ++i){
            udp_send_data((void*)(gSampleBuff + (i * UDP_BYTES_PER_PKT)), UDP_BYTES_PER_PKT);
        }
    }

    while(1)__asm__("wfi");
}


/* init process goes as follows:
 * 
 *  - initialize ADCBUF, MMW, EDMA and enet
 *  - synchronize mmwavelink
 *  - create the MMWave_execute task
 *  - open the device
 *  - create profile(s) 
 *  - add chirp configuration
 *  - configure the device to use profile(s)
 *  - create the main_task
 *  - and finally terminate itself
 *  
 */
static void init_task(void *args){
    int32_t err = 0;
    int32_t ret = 0;
    HWA_SrcDMAConfig dmacfg;

    Drivers_open();
    Board_driversOpen(); 

    DebugP_log("Init task launched\r\n");

    DebugP_log("Init HWA...\r\n");
    // assume for now that input memory will be at HWA base
    uint32_t hwaaddr = (uint32_t)SOC_virtToPhy((void*)hwa_getaddr(gHwaHandle[0]));
    hwa_init(gHwaHandle[0], &hwa_callback);
    DebugP_log("HWA address is %#x\r\n",hwaaddr);
    DebugP_log("Done.\r\n");


    DebugP_log("Init network...\r\n");
    network_init(NULL);
    DebugP_log("Done.\r\n");


    // init adc
    DebugP_log("Init adc...\r\n");
    gADCBufHandle = adcbuf_init();
    DebugP_assert(gADCBufHandle != NULL);
    for(int i = 0; i < 4; ++i){
        uint32_t adcaddr = (uint32_t)ADCBuf_getChanBufAddr(gADCBufHandle, i, &err);
        DebugP_log("Adcbuf address for channel %d is %#x\r\n", i, adcaddr);
    }
    DebugP_log("Done.\r\n");

    
    uint32_t adcaddr = (uint32_t)ADCBuf_getChanBufAddr(gADCBufHandle, 0, &err);

    // and mmw
    DebugP_log("Init mmw...\r\n");
    gMmwHandle = mmw_init(&err);
    DebugP_assert(gMmwHandle != NULL);
    DebugP_log("Done.\r\n");



    // and EDMA
    DebugP_log("Init edma...\r\n");
    edma_configure(gEdmaHandle[0],&edma_callback, (void*)hwaaddr, (void*)adcaddr, CHIRP_DATASIZE, 1, 1);
    edma_configure_hwa_l3(gEdmaHandle[0], &frame_done, (void*)&gSampleBuff, (void*)(hwaaddr+0x4000),  CHIRP_DATASIZE,  CHIRPS_PER_FRAME, 1);
    DebugP_log("Done.\r\n");



    // Not that any should happen but disable interrupts here 
    HwiP_disable();

    // ADC sampling done interrupt
    HwiP_Object hwiobj;
    HwiP_Params params;
    HwiP_Params_init(&params);
    params.intNum = CSL_MSS_INTR_RSS_ADC_CAPTURE_COMPLETE;
    params.args = NULL;
    params.callback = &chirp_isr;
    ret = HwiP_construct(&hwiobj, &params);
    if(ret != 0){ DebugP_log("Failed to construct\r\n");}

    // Push button interrupt
    gPushButtonBaseAddr = gpio_init(&btn_isr);

    ret = SemaphoreP_constructBinary(&gBtnPressedSem, 0);
    DebugP_assert(ret == 0);

    ret = SemaphoreP_constructBinary(&gFrameDoneSem, 0);
    DebugP_assert(ret == 0);


    DebugP_log("Synchronizing...\r\n");

    /* NOTE: According to the documentation a return value of 1
     * is supposed to mean synchronized, however MMWave_sync()
     * will under no circumstances return a value of 1.
     * So unless this is changed/fixed in a later version
     * treat 0 as a success and != 0 (<0) as a failure */
    while(MMWave_sync(gMmwHandle, &err) != 0);

    DebugP_log("Synced!\r\n");

    /* We must create a task that calls MMWave_exec at this point
     * otherwise the device will get stuck in an internal sync loop */
    gExecTask = xTaskCreateStatic(
        exec_task,      /*  Pointer to the function that implements the task. */
        "exec task",    /*  Text name for the task.  This is to facilitate debugging
                            only. */
        EXEC_TASK_SIZE, /*  Stack depth in units of StackType_t typically uint32_t
                            on 32b CPUs */
        NULL,           /*  We are not using the task parameter. */
        EXEC_TASK_PRI,  /*  task priority, 0 is lowest priority,
                            configMAX_PRIORITIES-1 is highest */
        gExecTaskStack, /*  pointer to stack base */
        &gExecTaskObj); /*  pointer to statically allocated task object memory */
    configASSERT(gExecTask != NULL);

    DebugP_log("Configuring mmw...\r\n");
    ret = mmw_open(gMmwHandle, &err);
    if(ret != 0){
        mmw_printerr("Failed to open device", err);
        fail();
    }

    gMmwProfiles[0] = mmw_create_profile(gMmwHandle, &err);
    ret = mmw_add_chirps(gMmwProfiles[0], &err);
    if(ret != 0){
        DebugP_logError("Failed to add chirps\r\n");
    }
    /*if(chirp == NULL){
        mmw_printerr("Failed to add chirp", err);
        fail();
    }*/

    ret = mmw_config(gMmwHandle, gMmwProfiles, &err);
    if (ret != 0){
        mmw_printerr("Failed to configure", err);
        fail();
    }

    DebugP_log("Configured!\r\n");

    DebugP_log("Creating main task...\r\n");
    gMainTask = xTaskCreateStatic(
        main_task,      /*  Pointer to the function that implements the task. */
        "main task",    /*  Text name for the task.  This is to facilitate debugging
                            only. */
        MAIN_TASK_SIZE, /*  Stack depth in units of StackType_t typically uint32_t
                            on 32b CPUs */
        NULL,           /*  We are not using the task parameter. */
        MAIN_TASK_PRI,  /*  task priority, 0 is lowest priority,
                            configMAX_PRIORITIES-1 is highest */
        gMainTaskStack, /*  pointer to stack base */
        &gMainTaskObj); /*  pointer to statically allocated task object memory */
    configASSERT(gMainTask != NULL);

    DebugP_log("Done. バイバイ\r\n");

    vTaskDelete(NULL);
}


void btn_isr(void *arg){
    uint32_t pending;
    uint32_t pin = (uint32_t)arg;

    pending = GPIO_getHighLowLevelPendingInterrupt(gPushButtonBaseAddr, pin);
    GPIO_clearInterrupt(GPIO_PUSH_BUTTON_BASE_ADDR, pin);
    if(pending){
        SemaphoreP_post(&gBtnPressedSem);
    }    
}


void chirp_isr(void *arg){
    int32_t err;
    edma_write();
}
#endif

int main(void) {
    /* init SOC specific modules */
    System_init();
    Board_init();
#ifdef SANDBOX
   gInitTask = xTaskCreateStatic(
            sandbox_main,   
            "init task", 
            INIT_TASK_SIZE,
            NULL,           
            INIT_TASK_PRI,  
            gInitTaskStack, 
            &gInitTaskObj); 
    configASSERT(gInitTask != NULL);
#else
    /* Create this at 2nd highest priority to initialize everything
     * the MMWave_execute task must have a higher priority than this */
   gInitTask = xTaskCreateStatic(
            init_task,   
            "init task", 
            INIT_TASK_SIZE,
            NULL,           
            INIT_TASK_PRI,  
            gInitTaskStack, 
            &gInitTaskObj); 
    configASSERT(gInitTask != NULL);
#endif
    vTaskStartScheduler();

    DebugP_assertNoLog(0);
}
