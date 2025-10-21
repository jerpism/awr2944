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
#include <math.h>


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
#include <dataprocessing.h>
#include <types.h>

extern void uart_dump_samples(void *buff, size_t n);

struct cfarcfg{
    uint8_t num_noise;
    uint8_t divfactor;
    uint32_t threshold;
};


extern void uart_read_cfarcfg(struct cfarcfg *cfarcfg);

/* Task related macros */
#define EXEC_TASK_PRI   (configMAX_PRIORITIES-1)     // must be higher than INIT_TASK_PRI
#define MAIN_TASK_PRI   (configMAX_PRIORITIES-3)
#define INIT_TASK_PRI   (configMAX_PRIORITIES-2)
#define EXEC_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define MAIN_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define DPC_TASK_SIZE   (4096U/sizeof(configSTACK_DEPTH_TYPE))
#define INIT_TASK_SIZE  (4096U/sizeof(configSTACK_DEPTH_TYPE))


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

HWA_Handle gHwaHandle[1];

#define GET_SAMPLE_IDX(chirp, rx, rbin ) ( (chirp * (NUM_RX_ANTENNAS) * NUM_RANGEBINS) + (rx * NUM_RANGEBINS) + (rbin) )
#define SQUARE_I16(x) (((int32_t)x) * ((int32_t)x))

struct detected_point{
    uint16_t range;
    uint16_t doppler;
};


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
SemaphoreP_Object gCfarDoneSem;
SemaphoreP_Object gFrameDoneSem;
SemaphoreP_Object gDopplerDoneSem;
SemaphoreP_Object gAngleDoneSem;

/* Rest of them */
volatile bool gState = 0; /* Tracks the current (intended) state of the RSS */
static uint32_t gPushButtonBaseAddr = GPIO_PUSH_BUTTON_BASE_ADDR;

int16imre_t gSampleBuff[NUM_TX_ANTENNAS][NUM_DOPPLER_CHIRPS][NUM_RX_ANTENNAS][NUM_RANGEBINS] __attribute__((section(".bss.dss_l3")));
uint16_t detmatrix[NUM_RANGEBINS][NUM_DOPPLER_CHIRPS];
uint16_t angledets[NUM_RANGEBINS][NUM_TX_ANTENNAS * NUM_RX_ANTENNAS];

// This can and should really be smaller since if we're detecting literally every single point, something is wrong
// but for now that can happen so keep this large
uint32_t gCfarResults[NUM_RANGEBINS * CHIRPS_PER_FRAME * NUM_RX_ANTENNAS] __attribute__((section(".bss.dss_l32")));



static inline void fail(void){
    DebugP_log("Failed\r\n");
    DebugP_assertNoLog(0);
    while(1) __asm__ volatile("wfi");
}


void edma_callback(Edma_IntrHandle handle, void *args){
   // printf("Edma cb\r\n");
    hwa_run(gHwaHandle[0]);
   // HWA_setDMA2ACCManualTrig(gHwaHandle[0], 0);
}


// TODO: combine these 2 cbs to one and use arguments instead
void hwa_cfar_cb(uint32_t intrIdx, uint32_t paramSet, void * arg){
    SemaphoreP_post(&gCfarDoneSem);
}


static volatile int chcounter = 0;
void hwa_callback(uint32_t intrIdx, uint32_t paramSet, void *arg){
    static volatile int next = 3;
    HWA_reset(gHwaHandle[0]);
    if(next == 6){
        chcounter++;
    }
    EDMA_setEvtRegion(EDMA_getBaseAddr(gEdmaHandle[0]), 0, next);



    if(next + 1 > 6){
        next = 3;
    }else{
        ++next;
    }
}



static void frame_done(Edma_IntrHandle handle, void *args){
    HWA_enable(gHwaHandle[0], 0);
    edma_reset_hwal3_param();
    SemaphoreP_post(&gFrameDoneSem);
    printf("Frame done\r\n");

}


void hwa_angle_cb(uint32_t intrIdx, uint32_t paramSet, void * arg){
    SemaphoreP_post(&gAngleDoneSem);
}


// TODO: this shouldn't and  won't stay here
// and all of this should be implemented with DMA anyways
// returns detected peak count
static int run_cfar(){
    int ret = 0;
    uint16_t *hwain = (uint16_t*)(hwa_getaddr(gHwaHandle[0]));
    DSSHWACCRegs *pregs = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
    uint16_t *dmatrix = &detmatrix[0][0];

    // First move our detmatrix to HWA input
    // currently it will contain 128 * 32 = 4096 data points 
    // which coincidentally is the max(+1) for CFARPEAKCNT so assume that limit won't be a problem for now
    for(int i = 0; i < NUM_DOPPLER_CHIRPS * NUM_RANGEBINS; ++i){
        *(hwain + i) =   *(dmatrix + i);
    }

    hwa_cfar_init(gHwaHandle[0], &hwa_cfar_cb);
    hwa_run(gHwaHandle[0]);
    SemaphoreP_pend(&gCfarDoneSem, SystemP_WAIT_FOREVER);

    // 12 bit register
    ret = pregs->CFAR_PEAKCNT & 0x00000fff;
    printf("Detected peaks %d\r\n",ret);

    return ret;

}


static int run_anglecfar(int16imre_t *angles, size_t len){
    int ret = 0;
    int16imre_t *hwain = (int16imre_t*)(hwa_getaddr(gHwaHandle[0]));
    uint32_t *hwaout = (uint32_t*)(hwa_getaddr(gHwaHandle[0]) + 0x1c000);

    DSSHWACCRegs *pregs = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;

    for(int i = 0; i < len; ++i){
        for(int j = 0; j < NUM_TX_ANTENNAS * NUM_RX_ANTENNAS; ++j){
            *(hwain + (i * NUM_TX_ANTENNAS * NUM_RX_ANTENNAS) + j) = *(angles + (i * NUM_TX_ANTENNAS * NUM_RX_ANTENNAS) + j);
        }
    }
    printf("len is %zu\r\n", len);

    hwa_anglecfar_init(gHwaHandle[0], &hwa_cfar_cb, len);
    HWA_reset(gHwaHandle[0]);
    hwa_run(gHwaHandle[0]);
    SemaphoreP_pend(&gCfarDoneSem, SystemP_WAIT_FOREVER);

    ret = pregs->CFAR_PEAKCNT & 0x00000fff;
    printf("Peakcnt is %d\r\n",ret);

    return ret;
}


static struct detected_point *construct_detlist(int peaks){
    uint32_t *hwaout = (uint32_t*)(hwa_getaddr(gHwaHandle[0]) + 0x10000);
    struct detected_point *points = malloc(sizeof(struct detected_point) * peaks);
    
    for(int i = 0; i < peaks; ++i){
        (points+i)->range = (*(hwaout+i) & 0x00fff000) >> 12U;
        (points+i)->doppler = (*(hwaout+i) & 0x00000fff);
    }

    return points;
}


static int16imre_t *run_anglefft(struct detected_point *points, size_t len){
    // First run a 2D FFT (doppler) on the detected ranges
    int16imre_t *hwain = (int16imre_t*)(hwa_getaddr(gHwaHandle[0]));
    int16imre_t *hwaout = (int16imre_t*)(hwa_getaddr(gHwaHandle[0]) + 0x4000);

    // This should probably be a statically allocated array instead
    int16imre_t *angleresults = malloc(len * NUM_RX_ANTENNAS * NUM_TX_ANTENNAS * sizeof(int16imre_t));

    SemaphoreP_constructBinary(&gAngleDoneSem, 0);


    // yes this reimplements a lot of the doppler functionality 
    // but all of this will be replaced with EDMA soon enough hopefully
    for(int i = 0; i < len; ++i){
        hwa_doppler_init(gHwaHandle[0], &hwa_doppler_cb);

        move_doppler_to_hwa(points[i].range);

        HWA_reset(gHwaHandle[0]);
        hwa_run(gHwaHandle[0]);

        SemaphoreP_pend(&gDopplerDoneSem, SystemP_WAIT_FOREVER);

        // For now assume that the doppler bin will be 0 since we're measuring stationary objects
        // Essentially rearranges [TX][RC][DC] into [DC][TX][RX]
        for(int j = 0; j < NUM_TX_ANTENNAS; ++j){
            for(int k = 0; k < NUM_RX_ANTENNAS; ++k){
               *(hwain + (j * NUM_RX_ANTENNAS) + k) = *(hwaout + (j * NUM_RX_ANTENNAS * NUM_DOPPLER_CHIRPS) + (k * NUM_DOPPLER_CHIRPS));
            }
        }

        hwa_angle_init(gHwaHandle[0], &hwa_angle_cb);

        HWA_reset(gHwaHandle[0]);
        hwa_run(gHwaHandle[0]);
        SemaphoreP_pend((&gAngleDoneSem), SystemP_WAIT_FOREVER);

        for(int j = 0; j < NUM_RX_ANTENNAS * NUM_TX_ANTENNAS; ++j){
            *(angleresults + (i * NUM_RX_ANTENNAS * NUM_TX_ANTENNAS) + j) = *(hwaout + j);
        }
    }
    return angleresults;
 
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
    uint8_t header[] = {1,2,3,4};
    uint8_t footer[] = {4,3,2,1};
    static bool firstrun = 1;
    int16_t tmp = 0;

    // TODO: grab this from sysconfig somehow but for now assume bank 2 will be output
    void *hwaout = (void*)(hwa_getaddr(gHwaHandle[0])+0x4000);

    HwiP_enable();
while(1){
    ClockP_usleep(5000);

    while(gState){
        // This might be unnecessary but we have encountered situations
        // in which the system ends up locked with interrupts seemingly disabled
        // so make sure they are re-enabled at the start of each frame 
        HwiP_enable();

        hwa_init(gHwaHandle[0], &hwa_callback);
        HWA_reset(gHwaHandle[0]);
        HWA_enable(gHwaHandle[0], 1U);


        mmw_start(gMmwHandle, &err);

        // Make sure wait ticks is not set to SystemP_WAIT_FOREVER
        // At times the device seems to end up in a deadlock forever looping in the idle task
        // and having a timeout here makes sure that the system never ends up stuck here 
        // this does mean that in some cases a duplicate frame may be sent but operation 
        // seems to resume normally afterwards
        SemaphoreP_pend(&gFrameDoneSem, 500);

        MMWave_stop(gMmwHandle, &err);

        dp_run_doppler(gSampleBuff, detmatrix);


        ClockP_usleep(50000);
        gState = 0;
        continue;


        int peaks = run_cfar();
        struct detected_point *points = construct_detlist(peaks);

        for(int i = 0; i < peaks; ++i){
            printf("R: %hu\r\n", points[i].range);
        }

        int16imre_t *angles = run_anglefft(points, peaks);
        uart_dump_samples(angles, 16);

        free(points);
        free(angles);


        while(1)__asm__("wfi");
        struct detected_point *angledet = NULL;

        int anglepeaks = run_anglecfar(angles, peaks);

        angledet = malloc(anglepeaks * sizeof(struct detected_point));
        uint32_t *hwaout = (uint32_t*)(hwa_getaddr(gHwaHandle[0]) + 0x1c000);

        for(int i = 0; i < anglepeaks; ++i){
            (angledet+i)->range = (*(hwaout+i) & 0x00fff000) >> 12U;
            (angledet+i)->doppler = (*(hwaout+i) & 0x00000fff); //just use the doppler for angle, no point in making another struct for this.
        }


        printf("Angle peaks %d\r\n", anglepeaks);
        for(int i = 0; i < anglepeaks; ++i){
            printf("Range %hu, Angle %hu\r\n",angledet[i].range, angledet[i].doppler);
        }

/*
        DSSHWACCRegs *pregs = (DSSHWACCRegs*)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
        uint16_t peakcnt = pregs->CFAR_PEAKCNT & 0x00000fff;
        printf("%hu\r\n",peakcnt);
*/

        while(1)__asm__("wfi");
        free(angledet);

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
    gHwaHandle[0] = HWA_open(0, NULL, &err);
    if(gHwaHandle[0] == NULL){
        DebugP_logError("Failed to open HWA\r\n");
        fail();
    }

    DebugP_log("Init task launched\r\n");

    DebugP_log("Init HWA...\r\n");
    // assume for now that input memory will be at HWA base
    uint32_t hwaaddr = (uint32_t)SOC_virtToPhy((void*)hwa_getaddr(gHwaHandle[0]));
    hwa_init(gHwaHandle[0], &hwa_callback);
    DebugP_log("HWA address is %#x\r\n",hwaaddr);
    DebugP_log("Done.\r\n");

  //  hwa_cfar_init(gHwaHandle[0], hwa_cfar_cb);

    ret = SemaphoreP_constructBinary(&gCfarDoneSem, 0);
    DebugP_assert(ret == 0);


    DebugP_log("Init network...\r\n");
  //  network_init(NULL);
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
    edma_configure_hwa_l3(gEdmaHandle[0], &frame_done, (void*)&gSampleBuff, (void*)(hwaaddr+0x4000),  CHIRP_DATASIZE,  NUM_DOPPLER_CHIRPS, 1);
   
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

    gMmwProfiles[0] = mmw_create_profile(gMmwHandle, 0, &err);
    gMmwProfiles[1] = mmw_create_profile(gMmwHandle, 1, &err);

    ret = mmw_add_chirps(gMmwProfiles[0], 0, &err);
    if(ret != 0){
        DebugP_logError("Failed to add chirps\r\n");
    }
 
    ret = mmw_add_chirps(gMmwProfiles[1], 1, &err);
    if(ret != 0){
        DebugP_logError("Failed to add chirps\r\n");
    }
 

    ret = mmw_config(gMmwHandle, gMmwProfiles, &err);
    if (ret != 0){
        mmw_printerr("Failed to configure\r\n", err);
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
        //SemaphoreP_post(&gBtnPressedSem);
        printf("Button\r\n");
        gState = !gState;
    }    
}


void chirp_isr(void *arg){
   EDMA_setEvtRegion(EDMA_getBaseAddr(gEdmaHandle[0]), 0, 1);
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
