
/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=8192
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=32768
-e_vectors  /* This is the entry of the application, _vector MUST be plabed starting address 0x0 */

/* This is the size of stack when R5 is in IRQ mode
 * In NORTOS,
 * - Here interrupt nesting is disabled as of now
 * - This is the stack used by ISRs registered as type IRQ
 * In FreeRTOS,
 * - Here interrupt nesting is enabled
 * - This is stack that is used initally when a IRQ is received
 * - But then the mode is switched to SVC mode and SVC stack is used for all user ISR callbacks
 * - Hence in FreeRTOS, IRQ stack size is less and SVC stack size is more
 */
__IRQ_STACK_SIZE = 256;
/* This is the size of stack when R5 is in IRQ mode
 * - In both NORTOS and FreeRTOS nesting is disabled for FIQ
 */
__FIQ_STACK_SIZE = 256;
__SVC_STACK_SIZE = 4096; /* This is the size of stack when R5 is in SVC mode */
__ABORT_STACK_SIZE = 256;  /* This is the size of stack when R5 is in ABORT mode */
__UNDEFINED_STACK_SIZE = 256;  /* This is the size of stack when R5 is in UNDEF mode */

SECTIONS
{
    /* This has the R5F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > R5F_VECS

    /* This has the R5F boot code until MPU is enabled,  this MUST be at a address < 0x80000000
     * i.e this cannot be placed in DDR
     */
    GROUP {
        .text.hwi: palign(8)
        .text.cache: palign(8)
        .text.mpu: palign(8)
        .text.boot: palign(8)
        .text:abort: palign(8) /* this helps in loading symbols when using XIP mode */
    } > MSS_L2

    /* This is rest of code. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .text:   {} palign(8)   /* This is where code resides */
        .rodata: {} palign(8)   /* This is where const's go */
    } > MSS_L2

    /* This is rest of initialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .data:   {} palign(8)   /* This is where initialized globals and static go */
    } > MSS_L2

    /* This is rest of uninitialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .bss:    {} palign(8)   /* This is where uninitialized globals go */
        RUN_START(__BSS_START)
        RUN_END(__BSS_END)
    } > SBL_RESERVED_L2_RAM | MSS_L2

    .sysmem: {} palign(8)  > SBL_RESERVED_L2_RAM | MSS_L2 /* This is where the malloc heap goes */
    .stack:  {} palign(8)  > SBL_RESERVED_L2_RAM | MSS_L2 /* This is where the main() stack goes */

    /* This is where the stacks for different R5F modes go */
    GROUP {
        .irqstack: {. = . + __IRQ_STACK_SIZE;} align(8)
        RUN_START(__IRQ_STACK_START)
        RUN_END(__IRQ_STACK_END)
        .fiqstack: {. = . + __FIQ_STACK_SIZE;} align(8)
        RUN_START(__FIQ_STACK_START)
        RUN_END(__FIQ_STACK_END)
        .svcstack: {. = . + __SVC_STACK_SIZE;} align(8)
        RUN_START(__SVC_STACK_START)
        RUN_END(__SVC_STACK_END)
        .abortstack: {. = . + __ABORT_STACK_SIZE;} align(8)
        RUN_START(__ABORT_STACK_START)
        RUN_END(__ABORT_STACK_END)
        .undefinedstack: {. = . + __UNDEFINED_STACK_SIZE;} align(8)
        RUN_START(__UNDEFINED_STACK_START)
        RUN_END(__UNDEFINED_STACK_END)
    } > SBL_RESERVED_L2_RAM | MSS_L2

    /* Sections needed for C++ projects */
    GROUP {
        .ARM.exidx:  {} palign(8)   /* Needed for C++ exception handling */
        .init_array: {} palign(8)   /* Contains function pointers called before main */
        .fini_array: {} palign(8)   /* Contains function pointers called after main */
    } > MSS_L2

    /* any data buffer needed to be put in L3 can be assigned this section name */
    .bss.dss_l3 {} > DSS_L3

    /* General purpose user shared memory, used in some examples */
  //  .bss.user_shared_mem (NOLOAD) : {} > USER_SHM_MEM
    /* this is used when Debug log's to shared memory are enabled, else this is not used */
//    .bss.log_shared_mem  (NOLOAD) : {} > LOG_SHM_MEM
    /* this is used only when IPC RPMessage is enabled, else this is not used */
    .bss.ipc_vring_mem   (NOLOAD) : {} > RTOS_NORTOS_IPC_SHM_MEM
    /* this is used only when Secure IPC is enabled */
    .bss.sipc_hsm_queue_mem   (NOLOAD) : {} > MAILBOX_HSM
    .bss.sipc_r5f_queue_mem   (NOLOAD) : {} > MAILBOX_R5F

    // For NDK packet memory
    .bss:ENET_CPPI_DESC        (NOLOAD) {} ALIGN (128) > CPPI_DESC

        .enet_dma_mem{
        *(*ENET_DMA_PKT_MEMPOOL)
    } (NOLOAD) {} ALIGN (128) > DSS_L3
    .bss:UDP_IPERF_SND_BUF  (NOLOAD) {} ALIGN (128) > DSS_L3

}
/* L2 memory map
 *     112K       16K        736K        16K           16K
 *   0x1C000     0x4000    0xB8000      0x4000       0x4000
 *  +----------------------------------------------------------+
 *  | SBL RES | CPPI DESC | MSS_L2 | USR_SHM_MEM | LOG_SHM_MEM |
 *  +----------------------------------------------------------+
 *  |         |           |        |             |             |
 * 0x10200000 0x1021C000  |        0x102D8000    0x102DC000    0x102E0000
 *                        0x10220000         
 */                        

MEMORY
{
    R5F_VECS  : ORIGIN = 0x00000000 , LENGTH = 0x00000040
    R5F_TCMA  : ORIGIN = 0x00000040 , LENGTH = 0x0000FFC0
    R5F_TCMB  : ORIGIN = 0x00080000 , LENGTH = 0x00010000
    SBL_RESERVED_L2_RAM (RW)   : origin=0x10200000 length= 0x00040000

    /* CPPI descriptor memory */
    CPPI_DESC : ORIGIN = 0x10240000, LENGTH = 0x00004000

    /* when using multi-core application's i.e more than one R5F active, make sure
     * this memory does not overlap with other R5F's
     */
    MSS_L2      : ORIGIN = (0x10244000) , LENGTH = 0x400000


    /* shared memories that are used by RTOS/NORTOS cores */
    /* On R5F,
     * - make sure there are MPU entries which maps below regions as non-cache
     * Might not be needed so these could be removed to get 32K extra for MMS_L2
     */
    //USER_SHM_MEM            : ORIGIN = (0x102DC000 - 0x4000), LENGTH = 0x00004000
    //LOG_SHM_MEM             : ORIGIN = (0x102E0000 - 0x4000), LENGTH = 0x00004000

    /* This is typically used to hold data IO buffers from accelerators like CSI, HWA */
    DSS_L3:   ORIGIN = 0x88000000, LENGTH = 0x000F0000

    /* 1st 512 B of DSS mailbox memory and MSS mailbox memory is used for IPC with R4 and should not be used by application */
    /* MSS mailbox memory is used as shared memory, we dont use bottom 32*6 bytes, since its used as SW queue by ipc_notify */
    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0xC5000200, LENGTH = 0xD40
    MAILBOX_HSM:    ORIGIN = 0x44000000 , LENGTH = 0x000003CE
    MAILBOX_R5F:    ORIGIN = 0x44000400 , LENGTH = 0x000003CE
}
