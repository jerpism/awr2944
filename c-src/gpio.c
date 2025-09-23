#include <stdint.h>
#include <stdbool.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/DebugP.h>
#include <ti/common/syscommon.h>

#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_board_config.h"

static uint32_t gLedAddr;

bool led_state(bool state){
    if(state == 1){
        GPIO_pinWriteHigh(gLedAddr, GPIO_LED_PIN);
    }else{
        GPIO_pinWriteLow(gLedAddr,GPIO_LED_PIN);
    }

    return state;
}


uint32_t gpio_init(void *btn_isr){
    int32_t ret = 0;
    uint32_t gpio_base = (uint32_t)AddrTranslateP_getLocalAddr(GPIO_PUSH_BUTTON_BASE_ADDR);
    const uint32_t pinnum = GPIO_PUSH_BUTTON_PIN;
    const uint32_t intrnum = GPIO_PUSH_BUTTON_INTR_LEVEL == GPIO_INTR_LEVEL_HIGH ? 
                        GPIO_PUSH_BUTTON_INTR_HIGH : GPIO_PUSH_BUTTON_INTR_LOW;

    const uint32_t ledaddr = (uint32_t)AddrTranslateP_getLocalAddr(GPIO_LED_BASE_ADDR);
    gLedAddr = ledaddr;
    const uint32_t ledpin = GPIO_LED_PIN;
    GPIO_setDirMode(ledaddr, ledpin, GPIO_LED_DIR);
    GPIO_setDirMode(gpio_base, pinnum, GPIO_PUSH_BUTTON_DIR);

    ret = GPIO_ignoreOrHonorPolarity(gpio_base, pinnum, GPIO_PUSH_BUTTON_TRIG_TYPE);
    if(ret != 0){ DebugP_log("Failed to ignore or honor polarity\r\n");}

    ret = GPIO_setTrigType(gpio_base, pinnum, GPIO_PUSH_BUTTON_TRIG_TYPE);
    if(ret != 0){ DebugP_log("Failed to set trig type\r\n");}

    ret = GPIO_markHighLowLevelInterrupt(gpio_base, pinnum, GPIO_PUSH_BUTTON_INTR_LEVEL);
    if(ret != 0){ DebugP_log("Failed to mark\r\n");}

    ret = GPIO_clearInterrupt(gpio_base, pinnum);
    if(ret != 0){ DebugP_log("Failed to clear\r\n");}

    ret = GPIO_enableInterrupt(gpio_base, pinnum);
    if(ret != 0){ DebugP_log("Failed to enable\r\n");}

    HwiP_Object hwiobj;
    HwiP_Params params;
    HwiP_Params_init(&params);
    params.intNum = intrnum;
    params.args = (void*)pinnum;
    params.callback = btn_isr;
    ret = HwiP_construct(&hwiobj, &params);
    if(ret != 0){ DebugP_log("Failed to construct\r\n");}
    HwiP_enable();

    return gpio_base;
}