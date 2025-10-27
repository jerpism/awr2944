#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <drivers/uart.h>
#include <hwa.h>
#include <kernel/dpl/DebugP.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

#define MAX_ARGS 32
#define CMD_CNT 4

extern HWA_Handle gHwaHandle[1];

// Implemented in main.c 
extern void sensor_start(void);
extern void sensor_stop(void);
extern void single_shot(void);

struct command {
    char *cmd;
    void (*fp)(int argc, char *argv[]);
};

static struct command cmd_table[CMD_CNT];

static void cfarCfg(int argc, char *argv[]){
    struct cfar_cfg cfg;

    // spooky VLA for halloween season
    // argv[0] is the command name so skip that
    int args[argc - 1];

    for(int i = 1; i < argc; ++i){
        args[i-1] = strtol(argv[i], NULL, 10);
    }


    // There's probably a better way to do this but initialize everything to -1 first
    // so we can check what arguments were actually provided 
    cfg.num_noise_l = -1;
    cfg.num_noise_r = -1;
    cfg.num_guard = -1;
    cfg.avg_div_fact = -1;
    cfg.thresh_divd = -1;
    cfg.thresh_divs = -1;


    // Again we don't care about the name of the command here
    switch(argc - 1){
        case 6:
        cfg.thresh_divs = args[5];

        case 5:
        cfg.thresh_divd = args[4];

        case 4:
        cfg.avg_div_fact = args[3];

        case 3:
        cfg.num_guard = args[2];

        case 2:
        cfg.num_noise_r = args[1];

        case 1:
        cfg.num_noise_l = args[0];

        default:
        break;


    }

    hwa_cfg_cfar(gHwaHandle[0], cfg);

    DebugP_log("Args provided\r\n");
    for(int i = 0; i < argc - 1; ++i){
        DebugP_log("%d\r\n",args[i]);
    }

}

static void start(int argc, char *argv[]){
    sensor_start();
}

static void stop(int argc, char *argv[]){
    sensor_stop();
}

static void single(int argc, char *argv[]){
    single_shot();
}

static inline void init_commands(void){
    int i = 0;

    cmd_table[i].cmd = "cfarCfg";
    cmd_table[i].fp = &cfarCfg;
    ++i;

    cmd_table[i].cmd = "start";
    cmd_table[i].fp = &start;
    ++i;

    cmd_table[i].cmd = "stop";
    cmd_table[i].fp = &stop;
    ++i;

    cmd_table[i].cmd = "single";
    cmd_table[i].fp = &single;
    ++i;
}


static int parse_cmd(char *cmd, char **out){
    int argc = 0;
    char *tmp;
    tmp = strtok(cmd, " ");
    while(tmp && argc < MAX_ARGS){
        out[argc++] = tmp;
        tmp = strtok(NULL, " ");
    }

    return argc;
}

// These 2 could be combined into a single function maybe
static inline void send_crlf(void){
    const char *str = "\r\n";
    // Do the same as UART_Transaction_init
    // sans the buffer and count of course
    const UART_Transaction trans = {
        .buf = str,
        .count = 3,
        .args = NULL,
        .timeout = SystemP_WAIT_FOREVER,
        .status = UART_TRANSFER_STATUS_SUCCESS
    };

    UART_write(gUartHandle[0], &trans);
}


static inline void send_backspace(void){
    const char *str = "\b \b";
    // Do the same as UART_Transaction_init
    // sans the buffer and count of course
    const UART_Transaction trans = {
        .buf = str,
        .count = 3,
        .args = NULL,
        .timeout = SystemP_WAIT_FOREVER,
        .status = UART_TRANSFER_STATUS_SUCCESS
    };

    UART_write(gUartHandle[0], &trans);

}

void send_prompt(void){
    const char *prompt = "$ ";
    UART_Transaction trans = {
        .buf = prompt,
        .count = 2,
        .args = NULL,
        .timeout = SystemP_WAIT_FOREVER,
        .status = UART_TRANSFER_STATUS_SUCCESS
    };
    UART_write( gUartHandle[0], &trans);
    
}

void cli_init(void){
    init_commands();
}

void cli_main(){
    int argc = 0;
    char *argv[MAX_ARGS];
    char buff[128];
    char c;
    int count = 0;


    UART_Transaction trans;
    UART_Transaction_init(&trans);
    trans.count = 1;
    trans.buf = &c;

    memset(buff, 0, sizeof(buff));
    send_prompt();

    while(1){
        UART_read( gUartHandle[0], &trans);

        if(count > 126){ break;}

        switch(c){
            case '\b':
            case 0x7F: // DEL
                if(count > 0){ --count; }
                send_backspace();
                continue;
            break;

            case '\r':
            case '\n':
                send_crlf();
                goto exit;
            break;

            default:
            break;
        }

        buff[count++] = c;
        UART_write(gUartHandle[0], &trans);

    }

exit:
    
    buff[count+1] = 0;
    count = 0;
    argc = parse_cmd(buff, argv);

    for(int i = 0; i < CMD_CNT; ++i){
        if(strcmp(argv[0], cmd_table[i].cmd) == 0){
            cmd_table[i].fp(argc, argv);
            break;
        }
    }



}