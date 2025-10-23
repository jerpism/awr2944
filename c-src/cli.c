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
#define CMD_CNT 1

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

    cfg.num_noise_l     = args[0];
    cfg.num_noise_r     = args[1];
    cfg.num_guard       = args[2];
    cfg.avg_div_fact    = args[3];
    cfg.thresh_divd     = args[4];
    cfg.thresh_divs     = args[5];
    DebugP_log("Args provided\r\n");
    for(int i = 0; i < argc - 1; ++i){
        DebugP_log("%d\r\n",args[i]);
    }

}

static inline void init_commands(void){
    int i = 0;

    cmd_table[i].cmd = "cfarCfg";
    cmd_table[i].fp = &cfarCfg;
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


void cli_main(){
    int argc = 0;
    char *argv[MAX_ARGS];
    char buff[128];
    char c;
    int count = 0;

    init_commands();

    UART_Transaction trans;
    UART_Transaction_init(&trans);
    trans.count = 1;
    trans.buf = &c;

    // TODO: might improve this with separate functions for special functionality as this is clumsy right now
    // but whatever, backspace works 
    while(1){
        UART_read( gUartHandle[0], &trans);

        if(count > 126){ break;}

        // Screen sends DEL so check for that too
        if(c == '\b' || c == 0x7F){
            if(count > 0){ --count; }

            c = '\b';
            UART_write(gUartHandle[0], &trans);
            c = ' ';
            UART_write(gUartHandle[0], &trans);
            c = '\b';
            UART_write(gUartHandle[0], &trans);
            continue;
        }

        if(c == '\r' || c == '\n'){
            c = '\r';
            UART_write(gUartHandle[0], &trans);
            c = '\n';
            UART_write(gUartHandle[0], &trans);

            break;
        }

        buff[count++] = c;
        UART_write(gUartHandle[0], &trans);

    }
    
    buff[count+1] = 0;

    argc = parse_cmd(buff, argv);

    for(int i = 0; i < CMD_CNT; ++i){
        if(strcmp(argv[0], cmd_table[i].cmd) == 0){
            cmd_table[i].fp(argc, argv);
            break;
        }
    }



}