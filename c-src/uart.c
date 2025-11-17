#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/uart.h>
#include <stdint.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"


// Writes out samples in a CSV format to the serial console
// buff should point to memory location (Typically HWA output register in this case)
// n    - number of samples 
void uart_dump_samples(void *buff, size_t n){
    int32_t transferOk;
    UART_Transaction transaction;
    size_t charlen;
    uint8_t *txbuff;
    int16_t *samples = (int16_t*)buff;

    UART_Transaction_init(&transaction);
    transaction.args = NULL;

    // Maximum possible length for each sample in characters
    // Includes space for sign, value, comma and nul terminator
    charlen = snprintf(NULL, 0, "%hd,", (int16_t)INT16_MIN) + 1;
    txbuff = calloc(charlen, sizeof(char));

    // Send STX
    *txbuff = 0x2;
    transaction.count = 1;
    transaction.buf = (void*)txbuff;
    UART_write(gUartHandle[0], &transaction);

    // write the actual numbers
    // just assume it'll be 16 bit complex so we have n*2 values 
    for(size_t i = 0; i < n * 2; ++i){
        // probaly a bad idea but it'll work for now
        transaction.count = snprintf((char*)txbuff, charlen, "%hd,", samples[i]);
        UART_write(gUartHandle[0], &transaction);

    }
    // send ETX
    memset(txbuff, 0, charlen * sizeof(char));
    *txbuff = 0x3;
    transaction.count = 1;
    UART_write(gUartHandle[0], &transaction);

    free(txbuff);
}
