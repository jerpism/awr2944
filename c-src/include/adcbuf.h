#ifndef ADCBUF_H
#define ADCBUF_H

#include <drivers/adcbuf.h>

/* Initialize the ADC Buffer 
 * sets some default settings that work with the MMWave DFE
 * handle will be set to the ADCBuf handle 
 * returns 0 on success, <0 otherwise */
ADCBuf_Handle adcbuf_init(void);

#endif /* ADCBUF_H */
