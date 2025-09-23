#ifndef GPIO_H
#define GPIO_H
#include <stdint.h>

uint32_t gpio_init(void *btn_isr);
bool led_state(bool state);

#endif /* GPIO_H */