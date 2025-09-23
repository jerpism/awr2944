#ifndef NETWORK_H
#define NETWORK_H
#include <stdint.h>
void network_init(void *args);
void udp_send_data(void *buff, size_t n);
#endif
