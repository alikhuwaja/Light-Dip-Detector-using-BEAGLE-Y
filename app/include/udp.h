#ifndef UDP_H
#define UDP_H

#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stddef.h>   

bool udp_start(uint16_t port, _Atomic bool *request_exit);
void udp_stop(void);
bool udp_send(const void *data, size_t len);

#endif 
