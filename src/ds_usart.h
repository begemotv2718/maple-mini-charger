#ifndef _DSO_USART_H
#define _DSO_USART_H
#include <stdint.h>
void dallas_usart_setup(void);
int dallas_reset(void);
void dallas_send(uint8_t *command,int len);
void dallas_recv(uint8_t *buffer, int len);
#endif
