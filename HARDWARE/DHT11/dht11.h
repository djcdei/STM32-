#ifndef __DHT11_H_
#define __DHT11_H_

#include "sys.h"

extern void dht11_init(void);
extern int32_t dht11_read(uint8_t *buf);

#endif
