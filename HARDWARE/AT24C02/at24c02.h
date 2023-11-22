#ifndef __AT24C02_H_
#define __AT24C02_H_
#include "sys.h"

#define AT24C02_SCL_W PBout(8)
#define AT24C02_SDA_W PBout(9)
#define AT24C02_SDA_R PBin(9)
extern void at24c02_init(void);
extern void at24c02_sda_pin_mode(GPIOMode_TypeDef pin_mode);
extern void at24c02_i2c_start(void);
extern void at24c02_i2c_stop(void); 
extern int32_t at24c02_write(uint8_t addr,uint8_t *buf,uint32_t len);
extern int32_t at24c02_read(uint8_t addr,uint8_t *buf,uint32_t len);
extern void at24c02_i2c_send_byte(uint8_t byte);
extern uint8_t at24c02_i2c_wait_ack(void);
#endif
