#ifndef __W25Q128_
#define __W25Q128_
#include "sys.h"
#define W25Q128_CS PBout(14)  // 片选引脚
#define W25Q128_SCLK PBout(3) // spi时钟线
#define W25Q128_MOSI PBout(5) // 主出从入
#define W25Q128_MISO PBin(4)  // 主入从出
extern void w25qxx_simulate_init(void);
extern void w25qxx_init(void);
extern uint8_t SPI1_SendByte(uint8_t txd);
extern void w25qxx_read_id(uint8_t *m_id, uint8_t *d_id);
extern void w25qxx_write_enable(void);
extern void w25qxx_write_disable(void);
extern uint8_t read_status_register(void);
extern void w25qxx_sector_erase(uint32_t sector_addr) ;// 选择扇区地址
extern void w25qxx_page_program(uint32_t page_addr, uint8_t *buf, uint32_t len);
extern void w25qxx_read_data(uint32_t addr, uint8_t *buf, uint32_t len);
#endif
