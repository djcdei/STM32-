#ifndef __FLASH_H_
#define __FLASH_H_
#include "sys.h"
#define FLASH_SAVE_ADDR ADDR_FLASH_SECTOR_4	  // ����4����
#define ADDR_FLASH_SECTOR_4 ((u32)0x08010000) // ����4��ʼ��ַ

typedef struct __flash_t
{
	uint8_t g_flash_buf[64]; // ��¼ʱ�����ʪ��ƴ���������ַ�����
	uint32_t offset;		 // �ṹ���������е�ƫ����
} flash_t;

extern void flash_init(void);
extern void flash_write(flash_t *flash_write_buf);
extern void flash_read(uint32_t num);
extern uint32_t flash_read_offset(uint32_t *start_addr);
#endif
