#include "flash.h"
#include <stdio.h>


/*flash��ʼ��������������д������*/
void flash_init(void)
{
	// ����FLASH�����ܸ�д���ݣ���������̣�
	FLASH_Unlock();
	// ��������4����λʱ���ڲ���32bit
	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
					FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	FLASH_EraseSector(FLASH_Sector_4, VoltageRange_3);

	// ����FLASH��ֻ�ܶ�������д��Ҳ���ܲ���
	FLASH_Lock();
}

/*�Լ���װ�Ľ�����д��flash�ĺ���*/
void flash_write(flash_t *flash_write_buf)
{
	int i = 0;
	FLASH_Unlock();

	for (i = 0; i < sizeof(flash_t) / 4; i++) // ����д��,һ����4���ֽڣ�����Ҫ����4
	{
		FLASH_ProgramWord(0x08010000 + flash_write_buf->offset * sizeof(flash_t) + i * 4, *((uint32_t *)flash_write_buf + i));
	}

	FLASH_Lock();
}

void flash_read(uint32_t num)
{
	int i = 0;
	volatile uint32_t *p;
	flash_t flash_read_buf;
	FLASH_Unlock();

	for (i = 0; i < num; i++)
	{
		flash_read_buf = *((volatile flash_t *)(0x08010000 + i * sizeof(flash_t)));
		p = (volatile uint32_t *)((0x08010000 + i * sizeof(flash_t)));
		if (*p == 0xffffffff)
		{
			printf("%08x\r\n:", *p);
		}
		else
		{
			my_USART_SendData(USART3, flash_read_buf.g_flash_buf);
		}
			
	}

	FLASH_Lock();
}

// ����flash����4��δ������ݵĵ�ַ���׵�ַ��ƫ����
uint32_t flash_read_offset(uint32_t *start_addr)
{
	volatile uint32_t *p = start_addr;
	uint32_t count = 0;
	FLASH_Unlock();
	while (*p != 0xffffffff)
	{
		count++;
		p = (uint32_t *)((uint8_t *)p + sizeof(flash_t));
	}
	FLASH_Lock();
	return count; // ���ص�ǰ���ݿ�
}


