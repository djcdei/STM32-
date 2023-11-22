#include "flash.h"
#include <stdio.h>


/*flash初始化（擦除）方便写入数据*/
void flash_init(void)
{
	// 解锁FLASH，才能改写数据（擦除、编程）
	FLASH_Unlock();
	// 擦除扇区4，单位时间内擦除32bit
	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
					FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	FLASH_EraseSector(FLASH_Sector_4, VoltageRange_3);

	// 锁定FLASH，只能读，不能写，也不能擦除
	FLASH_Lock();
}

/*自己封装的将数据写入flash的函数*/
void flash_write(flash_t *flash_write_buf)
{
	int i = 0;
	FLASH_Unlock();

	for (i = 0; i < sizeof(flash_t) / 4; i++) // 按字写入,一个字4个字节，所以要除以4
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

// 计算flash扇区4还未存放数据的地址到首地址的偏移量
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
	return count; // 返回当前数据块
}


