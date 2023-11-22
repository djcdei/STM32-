#include "w25q128.h"

static GPIO_InitTypeDef GPIO_InitStructure;

/*模拟SPI flash（W25Q128）初始化*/
void w25qxx_simulate_init(void)
{
	// 使能硬件PB4(看原理图)时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// 配置PB3( /CS ),PB5(MOSI),PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 //	配置为输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; // 	最高速度
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	 //	推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 	不使能内部上下拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 配置PB4(MISO)为输入模式，作为MISO引脚读取数据
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	 //	引脚PB4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //	输入模式
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 工作在模式3，时钟线引脚PB3初始电平为高电平
	W25Q128_SCLK = 1;
	W25Q128_MOSI = 1;
	W25Q128_CS = 1; // 拉高片选引脚
}

/*SPI flash（W25Q128）初始化*/
void w25qxx_init(void)
{
#if 0
	SPI_InitTypeDef SPI_InitStructure;
	//使能硬件SPI1和PB4(看原理图)时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//配置PB3,4,5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	 	//复用模式（引脚既可以被硬件控制，也可以被软件控制）
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; 	// 最高速度
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// 不使能内部上下拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 将PB3、4、5引脚分别复用为SPI1_SCK、SPI1_MISO、SPI1_MOSI
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
	//配置PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;//引脚PB14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 	//输出模式
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	PBout(14)=1;//拉高
	
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//SPI一般使用双线全双工，SPI总线能高速跟多个设备通信，速度快
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//M4工作在主机角色
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//看时序图，以最小的字节单元
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							//看时序图，CPOL=1，SPI时钟线空闲时为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						//看时序图,	CPHA=1,MISO设置为第二边沿（在这里为上升沿）采样数据
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//通过软件代码控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	//看从机的数据手册，不能超速，SCLK的频率=84MHz/16=5.25MHz
	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//看从机的数据手册
	SPI_Init(SPI1, &SPI_InitStructure);
	//使能SPI1工作
	SPI_Cmd(SPI1, ENABLE);
#else
	w25qxx_simulate_init(); // 模拟spi初始化
#endif
}

/*自己封装SPI发送数据函数（同步通信，发送完一个字节后同时也会返回数据）*/
uint8_t SPI1_SendByte(uint8_t txd)
{
#if 0
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI1, txd);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
#else
	int32_t i;
	uint8_t recv_data = 0;

	// 循环8位bit的数据，逐位发送(高位先发)
	for (i = 7; i >= 0; i--)
	{
		// 判断该bit位是否为高电平
		if (txd & (1 << i))
		{
			W25Q128_MOSI = 1; // MOSI设置为高电平
		}
		else
		{
			W25Q128_MOSI = 0;
		}

		// 设置时钟线为低电平，MOSI引脚输出数据
		W25Q128_SCLK = 0;
		delay_us(1);
		// 输出完数据后同时进行数据的接收（高位先收），就要拉高时钟线
		W25Q128_SCLK = 1; //
		delay_us(1);
		if (W25Q128_MISO)
		{
			recv_data |= (1 << i);
		}
	}
	return recv_data;
#endif
}

/*读取spi 设备id和厂商id数据*/
void w25qxx_read_id(uint8_t *m_id, uint8_t *d_id)
{
	// 片选引脚PB14输出低电平，从机开始工作
	PBout(14) = 0;
	// 发送0x90(看时序图90h)
	SPI1_SendByte(0x90);
	// 发送24bit地址，分三次发送
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);

	// 传递任意参数获取厂商id
	*m_id = SPI1_SendByte(0xFF);
	*d_id = SPI1_SendByte(0xFF);
	// 通信结束将CS（PB14）引脚拉高
	PBout(14) = 1;
}

// spi写使能 WRITE ENABLE
void w25qxx_write_enable(void)
{
	// cs引脚拉低，从机开始工作
	W25Q128_CS = 0;
	// 发送0x06
	SPI1_SendByte(0x06);
	// CS引脚为高电平，从机停止工作
	W25Q128_CS = 1;
}
// spi写不使能 WRITE DISABLE
void w25qxx_write_disable(void)
{
	W25Q128_CS = 0;
	SPI1_SendByte(0x04);
	W25Q128_CS = 1;
}

// 读寄存器 Read Status Register的状态
uint8_t read_status_register(void)
{
	uint8_t sta;
	W25Q128_CS = 0;
	SPI1_SendByte(0x05);
	// 发送任意8位bit数据，获取寄存器状态
	sta = SPI1_SendByte(0xFF);
	W25Q128_CS = 1;
	return sta;
}

// spi flash扇区擦除(sector erase)
void w25qxx_sector_erase(uint32_t sector_addr) // 选择扇区地址
{
	uint8_t sta;
	// 写使能
	w25qxx_write_enable();
	delay_us(1);
	W25Q128_CS = 0; // 从机开始工作
	// 发送0x20扇区擦除指令
	SPI1_SendByte(0x20);
	// 发送24bits的地址数据
	SPI1_SendByte((sector_addr >> 16) & 0xFF);
	SPI1_SendByte((sector_addr >> 8) & 0xFF);
	SPI1_SendByte(sector_addr & 0xFF);
	// 监测read status register寄存器的BUSY 比特位是否为0（表示数据擦除完毕）
	while (1)
	{
		sta = read_status_register();
		if ((sta & 0x01) == 0x00)
			break;
		delay_us(1);
	}

	w25qxx_write_disable();
}

// spi flash页编程(page program)参数：扇区首地址、数据首地址、数据长度
void w25qxx_page_program(uint32_t page_addr, uint8_t *buf, uint32_t len)
{
	uint8_t sta;
	uint8_t *p = buf;
	// 写使能
	w25qxx_write_enable();
	// 发送0x02页编程指令
	SPI1_SendByte(0x02);
	// 发送24bits的地址数据
	SPI1_SendByte((page_addr >> 16) & 0xFF);
	SPI1_SendByte((page_addr >> 8) & 0xFF);
	SPI1_SendByte(page_addr & 0xFF);

	// 判断len是否超过256.超过则要进行新一轮页编程
	len = len > 256 ? 256 : len;
	// 逐位写入buf中的数据
	while (len--)
	{
		SPI1_SendByte(*p);
		p++;
	}

	// CS引脚为高电平，从机停止工作
	W25Q128_CS = 1;

	delay_us(1);
	// 监测read status register寄存器的BUSY 比特位是否为0（表示数据写入完毕）
	while (1)
	{
		sta = read_status_register();
		if ((sta & 0x01) == 0x00)
			break;
		delay_us(1);
	}
	w25qxx_write_disable();
}
// 读取spi数据根据W25Q128S资料手册26页时序图编写,参数：起始地址、存放读取数据的缓冲区，要读取的数据长度
void w25qxx_read_data(uint32_t addr, uint8_t *buf, uint32_t len)
{
	uint8_t *p = buf;

	// CS引脚为低电平，从机开始工作
	W25Q128_CS = 0;

	// 先发送0x03 读取数据指令
	SPI1_SendByte(0x03);

	// 发送24bit地址（高地址先发送）巧妙的方法
	SPI1_SendByte((addr >> 16) & 0xFF);
	SPI1_SendByte((addr >> 8) & 0xFF);
	SPI1_SendByte(addr & 0xFF);

	while (len--)
	{
		// 每次读取1byte的数据
		*p = SPI1_SendByte(0xFF);
		p++;
	}
	// CS引脚为高电平，从机停止工作
	W25Q128_CS = 1;
}
