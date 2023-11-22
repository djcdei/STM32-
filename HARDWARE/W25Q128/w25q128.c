#include "w25q128.h"

static GPIO_InitTypeDef GPIO_InitStructure;

/*ģ��SPI flash��W25Q128����ʼ��*/
void w25qxx_simulate_init(void)
{
	// ʹ��Ӳ��PB4(��ԭ��ͼ)ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// ����PB3( /CS ),PB5(MOSI),PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 //	����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; // 	����ٶ�
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	 //	�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // 	��ʹ���ڲ�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// ����PB4(MISO)Ϊ����ģʽ����ΪMISO���Ŷ�ȡ����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	 //	����PB4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //	����ģʽ
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// ������ģʽ3��ʱ��������PB3��ʼ��ƽΪ�ߵ�ƽ
	W25Q128_SCLK = 1;
	W25Q128_MOSI = 1;
	W25Q128_CS = 1; // ����Ƭѡ����
}

/*SPI flash��W25Q128����ʼ��*/
void w25qxx_init(void)
{
#if 0
	SPI_InitTypeDef SPI_InitStructure;
	//ʹ��Ӳ��SPI1��PB4(��ԭ��ͼ)ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//����PB3,4,5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	 	//����ģʽ�����żȿ��Ա�Ӳ�����ƣ�Ҳ���Ա�������ƣ�
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed; 	// ����ٶ�
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// ��ʹ���ڲ�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// ��PB3��4��5���ŷֱ���ΪSPI1_SCK��SPI1_MISO��SPI1_MOSI
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
	//����PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;//����PB14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	 	//���ģʽ
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	PBout(14)=1;//����
	
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//SPIһ��ʹ��˫��ȫ˫����SPI�����ܸ��ٸ�����豸ͨ�ţ��ٶȿ�
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//M4������������ɫ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//��ʱ��ͼ������С���ֽڵ�Ԫ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							//��ʱ��ͼ��CPOL=1��SPIʱ���߿���ʱΪ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						//��ʱ��ͼ,	CPHA=1,MISO����Ϊ�ڶ����أ�������Ϊ�����أ���������
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//ͨ������������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	//���ӻ��������ֲᣬ���ܳ��٣�SCLK��Ƶ��=84MHz/16=5.25MHz
	
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//���ӻ��������ֲ�
	SPI_Init(SPI1, &SPI_InitStructure);
	//ʹ��SPI1����
	SPI_Cmd(SPI1, ENABLE);
#else
	w25qxx_simulate_init(); // ģ��spi��ʼ��
#endif
}

/*�Լ���װSPI�������ݺ�����ͬ��ͨ�ţ�������һ���ֽں�ͬʱҲ�᷵�����ݣ�*/
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

	// ѭ��8λbit�����ݣ���λ����(��λ�ȷ�)
	for (i = 7; i >= 0; i--)
	{
		// �жϸ�bitλ�Ƿ�Ϊ�ߵ�ƽ
		if (txd & (1 << i))
		{
			W25Q128_MOSI = 1; // MOSI����Ϊ�ߵ�ƽ
		}
		else
		{
			W25Q128_MOSI = 0;
		}

		// ����ʱ����Ϊ�͵�ƽ��MOSI�����������
		W25Q128_SCLK = 0;
		delay_us(1);
		// ��������ݺ�ͬʱ�������ݵĽ��գ���λ���գ�����Ҫ����ʱ����
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

/*��ȡspi �豸id�ͳ���id����*/
void w25qxx_read_id(uint8_t *m_id, uint8_t *d_id)
{
	// Ƭѡ����PB14����͵�ƽ���ӻ���ʼ����
	PBout(14) = 0;
	// ����0x90(��ʱ��ͼ90h)
	SPI1_SendByte(0x90);
	// ����24bit��ַ�������η���
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);

	// �������������ȡ����id
	*m_id = SPI1_SendByte(0xFF);
	*d_id = SPI1_SendByte(0xFF);
	// ͨ�Ž�����CS��PB14����������
	PBout(14) = 1;
}

// spiдʹ�� WRITE ENABLE
void w25qxx_write_enable(void)
{
	// cs�������ͣ��ӻ���ʼ����
	W25Q128_CS = 0;
	// ����0x06
	SPI1_SendByte(0x06);
	// CS����Ϊ�ߵ�ƽ���ӻ�ֹͣ����
	W25Q128_CS = 1;
}
// spiд��ʹ�� WRITE DISABLE
void w25qxx_write_disable(void)
{
	W25Q128_CS = 0;
	SPI1_SendByte(0x04);
	W25Q128_CS = 1;
}

// ���Ĵ��� Read Status Register��״̬
uint8_t read_status_register(void)
{
	uint8_t sta;
	W25Q128_CS = 0;
	SPI1_SendByte(0x05);
	// ��������8λbit���ݣ���ȡ�Ĵ���״̬
	sta = SPI1_SendByte(0xFF);
	W25Q128_CS = 1;
	return sta;
}

// spi flash��������(sector erase)
void w25qxx_sector_erase(uint32_t sector_addr) // ѡ��������ַ
{
	uint8_t sta;
	// дʹ��
	w25qxx_write_enable();
	delay_us(1);
	W25Q128_CS = 0; // �ӻ���ʼ����
	// ����0x20��������ָ��
	SPI1_SendByte(0x20);
	// ����24bits�ĵ�ַ����
	SPI1_SendByte((sector_addr >> 16) & 0xFF);
	SPI1_SendByte((sector_addr >> 8) & 0xFF);
	SPI1_SendByte(sector_addr & 0xFF);
	// ���read status register�Ĵ�����BUSY ����λ�Ƿ�Ϊ0����ʾ���ݲ�����ϣ�
	while (1)
	{
		sta = read_status_register();
		if ((sta & 0x01) == 0x00)
			break;
		delay_us(1);
	}

	w25qxx_write_disable();
}

// spi flashҳ���(page program)�����������׵�ַ�������׵�ַ�����ݳ���
void w25qxx_page_program(uint32_t page_addr, uint8_t *buf, uint32_t len)
{
	uint8_t sta;
	uint8_t *p = buf;
	// дʹ��
	w25qxx_write_enable();
	// ����0x02ҳ���ָ��
	SPI1_SendByte(0x02);
	// ����24bits�ĵ�ַ����
	SPI1_SendByte((page_addr >> 16) & 0xFF);
	SPI1_SendByte((page_addr >> 8) & 0xFF);
	SPI1_SendByte(page_addr & 0xFF);

	// �ж�len�Ƿ񳬹�256.������Ҫ������һ��ҳ���
	len = len > 256 ? 256 : len;
	// ��λд��buf�е�����
	while (len--)
	{
		SPI1_SendByte(*p);
		p++;
	}

	// CS����Ϊ�ߵ�ƽ���ӻ�ֹͣ����
	W25Q128_CS = 1;

	delay_us(1);
	// ���read status register�Ĵ�����BUSY ����λ�Ƿ�Ϊ0����ʾ����д����ϣ�
	while (1)
	{
		sta = read_status_register();
		if ((sta & 0x01) == 0x00)
			break;
		delay_us(1);
	}
	w25qxx_write_disable();
}
// ��ȡspi���ݸ���W25Q128S�����ֲ�26ҳʱ��ͼ��д,��������ʼ��ַ����Ŷ�ȡ���ݵĻ�������Ҫ��ȡ�����ݳ���
void w25qxx_read_data(uint32_t addr, uint8_t *buf, uint32_t len)
{
	uint8_t *p = buf;

	// CS����Ϊ�͵�ƽ���ӻ���ʼ����
	W25Q128_CS = 0;

	// �ȷ���0x03 ��ȡ����ָ��
	SPI1_SendByte(0x03);

	// ����24bit��ַ���ߵ�ַ�ȷ��ͣ�����ķ���
	SPI1_SendByte((addr >> 16) & 0xFF);
	SPI1_SendByte((addr >> 8) & 0xFF);
	SPI1_SendByte(addr & 0xFF);

	while (len--)
	{
		// ÿ�ζ�ȡ1byte������
		*p = SPI1_SendByte(0xFF);
		p++;
	}
	// CS����Ϊ�ߵ�ƽ���ӻ�ֹͣ����
	W25Q128_CS = 1;
}
