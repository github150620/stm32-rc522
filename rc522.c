// rc522.c
//
//      RC522 <--> STM32F103x
//   ----------------------------
//        RST <--> PA3
//   NSS(SDA) <--> PA4(SPI1_NSS)
//        SCK <--> PA5(SPI1_SCK)
//       MISO <--> PA6(SPI1_MISO)
//       MOSI <--> PA7(SPI1_MOSI)
//
#include "rc522.h"
#include "stm32f10x.h"

void SPI1_Init() {
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial     = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE);
}

void RC522_Reset(void) {
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	delay(10);
	GPIO_SetBits(GPIOA, GPIO_Pin_3);
	delay(10);
}

u8 RC522_Read(u8 addr) {
	u16 value;
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, ((addr<<1)&0x7E)|0x80);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	SPI_I2S_ReceiveData(SPI1);
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, 0);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	value = SPI_I2S_ReceiveData(SPI1);	

	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	return value & 0xff;
}

void RC522_Write(u8 addr, u8 value) {
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, (addr<<1)&0x7E);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	SPI_I2S_ReceiveData(SPI1);
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, value);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	SPI_I2S_ReceiveData(SPI1);

	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void RC522_SetRegBits(u8 reg, u8 mask) {
	u8 tmp;
	tmp = RC522_Read(reg);
	RC522_Write(reg, tmp | mask);
}

void RC522_ClearRegBits(u8 reg, u8 mask) {
	u8 tmp;
	tmp = RC522_Read(reg);
	RC522_Write(reg, tmp&(~mask));
}

