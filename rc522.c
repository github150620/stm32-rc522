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

// Table 5: MFRC522 Registers Overview
// Page 0: Command and Status
#define Reserved00            0x00    
#define CommandReg            0x01    
#define ComIEnReg             0x02    
#define DivIEnReg             0x03    
#define ComIrqReg             0x04    
#define DivIrqReg             0x05
#define ErrorReg              0x06    
#define Status1Reg            0x07    
#define Status2Reg            0x08    
#define FIFODataReg           0x09
#define FIFOLevelReg          0x0A
#define WaterLevelReg         0x0B
#define ControlReg            0x0C
#define BitFramingReg         0x0D
#define CollReg               0x0E
#define Reserved0F            0x0F
// Page 1: Command    
#define Reserved10            0x10
#define ModeReg               0x11
#define TxModeReg             0x12
#define RxModeReg             0x13
#define TxControlReg          0x14
#define TxAutoReg             0x15
#define TxSelReg              0x16
#define RxSelReg              0x17
#define RxThresholdReg        0x18
#define DemodReg              0x19
#define Reserved1A            0x1A
#define Reserved1B            0x1B
#define MifareReg             0x1C
#define Reserved1D            0x1D
#define Reserved1E            0x1E
#define SerialSpeedReg        0x1F
// Page 2: CFG    
#define Reserved20            0x20  
#define CRCResultRegM         0x21
#define CRCResultRegL         0x22
#define Reserved23            0x23
#define ModWidthReg           0x24
#define Reserved25            0x25
#define RFCfgReg              0x26
#define GsNReg                0x27
#define CWGsCfgReg            0x28
#define ModGsCfgReg           0x29
#define TModeReg              0x2A
#define TPrescalerReg         0x2B
#define TReloadRegH           0x2C
#define TReloadRegL           0x2D
#define TCounterValueRegH     0x2E
#define TCounterValueRegL     0x2F
// Page 3: TestRegister    
#define Reserved30            0x30
#define TestSel1Reg           0x31
#define TestSel2Reg           0x32
#define TestPinEnReg          0x33
#define TestPinValueReg       0x34
#define TestBusReg            0x35
#define AutoTestReg           0x36
#define VersionReg            0x37
#define AnalogTestReg         0x38
#define TestDAC1Reg           0x39  
#define TestDAC2Reg           0x3A   
#define TestADCReg            0x3B   
#define Reserved3C            0x3C   
#define Reserved3D            0x3D   
#define Reserved3E            0x3E   
#define Reserved3F            0x3F

// Table 11: CommIEnReg register (address 02h); reset value: 80h
#define IRqInv        0x80
#define TxIEn         0x40
#define RxIEn         0x20
#define IdleIEn       0x10
#define HiAlertIEn    0x08
#define LoAlertIEn    0x04
#define ErrIEn        0x02
#define TimerIEn      0x01

// Table 15: CommIRqReg register (address 04h); reset value: 14h
#define Set1        0x80
#define TxIRq       0x40
#define RxIRq       0x20
#define IdleIRq     0x10
#define HiAlertIRq  0x08
#define LoAlertIRq  0x04
#define ErrIRq      0x02
#define TimerIRq    0x01

// Table 17: DivIRqReg register (address 05h); reset value: X0h
#define Set2        0x80
#define MfinActIRq  0x10
#define CRCIRq      0x04

// Table 19: ErrorReg register (address 06h); reset value: 00h
#define WrErr       0x80
#define TempErr     0x40
#define BufferOvfl  0x10
#define CollErr     0x08
#define CRCErr      0x04
#define ParityErr   0x02
#define ProtocolErr 0x01

// Table 21: Status1Reg register (address 07h); reset value: 21h
#define CRCOk       0x40
#define CRCReady    0x20
#define IRq         0x10
#define TRunning    0x08
#define HiAlert     0x02
#define LoAlert     0x01

// Table 23: Status2Reg register (address 08h); reset value: 00h
#define TempSensClear  0x80
#define I2CForceHS     0x40
#define MFCrypto1On    0x08
#define ModemState     0x07

// Table 27: FIFOLevelReg register (address 0Ah); reset value: 00h
#define FlushBuffer    0x80
#define FIFOLevel      0x7F

// Table 31: ControlReg register (address 0Ch); reset value: 10h
#define TStopNow       0x80
#define TStartNow      0x40
#define RxLastBits     0x07

// Table 33: BitFramingReg register (address 0Dh); reset value: 00h
#define StartSend      0x80
#define RxAlign        0x70
#define TxLastBits     0x07

// Table 35: CollReg register (address 0Eh); reset value: XXh
#define ValuesAfterColl  0x80
#define CollPosNotValid  0x20
#define CollPos          0x1F

// Table 149: Command overview
#define Idle              0x00
#define Mem               0x01
#define GenerateRandmID   0x02
#define CalcCRC           0x03
#define Transmit          0x04
#define NoCmdChange       0x07
#define Receive           0x08
#define Transceive        0x0C
#define MFAuthent         0x0E
#define SoftReset         0x0F

#define RBUF_MAX_LEN 18

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

u8 RC522_SPIWriteAndRead(u8 data) {
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, data);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	return SPI_I2S_ReceiveData(SPI1);	
}

u8 RC522_ReadReg(u8 addr) {
	u16 value;
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	RC522_SPIWriteAndRead(((addr<<1)&0x7E)|0x80);
	value = RC522_SPIWriteAndRead(0);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	return value & 0xff;
}

void RC522_WriteReg(u8 addr, u8 value) {
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	RC522_SPIWriteAndRead((addr<<1)&0x7E);
	RC522_SPIWriteAndRead(value);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void RC522_SetRegBits(u8 reg, u8 mask) {
	u8 tmp;
	tmp = RC522_ReadReg(reg);
	RC522_WriteReg(reg, tmp | mask);
}

void RC522_ClearRegBits(u8 reg, u8 mask) {
	u8 tmp;
	tmp = RC522_ReadReg(reg);
	RC522_WriteReg(reg, tmp&(~mask));
}

void RC522_ReadFIFO(u8 *buf, u8* len) {
	*len = RC522_ReadReg(FIFOLevelReg);
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	RC522_SPIWriteAndRead(((FIFODataReg<<1)&0x7E)|0x80);
	for (u8 i=0;i<*len;i++) {
		buf[i] = RC522_SPIWriteAndRead(((FIFODataReg<<1)&0x7E)|0x80);
	}
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void RC522_WriteFIFO(u8 *buf, u8 len) {	
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	RC522_SPIWriteAndRead((FIFODataReg<<1)&0x7E);
	for (u8 i=0;i<len;i++) {
		RC522_SPIWriteAndRead(buf[i]);
	}
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void RC522_AntennaOn(void) {
	u8 v;
	v = RC522_ReadReg(TxControlReg);
	if (!(v&0x03)) {
		RC522_SetRegBits(TxControlReg, 0x03);
	}
}

void RC522_AntennaOff(void) {
	RC522_ClearRegBits(TxControlReg, 0x03);
}

void RC522_Reset(void) {
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	delay(100);
	GPIO_SetBits(GPIOA, GPIO_Pin_3);
	delay(100);
}

void RC522_Init(void) {
	SPI1_Init();
	RC522_Reset();
	
	RC522_WriteReg(CommandReg, 0x0f);
	while (RC522_ReadReg(CommandReg)&0x10) {
	}
	
	RC522_WriteReg(ModeReg, 0x3d);

	RC522_WriteReg(TReloadRegL, 30);
	RC522_WriteReg(TReloadRegH, 0);
	
	RC522_WriteReg(TModeReg, 0x8d);
	RC522_WriteReg(TPrescalerReg, 0x3e);
	RC522_WriteReg(TxAutoReg, 0x40);
	
	RC522_WriteReg(ComIEnReg, 0x77|0x80);	
}

void RC522_Config(void) {
	RC522_ClearRegBits(Status2Reg, 0x08);
	RC522_WriteReg(ModeReg, 0x3d);
	RC522_WriteReg(RxSelReg, 0x86);
	RC522_WriteReg(RFCfgReg, 0x7f);
	RC522_WriteReg(TReloadRegL, 30);
	RC522_WriteReg(TReloadRegH, 0);
	RC522_WriteReg(TModeReg, 0x8d);
	RC522_WriteReg(TPrescalerReg, 0x3e);
	RC522_AntennaOff();
	delay(200);
	RC522_AntennaOn();
}

u8 RC522_Transceive(u8 *wbuf, u8 wLen, u8 *rbuf, u8 *rLen) {
	u8 err = 0;
	u8 value;
	
	RC522_SetRegBits(FIFOLevelReg, FlushBuffer);
	RC522_ClearRegBits(ComIrqReg, Set1);
	RC522_WriteFIFO(wbuf, wLen);
	RC522_WriteReg(CommandReg, Transceive);
	RC522_SetRegBits(BitFramingReg, StartSend);
	while (1) {
		value = RC522_ReadReg(ComIrqReg);
		if (value&RxIRq) {
			RC522_ReadFIFO(rbuf, rLen);
			err = 0;
			break;
		}
		if (value&IdleIRq) {
			*rLen = 0;
			err = 0;
			break;
		}
		if (value&ErrIRq) {
			printf("[ERROR] RC522_Transceive(), ErrorReg: %02X\n", RC522_ReadReg(ErrorReg));
			err = 1;
			break;
		}
		if (value&TimerIRq) {
			err = 2;
			break;
		}
	}
	RC522_SetRegBits(ControlReg, TStopNow);
	RC522_WriteReg(CommandReg, Idle);
	return err;
}

u8 RC522_MFAuthent(u8 *wbuf, u8 wLen) {
	u8 err = 0;
	u8 value;

	RC522_SetRegBits(FIFOLevelReg, FlushBuffer);
	RC522_ClearRegBits(ComIrqReg, Set1);
	RC522_WriteFIFO(wbuf, wLen);
	RC522_WriteReg(CommandReg, MFAuthent);

	while (1) {
		value = RC522_ReadReg(ComIrqReg);
		if (value&IdleIRq) {
			err = 0;
			break;
		}
		if (value&ErrIRq) {
			printf("[Error] RC522_MFAuthent(), ErrorReg: %02X\n", RC522_ReadReg(ErrorReg));
			err = 1;
			break;
		}
		if (value&TimerIRq) {
			err = 2;
			break;
		}
	}
	RC522_SetRegBits(ControlReg, TStopNow);
	RC522_WriteReg(CommandReg, Idle);
	return err;
}

u8 RC522_Request(u16 *type) {
	u8 err;
	u8 wbuf[2];
	u8 rbuf[10];
	u8 rlen;
	
	//wbuf[0] = 0x26;  // REQA
	wbuf[0] = 0x52;  // WAKE-UP
	RC522_WriteReg(BitFramingReg, 0x07&TxLastBits); // REQA or WAKE-UP command is only 7 bits.	
	err = RC522_Transceive(wbuf, 1, rbuf, &rlen);
	if (err != 0) {
		return 1;
	}
	
	if (rlen != 2) {
		return 2;
	}

	*type = rbuf[0];
	*type <<= 8;
	*type |= rbuf[1];

	return 0;
}

u8 RC522_Anticoll(u8 *id) {
	u8 wbuf[2];
	u8 rbuf[RBUF_MAX_LEN];
	u8 rlen;
	u8 err;
	u8 check;
		
	RC522_ClearRegBits(CollReg, ValuesAfterColl);
	RC522_ClearRegBits(Status2Reg, MFCrypto1On);
	
	wbuf[0] = 0x93; // SEL
	wbuf[1] = 0x20; // NVB
	RC522_WriteReg(BitFramingReg, 0&RxLastBits);
	err = RC522_Transceive(wbuf, 2, rbuf, &rlen);
	if (err != 0) {
		return 1;
	}
	
	if (rlen != 5) {
		return 2;
	}
	
	check = 0;
	for (u8 i=0;i<5;i++) {
		id[i] = rbuf[i];
		check ^= rbuf[i];
	}
	
	if (check != 0x00) {
		return 3;
	}
	
	printf("CollReg: 0x%02X\n", RC522_ReadReg(CollReg));
	
	RC522_SetRegBits(CollReg, ValuesAfterColl);

	return 0;
}

void RC522_CalcCRC(u8 *buf, u8 len, u8 *crc) {
	u8 value;
	RC522_ClearRegBits(DivIrqReg, CRCIRq);
	RC522_WriteFIFO(buf, len);
	RC522_WriteReg(CommandReg, CalcCRC);
	while (1) {
		value = RC522_ReadReg(DivIrqReg);
		if (value&CRCIRq) {
			break;
		}
	}
	crc[0] = RC522_ReadReg(CRCResultRegL);
	crc[1] = RC522_ReadReg(CRCResultRegM);
	RC522_WriteReg(CommandReg, Idle);
}

u8 RC522_Select(u8 *id) {
	u8 wbuf[9];
	u8 rbuf[RBUF_MAX_LEN];
	u8 rlen;
	u8 err;
	
	wbuf[0] = 0x93;  // SEL
	wbuf[1] = 0x70;  // NVB
	wbuf[2] = id[0]; // UID CLn
	wbuf[3] = id[1];
	wbuf[4] = id[2];
	wbuf[5] = id[3];
	wbuf[6] = id[4];
	RC522_CalcCRC(wbuf, 7, &wbuf[7]);
	
	err = RC522_Transceive(wbuf, 9, rbuf, &rlen);
	if (err != 0) {
		return 1;
	}

	if (rlen != 3) {
		return 2;
	}

	return 0;	
}

u8 RC522_Auth(u8 mode, u8 addr, u8 *key, u8 *id) {
	u8 wbuf[12];
	u8 err;
	
	wbuf[0] = mode;
	wbuf[1] = addr;
	memcpy(&wbuf[2], key, 6);
	memcpy(&wbuf[8], id, 4);
	err = RC522_MFAuthent(wbuf, 12);
	if (err != 0) {
		return 1;
	}

	return 0;
}

u8 RC522_Read(u8 addr, u8 *values) {
	u8 wbuf[4];
	u8 rbuf[RBUF_MAX_LEN];
	u8 rlen;
	u8 err;
	
	wbuf[0] = 0x30; // READ
	wbuf[1] = addr;
	RC522_CalcCRC(wbuf, 2, &wbuf[2]);
	RC522_SetRegBits(Status2Reg, MFCrypto1On);
	err = RC522_Transceive(wbuf, 4, rbuf, &rlen);
	if (err != 0) {
		return 1;
	}

	if (rlen != 18) {
		return 2;
	}
	
	memcpy(values, rbuf, 16);
	
	return 0;
}

u8 RC522_Write(u8 addr, u8 *values) {
	u8 wbuf[18];
	u8 rbuf[RBUF_MAX_LEN];
	u8 rlen;
	u8 err;
	
	wbuf[0] = 0xA0; // WRITE
	wbuf[1] = addr;
	RC522_CalcCRC(wbuf, 2, &wbuf[2]);
	RC522_SetRegBits(Status2Reg, MFCrypto1On);
	err = RC522_Transceive(wbuf, 4, rbuf, &rlen);
	if (err != 0) {
		return 1;
	}
	
	if ((rlen&0x0F) != 1) {
		return 2;
	}
	
	if (rbuf[0] != 0x0A) {
		return 3;
	}
	
	memcpy(wbuf, values, 16);
	RC522_CalcCRC(wbuf, 16, &wbuf[16]);
	err = RC522_Transceive(wbuf, 18, rbuf, &rlen);
	if (err != 0) {
		return 4;
	}
	
	if ((rlen&0x0F) != 1) {
		return 5;
	}
	
	if (rbuf[0] != 0x0A) {
		return 6;
	}

	return 0;
}	

u8 RC522_Halt(void) {
	u8 wbuf[4];
	u8 rbuf[RBUF_MAX_LEN];
	u8 rlen;
	u8 err;
	
	wbuf[0] = 0x50; // HALT
	wbuf[1] = 0;
	RC522_CalcCRC(wbuf, 2, &wbuf[2]);
	RC522_ClearRegBits(Status2Reg, MFCrypto1On);
	err = RC522_Transceive(wbuf, 4, rbuf, &rlen);
	if (err != 0) {
		return 1;
	}

	if (rlen != 1) {
		return 2;
	}
	
	return 0;
}
