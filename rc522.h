#ifndef _RC522_H
#define _RC522_H

#include "stm32f10x.h"

u8 RC522_SectorDefault[16] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  // KEYA
	0xff, 0x07, 0x80, 0x69,              // Privilege
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  // KEYB
};

void RC522_Reset  (void);
void RC522_Init   (void);
void RC522_Config (void);

u8 RC522_Request  (u16 *type);
u8 RC522_Anticoll (u8 *id);
u8 RC522_Select   (u8 *id);
u8 RC522_Auth     (u8 mode, u8 addr, u8 *key, u8 *id);
u8 RC522_Read     (u8 addr, u8 *values);
u8 RC522_Write    (u8 addr, u8 *values);
u8 RC522_Halt     (void);

#endif
