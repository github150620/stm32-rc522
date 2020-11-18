#include "stm32f10x.h"
#include "rc522.h"

int main(void) {
	u8 err;
	u16 type;
	u8 id[5];
	u8 buf[18];
	u8 addr;
	u8 i;
	
	USART1_Init();
	printf("Init RC522...\n");
	RC522_Init();
	printf("Config RC522...\n");
	RC522_Config();
	
	while (1) {
		printf(".");
		type = 0;
		err = RC522_Request(&type);
		if (err != 0) {
			delay(10000000);
			continue;
		}
		printf("Request: %d\n", err);
		printf("type: %04X\n", type);

		printf("\nAnticoll\n");
		err = RC522_Anticoll(id);
		printf("Anticoll: %d\n", err);
		if (err != 0) {
			delay(10000000);
			continue;
		}
		printf("ID: %02X %02X %02X %02X %02X\n", id[0], id[1], id[2], id[3], id[4]);

		printf("\nSelect\n");
		err = RC522_Select(id);
		printf("Select: %d\n", err);
		if (err != 0) {
			delay(10000000);
			continue;
		}
		
		printf("\nRead\n");
		for (addr=0;addr<16;addr++){
			printf("%02x: ", addr);
			err = RC522_Auth(0x61, addr, KEYB, id);
			if (err != 0) {
				printf("Auth: %d\n", err);
				break;
			}

			err = RC522_Read(addr, buf);
			if (err != 0) {
				printf("Read: %d\n", err);
				break;
			}
			printf("%02X %02X %02X %02X %02X %02X %02X %02X ", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
			printf("%02X %02X %02X %02X %02X %02X %02X %02X\n", buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
		}
		
		/*
		printf("\nWrite\n");
		addr = 1;
		err = RC522_Auth(0x60, addr, KEYA, id);
		if (err != 0) {
			printf("Auth: %d\n", err);
		} else {
			err = RC522_Write(addr, block);
			if (err != 0) {
				printf("Write: %d\n", err);
			}
		}
		*/

		printf("\nHalt\n");
		err = RC522_Halt();
		printf("Halt: %d\n", err);
		
		delay(50000000);
	}
}
