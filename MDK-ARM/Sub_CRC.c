#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "stm32f4xx_hal_rtc.h"

extern UART_HandleTypeDef huart1;

uint16_t M24SR_UpdateCrc(uint8_t ch, uint16_t *pwCrc){

	ch = (ch^(uint8_t)((*pwCrc) & 0x00ff));
	ch = (ch^(ch<<4));
	*pwCrc = (*pwCrc >> 8)^((uint16_t)ch<<8)^((uint16_t)ch<<3)^((uint16_t)ch>>4);
	
	return(*pwCrc);

}

void M24SR_ComputeCrc(uint8_t *Data, uint8_t Lenght){
	uint8_t chBlock;
	uint16_t wCrc;
	uint8_t outBuf[2];
	u8 uart[100];
	
	wCrc = 0x6363;
	
	do{
		chBlock = *Data++;
		M24SR_UpdateCrc(chBlock, &wCrc);
	} while(--Lenght);
	
	//u162array(&outBuf[0],wCrc);
	//invertiArray(&outBuf[0],2);	
	outBuf[0] = wCrc & 0x00ff;
	outBuf[1] = (wCrc & 0xff00) >> 8;
	
	//sprintf(uart,"CRC: %x %x\n",outBuf[0], outBuf[1]);
	//HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	Data[Lenght] = outBuf[0];
	Data[Lenght+1] = outBuf[1];
}

u16 CRC_flash(u32 addStart, u32 lenght){
	uint8_t chBlock;
	uint16_t wCrc;
	u8 uart[100];
	u32 offset = 0;
	u8 bytes[16];
	int i = 0;
	
	wCrc = 0x6363;
	//lenght = 250;
	
	do{
		chBlock = *(u8*)(addStart + offset);
		//bytes[i] = chBlock;
		M24SR_UpdateCrc(chBlock, &wCrc);
		offset = offset + 1;
		i++;
		resetWD();
	} while(--lenght);
	
	//sprintf(uart,"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8], bytes[9], bytes[10], bytes[11], bytes[12], bytes[13], bytes[14], bytes[15]);
	//sprintf(uart,"%x %x\n", bytes[0], bytes[1]);
	//HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	sprintf(uart,"CRC: %x\n",wCrc);
	HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	return wCrc;
	
}