#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "string.h"

//periferiche
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c2;
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern u8 emergenza;
extern u8 BTattivo;
//update attivo
u8 updateAttivo = 0;

//pacchetti totali
u16 paccTot = 0;

extern u8 riavvio;

extern u16 riavvioForzato;

extern u32 lastSMS;
extern u8 lastNumber[20];

u16 packTotGSM = 0;
u8 updateGSMatt = 0;
u16 NpackRecGSM = 0;
u8 packRecFlag = 0;
u8 packGSM[500];
u16 crcGSM;
u16 updateGSMvers;
u8 downloadNewPackFlag = 0;
u8 bytes16[16] = {0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

extern u8 riavvia;

extern u8 serialeDaScrivere[4];

u8 progPacchetto(u8 *inBuf, u16 nPacchetto, u16 paccTot){
	
	u32 address;
	u32 progVar;
	static u8 app;
	int i;
	u8 OK[4] = "OK\r\n";
	u8 uart[100];
	

	

	HAL_FLASH_Unlock();

	
	sprintf(uart,"posizione 1, %d\n",nPacchetto);
	HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	if(nPacchetto > paccTot){
		return 0;
	}
	updateAttivo = 5;
	
	riavvioForzato = timeoutModulo;
	

	if(nPacchetto < paccTot/2){ 
		
		//operazioni primo pacchetto
		if(nPacchetto == 0){		
			//controllo su che pezzo di flash sono
			address = *(u32*)0x08008000;
			if(address == 0x11111111){

				app = 1;
			}
			else{

				app = 0;		
			}
			
			//pacchetto[2+i] + pacchetto[3+i]*256 + pacchetto[4+i]*65536 + pacchetto[5+i]*16777216;
			for(i=0;i<18;i++){
				progVar = inBuf[4*i] + inBuf[4*i+1]*256 + inBuf[4*i+2]*65536 + inBuf[4*i+3]*16777216;
				address = 0x08008200 + 229376*app + 4*i;
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
				FLASH_WaitForLastOperation(1000);			
			}
			
			//mando OK
			if(BTattivo == 1){
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
			}
			
		}
		
		else if(nPacchetto != 0 && nPacchetto != ((paccTot/2)-1)){
			//scrivo il pacchetto
			for(i=0;i<18;i++){
				progVar = inBuf[4*i] + inBuf[4*i+1]*256 + inBuf[4*i+2]*65536 + inBuf[4*i+3]*16777216;
				address = 0x08008200 + 229376*app + 72*nPacchetto + 4*i;
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);	
				FLASH_WaitForLastOperation(1000);				
			}
			
			//mando OK
			if(BTattivo == 1){
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
			}
			
		}
		else {
			//scrivo l'ultimo pacchetto
			for(i=0;i<18;i++){
				progVar = inBuf[4*i] + inBuf[4*i+1]*256 + inBuf[4*i+2]*65536 + inBuf[4*i+3]*16777216;
				address = 0x08008200 + 229376*app + 72*nPacchetto + 4*i;
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
			FLASH_WaitForLastOperation(1000);				
			}
			
			progVar = 0x77777777;
			address = 0x08008000;
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
			FLASH_WaitForLastOperation(1000);		
						
			//mando OK
			if(BTattivo == 1){
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				
			}
			HAL_UART_Transmit(&huart1,&OK[0],4,1000);
			
		}
		
	}
	else{
		if(BTattivo == 1){
			HAL_UART_Transmit(&huart2,&OK[0],4,1000);
		}
		
		if(nPacchetto == paccTot-1){
				HAL_Delay(1000);
				//riavvioUMR();
				HAL_TIM_Base_Stop(&htim4);		
		}
	
	}
		
}


u8 progPacchetto3(u8 *inBuf, u16 nPacchetto, u16 paccTot){
	
	u32 address;
	u32 progVar;
	static u8 app;
	int i;
	u8 OK[4] = "OK\r\n";
	u8 uart[100];
	

	HAL_FLASH_Unlock();
	
	
	sprintf(uart,"posizione 2, %d\n",nPacchetto);
	HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	if(nPacchetto > paccTot){
		return 0;
	}
	updateAttivo = 5;
	
	
	riavvioForzato = timeoutModulo;

	if(nPacchetto < paccTot/2){ 
		
		if(BTattivo == 1){
			HAL_UART_Transmit(&huart2,&OK[0],4,1000);
		}
		
		
	}
	else{
		nPacchetto = nPacchetto - (paccTot/2);
		//operazioni primo pacchetto
		if(nPacchetto == 0){		
			//controllo su che pezzo di flash sono
			address = *(u32*)0x08008000;
			if(address == 0x11111111){

				app = 1;
			}
			else{

				app = 0;		
			}
			
			//pacchetto[2+i] + pacchetto[3+i]*256 + pacchetto[4+i]*65536 + pacchetto[5+i]*16777216;
			for(i=0;i<18;i++){
				progVar = inBuf[4*i] + inBuf[4*i+1]*256 + inBuf[4*i+2]*65536 + inBuf[4*i+3]*16777216;
				address = 0x08008200 + 229376*app + 4*i;
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
				FLASH_WaitForLastOperation(1000);			
			}
			
			//mando OK
			if(BTattivo == 1){
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
			}
			
		}
		
		else if(nPacchetto != 0 && nPacchetto != ((paccTot/2)-1)){
			//scrivo il pacchetto
			for(i=0;i<18;i++){
				progVar = inBuf[4*i] + inBuf[4*i+1]*256 + inBuf[4*i+2]*65536 + inBuf[4*i+3]*16777216;
				address = 0x08008200 + 229376*app + 72*nPacchetto + 4*i;
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);	
				FLASH_WaitForLastOperation(1000);				
			}
			
			//mando OK
			if(BTattivo == 1){
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
			}
			HAL_UART_Transmit(&huart1,&OK[0],4,1000);
			
		}
		else {
			//scrivo l'ultimo pacchetto
			for(i=0;i<18;i++){
				progVar = inBuf[4*i] + inBuf[4*i+1]*256 + inBuf[4*i+2]*65536 + inBuf[4*i+3]*16777216;
				address = 0x08008200 + 229376*app + 72*nPacchetto + 4*i;
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
			FLASH_WaitForLastOperation(1000);				
			}
			
			progVar = 0x77777777;
			address = 0x08040000;
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
			FLASH_WaitForLastOperation(1000);	
			
			//mando OK
			if(BTattivo == 1){
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
			}
			
			
			HAL_Delay(1000);
			HAL_TIM_Base_Stop(&htim4);
			
			
			
		}

	}
		
}

void formattaFlashInterna(void){
	u32 address,address3;
	
	HAL_FLASH_Unlock();
	address = *(u32*)0x08008000;
	address3 = *(u32*)0x08008200;
	
	if(address != 0x11111111){
			FLASH_Erase_Sector(FLASH_SECTOR_2,VOLTAGE_RANGE_3);
			resetWD();
			FLASH_WaitForLastOperation(1000);
			FLASH_Erase_Sector(FLASH_SECTOR_3,VOLTAGE_RANGE_3);
			resetWD();
			FLASH_WaitForLastOperation(1000);
			FLASH_Erase_Sector(FLASH_SECTOR_4,VOLTAGE_RANGE_3);
			resetWD();
			FLASH_WaitForLastOperation(1000);
			FLASH_Erase_Sector(FLASH_SECTOR_5,VOLTAGE_RANGE_3);
			resetWD();
			FLASH_WaitForLastOperation(1000);		
	}
	
	address = *(u32*)0x08040000;
	address3 = *(u32*)0x08040200;
	
		if(address != 0x11111111){
			FLASH_Erase_Sector(FLASH_SECTOR_6,VOLTAGE_RANGE_3);
			resetWD();
			FLASH_WaitForLastOperation(1000);
			FLASH_Erase_Sector(FLASH_SECTOR_7,VOLTAGE_RANGE_3);
			resetWD();
			FLASH_WaitForLastOperation(1000);	
	}
	
}

void scriviSeriale(void){
	u32 address = 0x08007ff8;
	u32 progVar = 0x30303030;
	
		
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
	FLASH_WaitForLastOperation(1000);
	
	progVar = serialeDaScrivere[0]<<24;
	progVar |= serialeDaScrivere[1]<<16;
	progVar |= serialeDaScrivere[2]<<8;
	progVar |= serialeDaScrivere[3];
	
	address = 0x08007ffc;
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
	FLASH_WaitForLastOperation(1000);
	
}


void JumpToApp(int app){
	uint32_t app1,app2,address;
	
	if(app == 1){
		address = 0x08008200;
	}
	else if(app == 3){
		address = 0x08040200;
	}
	else{
		return;
	}
	//address = APP2_BASE_ADDRESS;
	void (*app_reset_handler)(void);
	uint32_t msp_value = *(volatile uint32_t *)address;
	
	__set_MSP(msp_value);
	
	uint32_t resethandler_address = *(volatile uint32_t *) (address + 4);
	
	app_reset_handler = (void*) resethandler_address;
	
	SCB->VTOR = address;
	
	app_reset_handler();
	
	return;	
}

void installaPacchettoGSM(void){
	u32 address;
	u32 progVar;
	int i = 0;
	u8 uart[100];
	
	address = 0x08040200;
	
	while(i < 240){
		//progVar = packGSM[i] + packGSM[i+1]*256 + packGSM[i+2]*65536 + packGSM[i+3]*16777216;
		progVar = packGSM[i+3];
		progVar = (progVar << 8) | packGSM[i+2];
		progVar = (progVar << 8) | packGSM[i+1];
		progVar = (progVar << 8) | packGSM[i];
		
		/*if(progVar <= 0x0f){
			sprintf(uart,"0000000%X",progVar);
		}
		else if(progVar > 0x0f && progVar <= 0xff){
			sprintf(uart,"000000%X",progVar);
		}
		else if(progVar > 0xff && progVar <= 0xfff){
			sprintf(uart,"00000%X",progVar);
		}
		else if(progVar > 0xfff && progVar <= 0xffff){
			sprintf(uart,"0000%X",progVar);
		}
		else if(progVar > 0xffff && progVar <= 0xfffff){
			sprintf(uart,"000%X",progVar);
		}
		else if(progVar > 0xfffff && progVar <= 0xffffff){
			sprintf(uart,"00%X",progVar);
		}
		else if(progVar > 0xffffff && progVar <= 0xfffffff){
			sprintf(uart,"0%X",progVar);
		}
		else{
			sprintf(uart,"%X",progVar);
		}*/
		
		
		address = 0x08040200 + (NpackRecGSM-1)*240 + i;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
		FLASH_WaitForLastOperation(1000);

		//sprintf(uart,"%d\n",address);
		//HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
		
		i = i+4;
		
	}
	
	//HAL_UART_Transmit(&huart1,(u8*)"\n",1,100);
	
}

u8 posizioneFlash(void){
	
	u32 address;
	u8 app = 0;
	
	address = *(u32*)0x08008000;
	if(address == 0x11111111){
			app = 1;
	}
	
	return app;
}

void validazioneUpdate(void){
	u32 address;
	u32 validatore = 0x77777777;
	
	
	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"download finished",17);
	
	if(posizioneFlash() == 1){
		address = 0x08040000;
	}
	else{
		address = 0x08008000;
	}
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,validatore);
	FLASH_WaitForLastOperation(1000);	
	
	//HAL_TIM_Base_Stop(&htim4);	
	riavvia = 1;
	
}

void testFlash(void){
	u32 address;
	u32 progVar;
	int i = 0;
	u8 uart[100];
	
	address = 0x08040200;
	
	while(i < 16){
		//progVar = packGSM[i]*16777216 + packGSM[i+1]*65536 + packGSM[i+2]*256 + packGSM[i+3];
		progVar = bytes16[i+3];
		progVar = (progVar << 8) | bytes16[i+2];
		progVar = (progVar << 8) | bytes16[i+1];
		progVar = (progVar << 8) | bytes16[i];
		
		address = 0x08040200 + i;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
		FLASH_WaitForLastOperation(1000);
		
		sprintf(uart,"%x %x\n",progVar,address);
		HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
		
		
		i = i+4;
		
	}

}
