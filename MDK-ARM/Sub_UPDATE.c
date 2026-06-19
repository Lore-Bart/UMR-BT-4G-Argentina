#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "string.h"
#include "stdio.h"

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


#define BT_UPDATE_TIMEOUT_TICKS          15U
#define BT_UPDATE_PAYLOAD_BYTES          72U
#define BT_UPDATE_WORDS_PER_PACKET       18U
#define BT_UPDATE_APP_SLOT_SIZE_BYTES    229376UL
#define BT_UPDATE_APP1_MARKER_ADDR       0x08008000UL
#define BT_UPDATE_APP1_BASE_ADDR         0x08008200UL
#define BT_UPDATE_APP3_MARKER_ADDR       0x08040000UL
#define BT_UPDATE_APP_VALID_MARKER       0x77777777UL
#define BT_UPDATE_ACTIVE_MARKER          0x11111111UL


static void btUpdateWaitInternalFlashReadyWD(void)
{
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET){
		resetWD();
	}
}

u8 internalFlashEraseSectorWD(uint32_t sector)
{
	/* FLASH_Erase_Sector() e HAL_FLASHEx_Erase() sono bloccanti: su sector erase
	 * possono superare 1 s e far scattare il watchdog esterno. Qui avvio l'erase
	 * a registro e attendo BSY alimentando resetWD().
	 */
	HAL_FLASH_Unlock();
	btUpdateWaitInternalFlashReadyWD();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	FLASH->CR &= ~(FLASH_CR_SNB | FLASH_CR_PSIZE);
	FLASH->CR |= FLASH_CR_SER | (sector << 3) | FLASH_PSIZE_WORD;
	FLASH->CR |= FLASH_CR_STRT;

	btUpdateWaitInternalFlashReadyWD();
	FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);

	if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR) != RESET){
		return 0;
	}
	return 1;
}

static u32 btUpdateWordFromPacket(u8 *inBuf, int wordIndex)
{
	u32 progVar;
	int i = wordIndex * 4;
	progVar = inBuf[i+3];
	progVar = (progVar << 8) | inBuf[i+2];
	progVar = (progVar << 8) | inBuf[i+1];
	progVar = (progVar << 8) | inBuf[i];
	return progVar;
}

static u8 btUpdateProgramWordVerified(u32 address, u32 progVar)
{
	int retry;
	for(retry=0; retry<3; retry++){
		if(*(volatile u32*)address == progVar){
			return 1;
		}
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
		btUpdateWaitInternalFlashReadyWD();
		if(*(volatile u32*)address == progVar){
			return 1;
		}
	}
	return 0;
}

static void btUpdateSendOK(void)
{
	u8 OK[4] = "OK\r\n";
	if(BTattivo == 1){
		HAL_UART_Transmit(&huart2,&OK[0],4,1000);
	}
}

static void btUpdateSendERR(void)
{
	u8 ER[4] = "ER\r\n";
	if(BTattivo == 1){
		HAL_UART_Transmit(&huart2,&ER[0],4,1000);
	}
}

static void btUpdateRebootAfterFinalOK(void)
{
	/* Il protocollo Android non viene modificato: prima viene inviato l'OK finale,
	   poi si lascia un breve margine alla UART/BT e si forza il reset software. */
	inviaDebug("BT update final OK sent, software reboot now\n");
	resetWD();
	HAL_Delay(500);
	HAL_FLASH_Lock();
	__disable_irq();
	NVIC_SystemReset();
	while(1){ }
}

static u8 btUpdateProgram72Verified(u32 baseAddress, u16 packetIndex, u8 *inBuf)
{
	int i;
	u32 address;
	u32 progVar;
	for(i=0;i<BT_UPDATE_WORDS_PER_PACKET;i++){
		progVar = btUpdateWordFromPacket(inBuf,i);
		address = baseAddress + ((u32)BT_UPDATE_PAYLOAD_BYTES * packetIndex) + ((u32)4 * i);
		if(btUpdateProgramWordVerified(address,progVar) == 0){
			u8 uart[120];
			sprintf(uart,"BT update flash verify failed pkt %u addr 0x%08X\n", (unsigned int)packetIndex, (unsigned int)address);
			inviaDebug(uart);
			return 0;
		}
		resetWD();
	}
	return 1;
}

extern u8 riavvia;

extern u8 serialeDaScrivere[4];

u8 progPacchetto(u8 *inBuf, u16 nPacchetto, u16 paccTot){
	
	u32 address;
	static u8 app = 0;
	static u8 appReady = 0;
	u16 halfPackets;
	u32 baseAddress;
	u8 uart[120];
	
	HAL_FLASH_Unlock();
	
	if(paccTot < 2 || nPacchetto >= paccTot){
		sprintf(uart,"BT update invalid packet %u/%u\n", (unsigned int)nPacchetto, (unsigned int)paccTot);
		inviaDebug(uart);
		btUpdateSendERR();
		updateAttivo = 0;
		emergenza = 3;
		return 0;
	}
	
	updateAttivo = BT_UPDATE_TIMEOUT_TICKS;
	riavvioForzato = timeoutModulo;
	halfPackets = paccTot/2;
	
	/* Questa funzione programma la prima meta' del file update, mantenendo il protocollo app originale. */
	if(nPacchetto < halfPackets){
		if(nPacchetto == 0 || appReady == 0){
			address = *(volatile u32*)BT_UPDATE_APP1_MARKER_ADDR;
			if(address == BT_UPDATE_ACTIVE_MARKER){
				app = 1;
			}
			else{
				app = 0;
			}
			appReady = 1;
			sprintf(uart,"BT update target slot base: 0x%08X\n", (unsigned int)(BT_UPDATE_APP1_BASE_ADDR + (BT_UPDATE_APP_SLOT_SIZE_BYTES * app)));
			inviaDebug(uart);
		}
		baseAddress = BT_UPDATE_APP1_BASE_ADDR + (BT_UPDATE_APP_SLOT_SIZE_BYTES * app);
		
		if(btUpdateProgram72Verified(baseAddress,nPacchetto,inBuf) == 0){
			btUpdateSendERR();
			updateAttivo = 0;
			emergenza = 3;
			return 0;
		}
		
		if(nPacchetto == (halfPackets-1)){
			if(btUpdateProgramWordVerified(BT_UPDATE_APP1_MARKER_ADDR,BT_UPDATE_APP_VALID_MARKER) == 0){
				inviaDebug("BT update marker verify failed app1\n");
				btUpdateSendERR();
				updateAttivo = 0;
				emergenza = 3;
				return 0;
			}
			inviaDebug("BT update first image half programmed and verified\n");
		}
		
		btUpdateSendOK();
		return 1;
	}
	else{
		/* Seconda meta' ignorata su questo slot, come da protocollo originale. */
		btUpdateSendOK();
		if(nPacchetto == paccTot-1){
			inviaDebug("BT update completed, reboot requested\n");
			btUpdateRebootAfterFinalOK();
		}
		return 1;
	}
}


u8 progPacchetto3(u8 *inBuf, u16 nPacchetto, u16 paccTot){
	
	u32 address;
	static u8 app = 0;
	static u8 appReady = 0;
	u16 halfPackets;
	u16 localPacket;
	u32 baseAddress;
	u8 uart[120];
	
	HAL_FLASH_Unlock();
	
	if(paccTot < 2 || nPacchetto >= paccTot){
		sprintf(uart,"BT update invalid packet %u/%u\n", (unsigned int)nPacchetto, (unsigned int)paccTot);
		inviaDebug(uart);
		btUpdateSendERR();
		updateAttivo = 0;
		emergenza = 3;
		return 0;
	}
	
	updateAttivo = BT_UPDATE_TIMEOUT_TICKS;
	riavvioForzato = timeoutModulo;
	halfPackets = paccTot/2;
	
	/* Questa funzione programma la seconda meta' del file update, mantenendo il protocollo app originale. */
	if(nPacchetto < halfPackets){
		btUpdateSendOK();
		return 1;
	}
	else{
		localPacket = nPacchetto - halfPackets;
		if(localPacket == 0 || appReady == 0){
			address = *(volatile u32*)BT_UPDATE_APP1_MARKER_ADDR;
			if(address == BT_UPDATE_ACTIVE_MARKER){
				app = 1;
			}
			else{
				app = 0;
			}
			appReady = 1;
			sprintf(uart,"BT update target slot base: 0x%08X\n", (unsigned int)(BT_UPDATE_APP1_BASE_ADDR + (BT_UPDATE_APP_SLOT_SIZE_BYTES * app)));
			inviaDebug(uart);
		}
		baseAddress = BT_UPDATE_APP1_BASE_ADDR + (BT_UPDATE_APP_SLOT_SIZE_BYTES * app);
		
		if(btUpdateProgram72Verified(baseAddress,localPacket,inBuf) == 0){
			btUpdateSendERR();
			updateAttivo = 0;
			emergenza = 3;
			return 0;
		}
		
		if(localPacket == (halfPackets-1)){
			if(btUpdateProgramWordVerified(BT_UPDATE_APP3_MARKER_ADDR,BT_UPDATE_APP_VALID_MARKER) == 0){
				inviaDebug("BT update marker verify failed app3\n");
				btUpdateSendERR();
				updateAttivo = 0;
				emergenza = 3;
				return 0;
			}
			inviaDebug("BT update second image half programmed and verified\n");
			btUpdateSendOK();
			btUpdateRebootAfterFinalOK();
			return 1;
		}
		
		btUpdateSendOK();
		return 1;
	}
}


void formattaFlashInterna(void){
	u32 address,address3;
	
	HAL_FLASH_Unlock();
	address = *(u32*)0x08008000;
	address3 = *(u32*)0x08008200;
	
	if(address != 0x11111111){
			internalFlashEraseSectorWD(FLASH_SECTOR_2);
			internalFlashEraseSectorWD(FLASH_SECTOR_3);
			internalFlashEraseSectorWD(FLASH_SECTOR_4);
			internalFlashEraseSectorWD(FLASH_SECTOR_5);		
	}
	
	address = *(u32*)0x08040000;
	address3 = *(u32*)0x08040200;
	
		if(address != 0x11111111){
			internalFlashEraseSectorWD(FLASH_SECTOR_6);
			internalFlashEraseSectorWD(FLASH_SECTOR_7);	
	}
	
}

void scriviSeriale(void){
	u32 address = 0x08007ff8;
	u32 progVar = 0x30303030;
	
		
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
	btUpdateWaitInternalFlashReadyWD();
	
	progVar = serialeDaScrivere[0]<<24;
	progVar |= serialeDaScrivere[1]<<16;
	progVar |= serialeDaScrivere[2]<<8;
	progVar |= serialeDaScrivere[3];
	
	address = 0x08007ffc;
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,progVar);
	btUpdateWaitInternalFlashReadyWD();
	
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
