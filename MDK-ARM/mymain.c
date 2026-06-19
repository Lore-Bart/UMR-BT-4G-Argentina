#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "stdio.h"
#include "string.h"

/* Richiesta livello segnale SIM7600: prima lettura 10 s dopo aggancio rete, poi ogni 5 minuti. */
#define SIGNAL_REQUEST_FIRST_DELAY_MS   10000UL
#define SIGNAL_REQUEST_INTERVAL_MS      300000UL

/* SMS: forziamo text mode dopo aggancio modulo e dopo eventuale riaggancio rete. */
#define SMS_TEXTMODE_FIRST_DELAY_MS     15000UL
#define SMS_TEXTMODE_RETRY_MS           60000UL

//periferiche
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c2;
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

//variabili Rx/Tx
extern uint8_t tx[200];
extern uint8_t rxBT[500];
extern uint8_t rx4G[500];


extern u8 risultatoTestMemorie[4];


//salvataggio attivato
extern u8 salvataggioLoad;
extern u8 salvataggioMeas;

//identificativo
u8 identificativo[16] = "UMR             ";

//coordinate
double latitudineD,longitudineD;
extern long latitudine,longitudine;

//numeri
u8 numeroAllarmi[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
u8 numeroDevice[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//versione software
u16 software = 21;
u8 XL = 0;	

extern u8 BTattivo;
extern uint8_t messaggioRecBT[100];

//NFC
int sizeNFC = 0;
u8 arrayNFC[16];
u8 offsetNFC[2];

extern u8 avviaTestMemorie;

//NFC after
int sizeNFCafter = 0;
u8 arrayNFCafter[16];
u8 offsetNFCafter[2];


extern u16 paccTot;

//regola ora
extern u32 regolaOra;

extern u8 rxTest[500];

//flag format
u8 formatGuasti = 0;
u16 formatNeutro = 0;

//messaggi
u8 messaggioTampering = 0;

extern u8 programmaPacchetto;

extern u8 emergenza;

u8 riavvio = 0;

extern u8 avvioConcluso;

//SMS polling
extern u8 inviaSMSpollFlag;
extern u8 inviaSMSpollFlagCoda;
extern u8 statoModulo;

//internet
extern u8 statoInternet;
extern u8 aggiungiTensioniDBflag;
extern u8 aggiungiMeasProfileDBflag;
extern u8 aggiungiLoadProfileDBflag;
extern u8 aggiungiGuastoDBflag;
extern u8 aggiungiUnderDBflag;
extern u8 aggiungiOverDBflag;
extern u8 aggiungiNeutroStartDBflag;
extern u8 aggiungiNeutroEndDBflag;


//controlla rete
extern u8 controllaRete;
//controllo antifurto
extern u8 controlloFurto;
//cambio nome BT
extern u8 cambioNomeBT;
extern u8 inibitNFC;
extern u8 controlloBatteriaFlag;
extern u8 spegniLed;
extern u32 sniff32;
extern u8 batteryLevel;
extern long tempo;
extern u8 salvaSeriale;
extern u8 univoco[5];
extern u8 aggiungiIntrusioneDBflag;
extern u8 aggiungiRebootDBflag;
extern u8 aggiungiDebugDBflag;
extern u8 retePrivata;

extern long timerBT;
extern u8 holdDisBT;
extern u8 BTon;
extern u8 forzaAtt;
extern u8 forzaDis;

extern u16 packTotGSM;
extern u8 updateGSMatt;
extern u16 NpackRecGSM;
extern u8 packRecFlag;
extern u8 packGSM[500];
extern u16 crcGSM;
extern u16 updateGSMvers;
extern u8 downloadNewPackFlag;
extern u8 uartPack[500];
extern u16 sizePack;

extern u32 lastSMS;
extern u8 lastNumber[20];
extern u8 checkNeutroFlag;
u8 checkTensioni = 0;

extern u8 indicationSMSarrivato[100];
extern u8 indicationSMSflag;
extern u8 smsTextModeReady;

extern u8 stato4G;
extern u8 aggioraOrarioNTPflag;

extern u8 RTCpollingFlag;

extern u8 timerModuloESC;

extern u8 disattivaInternetFlag;

u8 testAPNflag = 0;

static u8 smsTextSchedulerArmed = 0;
static u32 nextSmsTextModeTick = 0;

u8 riavvia = 0;

extern long cicliMain;

extern u8 produzione;
extern u16 timerProduzione;
extern u8 serialeDaScrivere[4];

void Print_ResetFlags(UART_HandleTypeDef *huart)
{
    uint32_t flags = RCC->CSR;
    char msg[128];

    // Stampa valore grezzo
    snprintf(msg, sizeof(msg),
             "\r\nRESET FLAGS: 0x%08lX\r\n", flags);
    HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Decodifica
    if (flags & RCC_CSR_PORRSTF)
        HAL_UART_Transmit(huart, (uint8_t*)" - Power-On Reset\r\n", 18, HAL_MAX_DELAY);

    if (flags & RCC_CSR_PINRSTF)
        HAL_UART_Transmit(huart, (uint8_t*)" - NRST Pin Reset\r\n", 18, HAL_MAX_DELAY);

    if (flags & RCC_CSR_SFTRSTF)
        HAL_UART_Transmit(huart, (uint8_t*)" - Software Reset\r\n", 19, HAL_MAX_DELAY);

    if (flags & RCC_CSR_IWDGRSTF)
        HAL_UART_Transmit(huart, (uint8_t*)" - Independent Watchdog Reset\r\n", 31, HAL_MAX_DELAY);

    if (flags & RCC_CSR_WWDGRSTF)
        HAL_UART_Transmit(huart, (uint8_t*)" - Window Watchdog Reset\r\n", 27, HAL_MAX_DELAY);

    if (flags & RCC_CSR_BORRSTF)
        HAL_UART_Transmit(huart, (uint8_t*)" - Brown-Out Reset\r\n", 20, HAL_MAX_DELAY);

    if (flags & RCC_CSR_LPWRRSTF)
        HAL_UART_Transmit(huart, (uint8_t*)" - Low-Power Reset\r\n", 19, HAL_MAX_DELAY);

    // Cancella i flag per i reset successivi
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void print_reset_cause(void)
{
uint32_t csr = RCC->CSR; // 1) legge la causa del reset (prima che venga toccata da HAL)
char msg[100] = {0};


// 2) decodifica testuale
strcat(msg, "RESET CAUSE: ");


if (csr & RCC_CSR_BORRSTF) strcat(msg, "BOR "); // Brown-Out
if (csr & RCC_CSR_PORRSTF) strcat(msg, "POR "); // Power-On
if (csr & RCC_CSR_PINRSTF) strcat(msg, "PIN "); // Pin NRST
if (csr & RCC_CSR_SFTRSTF) strcat(msg, "SFT "); // Software reset
if (csr & RCC_CSR_IWDGRSTF) strcat(msg, "IWDG "); // Independent watchdog
if (csr & RCC_CSR_WWDGRSTF) strcat(msg, "WWDG "); // Window watchdog
if (csr & RCC_CSR_LPWRRSTF) strcat(msg, "LPWR "); // Low-power reset


strcat(msg, "\r\n");


// 3) invio su UART (o BT)
HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);


// 4) ora posso cancellarli
__HAL_RCC_CLEAR_RESET_FLAGS();
}

void HAL_Delay(uint32_t Delay)
{
	uint32_t tickstart = HAL_GetTick();
	uint32_t wait = Delay;

	if (wait < HAL_MAX_DELAY) wait++;
	uint32_t tickOld = tickstart;


	while ((uint32_t)(HAL_GetTick() - tickstart) < wait)
	{
		uint32_t tick = HAL_GetTick();


		if ((uint32_t)(tick - tickOld) >= 100)
		{
			tickOld = tick;

			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
			for (volatile uint8_t d = 0xFF; d != 0; d--) { __NOP(); }
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
		}
	}
}


int mymain(void){
		uint8_t uart[100];
		u8 offset[2];
		u16 beforeOffset;	
		u8 formattatore[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		u8 messaggio[100];
		u16 pacchettoN;
		u32 app = 0;
		u8 name[31];
		u32 corrente[6];
		u16 diff[6];
		u8 addressFram[2];
		int u = 0;
		u32 tick, tickOld;
		u32 nextSignalRequestTick = 0;
		u8 signalSchedulerArmed = 0;
			//da cancellare
			u8 uartmsg[100];
			
	
	//Print_ResetFlags(&huart1);
		print_reset_cause(); // <<<<< QUI

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET); //LED VERDE ACCESO
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET); //LED ROSSO SPENTO
		
			
	//HAL_TIM_Base_Start_IT(&htim4); //attivo watchdog(sogliaCorrenteA == 0 && sogliaCorrenteB == 0)
	__HAL_RTC_WAKEUPTIMER_ENABLE(&hrtc); //avvio l'orologio
	
	
	HAL_UART_Receive_DMA(&huart2,&rxBT[0],500); //inizializzo il DMA UART
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); //avvio l'interrupt UART
	
		
		HAL_UART_Receive_DMA(&huart1,&rxTest[0],500); //inizializzo il DMA UART
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //avvio l'interrupt UART
	
	HAL_UART_Receive_DMA(&huart6,&rx4G[0],500); //inizializzo il DMA UART
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); //avvio l'interrupt UART
	

			
	boot(); //inizializzo le periferiche
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET); //attiva batteria
	HAL_UART_Transmit(&huart1,(u8*)"batteria attivata\n",18,100);
	formattaFlashInterna();
	//estremiDSTuniv(); //calcolo gli estremi per l'ora legale (li devo controllare una volta al giorno);
	
	

	__HAL_RTC_WAKEUPTIMER_ENABLE_IT(&hrtc,RTC_IT_WUT); //avvio l'interrupt sull'orologio
	
	//latitudine double
	latitudineD = latitudine;	latitudineD /= 1000000;
	longitudineD = longitudine; longitudineD /= 1000000;
	
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
	app = *(u32*)0x08008000;
	
	//cambioNomeBTfunction((u8*)"UMR-BT\0");
	
	tickOld = HAL_GetTick();
	
	//attivo la modalit testo per gli SMS
	//invia4G("AT+CMGF=1\r");
	
	//inviaDebug("primo avvio\n");
	//primoAvvio();
	
	//statoInternet = 0;
	
	
	
	while(1){//(riavvio == 0){
		
		cicliMain++;
		
		if(RTCpollingFlag == 1){
			RTCpollingFlag = 0;
			RTCpolling();
		}
		
		//sovracorrenti
		tick = HAL_GetTick();
		if(tick-tickOld > 100){
			exTimerGuasti();
			tickOld = tick;
		}		
		
		//connessione internet
		if(stato4G > 2 && statoModulo == 0 && statoInternet == 1 && avvioConcluso == 1){
			//statoModulo++; inviaDebug("statoModulo++\n");
			HAL_UART_Transmit(&huart6,(u8*)"AT+NETCLOSE\r\r",12,100);
			HAL_Delay(1000);
			//statoModulo--; inviaDebug("statoModulo--\n");
			connettiInternet();
		}
		
		resetWD();
		
		/*
		 * Inizializzazione SMS text mode.
		 * Deve avvenire dopo che il modem e' acceso/agganciato e solo quando
		 * la UART 4G e' libera, cosi' i comandi SMS arrivano come testo e non PDU.
		 */
		if(avvioConcluso == 1 && stato4G > 1){
			if(smsTextSchedulerArmed == 0){
				nextSmsTextModeTick = HAL_GetTick() + SMS_TEXTMODE_FIRST_DELAY_MS;
				smsTextSchedulerArmed = 1;
			}
			if(smsTextModeReady == 0 && statoModulo == 0 && updateGSMatt == 0 && ((int32_t)(HAL_GetTick() - nextSmsTextModeTick) >= 0)){
				initSMStextMode();
				nextSmsTextModeTick = HAL_GetTick() + SMS_TEXTMODE_RETRY_MS;
			}
		}
		else{
			smsTextSchedulerArmed = 0;
			smsTextModeReady = 0;
		}
		
		/*
		 * Richiesta periodica livello segnale.
		 * Parte solo quando il modulo 4G e' agganciato e statoModulo e' libero,
		 * quindi non collide con connessione internet, HTTP/MySQL, SMS, NTP o aggiornamento GSM.
		 */
		if(avvioConcluso == 1 && stato4G > 1){
			if(signalSchedulerArmed == 0){
				nextSignalRequestTick = HAL_GetTick() + SIGNAL_REQUEST_FIRST_DELAY_MS;
				signalSchedulerArmed = 1;
			}

			if(statoModulo == 0 && ((int32_t)(HAL_GetTick() - nextSignalRequestTick) >= 0)){
				requestSignal();
				nextSignalRequestTick = HAL_GetTick() + SIGNAL_REQUEST_INTERVAL_MS;
			}
		}
		else{
			signalSchedulerArmed = 0;
		}
		
		//aggiungi a database
		if(statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && updateGSMatt == 0 && aggiungiIntrusioneDBflag == 1){
			aggiungiIntrusioneDBflag = 0;
			aggiungiIntrusioneDB(0);
		}
		if(statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && updateGSMatt == 0 && aggiungiMeasProfileDBflag == 1){
			aggiungiMeasProfileDBflag = 0;
			aggiungiMeasProfileDB(0);
		}
		if(statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && updateGSMatt == 0 && aggiungiLoadProfileDBflag == 1){
			aggiungiLoadProfileDBflag = 0;
			aggiungiLoadProfileDB(0);
		}
		if(statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && updateGSMatt == 0 && aggiungiGuastoDBflag == 1){
			aggiungiGuastoDBflag = 0;
			aggiungiGuastoDB(0,&corrente[0]);
		}
		if(statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && updateGSMatt == 0 && aggiungiUnderDBflag == 1){
			aggiungiUnderDBflag = 0;
			aggiungiUnderDB(0,&corrente[0]);
		}
		if(statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && updateGSMatt == 0 && aggiungiOverDBflag == 1){
			aggiungiOverDBflag = 0;
			aggiungiOverDB(0,&corrente[0]);
		}
		if(statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && updateGSMatt == 0 && aggiungiNeutroStartDBflag == 1){
			aggiungiNeutroStartDBflag = 0;
			aggiungiNeutroStartDB(0,&diff[0]);
		}
		if(statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && updateGSMatt == 0 && aggiungiNeutroEndDBflag == 1){
			aggiungiNeutroEndDBflag = 0;
			aggiungiNeutroEndDB(0,0,&diff[0],&diff[0]);
		}

		if(statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && updateGSMatt == 0 && aggiungiRebootDBflag == 1){
			aggiungiRebootDBflag = 0;
			aggiungiRebootDB(0);
		}
		if(statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && updateGSMatt == 0 && aggiungiDebugDBflag == 1){
			aggiungiDebugDBflag = 0;
			aggiungiDebugDB(0);
		}
		
		if(timerModuloESC == 0){
			timerModuloESC = timerModuloESCinit;
			sprintf(uart,"\x1b");
			invia4G(uart);
			HAL_Delay(200);
			invia4G("\r");
			statoModulo = 0;
			/*
			 * Se il timeout avviene mentre siamo in connessione internet,
			 * non lasciamo statoInternet bloccato a 2: torniamo a 1 per permettere
			 * un nuovo tentativo pulito dal main loop.
			 */
			if(statoInternet == 2){
				statoInternet = 1;
			}
		}
		
		if(disattivaInternetFlag == 1){
			disattivaInternetFlag = 0;
			HAL_UART_Transmit(&huart6,(u8*)"AT+NETCLOSE\r\r",12,100);			
		}
		

		if(riavvia == 1 && inviaSMSpollFlag == 0 && inviaSMSpollFlagCoda == 0){
			riavvia = 0;
			rebootSMS();			
		}
		//aggiornamento GSM
		
		//ricezione del pacchetto
		if(updateGSMatt != 0 && downloadNewPackFlag != 0 && statoModulo == 0 && statoInternet == 3 && avvioConcluso == 1 && BTattivo == 0 && inviaSMSpollFlag == 0 && inviaSMSpollFlagCoda == 0){ //flag download pacchetto
			if(NpackRecGSM <= packTotGSM){
				downloadPacchetto(NpackRecGSM);
			}
			downloadNewPackFlag = 0;
		}
		
		//installazione del pacchetto
		if(updateGSMatt != 0 && packRecFlag != 0 && statoModulo == 0){
			
			packRecFlag = 0;
			
			if(NpackRecGSM != 0){
				installaPacchettoGSM();
			}

			if(NpackRecGSM == 0){
				NpackRecGSM++;
				downloadNewPackFlag = 1;
			}
			else if(NpackRecGSM != 0 && NpackRecGSM < packTotGSM){
				if(sizePack > 400){
					NpackRecGSM++;
					downloadNewPackFlag = 1;
				}
				else{
					downloadNewPackFlag = 1;
				}
			}
			else{
				updateGSMatt = 0;
				NpackRecGSM = 0;
				HAL_UART_Transmit(&huart1,(u8*)"test",4,100);
				if(CRC_flash(0x08040200,240*packTotGSM) == crcGSM){//validazione CRC
					validazioneUpdate();
				}
				else{
					inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"CRC error",9);
				}				
			}
		}
		
		
		//cambio nome BT
		if(cambioNomeBT == 1 && statoModulo == 0 && BTattivo == 0){
			cambioNomeBT = 0;
			inviaDebug((u8*)"cambio nome BT");			
			HAL_UART_Transmit(&huart2,(u8*)"$$$",3,100);
			HAL_Delay(500);
			HAL_UART_Transmit(&huart2,(u8*)"SN,",3,100);
			HAL_UART_Transmit(&huart2,identificativo,16,100);
			HAL_UART_Transmit(&huart2,(u8*)"\r",1,100);
			HAL_Delay(500);
			HAL_UART_Transmit(&huart2,(u8*)"---\r",4,100);
			HAL_Delay(500);
		}

		//controllo antifurto
		if(controlloFurto == 1 && BTattivo == 0){
			controlloFurto = 0;
			controlloAntifurtoProva();
			spegniLed = 0;
		}
		
		
		//salvataggio da effettuate
		if(salvataggioLoad == 1){
			salvataggioLoad = 0;
			salvaLoad();			
			}
		if(salvataggioMeas == 1){
			salvataggioMeas = 0;
			salvaMeas();
			}		
		if(avviaTestMemorie == 1){
			testMemorie();
		}
			
		if(regolaOra != 0){
			//HAL_Delay(1000);
			sethour(regolaOra);
			regolaOra = 0;
			//estremiDSTuniv();
		}
		if(programmaPacchetto != 0){
			programmaPacchetto = 0;
			pacchettoN = array2u16(&messaggioRecBT[0]);
			if(app == 0x11111111){
				progPacchetto3(&messaggioRecBT[2],pacchettoN,paccTot);
			}
			else{
				progPacchetto(&messaggioRecBT[2],pacchettoN,paccTot);
			}			
		}
		//invio SMS
		if(statoModulo == 0 && inviaSMSpollFlag == 1 && stato4G > 1){
			inviaDebug("invio SMS\n");
			inviaSMSpoll(1);
			inviaSMSpollFlag = 0;
		
		}
		//invio SMS coda
		if(statoModulo == 0 && inviaSMSpollFlagCoda == 1 && stato4G > 1){
			inviaSMSpoll(2);
			inviaSMSpollFlagCoda = 0;
		}
		//aggiornamento orogolo NTP
		if(statoModulo == 0 && aggioraOrarioNTPflag == 1 && stato4G > 1){
			aggioraOrarioNTPflag = 0;
			invia4G("AT+CCLK?\r");			
		}
		
		//lettura SMS
		if(indicationSMSflag == 1 && statoModulo == 0){
			if(smsTextModeReady == 0){
				initSMStextMode();
			}
			else{
				indicationSMSflag = 0;
				leggiSMS(&indicationSMSarrivato[0]);
			}
		}
		if(checkNeutroFlag == 1){
			checkNeutroFlag = 0;
			checkNeutro();
		}
		
		if(checkTensioni == 1){
			checkTensioni = 0;
			checkUnderVoltage();
			checkOverVoltage();
		}
		
		if(controlloBatteriaFlag == 1){
			controllaBatteria();
			controlloBatteriaFlag = 0;
		}
		
		//produzione
		if(produzione == 2){
			if(timerProduzione == 2){ //cancello vecchio seriale
				inviaDebug("cancellazione seriale\n");
				HAL_FLASH_Unlock();
				resetWD();
				FLASH_Erase_Sector(FLASH_SECTOR_1,VOLTAGE_RANGE_3);
				resetWD();
				FLASH_WaitForLastOperation(1000);
				timerProduzione++;
			}
			else if(timerProduzione == 5){ //scrivo nuovo seriale
				inviaDebug("scrittura seriale\n");
				scriviSeriale();
				timerProduzione++;
			}
			else if(timerProduzione == 8){
				eseguiComandoTest("0000C87");
				inviaDebug("calibrazione tensione\n");
				eseguiComandoTest("0000C201");
				timerProduzione++;
			}
			else if(timerProduzione == 11){
				inviaDebug("calibrazione corrente\n");
				eseguiComandoTest("0000C203");
				timerProduzione++;
			}
			else if(timerProduzione == 14){
				inviaDebug("calibrazione potenza\n");
				eseguiComandoTest("0000C204");
				timerProduzione++;
			}
			else if(timerProduzione == 17){
				inviaDebug("calibrazione guasti\n");
				eseguiComandoTest("0000C21");
				timerProduzione++;
			}
			else if(timerProduzione == 20){
				inviaDebug("ricalibrazione tensione\n");
				eseguiComandoTest("0000C201");
				timerProduzione++;
			}
			else if(timerProduzione == 23){
				inviaDebug("ricalibrazione corrente\n");
				eseguiComandoTest("0000C203");
				timerProduzione++;
			}
			else if(timerProduzione == 27){
				inviaDebug("ricalibrazione potenza\n");
				eseguiComandoTest("0000C204");
				timerProduzione++;
			}
			else if(timerProduzione == 30){
				produzione = 3;
				inviaDebug("ricalibrazione guasti\n");
				eseguiComandoTest("0000C21");
			}
		}
		
		
	}

}

