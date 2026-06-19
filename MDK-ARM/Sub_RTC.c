#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "time.h"

#define durataAvvio 20

extern u8 IDconnesso;


//periferiche
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c2;
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;

extern long corrE1[3],corrE2[3]; //calibrazione energia attiva
extern long corrR1[3],corrR2[3];
extern long corrI1[3],corrI2[3];
extern long corrV[3],corrV[3];

//inibizione sovracorrenti e neutro
extern u8 inibitOverI;
extern int inibitN;

//variabili data e ora
RTC_DateTypeDef currentDate;
RTC_TimeTypeDef currentTime;
uint32_t myTimeVar,dstA,dstB; //posix ed estremi
u8 DST = 0; //Flag DST Attivo
u32 seconds = 0; //secondi dall'accensione
u32 secondiLoad = 0; //secondi persi load
u32 secondiMeas = 0; //secondi persi meas

//flag auto sync
u8 autoSyncActive = 0;



//attivazione antifurto
extern u8 antifurtoAttivo;


//inibizione dopo formattazione
u8 inibizione = 0;

//ora legale universale
u8 DSTon = 0;
u8 DSThourStart = 0;
u8 DSTdayStart = 0;
u8 DSTweekStart = 0;
u8 DSTmonthStart = 0;
u8 DSTdayStop = 0;
u8 DSTweekStop = 0;
u8 DSTmonthStop = 0;
u8 DSThourStop = 0;

//soglie
extern u16 sogliaNeutro;

//avvio concluso
u8 avvioConcluso = 0;

//flag auto sync
extern u8 autoSyncActive;

extern u8 BTattivo;
extern uint32_t indiceLoad;
extern uint32_t indiceMeas;
extern uint8_t messaggioRecBT[100];
extern u32 contatore;

extern u16 calibAntifurto;
extern u8 antifurtoScattato;
extern u16 nIntrusioni;
extern u8 antifurtoAttivo;
extern u16 misuraLuce;

extern uint8_t LoadActive;
extern uint8_t MeasActive;

extern uint8_t rxBT[500];
extern u8 lastNumber[10];
extern u8 numeroAllarmi[20];

extern u8 sniff[50];
extern u8 guasti;
extern u8 eventiNeutro;
extern u8 cancellaSMS;
extern u32 sniff32;

//da cancellare
extern u8 identificativo[16];
extern u8 antifurtoAttivo;
extern u16 calibAntifurto;
extern long latitudine;
extern long longitudine;
extern u8 formatGuasti;
extern u16 formatNeutro;
extern u8 messaggioTampering;

//variabili V,I,P,Q NON CORRETTI
extern u32 Vnc[3];
extern u32 I1nc[3],I2nc[3];
extern u32 P1nc[3],P2nc[3];

//variabili energie NON CORRETTI
extern long E1nc[3],E2nc[3];
extern long R1nc[3],R2nc[3];

//variabili misurandi
extern u32 V[3]; //tensioni
extern long I1[3],I2[3]; //correnti
extern uint16_t phi1[3],phi2[3]; //phi
extern long P1[3],P2[3]; //potenze attive
extern long Q1[3],Q2[3]; //potenze reattive
extern int cosphi1[3],cosphi2[3]; //cosfi

//energie
extern long E1[3],E2[3],R1[3],R2[3];

extern long offE1[3],offE2[3]; //offset energia attiva

extern u8 calib;

extern uint32_t E1p[3],E2p[3]; //energie attive positive
extern uint32_t E1n[3],E2n[3]; //energie attive negative
extern uint32_t R1p[3],R2p[3]; //energie reattive positive
extern uint32_t R1n[3],R2n[3]; //energie reattive negative

extern u32 contatore;
extern u32 acq[6];
extern u32 correnteGuasto[6];

//soglie
extern u16 sogliaCorrenteA;
extern u16 sogliaCorrenteB;

extern u8 simulaGuasto;
extern u8 simulaNeutro;

//inibizione guasto
extern u8 inibitGuasto;

extern u8 riavvio;
extern u32 correnteGuasto[6];

extern u32 calibrazioneI[6];

extern u8 batteryLevel, batteriaInCarica;

extern u32 lettureV[3];

extern long contaguasto;

extern u32 sniffNeutro;
u8 sonopassato = 0;

u8 smstest = 0;
extern u8 inibitGuastoSMS;

extern u32 E1pTOT,E2pTOT,E1nTOT,E2nTOT,R1pTOT,R2pTOT,R1nTOT,R2nTOT;

extern u8 mettoilmeno;
extern u8 statoModulo;

extern u8 azzeraLineaA;
extern u8 azzeraLineaB;
u32 sniffNuovo;
u32 contatoreAvvio = 0;

u8 emergenza = 0;

u16 riavvioForzato = timeoutModulo;
u16 riavvioForzato2 = timeoutModulo2;

u16 riavvioSMS = timeoutSMS;
u16 riavvioSMS2 = timeoutSMS2;

//internet
extern u8 statoInternet;

//richiesta segnale
extern long contaGuasto;
extern u8 formatGuasti;

//NFC
extern int sizeNFC;
extern u8 arrayNFC[16];
extern u8 offsetNFC[2];
extern int sizeNFCafter;
extern u8 arrayNFCafter[16];
extern u8 offsetNFCafter[2];

extern u8 testEstrazione;
extern u8 segnaleGSM;

//formattazione Flash
u8 cancellaLoad = 0;
u8 cancellaMeas = 0;

extern u8 testSalvataggio;

extern u8 smsAttivi;

u8 controllaRete = 0;
u8 controlloFurto = 0;
u8 controlloBatteriaFlag = 0;

extern u8 eventoNeutro;
extern u8 eventiNeutro;

extern u8 inibitNFC;

extern u8 paginaLoadSost;
extern u8 paginaMeasSost;

extern long quanteVolteGuasti;

u8 spegniLed = 0;

u8 alimentatore = 1;
u8 lowpower = 0;

extern u8 inizializzaAntifurto;
long timerBT = 0;

//controllo update
extern u8 updateAttivo;

extern long temperatura;
extern u16 timeCarica;
extern double tensioneB;

double misura[6];
extern u8 aggiungiNeutroStartDBflag;
extern u8 triangolo;

extern u8 APN[30];
extern u8 mySQL[50];
extern u8 userSQL[20];
extern u8 pwSQL[20];
extern u8 attivaInternetFlag;

extern u8 univoco[5];
extern u8 salvataggioLoad;
extern u8 userAPN[30];
extern u8 pwAPN[30];
extern u8 retePrivata;
extern u32 sogliaI;
extern u8 sogliaPers;
extern u32 Ipot;
extern u16 eccezione;
extern u8 password[17];
extern u8 inviaSMSpollFlag;
extern u8 holdDisBT;
extern u8 forzaDis;
extern u8 BTon;

extern u16 packTotGSM;
extern u8 updateGSMatt;
extern u16 NpackRecGSM;
extern u8 packRecFlag;
extern u8 packGSM[500];
extern u16 crcGSM;
extern u16 updateGSMvers;
extern u8 downloadNewPackFlag;
extern u8 upAddressGSM[200];
extern u8 checkTensioni;
extern u8 overSavings;
extern u8 debugDB;
extern u32 regolaOra;
extern u8 addressNTP[50];

long cicliMain = 0;
//u32 tick1,tick2;

u8 checkNeutroFlag = 0;

u8 timerModulo4G = 30;

extern int netCount;
u8 stato4G = 0;

u8 RTCpollingFlag = 0;

u8 emergenzaCollaudo = 0;

u8 produzione = 0;
u16 timerProduzione = 0;
u8 serialeDaScrivere[4] = "0850";

u8 timerModuloESC = timerModuloESCinit;


void RTC_WKUP_IRQHandler(void)
{
	u8 deleteSMS[19] = "AT+QMGDA= DEL ALL\x22\r";
	static u16 time = 0;	
	static u8 avvio = 0;
	static u8 a = 0;
	//per NFC
	u16 beforeOffset;	
	u8 addressFram[2] = {1,100};
	u8 offset[2];
	static u8 collaudo = 0;
	static u8 timerCollaudo = 3;
	u8 uart[50];
	
	static u8 riavvio = 0;
	u8 addressFlash[3] = {0,0,0}; //per formattazione flash
	
	static u8 annuncio4G = 0;
	static int accensione = 0;
	
	

	
	u8 stringa1[50] = "ciao come va stronzo";

		
		
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
	
	deleteSMS[9] = 0x22;
	UpdateTime();
	timerBT++;
	contatoreAvvio++;
	
	if(time<300){
		time++;
		}
		
	//controllo stato alimentatore
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)){
		alimentatore = 1;
	}
	else{
		alimentatore = 0;
		if(produzione != 0 || time < 299){
			alimentatore = 1;
			}
	}
	
	
	if(riavvio == 1){
		riavvio = 2;
	}
	else if(riavvio == 2){
		riavvio = 0;
		//adeinit();
		adeinit3();
		avvioConcluso = 0;
		avvio = 0;
	}
	
	
	////stato4G (0 = off,1 = acceso senza connessione,2 = 2G/3G,3 = 4G)
	if(annuncio4G != 2){	
		if(netCount < 2 && HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2) == 1){
			stato4G = 0;
		}
		else if(netCount < 2 && HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2) == 0){
			stato4G = 1;
		}
		else if(netCount >= 2 && netCount < 4){
			stato4G = 2;
		}
		else if(netCount >= 4){
			stato4G = 3;
		}
		netCount = 0;
		
		if(annuncio4G == 0 && stato4G > 1){
			inviaDebug((u8*)"modulo 4G attivo\n");
			annuncio4G = 1;
		}
		
		if(annuncio4G == 1 && stato4G == 3){
			inviaDebug((u8*)"connessione 4G attiva\n");
			annuncio4G = 2;
			HAL_NVIC_DisableIRQ(EXTI2_IRQn);		
		}
	}
	
	//modalit low power
	if(avvioConcluso == 1 && alimentatore == 0 && lowpower == 0){
		lowpower = 1;
		////disabilito solo il secondo ADE
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);  //ADE1
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);  //ADE2
		
		////disabilito i rivelatori di picco
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
		
	}
	else if(avvioConcluso == 1 && alimentatore != 0 && lowpower != 0){
		lowpower = 0;
		
		////abilito il secondo ADE
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);  //ADE1
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);  //ADE2
		
		riavvio = 1;
		
		//abilito i rivelatori di picco
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
		inibitGuastoSMS = 5;
		
	}
	
		
	if(avvio < (durataAvvio-10)){
		avvio++;
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_9);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10);
		resetWD();
		
	}
	else if (avvio == (durataAvvio-10)){
		avvio++;
			acquisizioni();
			acquisizioni3();
			//HAL_TIM_Base_Init(&htim3);
			//HAL_TIM_Base_Start_IT(&htim3);
			azzeraRegistriADE1();
			azzeraRegistriADE3();
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_9);
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10);		
			resetWD();
		
			sprintf(uart,"produzione: %d\n",produzione);
			inviaDebug(uart);
	}
	else if(avvio > (durataAvvio-10) && avvio < durataAvvio){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_9);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10);
		avvio++;
		resetWD();
	}
	else if(avvio == durataAvvio){
		avvio++;
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
		avvioConcluso = 1;	
		
		if(VAR != 0){
			triangolo = 2;
		}
		resetWD();
		
	}
	else{
			
			
			if(lowpower == 0){
				if(riavvio == 0){
					acquisizioni();
					acquisizioni3();		
					//sprintf(uart,"%d %d %d\n", Vnc[0],Vnc[1],Vnc[2]);
					//inviaDebug(uart);
					//inviaDebug("acquisizione\n");
				}
			}
			RTCpollingFlag = 1;
			salvataggio();
			if(produzione == 5){
				invia4G("AT+CPSI?\r");
				produzione = 0;
				}
				
	}
	
	if(produzione == 1){
		if(statoInternet == 3 && collaudo == 0){
			collaudo = 1;
			preparaLoad();
			preparaMeas();
		}
		
		if(statoInternet == 2 && timerCollaudo != 0){
			timerCollaudo--;
		}
		else if(statoInternet == 2 && timerCollaudo == 0){
			emergenzaCollaudo = 1;
		}
	}
	
	//sprintf(uart,"netcount %d\n", netCount);
	//inviaDebug(uart);
	
	if(statoModulo != 0 && timerModuloESC != 0 && avvioConcluso == 1){
		timerModuloESC--;
	}
	
		
}


void RTCpolling(void){
	
		static u8 rigaLoad = 255;
		static u8 rigaMeas = 255;
		u8 uart[100];
		u16 beforeOffset;	
		u8 offset[2];
		u8 addressFlash[3] = {0,0,0}; //per formattazione flash
	
		u8 formattatore[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		u8 addressFram[2] = {1,100};
		
		if(currentTime.Seconds == 0 && produzione == 0){
		sprintf(uart,"minuto: %d\n",currentTime.Minutes);
		inviaDebug(uart);
		/*
		 * La richiesta del livello segnale non viene piu' fatta qui ogni minuto.
		 * Ora e' gestita dal main loop, solo quando il modulo 4G e' libero.
		 */
		}
		
		//sprintf(uart,"guasto %u %u %u %u %u %u\n corrente %ld %ld %ld %ld %ld %ld\n", acquisizioneADC(1),acquisizioneADC(2),acquisizioneADC(3),acquisizioneADC(4),acquisizioneADC(5),acquisizioneADC(6),I1[0],I1[1],I1[2],I2[0],I2[1],I2[2]);
		//inviaDebug(uart);
		
		/*if(lowpower == 0){
				if(riavvio == 0){
					acquisizioni();
					acquisizioni3();					
				}
			}*/
	
		
			if(lowpower == 0){
				if(riavvio == 0){
					elabMisure();
					elabMisure3();
					calcolatutto();
					calcolatutto3();
					checkTensioni = 1;
					produzioneFun();					
				}
			}
			else{
				leggiTensioni();
				azzeraMisurandi();
				checkTensioni = 1;
			}
		
		//salvataggi
			/*if(inibizione == 0 && updateGSMatt == 0 && updateAttivo == 0){
				salvataggio(); //salvataggio load e meas profiles
			}
			else{
				inibizione--;
			}*/
		
			//controlloBatteria
			if(currentTime.Seconds == 0 || currentTime.Seconds == 30 || currentTime.Seconds == 15 || currentTime.Seconds == 45){
				controlloBatteriaFlag = 1;
				addressFram[0] = 2; addressFram[1] = 231;
				saveU16fram(timeCarica,&addressFram[0]);
			}
		  

			//evento neutro (squilibrio tensioni)
			afterNeutro(); // scrittura evento neutro su NFC
			if((currentTime.Seconds == 25 || currentTime.Seconds == 55) && inibitN == 0 && sogliaNeutro != 0 && updateAttivo == 0){
				checkNeutroFlag = 1;				
			}
			
			if(inibitN != 0){ //inibizione controllo neutro durante cancellazione dati
				inibitN--;
			}		
			
		
		//gestione antifurto	
		if(antifurtoAttivo == 1 &&  multiplocinque(myTimeVar + 1) == 1 && BTattivo == 0){
			spegniLed = 1;
		}
		else if(antifurtoAttivo == 1  && multiplocinque(myTimeVar) == 1 && BTattivo == 0){
			controlloFurto = 1;			
		}
		else{
			spegniLed = 0;
		}
		
			//gestione guasti
	if(inibitGuasto != 0 && inibitGuasto != 255){ //inibizione controllo guasti dopo invio allarme
		inibitGuasto--;
		if(inibitGuasto == 2){
			ricalcolaSoglie();			
		}
	}
	

	
		//cancellazione flash
		if((cancellaLoad + cancellaMeas) <= 31 && (cancellaLoad + cancellaMeas) > 0){
			if(cancellaLoad > 0){
				cancellaLoad--;
				blockErase(cancellaLoad);
				inibizione = 2;
			}
			if(cancellaMeas > 0){
				cancellaMeas--;
				blockErase(cancellaMeas+32);
				inibizione = 2;
			}			
		}
		else if((cancellaLoad + cancellaMeas) > 31){
			formatFlash();
			inibizione = 35;
			cancellaLoad = 0;
			cancellaMeas = 0;
		}
		/*
		 * Gestione NFC.
		 * La cancellazione deve avere precedenza sulle normali scritture in coda:
		 * se si scrivesse prima un vecchio evento pendente, questo potrebbe
		 * ricomparire subito dopo il comando di erase.
		 */
		if(formatGuasti != 0){
			formatGuasti--;
			beforeOffset = 4096 + (formatGuasti)*16;
			u162array(&offset[0],beforeOffset);
			resetWD();
			writeNFC32(&formattatore[0],16,&offset[0]);
			if(formatGuasti == 0){
				if(inibitGuasto == 255){ inibitGuasto = 0; }
				inibitGuastoSMS = 0;
			}
		}
		else if(formatNeutro != 0){
			formatNeutro--;	
			beforeOffset = 64 + (formatNeutro)*16;
			u162array(&offset[0],beforeOffset);
			resetWD();
			writeNFC32(&formattatore[0],16,&offset[0]);
			if(formatNeutro == 0){
				inibitN = 0;
			}
		}
		else if(sizeNFCafter != 0 && sizeNFC == 0){
			resetWD();
			writeNFC32(&arrayNFCafter[0],sizeNFCafter,&offsetNFCafter[0]);
			sizeNFCafter = 0;
		}
		else if(sizeNFC != 0){
			resetWD();
			writeNFC32(&arrayNFC[0],sizeNFC,&offsetNFC[0]);
			sizeNFC = 0;			
		}

		
		
		//sostituzione FRAM FLASH dei profili
		if(paginaLoadSost != 255){
			if(rigaLoad < 16){
				fram2flashLoad(paginaLoadSost,rigaLoad);
				rigaLoad++;
			}
			else if (rigaLoad == 16){
				rigaLoad = 255;
				paginaLoadSost = 255;
			}
			else{
				rigaLoad = 0;
				u162array(&addressFlash[0],paginaLoadSost*16);
				FormatSector(&addressFlash[0]);
			}
		}
		
		if(paginaMeasSost != 255){
			if(rigaMeas < 16){
				fram2flashMeas(paginaMeasSost,rigaMeas);
				rigaMeas++;
			}
			else if (rigaMeas == 16){
				rigaMeas = 255;
				paginaMeasSost = 255;
			}
			else{
				rigaMeas = 0;
				u162array(&addressFlash[0],(paginaMeasSost*16)+8192); // aggiungere offset
				FormatSector(&addressFlash[0]);
			}
		}
		
		//controllo scheda durante update BT
		if(updateAttivo > 0){
			updateAttivo--;
			if(updateAttivo == 0){
				emergenza = 3;
			}
		}
		
		//test di salvataggio
		if(avvioConcluso == 1 && testSalvataggio == 1 && multiplocinque(myTimeVar)){
			preparaMeas();
			preparaLoad();
		}
		if(avvioConcluso == 1 && testSalvataggio == 1 && multiplocinque(myTimeVar+2)){
			sprintf(uart,"load: %d meas: %d\n", indiceLoad,indiceMeas);
			HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
		}
		
		//sprintf(uart,"guasto %u %u %u %u %u %u\n corrente %ld %ld %ld %ld %ld %ld\n energie %ld %ld %ld %ld %ld %ld", acquisizioneADC(1),acquisizioneADC(2),acquisizioneADC(3),acquisizioneADC(4),acquisizioneADC(5),acquisizioneADC(6),I1[0],I1[1],I1[2],I2[0],I2[1],I2[2],E1nc[0],E1nc[1],E1nc[2],E2nc[0],E2nc[1],E2nc[2]);
		//inviaDebug(uart);
		//controlloBatteriaFlag = 1;
}


void produzioneFun(void){
	
	
		
	if(produzione == 0){
		return;
	}
	else if(produzione == 2){
		timerProduzione++;
	}
	else if(produzione == 3){
		if(timerProduzione < 32){
			timerProduzione++;
		}
		else if(timerProduzione >= 32 && timerProduzione < 40){
				u8 uart[50];
				sprintf(uart,"tensioni %.2f %.2f %.2f\n",(float)V[0]/100,(float)V[1]/100,(float)V[2]/100);
				inviaDebug(uart);
				sprintf(uart,"correnti %.2f %.2f %.2f %.2f %.2f %.2f\n",(float)I1[0]/100,(float)I1[1]/100,(float)I1[2]/100,(float)I2[0]/100,(float)I2[1]/100,(float)I2[2]/100);
				inviaDebug(uart);
				sprintf(uart,"Potenza A %.2f %.2f %.2f %.2f %.2f %.2f\n",(float)P1[0]/100,(float)P1[1]/100,(float)P1[2]/100,(float)P2[0]/100,(float)P2[1]/100,(float)P2[2]/100);
				inviaDebug(uart);
				sprintf(uart,"Potenza R %.2f %.2f %.2f %.2f %.2f %.2f\n",(float)Q1[0]/100,(float)Q1[1]/100,(float)Q1[2]/100,(float)Q2[0]/100,(float)Q2[1]/100,(float)Q2[2]/100);
				inviaDebug(uart);
				sprintf(uart,"phi %d %d %d %d %d %d\n\n",phi1[0],phi1[1],phi1[2],phi2[0],phi2[1],phi2[2]);
				inviaDebug(uart);
				timerProduzione++;				
		}
		else{
			eseguiComandoTest("0000Caa");
			inviaDebug("\nFINE PRODUZIONE\n");
			produzione = 0;
		}
	}
		

}


//aggiornamento orario
void UpdateTime(void){
	u8 uart[100];
	
	
	HAL_RTC_GetTime(&hrtc,&currentTime,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&currentDate,RTC_FORMAT_BIN);
	


	myTimeVar = timetoposix(currentDate,currentTime);
	//myTimeVar += 3600; //CET
	
	/*if(dstA < dstB){
		if(myTimeVar >= dstA && myTimeVar <= dstB){
			myTimeVar += 3600;
			DST = 1;
		}
		else{
			DST = 0;
		}
	}
	else if (dstB < dstA){
		if(myTimeVar < dstB || myTimeVar > dstA){
			myTimeVar += 3600;
			DST = 1;
		}
		else{
			DST = 0;
		}
	}*/
	
	oraEstesa(myTimeVar);
	
	seconds++;
	secondiLoad++;
	secondiMeas++;
	
}


//ora estesa non posix
void oraEstesa(uint32_t A){
	
			
			RTC_DateTypeDef setDate;
			RTC_TimeTypeDef setTime;			
	
			int i = -27;
			int m = 1;
			int g = 1;
			int h = 0;
			int min = 0;
			int s = 0;
			int b = 0;
			
	
			A = A - 94694400;
			//A -= 3600; //ADJ
		
			while(A >= 126230400){
				A = A -126230400;
				i = i + 4;
				}
						
			if(A >= 31536000){
				A = A - 31536000;
				i = i + 1;
			}
			if(A >= 31536000){
				A = A - 31536000;
				i = i + 1;
			}
			
			if(A >= 31622400){
				A = A - 31622400;
				b = 1;
				i = i + 1;
			}
			

			
			//mesi
			
			//gennaio 31
			if(A >= 2678400){
				A = A - 2678400;
				m = m+1;			
			}
			
			//febbraio 28
			if(A >= 2419200 && b == 0 && m == 2){
				A = A - 2419200;
				m = m+1;			
			}
			
			if(A >= 2505600 && b == 1 && m == 2){
				A = A - 2505600;
				m = m+1;			
			}
			
			//marzo 31
			if(A >= 2678400 && m == 3){
				A = A - 2678400;
				m = m+1;			
			}			
			
			//aprile 30
			if(A >= 2592000 && m == 4){
				A = A - 2592000;
				m = m+1;			
			}

			//maggio 31
			if(A >= 2678400 && m == 5){
				A = A - 2678400;
				m = m+1;			
			}

			//giugno 30
			if(A >= 2592000 && m == 6){
				A = A - 2592000;
				m = m+1;			
			}

			//luglio 31
			if(A >= 2678400 && m == 7){
				A = A - 2678400;
				m = m+1;			
			}

			//agosto 31
			if(A >= 2678400 && m == 8){
				A = A - 2678400;
				m = m+1;			
			}

			//settembre 30
			if(A >= 2592000 && m == 9){
				A = A - 2592000;
				m = m+1;			
			}
		
			//ottobre 31
			if(A >= 2678400 && m == 10){
				A = A - 2678400;
				m = m+1;			
			}

			//novembre 30
			if(A >= 2592000 && m == 11){
				A = A - 2592000;
				m = m+1;			
			}
			
			
			//giorni
			while(A >= 86400){
				A = A-86400;
				g = g + 1;
			}
			
			//ore
			while(A >= 3600){
				A = A - 3600;
				h = h + 1;
			}
			
			//minuti
			while(A >= 60){
				A = A - 60;
				min = min + 1;
			}
			
			s = A;
			
			
			currentDate.Year = i;
			currentDate.Month = m;
			currentDate.Date = g;
			currentDate.WeekDay = WhatWeekDay(setDate);
			currentTime.Hours = h;
			currentTime.Minutes = min;
			currentTime.Seconds = s;
			
}

u32 giornoSettimana(RTC_DateTypeDef data){

	RTC_TimeTypeDef ora;
	u32 posix;

	posix = timetoposix(data,ora);
	posix /= 86400;
	while(posix >= 7){
		posix = posix - 7;
	}

	if(posix >= 3){
		posix = posix-3;
	}
	else{
		posix = posix+4;
	}


	return posix;
}

//calcolo estremi DST
void estremiDSTuniv(void){
	RTC_DateTypeDef data;
	RTC_TimeTypeDef ora;
	u8 giorniMese = 0;
	u8 weekDay = 0;
	
	//calcolo inizio
	
	HAL_RTC_GetTime(&hrtc,&ora,RTC_FORMAT_BIN); //prendo la data e l'ora attuali
	HAL_RTC_GetDate(&hrtc,&data,RTC_FORMAT_BIN);
	
	if(DSTdayStart == 0 || DSTweekStart == 0 || DSTmonthStart == 0){ //se non impostato niente considero di essere in Italia
		estremiDST();
		return;
	}
	
	switch(DSTmonthStart){
		case 1:
			giorniMese = 31;
			break;
		case 2:
			giorniMese = 28 + bisestile(data.Year);
			break;
		case 3:
			giorniMese = 31;
			break;
		case 4:
			giorniMese = 30;
			break;
		case 5:
			giorniMese = 31;
			break;
		case 6:
			giorniMese = 30;
			break;
		case 7:
			giorniMese = 31;
			break;
		case 8:
			giorniMese = 31;
			break;
		case 9:
			giorniMese = 30;
			break;
		case 10:
			giorniMese = 31;
			break;
		case 11:
			giorniMese = 30;
			break;
		case 12:
			giorniMese = 31;
			break;	
	}
	
	data.Date = 1;
	data.Month = DSTmonthStart;
	weekDay = DSTdayStart; if(weekDay == 7){weekDay = 0;}
	
	if(DSTweekStart < 4){ //controllo se devo fare il cambio la prima,seconda o la terza settimana
		while(WhatWeekDay(data) != weekDay){
			data.Date++;
		}
		data.Date += (DSTweekStart-1)*7;
	}
	else{
		data.Date = giorniMese;
		while(WhatWeekDay(data) != weekDay){
			data.Date--;
		}
	}
	
	ora.Hours = DSThourStart; ora.Minutes = 0; ora.Seconds = 0;
	dstA = timetoposix(data,ora);
	
	//calcolo fine
	
	HAL_RTC_GetTime(&hrtc,&ora,RTC_FORMAT_BIN); //prendo la data e l'ora attuali
	HAL_RTC_GetDate(&hrtc,&data,RTC_FORMAT_BIN);
	
	switch(DSTmonthStop){
		case 1:
			giorniMese = 31;
			break;
		case 2:
			giorniMese = 28 + bisestile(data.Year);
			break;
		case 3:
			giorniMese = 31;
			break;
		case 4:
			giorniMese = 30;
			break;
		case 5:
			giorniMese = 31;
			break;
		case 6:
			giorniMese = 30;
			break;
		case 7:
			giorniMese = 31;
			break;
		case 8:
			giorniMese = 31;
			break;
		case 9:
			giorniMese = 30;
			break;
		case 10:
			giorniMese = 31;
			break;
		case 11:
			giorniMese = 30;
			break;
		case 12:
			giorniMese = 31;
			break;	
	}
	
	data.Date = 1;
	data.Month = DSTmonthStop;
	weekDay = DSTdayStop; if(weekDay == 7){weekDay = 0;}

	if(DSTweekStop < 4){ //controllo se devo fare il cambio la prima,seconda o la terza settimana
		while(WhatWeekDay(data) != weekDay){
			data.Date++;
		}
		data.Date += (DSTweekStop-1)*7;
	}
	else{
		data.Date = giorniMese;
		while(WhatWeekDay(data) != weekDay){
			data.Date--;
		}
	}
	
	ora.Hours = DSThourStop-1; ora.Minutes = 0; ora.Seconds = 0;
	dstB = timetoposix(data,ora);
	
}

//estremi DST Italia
void estremiDST(void){
	RTC_DateTypeDef data;
	RTC_TimeTypeDef ora;
		
	data.Date = 31;
	data.Month = 3;
	HAL_RTC_GetTime(&hrtc,&ora,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&data,RTC_FORMAT_BIN);
	
	data.Date = 31;
	data.Month = 3;
	
	while(WhatWeekDay(data) != 0){
		data.Date--;
	}
	
	ora.Hours = 2;
	ora.Minutes = 0;
	ora.Seconds = 0;

	dstA = timetoposix(data,ora);
	
	data.Date = 31;
	data.Month = 10;
	
		while(WhatWeekDay(data) != 0){
		data.Date--;
	}
		
	dstB = timetoposix(data,ora);

}

struct dstTime estremiDSTposix(u32 posix){
	RTC_DateTypeDef data;
	RTC_TimeTypeDef ora;
	struct dstTime dstLocal;
	u8 uart[100];
	
	
	data.Date = 31;
	data.Month = 3;
	
	data = posix2date(posix);
	
	data.Date = 31;
	data.Month = 3;
	
	while(WhatWeekDay(data) != 0){
		data.Date--;
	}
	
	ora.Hours = 2;
	ora.Minutes = 0;
	ora.Seconds = 0;

	dstLocal.A = timetoposix(data,ora);
	
	data.Date = 31;
	data.Month = 10;
	
		while(WhatWeekDay(data) != 0){
		data.Date--;
	}
		
	dstLocal.B = timetoposix(data,ora);
	
	//sprintf(uart,"A = %d  B = %d\n",dstLocal.A,dstLocal.B);
	//inviaDebug(uart);
	
	return dstLocal;

}

//giorno della settimana
uint8_t WhatWeekDay(RTC_DateTypeDef data){
	struct tm currenttime1;
	RTC_TimeTypeDef ora;
	uint32_t posix;
	uint8_t week;
	
	ora.Hours = 10;
	ora.Minutes =  10;
	ora.Seconds = 10;
	
	posix = timetoposix(data,ora);
	currenttime1 = *localtime(&posix);
	week = currenttime1.tm_wday;

		return week;	
}

//da orario esteso a posix
uint32_t timetoposix(RTC_DateTypeDef data, RTC_TimeTypeDef ora){
	
	long secondi;
	float bis1;
	int bis2,giorni;
	
	//calcolo numero anni bisestili
	bis1 = (data.Year + 28)/4;
	bis2 = bis1;
	bis2 = bis2 +1;
	
	//calcolo giorni fino a 1 gennaio
	giorni = (data.Year + 30)*365 + bis2;
	
	//aggiungo il numero di giorni fino al mese precedente
	
	switch(data.Month){
	case 1:
		giorni = giorni;
		break;
	case 2:
		giorni = giorni + 31;
		break;
	case 3:
		giorni = giorni + 59;
		break;
	case 4:
		giorni = giorni + 90;
		break;
	case 5:
		giorni = giorni + 120;
		break;
	case 6:
		giorni = giorni + 151;
		break;
	case 7:
		giorni = giorni + 181;
		break;
	case 8:
		giorni = giorni + 212;
		break;
	case 9:
		giorni = giorni + 243;
		break;
	case 10:
		giorni = giorni + 273;
		break;
	case 11:
		giorni = giorni + 304;
		break;
	case 12:
		giorni = giorni + 334;
		break;
	}

	//calcolo i secondi
	secondi = (((giorni + data.Date - 1) * 24 + (ora.Hours - 1)) * 60 + ora.Minutes)*60 + ora.Seconds;
	secondi += 3600;

	return(secondi);
}

//calcolo se un anno  bisestile
u8 bisestile(u8 anno){

	u8 bis = 0;
	u8 supporto;
	
	supporto = anno;
	supporto /= 4;
	supporto *= 4;
	
	if(supporto == anno){
		bis = 1;
	}
	
	return bis;
}




//imposta orario
void sethour(uint32_t A){

			RTC_DateTypeDef setDate;
			RTC_TimeTypeDef setTime;
			u8 uart[100];

			int i = -27;
			int m = 1;
			int g = 1;
			int h = 0;
			int min = 0;
			int s = 0;
			int b = 0;

			//A = A - 3600*DST;

			sprintf(uart,"input %d\n\n", A);
	    HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
			A = A - 94694400;
			//A += 3600; //ADJ

			while(A >= 126230400){
				A = A -126230400;
				i = i + 4;
				}

			if(A >= 31536000){
				A = A - 31536000;
				i = i + 1;
			}
			if(A >= 31536000){
				A = A - 31536000;
				i = i + 1;
			}

			if(A >= 31536000){
				A = A - 31536000;
				b = 1;
				i = i + 1;
			}



			//mesi

			//gennaio 31
			if(A >= 2678400){
				A = A - 2678400;
				m = m+1;
			}

			//febbraio 28
			if(A >= 2419200 && b == 0 && m == 2){
				A = A - 2419200;
				m = m+1;
			}

			if(A >= 2505600 && b == 1 && m == 2){
				A = A - 2505600;
				m = m+1;
			}

			//marzo 31
			if(A >= 2678400 && m == 3){
				A = A - 2678400;
				m = m+1;
			}

			//aprile 30
			if(A >= 2592000 && m == 4){
				A = A - 2592000;
				m = m+1;
			}

			//maggio 31
			if(A >= 2678400 && m == 5){
				A = A - 2678400;
				m = m+1;
			}

			//giugno 30
			if(A >= 2592000 && m == 6){
				A = A - 2592000;
				m = m+1;
			}

			//luglio 31
			if(A >= 2678400 && m == 7){
				A = A - 2678400;
				m = m+1;
			}

			//agosto 31
			if(A >= 2678400 && m == 8){
				A = A - 2678400;
				m = m+1;
			}

			//settembre 30
			if(A >= 2592000 && m == 9){
				A = A - 2592000;
				m = m+1;
			}

			//ottobre 31
			if(A >= 2678400 && m == 10){
				A = A - 2678400;
				m = m+1;
			}

			//novembre 30
			if(A >= 2592000 && m == 11){
				A = A - 2592000;
				m = m+1;
			}


			//giorni
			while(A >= 86400){
				A = A-86400;
				g = g + 1;
			}

			//ore
			while(A >= 3600){
				A = A - 3600;
				h = h + 1;
			}

			//minuti
			while(A >= 60){
				A = A - 60;
				min = min + 1;
			}

			s = A;


			setDate.Year = i;
			setDate.Month = m;
			setDate.Date = g;
			//setDate.WeekDay = giornoSettimana(setDate);
			setTime.Hours = h;
			setTime.Minutes = min;
			setTime.Seconds = s;
			
			/*setDate.Year = 21;
			setDate.Month = 11;
			setDate.Date = 4;
			//setDate.WeekDay = giornoSettimana(setDate);
			setTime.Hours = 0;
			setTime.Minutes = 26;
			setTime.Seconds = 0;*/

			HAL_RTC_SetDate(&hrtc,&setDate,RTC_FORMAT_BIN);
			HAL_RTC_SetTime(&hrtc,&setTime,RTC_FORMAT_BIN);
			
			
			UpdateTime();

			sprintf(uart,"set: %d %d %d     %d %d %d\n%d\n\n", setDate.Date, setDate.Month, setDate.Year, setTime.Hours,setTime.Minutes, setTime.Seconds,myTimeVar);
	    HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
}


RTC_DateTypeDef posix2date(u32 A){

			RTC_DateTypeDef date;
			
			u8 uart[100];

			int i = -27;
			int m = 1;
			int g = 1;
			int b = 0;

			//A = A - 3600*DST;

			A = A - 94694400;
			//A += 3600; //ADJ

			while(A >= 126230400){
				A = A -126230400;
				i = i + 4;
				}

			if(A >= 31536000){
				A = A - 31536000;
				i = i + 1;
			}
			if(A >= 31536000){
				A = A - 31536000;
				i = i + 1;
			}

			if(A >= 31536000){
				A = A - 31536000;
				b = 1;
				i = i + 1;
			}

			//mesi

			//gennaio 31
			if(A >= 2678400){
				A = A - 2678400;
				m = m+1;
			}

			//febbraio 28
			if(A >= 2419200 && b == 0 && m == 2){
				A = A - 2419200;
				m = m+1;
			}

			if(A >= 2505600 && b == 1 && m == 2){
				A = A - 2505600;
				m = m+1;
			}

			//marzo 31
			if(A >= 2678400 && m == 3){
				A = A - 2678400;
				m = m+1;
			}

			//aprile 30
			if(A >= 2592000 && m == 4){
				A = A - 2592000;
				m = m+1;
			}

			//maggio 31
			if(A >= 2678400 && m == 5){
				A = A - 2678400;
				m = m+1;
			}

			//giugno 30
			if(A >= 2592000 && m == 6){
				A = A - 2592000;
				m = m+1;
			}

			//luglio 31
			if(A >= 2678400 && m == 7){
				A = A - 2678400;
				m = m+1;
			}

			//agosto 31
			if(A >= 2678400 && m == 8){
				A = A - 2678400;
				m = m+1;
			}

			//settembre 30
			if(A >= 2592000 && m == 9){
				A = A - 2592000;
				m = m+1;
			}

			//ottobre 31
			if(A >= 2678400 && m == 10){
				A = A - 2678400;
				m = m+1;
			}

			//novembre 30
			if(A >= 2592000 && m == 11){
				A = A - 2592000;
				m = m+1;
			}


			//giorni
			while(A >= 86400){
				A = A-86400;
				g = g + 1;
			}

			date.Year = i;
			date.Month = m;
			date.Date = g;
			
			return date;

}


u32 isDST(u32 A){
	
	struct dstTime dstLocal;
	
	/* Versione LITE Argentina: nessuna ora legale.
	 * Se DSTon e' 0 non applico alcuna correzione.
	 */
	if(DSTon == 0){
		DST = 0;
		return A;
	}
	
	dstLocal = estremiDSTposix(A);
	
	if(dstLocal.A < dstLocal.B){
		if(A > dstLocal.A && A < dstLocal.B){
			A -= 3600;
		}
	}
	else{
		if(A < dstLocal.A || A > dstLocal.B){
			A -= 3600;
		}
	}	
	
	return A;
}

void impostaOra(u32 A){
	
	//A = isDST(A);
	regolaOra = A;

}



