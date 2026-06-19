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
extern ADC_HandleTypeDef hadc1;



//numeri
extern u8 lastNumber[10];
extern u8 numeroAllarmi[20];

//sovracorrenti
u32 calibrazioneI[6];
u32 acquisizioneI[6];
u32 correnteI[6];
u8 guasti = 0; //aggiungere in FRAM

//identificativo
extern u8 identificativo[16];

//inibizione sovracorrenti
u8 inibitOverI = 0;

//autotune
u8 eventoAvvenuto = 0;

//soglie
u16 sogliaCorrenteA = 1000;
u16 sogliaCorrenteB = 1000;

//data e ora
extern uint32_t myTimeVar;

//BT attivo
extern u8 BTattivo;

//calibrazione guasti
u8 calibGuasti = 0;

u32 contatore = 0;

//format guasti
extern u8 formatGuasti;

//acquisizioni
u32 acq[6];
u32 correnteGuasto[6];

//inibizione guasto
u8 inibitGuasto = 0;

//simulatore
u8 simulaGuasto = 0;

//invio eventi
u8 invio[1600];

u8 inibitGuastoSMS = 0;

extern u8 sonopassato;

extern double latitudineD,longitudineD;

static u8 validEventTimestamp(u32 timestamp)
{
	/*
	 * 0xFFFFFFFF corrisponde al 07/02/2106: e' il classico valore
	 * di EEPROM/FRAM non realmente inizializzata o non cancellata.
	 * Non deve essere conteggiato ne' visualizzato come evento.
	 */
	if(timestamp == 0UL) return 0;
	if(timestamp == 0xFFFFFFFFUL) return 0;
	return 1;
}

long contaguasto = 0;

long contaGuasto = 0;

extern u8 lowpower;

long quanteVolteGuasti = 0;

extern u8 updateAttivo;

extern u16 eccezione;

extern u8 alimentatore;

extern int netCount;

void salvaGuastoFake(void){

	u8 array[16] = {0,0,0,0,0,5,0,5,0,5,0,5,0,5,0,5};
	u8 offset[2];
	u16 beforeOffset;
	u16 beforeFram;
	u8 addressfram[2] = {1,18};
	
	u322array(&array[0],myTimeVar);
	
	//beforeOffset = 4096 + 16*guasti;
	//u162array(&offset[0],beforeOffset);
	//guasti++;
	//saveArrayFram(&guasti,&addressfram[0],1);
	//writeNFC(&array[0],16,&offset[0]);
	
	u322array(&array[0],myTimeVar);
	
	beforeOffset = 4096 + 16*guasti;
	u162array(&offset[0],beforeOffset);
	beforeFram = 6144 + 16*guasti;
			
			if(guasti < 99){
				guasti++;
			}
			else{
				guasti = 0;
			}
			
			//__disable_irq();
			saveArrayFram(&guasti,&addressfram[0],1);
			u162array(&addressfram[0],beforeFram);
			saveArrayFram(&array[0],&addressfram[0],16);
			writeNFC(&array[0],16,&offset[0]);


}

void acquisciCorrenti(void){
	
}

void exTimerGuasti(void){
	int i = 0;
	u8 arrayEvento[16];
	u8 offset[2];
	u16 beforeOffset;
	u8 addressfram[2] = {1,18};
	u32 maxValue;
	u8 sms[200];
	u16 beforeFram;
	u8 inviaTutto[100];
		
	acquisizioneTemp();
	
	//netCount++;
	//contaNet();
	
	if(lowpower != 0){
		return;
	}
	
	if(updateAttivo > 0){
		return;
	}
	
	if(inibitGuasto != 0 || inibitGuastoSMS != 0 || (sogliaCorrenteA == 0 && sogliaCorrenteB == 0)){
		return;
	}
	
	//quanteVolteGuasti++;
	
		while(i<6){
			acq[i] = acquisizioneADC(i+1);
			i++;
		}
		i = 0;
		
		if(eccezione == 1){
			acq[1] = acq[1] - 390;
		}

		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
		delay(20);
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
		
		//return;
		
		quanteVolteGuasti = calibrazioneI[0];

	
		while(i<6){
			correnteGuasto[i] = acq[i]*1000/calibrazioneI[i];
			i++;			
		}
		i = 0;
		

		
		if(simulaGuasto == 1){
			simulaGuasto = 0;
			correnteGuasto[0] = 1000;
			HAL_UART_Transmit(&huart1,(u8*)"guasto",6,1000);
		}
		
		
		if(((correnteGuasto[0] >= sogliaCorrenteA || correnteGuasto[1] >= sogliaCorrenteA || correnteGuasto[2] >= sogliaCorrenteA) && sogliaCorrenteA > 0)	|| ((correnteGuasto[3] >= sogliaCorrenteB || correnteGuasto[4] >= sogliaCorrenteB || correnteGuasto[5] >= sogliaCorrenteB) && sogliaCorrenteB > 0)){
			u322array(&arrayEvento[0],myTimeVar);
			u162array(&arrayEvento[4],correnteGuasto[0]);
			u162array(&arrayEvento[6],correnteGuasto[1]);
			u162array(&arrayEvento[8],correnteGuasto[2]);
			u162array(&arrayEvento[10],correnteGuasto[3]);
			u162array(&arrayEvento[12],correnteGuasto[4]);
			u162array(&arrayEvento[14],correnteGuasto[5]);
		
			maxValue = correnteGuasto[0];
			
			while(i<5){
				if(maxValue < correnteGuasto[i+1]){
					maxValue = correnteGuasto[i+1];
				}
				i++;
			}
			i=0;
		
			beforeOffset = 4096 + 16*guasti;
			u162array(&offset[0],beforeOffset);
			beforeFram = 6144 + 16*guasti;
			
			if(guasti < 99){
				guasti++;
			}
			else{
				guasti = 0;
			}
			
			//__disable_irq();
			saveArrayFram(&guasti,&addressfram[0],1);
			u162array(&addressfram[0],beforeFram);
			saveArrayFram(&arrayEvento[0],&addressfram[0],16);
			writeNFC(&arrayEvento[0],16,&offset[0]);
			//__enable_irq();
		
		//invio SMS
		sprintf(sms,"Alarm!\ndetected overcurrent!\nUMR: ----------------\ncurrent maximum value: %d A\nlat: %.3f  long: %.3f",maxValue,latitudineD,longitudineD);
		copiaArray(&sms[34],&identificativo[0],16);			
		inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),&sms[0],strlen(sms));
			
		//aggiungiDB
		aggiungiGuastoDB(1,&correnteGuasto[0]);
			
		//disattivo il Timer e attivo inibizione
		inibitGuasto = 60;
		//eventoAvvenuto = 1;			
		}
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
		delay(20);
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
			
}


void TIM3_IRQHandler(void){
	
	HAL_TIM_IRQHandler(&htim3);
	
	exTimerGuasti();
}


//ricalcola soglie
void ricalcolaSoglie(void){
	int i  = 0;
	u32 sogliaMax;
	u8 array[4];
	u8 addressFram[2] = {1,43};
	u8 offset[2] = {0,34};
	u8 modif = 0;
	u8 sms[100];
	
	if(alimentatore == 0){
		return;
	}
	
	while(i<6){
			acq[i] = acquisizioneADC(i+1);
			i++;
		}
		i = 0;
		//contatore++;
	
		if(eccezione == 1){
			acq[1] = acq[1] - 390;
		}
		
		while(i<6){
			correnteGuasto[i] = acq[i]*1000/calibrazioneI[i];
			i++;
		}
		i = 0;
		
		if(simulaGuasto == 1){
			simulaGuasto = 0;
			correnteGuasto[0] = 700;
		}
		
		if(sogliaCorrenteA != 0){
			if(correnteGuasto[0] >= sogliaCorrenteA || correnteGuasto[1] >= sogliaCorrenteA || correnteGuasto[2] >= sogliaCorrenteA){
				sogliaMax = correnteGuasto[0];
				if(sogliaMax < correnteGuasto[1]){sogliaMax = correnteGuasto[1];}
				if(sogliaMax < correnteGuasto[2]){sogliaMax = correnteGuasto[2];}
				
				sogliaCorrenteA = sogliaMax + 15;
				modif = 1;
			}
		}
		
		if(sogliaCorrenteB != 0){
			if(correnteGuasto[3] >= sogliaCorrenteB || correnteGuasto[4] >= sogliaCorrenteB || correnteGuasto[5] >= sogliaCorrenteB){
				sogliaMax = correnteGuasto[3];
				if(sogliaMax < correnteGuasto[4]){sogliaMax = correnteGuasto[4];}
				if(sogliaMax < correnteGuasto[5]){sogliaMax = correnteGuasto[5];}
				
				sogliaCorrenteB = sogliaMax + 15;
				modif = 1;
			}
		}
		
		//sonopassato = modif;
		
		if(modif==1){
			sprintf(sms,"overcurrent threshold modified: thrA: %dA  thrB: %dA",sogliaCorrenteA,sogliaCorrenteB);
			inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),&sms[0],strlen(sms));
		}
				
		if(modif == 1){
			u162array(&array[0],sogliaCorrenteA);
			u162array(&array[2],sogliaCorrenteB);
			saveArrayFram(&array[0],&addressFram[0],4);
			writeNFC(&array[0],4,&offset[0]);
		}
		
}


//calibrazione
void calibOverI(void){ //li calibro con 100 A
	u8 addressFram[2] = {1,19};
	int i  = 0;
	u8 uart[100];
		
		while(i<6){
			acq[i] = acquisizioneADC(i+1);
			i++;
		}
		i = 0;
		
		if(eccezione == 1){
			acq[1] = acq[1] - 390;
		}
		
		while(i<6){
			//calibrazioneI[i] = acq[i]*1000/100;
			calibrazioneI[i] = acq[i]*1000/75;
			saveU32fram(calibrazioneI[i],&addressFram[0]);
			addressFram[1] += 4;
			i++;
		}
		i = 0;
	
	
}




//modifica soglia A
void modificaSogliaA(int value){
	u8 addressFram[2] = {1,43};
	u8 offset[2] = {0,34};
	u8 array[10];
	
	
	sogliaCorrenteA = value;
	u162array(&array[0],value);
	saveArrayFram(&array[0],&addressFram[0],2);
	writeNFC(&array[0],2,&offset[0]);
}


//modifica soglia B
void modificaSogliaB(int value){
	u8 addressFram[2] = {1,45};
	u8 offset[2] = {0,36};
	u8 array[10];

	
	
	sogliaCorrenteB = value;
	u162array(&array[0],value);
	saveArrayFram(&array[0],&addressFram[0],2);
	writeNFC(&array[0],2,&offset[0]);
}

void downloadGuasti(void){
	int i = 0;
	u16 beforeFram;
	u8 addressFram[2];
	u16 contatore = 0;
	u8 array[16];
	u32 timestamp;
	
	while(i<100){		
		beforeFram = 6144 + 16*i;
		u162array(&addressFram[0],beforeFram);
		ReadArrayFram(&array[0],&addressFram[0],16);
		timestamp = array2u32(&array[0]);
		
		if(validEventTimestamp(timestamp)){
			copiaArray(&invio[contatore*16],&array[0],16);
			contatore++;
		}		
		i++;
	}
	i = 0;
		
	//invio numero eventi
	u162array(&array[0],contatore);
	if(BTattivo == 1){
		HAL_UART_Transmit(&huart2,&array[0],2,1000);
	}
	i = 0;
	
	while(i<contatore){
		HAL_UART_Transmit(&huart2,&invio[i*16],16,1000);
		i++;
		resetWD();
	}
	
}


//cancella tutti i guasti salvati
void formattaGuasti(void){
	u8 addressFram[2] = {1,18};
	int i = 0;
	u8 formattatore[256];

	/*
	 * La cancellazione dei guasti coinvolge sia FRAM sia area NDEF NFC.
	 * Durante l'operazione blocchiamo la generazione di nuovi eventi e
	 * scartiamo eventuali scritture NFC ancora in coda, altrimenti un evento
	 * vecchio potrebbe essere riscritto dopo l'azzeramento.
	 */
	inibitGuasto = 255;
	inibitGuastoSMS = 255;
	clearNFCpending();

	while(i<256){ //genero array per formattare FRAM
		formattatore[i] = 0;
		i++;
	}
	i=0;
	
	guasti = 0;
	saveArrayFram(&guasti,&addressFram[0],1);
	
	addressFram[0] = 24;	addressFram[1] = 0;
	while(i<7){
		saveArrayFram(&formattatore[0],&addressFram[0],256);
		resetWD();
		i++;
		addressFram[0]++;
	}
	
	/*
	 * NFC: 100 eventi da 16 byte = 1600 byte.
	 * Cancellazione robusta progressiva: 25 chunk da 64 byte,
	 * con verifica di lettura dopo ogni scrittura.
	 */
	formatGuasti = 100;
	inviaDebug((u8*)"erase overcurrent/fault events started\n");
	
}


//download ultimo evento sovracorrente
void ultimoGuasto(u8 *outBuf){
	u8 addressFram[2];
	u16 beforeFram;
	u8 array[16];
	u32 timestamp;
	int i;
	int j;
	int found = 0;
	
	for(j=0;j<16;j++){
		outBuf[j] = 0;
	}
	
	/* Non usare direttamente guasti-1: dopo cancellazioni o memoria sporca
	 * puo' puntare a uno slot vuoto/0xFFFFFFFF. Cerchiamo l'ultimo
	 * evento realmente valido.
	 */
	for(i=0;i<100;i++){
		beforeFram = 6144 + 16*i;
		u162array(&addressFram[0],beforeFram);
		ReadArrayFram(&array[0],&addressFram[0],16);
		timestamp = array2u32(&array[0]);
		if(validEventTimestamp(timestamp)){
			copiaArray(&outBuf[0],&array[0],16);
			found = 1;
		}
	}
	
	if(found == 0){
		for(j=0;j<16;j++){
			outBuf[j] = 0;
		}
	}
}











