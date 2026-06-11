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

//BT attivo
extern u8 BTattivo;

//identificativo
extern u8 identificativo[16];

//numeri
extern u8 numeroAllarmi[20];
extern u8 numeroDevice[20];

//variabili temporali
extern u32 myTimeVar;

//attivazione antifurto
u8 antifurtoAttivo = 0;

//calibrazione antifurto
u16 calibAntifurto = 0;

u16 sogliaAntifurto = 50;

u16 misuraLuce = 0;

u8 inizializzaAntifurto = 0;

//invio messaggio
extern u8 messaggioTampering;

extern double latitudineD,longitudineD;

extern u8 spegniLed;

//antifurto scattato
u8 antifurtoScattato = 0;
u16 nIntrusioni = 0;


void attivazioneAntifurto(void){
	u8 addressFram[2] = {1,86};
	
	if(BTattivo == 1){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
	}
	delay(500);
	
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
	delay(100);
	calibAntifurto = acquisizioneADC(0);
	saveU16fram(calibAntifurto,&addressFram[0]);
	

	if(BTattivo == 1){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
	}

}


void controlloAntifurtoProva(void){
int diff;
	u16 acq;
	u8 messaggio[100];
	u8 addressFram[2] = {1,90};
	u16 addressFramInt = 1024;
	

	if(spegniLed == 0){
		return;
	}
	
	acq = acquisizioneADC(0);
	misuraLuce = acq;
	
	if(inizializzaAntifurto == 1){
		inizializzaAntifurto = 0;
		calibAntifurto = acq;
		addressFram[1] = 86;
		saveU16fram(calibAntifurto,&addressFram[0]);
		addressFram[1] = 90;
	}
	
	if(acq > calibAntifurto && antifurtoScattato == 0){
		return;
	}
	else{
		if(acq < calibAntifurto){
			diff = calibAntifurto - acq;	
		}
		else{
			diff = 0;
		}
		
	}
	
	if(antifurtoScattato == 0){
			if(diff > sogliaAntifurto){
								
				antifurtoScattato = 1; //salvo che l'antifurto è scattato
				saveArrayFram(&antifurtoAttivo,&addressFram[0],1);			
				
				addressFramInt += nIntrusioni*8; u162array(&addressFram[0], addressFramInt); //indirizzo per il salvataggio dell'evento
				u322array(&messaggio[0],myTimeVar); //creo l'evento
				saveArrayFram(&messaggio[0],&addressFram[0],4); //salvo l'evento
				
				if(nIntrusioni == 99){ //incremento numero intrusioni
					nIntrusioni = 0;
				}					
				else{
					nIntrusioni++;				
				}
				
				addressFram[0] = 1; addressFram[1] = 88;
				u162array(&messaggio[0],nIntrusioni);
				saveArrayFram(&messaggio[0],&addressFram[0],2); //salvo numero intrusioni			
				
				//invio SMS
				sprintf(messaggio,"Alarm!\nthis cabinet could be open: ----------------\nlat: %.3f  long: %.3f",latitudineD,longitudineD);
				copiaArray(&messaggio[35],&identificativo[0],16);
				inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),&messaggio[0],strlen(messaggio));
				aggiungiIntrusioneDB(1);
			}
	}
	else{
			if(diff < sogliaAntifurto){
				antifurtoScattato = 0; //salvo che l'antifurto è rientrato
				saveArrayFram(&antifurtoAttivo,&addressFram[0],1);
				
				addressFramInt += 4 + (nIntrusioni-1)*8; u162array(&addressFram[0], addressFramInt); //indirizzo per il salvataggio dell'evento
				u322array(&messaggio[0],myTimeVar); //creo l'evento
				saveArrayFram(&messaggio[0],&addressFram[0],4); //salvo l'evento
				
				antifurtoAttivo = 0;
				addressFram[0] = 1; addressFram[1] = 85;
				saveArrayFram(&antifurtoAttivo,&addressFram[0],1);
				
			}	
	
	}	

}




void controlloAntifurto(void){
	int diff;
	u16 acq;
	u8 messaggio[100];
	u8 addressFram[2] = {1,90};
	u16 addressFramInt = 1024;
	
	ADC_ChannelConfTypeDef canale;
	
	//imposto il canale
	canale.Channel = ADC_CHANNEL_0;
	HAL_ADC_ConfigChannel(&hadc1, &canale);
	
	
	//effettuo la misura
	HAL_ADC_ConfigChannel(&hadc1, &canale);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,1000);
		acq = HAL_ADC_GetValue(&hadc1);
		diff = acq	-	calibAntifurto;
	
	//effettuo il confronto
	if(antifurtoScattato == 0){
			if(diff > sogliaAntifurto){
				sprintf(messaggio,"Alarm!\n this cabinet has been opened: ----------------\nlat: %3f  long%3f",latitudineD,longitudineD);
				copiaArray(&messaggio[38],&identificativo[0],16);
				inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),&messaggio[0],100);
				
				//aggiungiDB
				aggiungiIntrusioneDB(1);
				
				antifurtoScattato = 1; //salvo che l'antifurto è scattato
				saveArrayFram(&antifurtoAttivo,&addressFram[0],1);			
				
				addressFramInt += nIntrusioni*8; u162array(&addressFram[0], addressFramInt); //indirizzo per il salvataggio dell'evento
				u322array(&messaggio[0],myTimeVar); //creo l'evento
				saveArrayFram(&messaggio[0],&addressFram[0],4); //salvo l'evento
				
				if(nIntrusioni == 255){ //incremento numero intrusioni
					nIntrusioni = 0;
				}					
				else{
					nIntrusioni++;				
				}
				
				addressFram[0] = 1; addressFram[1] = 88;
				u162array(&messaggio[0],nIntrusioni);
				saveArrayFram(&messaggio[0],&addressFram[0],2); //salvo numero intrusioni
			
			
			}
	}
	else{
			if(diff < sogliaAntifurto){
				antifurtoScattato = 0; //salvo che l'antifurto è rientrato
				saveArrayFram(&antifurtoAttivo,&addressFram[0],1);
				
				addressFramInt += 4 + nIntrusioni*8; u162array(&addressFram[0], addressFramInt); //indirizzo per il salvataggio dell'evento
				u322array(&messaggio[0],myTimeVar); //creo l'evento
				saveArrayFram(&messaggio[0],&addressFram[0],4); //salvo l'evento
			
			}	
	
	}	
}


void downloadIntrusioni(void){

u8 addressFram[2];
u16 addressFramInt;
int i = 0;
int contatore = 0;
u8 array[8];
	
	for(i=0;i<100;i++){ //conteggio eventi		
		addressFramInt = 1024 + i*8; u162array(&addressFram[0],addressFramInt);
		ReadArrayFram(&array[0],&addressFram[0],1);
		if(array[0] != 0){
			contatore++;
		}	
	}
	
	u162array(&array[0],contatore); //invio contatore
	if(BTattivo == 1){
		HAL_UART_Transmit(&huart2,&array[0],2,1000);
	}

	
	for(i=0;i<100;i++){
		resetWD();
		addressFramInt = 1024 + i*8; u162array(&addressFram[0],addressFramInt);
		ReadArrayFram(&array[0],&addressFram[0],8);
		if(array[0] != 0 && BTattivo == 1){
			HAL_UART_Transmit(&huart2,&array[0],8,1000);
		}
	}

}

//cancella eventi neutro
void formattaTampering(void){
	
	int i = 0;
	u8 addressFram[2] = {1,88};
	u8 formattatore[256];
		
	nIntrusioni = 0;
	
	
	u162array(&formattatore[0],nIntrusioni);
	saveArrayFram(&formattatore[0],&addressFram[0],2);
		
	while(i<256){ //genero array per formattare FRAM
		formattatore[i] = 0;
		i++;
	}
	i=0;
	
	addressFram[0] = 4;	addressFram[1] = 0;
	while(i<4){
		saveArrayFram(&formattatore[0],&addressFram[0],256);
		i++;
		addressFram[0]++;
	}
	
}










