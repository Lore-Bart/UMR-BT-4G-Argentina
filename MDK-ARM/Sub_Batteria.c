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

extern u32 V[3];

extern u32 tempo;

//identificativo
extern u8 identificativo[16];

extern double latitudineD,longitudineD;

u8 batteryLevel = 5;
u8 batteriaInCarica = 0;
u8 messaggioBatteria = 2;

u16 timeCarica = 0;

extern u8 alimentatore;

extern u8 BTattivo;

extern u32 sniff32;
u8 mettoilmeno = 0;

double tensioneB;
int tensioneBint;
long temperatura = 0;

double controllaBatteriaProva(void){
	u8 rimettiInCarica = 0;
	u32 acquisizione;
	double tensione;
	
	//tolgo dalla carica
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1) != 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		rimettiInCarica = 1;
		delay(50);
	}
	
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	delay(50);
	acquisizione = acquisizioneADC(9);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);

	if(rimettiInCarica == 1){
		delay(50);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		rimettiInCarica = 0;
	}
	
	tensione = acquisizione;
	tensione *= 0.003071;
	
	
	
	return tensione;
	
}

u32 controllaBatteriaProva5(void){
	u8 rimettiInCarica = 0;
	u32 acquisizione;
	
	//tolgo dalla carica
	/*if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1) != 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		rimettiInCarica = 1;
		delay(50);
	}*/
	
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	delay(50);
	acquisizione = acquisizioneADC(9);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);

	/*if(rimettiInCarica == 1){
		delay(50);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		rimettiInCarica = 0;
	}*/
	
	return acquisizione;
	
}

void controllaBatteria(void){
	double stampella;
	u32 acquisizione;
	u8 sms[200];
	u8 rimettiInCarica = 0;
	u8 addressFram[2] = {1,91};
	u8 uart[100];
	
	u8 x;
	
	
	if(batteriaInCarica != 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		rimettiInCarica = 1;
	}
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	delay(400);
	acquisizione = acquisizioneADC(9);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	
	if(batteriaInCarica != 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
	}
	
	tensioneB = acquisizione;
	tensioneB *= 0.003071;
	//tensioneB += ((tensioneB/97.37))*0.5;	
	
	if(alimentatore == 0){
		tensioneB += 0.15;
	}
	//tensioneB += 0.90;
	
	//return;
	
	//sprintf(uart,"batteria: %f, %d\n",tensioneB,acquisizione);
	//HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	tensioneB *= 10;
	tensioneBint = tensioneB;
	tensioneB /= 10;
	
	//batteria
	if(tensioneB > 8.1){
		batteryLevel = 100; //1800
	}
	else if(tensioneB > 7.95 && tensioneB <= 8.1){ //1620
		batteryLevel = 90;
		//if(batteriaInCarica == 0){timeCarica = 1620;}
		//else if(batteriaInCarica == 1 && timeCarica < 1620){timeCarica = 1620;}
	}
	else if(tensioneB > 7.78 && tensioneB <= 7.95){ //1440
		batteryLevel = 80;
		//if(batteriaInCarica == 0){timeCarica = 1440;}
		//else if(batteriaInCarica == 1 && timeCarica < 1440){timeCarica = 1440;}
	}
	else if(tensioneB > 7.65 && tensioneB <= 7.78){ //1260
		batteryLevel = 70;
		//if(batteriaInCarica == 0){timeCarica = 1260;}
		//else if(batteriaInCarica == 1 && timeCarica < 1260){timeCarica = 1260;}
	}
	else if(tensioneB > 7.57 && tensioneB <= 7.65){ //1080
		batteryLevel = 60;
		//if(batteriaInCarica == 0){timeCarica = 1080;}
		//else if(batteriaInCarica == 1 && timeCarica < 1080){timeCarica = 1080;}
	}
	else if(tensioneB > 7.50 && tensioneB <= 7.57){ //900
		batteryLevel = 50;
		//if(batteriaInCarica == 0){timeCarica = 900;}
		//else if(batteriaInCarica == 1 && timeCarica < 900){timeCarica = 900;}
	}
	else if(tensioneB > 7.45 && tensioneB <= 7.50){ //720
		batteryLevel = 40;
		//if(batteriaInCarica == 0){timeCarica = 720;}
		//else if(batteriaInCarica == 1 && timeCarica < 720){timeCarica = 720;}
	}
	else if(tensioneB > 7.41 && tensioneB <= 7.45){ //540
		batteryLevel = 30;
		//if(batteriaInCarica == 0){timeCarica = 540;}
		//else if(batteriaInCarica == 1 && timeCarica < 540){timeCarica = 540;}
	}
	else if(tensioneB > 7.30 && tensioneB <= 7.41){ //360
		batteryLevel = 20;
		//if(batteriaInCarica == 0){timeCarica = 360;}
		//else if(batteriaInCarica == 1 && timeCarica < 360){timeCarica = 360;}
	}
	else if(tensioneB > 7.05 && tensioneB <= 7.30){ //180
		batteryLevel = 10;
		//if(batteriaInCarica == 0){timeCarica = 180;}	
		//else if(batteriaInCarica == 1 && timeCarica < 180){timeCarica = 180;}		
	}
	else{
		batteryLevel = 5;
		//if(batteriaInCarica == 0){timeCarica = 0;}	//0	
		//else if(batteriaInCarica == 1 && timeCarica < 0){timeCarica = 0;}		
	}
	
	
	//sgancio batteria
	if(tensioneB < 6.8 && alimentatore == 0){
		//HAL_UART_Transmit(&huart1,(u8*)"DISCONNESSA\n",12,100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	else{
		//HAL_UART_Transmit(&huart1,(u8*)"CONNESSA\n",9,100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
	}
	
	
	//gestione carica della batteria
	
	/*if(batteriaInCarica == 1){
		timeCarica += 1;
	}*/
	
	//controllo se c'è tensione
	if(alimentatore != 0){
		if(batteryLevel <= 70){
			batteriaInCarica = 1; ////////
		}
		else if(batteryLevel >= 100){
			batteriaInCarica = 0;
		}
	}
	else{
		batteriaInCarica = 0;
	}
	

	acquisizioneTemp();
	
	if(temperatura < 0 || temperatura > 75){ //sgancia a circa 70°C
		batteriaInCarica = 0;
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
	}

	
	if(batteriaInCarica == 1 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1) == 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
	}
	
	if(batteriaInCarica == 0 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1) != 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
	}
	
	
	
	//invio messaggi
	if(batteryLevel >= 40 &&messaggioBatteria != 0){
		messaggioBatteria = 0;
		saveArrayFram(&messaggioBatteria,&addressFram[0],1);
	}
	if(batteryLevel <= 30 && batteryLevel > 10 && messaggioBatteria == 0 && BTattivo == 0 && alimentatore == 0 && V[0] < (20000 - VAR*10000)){
		sprintf(sms,"battery alarm!\ndevice: ----------------\nlat: %.3f  long: %.3f\nresidual charge: 30-",latitudineD,longitudineD);
		copiaArray(&sms[23],&identificativo[0],16);
		sms[strlen(sms)-1] = 37;
		//inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),&sms[0],strlen(sms));
		messaggioBatteria = 1;	
		saveArrayFram(&messaggioBatteria,&addressFram[0],1);
	}
	if(batteryLevel <= 10 && messaggioBatteria != 2 && BTattivo == 0 && alimentatore == 0 && V[0] < (20000 - VAR*10000)){
		sprintf(sms,"battery alarm!\ndevice: ----------------\nlat: %.3f  long: %.3f\nresidual charge: 10-",latitudineD,longitudineD);
		copiaArray(&sms[23],&identificativo[0],16);
		sms[strlen(sms)-1] = 37;
		//inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),&sms[0],strlen(sms));
		messaggioBatteria = 2;	
		saveArrayFram(&messaggioBatteria,&addressFram[0],1);
	}
	
	
	

}


long acquisizioneTemp(void){
	ADC_ChannelConfTypeDef sConfig;
	u32 acquisizione = 0;
	double tempLocalD;
	long tempLocal;
	
	

	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	} 
					
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,5) == HAL_OK){
		acquisizione = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
					
	
	
	tempLocalD = acquisizione;
	
	tempLocalD *= 0.000805664;
	tempLocalD -= 0.76;
	tempLocalD /= 0.0025;
	tempLocalD += 25;
	
	tempLocalD +=0.5;
	tempLocal = tempLocalD;
	
	temperatura = tempLocal;
	
	return tempLocal;
	
}






