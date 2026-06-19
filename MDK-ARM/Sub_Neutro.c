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

//identificativo
extern u8 identificativo[16];

//BT attivo
extern u8 BTattivo;

//variabili misurandi
extern u32 V[3]; //tensioni

//evento neutro
u8 eventoNeutro = 0;

//numero eventi
u8 eventiNeutro = 0;

//data e ora
extern uint32_t myTimeVar;

//numero
extern u8 numeroAllarmi[20];

//soglie
u16 sogliaNeutro = 0;

//inibizione
int inibitN = 0;

//format flag
extern u16 formatNeutro;

//flag scrittura Neutro
u8 scritturaNeutro = 0;
u8 arrayNeutro[16];
u8 offsetNeutro[2];

extern u8 lowpower;

u8 simulaNeutro = 0;

u32 sniffNeutro = 0;

extern double latitudineD,longitudineD;

u16 underVoltageTH = 0;
u8 underVoltageEvent = 0;
u16 overVoltageTH = 0;
u8 overVoltageEvent = 0;

static u8 validNeutroTimestamp(u32 timestamp)
{
	/* 0xFFFFFFFF viene visualizzato come 07/02/2106: non e' un evento valido. */
	if(timestamp == 0UL) return 0;
	if(timestamp == 0xFFFFFFFFUL) return 0;
	return 1;
}


void afterNeutro(void){
	
	
	if(scritturaNeutro ==  1){
		scritturaNeutro = 0;
		writeNFC(&arrayNeutro[0],16,&offsetNeutro[0]);
	}
	else if(scritturaNeutro == 2){
		scritturaNeutro = 1;	
	}

}

void salvaNeutroFake(void){
	
	u8 array1[16] = {0,0,0,0,0x4e,0x20,0x4e,0x20,0x4e,0x20,0,0,0,0,0x4e,0x20};
	u8 array3[16] = {0x4e,0x20,0x4e,0x20,0,0,0,0,0x4e,0x30,0x4e,0x30,0x4e,0x30,0,0};
	u16 beforeOffset;
	u8 offset[2];
	static u8 flag = 0;
	u8 addressFram[2] = {1,17};
	
	if(flag == 0){
		u322array(&array1[0],myTimeVar);
		u322array(&array1[10],myTimeVar);
		beforeOffset = 64 + eventiNeutro*32;
		u162array(&offset[0],beforeOffset);
		writeNFC(&array1[0],16,&offset[0]);
		
		eventiNeutro++;
		saveArrayFram(&eventiNeutro,&addressFram[0],2);
		flag = 1;
	}
	else{
		u322array(&array3[4],myTimeVar);
		beforeOffset = 64 + eventiNeutro*32 - 16;
		u162array(&offset[0],beforeOffset);
		writeNFC(&array3[0],16,&offset[0]);
		flag = 0;
	}
	


}

void checkNeutro(void){
	u16 diff[3];
	static u16 diffMax[3];
	static u16 diffMaxAss;
	u8 array[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	u8 addressFram[2];
	u16 beforeFram;
	u8 offset[2];
	u16 beforeOffset;
	int i = 0;
	static u32 epochMax;
	u8 sms[100];
	double max;	
	long sogliaNeutroLocal;
		
	sogliaNeutroLocal = sogliaNeutro;
		
	if(lowpower != 0){
		return;
	}
		
	diff[0] = abs(V[0]-V[1]);
	diff[1] = abs(V[1]-V[2]);
	diff[2] = abs(V[2]-V[0]);
	
	if(simulaNeutro == 1){
		eventoNeutro = 0;
		simulaNeutro = 0;
		diff[0] = 35000;
		simulaNeutro = 2;
	}
	else if(simulaNeutro == 2){
		diff[0] = 38000;
		simulaNeutro = 0;		
	}
	
	if(eventoNeutro == 0 && (diff[0] >= sogliaNeutroLocal || diff[1] >= sogliaNeutroLocal || diff[2] >= sogliaNeutroLocal)){
		u322array(&array[0],myTimeVar);
		u162array(&array[4],diff[0]);
		u162array(&array[6],diff[1]);
		u162array(&array[8],diff[2]);
		
		
		
		diffMax[0] = diff[0];
		diffMax[1] = diff[1];
		diffMax[2] = diff[2];
	
		diffMaxAss = diffMax[0];
		if(diffMaxAss < diffMax[1]){diffMaxAss = diffMax[1];}
		if(diffMaxAss < diffMax[2]){diffMaxAss = diffMax[2];}
		
		//sniffNeutro = diffMax[0];
		max = diffMaxAss;	max /= 100;
		
		epochMax = myTimeVar;
		
		addressFram[0] = 1; addressFram[1] = 16;
		eventoNeutro = 1;
		saveArrayFram(&eventoNeutro,&addressFram[0],1);
		
		beforeFram = 2048 + eventiNeutro*32;
		u162array(&addressFram[0],beforeFram);
		saveArrayFram(&array[0],&addressFram[0],32);
		
		beforeOffset = 64 + eventiNeutro*32;
		u162array(&offset[0],beforeOffset);
		writeNFC32(&array[0],16,&offset[0]);
		beforeOffset = 64 + eventiNeutro*32+16;
		u162array(&offsetNeutro[0],beforeOffset);
		copiaArray(&arrayNeutro[0],&array[16],16); //scritturaNeutro = 2;		
		writeNFC32(&arrayNeutro[0],16,&offsetNeutro[0]);

		sprintf(sms,"alarm!\nneutral event started\nUMR: ----------------\nmaximum voltage difference: %.2f V\nlat: %.3f  long: %.3f",max,latitudineD,longitudineD);
		copiaArray(&sms[34],&identificativo[0],16);
		inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),sms,strlen(sms));
		aggiungiNeutroStartDB(1,&diff[0]);
		//inviaSMSprova();
		
	}

	
	else if(eventoNeutro == 1 && diff[0] < sogliaNeutroLocal && diff[1] < sogliaNeutroLocal && diff[2] < sogliaNeutroLocal){
		
		addressFram[0] = 1; addressFram[1] = 16;
		eventoNeutro = 0;
		saveArrayFram(&eventoNeutro,&addressFram[0],1);
		
		beforeFram = 2048 + eventiNeutro*32;
		u162array(&addressFram[0],beforeFram);
		ReadArrayFram(&array[0],&addressFram[0],32);
		
		u322array(&array[10],epochMax);
		u162array(&array[14],diffMax[0]);
		u162array(&array[16],diffMax[1]);
		u162array(&array[18],diffMax[2]);
		u322array(&array[20],myTimeVar);
		u162array(&array[24],diff[0]);
		u162array(&array[26],diff[1]);
		u162array(&array[28],diff[2]);
		
	
		beforeFram = 2048 + eventiNeutro*32;
		u162array(&addressFram[0],beforeFram);
		saveArrayFram(&array[0],&addressFram[0],32);
		
		beforeOffset = 64 + eventiNeutro*32;
		u162array(&offset[0],beforeOffset);
		writeNFC32(&array[0],16,&offset[0]);
		beforeOffset = 64 + eventiNeutro*32+16;
		u162array(&offsetNeutro[0],beforeOffset);
		copiaArray(&arrayNeutro[0],&array[16],16); //scritturaNeutro = 2;
		writeNFC32(&arrayNeutro[0],16,&offsetNeutro[0]);

		sprintf(sms,"neutral event ended\nUMR: ----------------\nlat: %.3f  long: %.3f",latitudineD,longitudineD);
		copiaArray(&sms[25],&identificativo[0],16);
		inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),sms,strlen(sms));
		aggiungiNeutroEndDB(1,epochMax,&diffMax[0],&diff[0]);
	//aumentare evento
		
		if(eventiNeutro == 99){
			eventiNeutro = 0;
		}
		else{
			eventiNeutro++;
		}
		addressFram[0] = 1;	addressFram[1] = 17;
		saveArrayFram(&eventiNeutro,&addressFram[0],1);
		
	}
	
	if(diff[0] > diffMaxAss || diff[1] > diffMaxAss || diff[2] > diffMaxAss){
		
		diffMax[0] = diff[0];
		diffMax[1] = diff[1];
		diffMax[2] = diff[2];
		
		diffMaxAss = diffMax[0];
		if(diffMaxAss > diffMax[1]){diffMaxAss = diffMax[1];}
		if(diffMaxAss > diffMax[2]){diffMaxAss = diffMax[2];}
		
		epochMax = myTimeVar;
				
	}
	
}


void modificaSogliaN(u16 value){
	u8 addressFram[2] = {0,249};
	u8 array[2];
	u8 offset[2] = {0,32};
	
	sogliaNeutro = value;
	u162array(&array[0],value);
	saveArrayFram(&array[0],&addressFram[0],2);
	writeNFC(&array[0],2,&offset[0]);
}


void modificaSogliaUnderVoltage(u16 value){
	u8 addressFram[2] = {2,250};
	
	/* Accetta sia volt interi (es. 190) sia centivolt interni (es. 19000).
	 * 0 mantiene il significato di soglia disabilitata.
	 */
	if(value > 0 && value < 600){
		value = value * 100;
	}
	underVoltageTH = value;
	saveU16fram(underVoltageTH,&addressFram[0]);
}


void modificaSogliaOverVoltage(u16 value){
	u8 addressFram[2] = {2,252};
	
	/* Accetta sia volt interi (es. 250) sia centivolt interni (es. 25000).
	 * 0 mantiene il significato di soglia disabilitata.
	 */
	if(value > 0 && value < 600){
		value = value * 100;
	}
	overVoltageTH = value;
	saveU16fram(overVoltageTH,&addressFram[0]);
}


void estrazioneNeutro(void){
	int i = 0;
	u16 beforeFram;
	u8 addressFram[2];
	int contatore = 0;
	u8 data[32];
	u32 local;
	
	//conteggio eventi
	while(i<100){
		beforeFram = 2048 + 32*i;
		u162array(&addressFram[0],beforeFram);
		ReadArrayFram(&data[0],&addressFram[0],4);
		local = array2u32(&data[0]);
		if(validNeutroTimestamp(local)){
			contatore++;
		}
		i++;
	}
	i = 0;
	//invio contatore
	u162array(&data[0],contatore);
	if(BTattivo == 1){
		HAL_UART_Transmit(&huart2,&data[0],2,1000);
		delay(10);
		resetWD();
	}
	
	data[30] = 0x0d;	data[31] = 0x0a;
	//invio eventi
	while(i<100){
		beforeFram = 2048 + 32*i;
		u162array(&addressFram[0],beforeFram);
		ReadArrayFram(&data[0],&addressFram[0],30);
		local = array2u32(&data[0]);
		if(validNeutroTimestamp(local) && BTattivo == 1){
			data[30] = 0x0d;	data[31] = 0x0a;
			HAL_UART_Transmit(&huart2,&data[0],32,1000);
			delay(10);
			resetWD();
		}
		i++;
	}

}


//cancella eventi neutro
void formattaNeutro(void){
	int i = 0;
	u8 addressFram[2] = {1,16};
	u8 formattatore[256];
	
	/*
	 * La cancellazione degli eventi neutro/squilibrio tensioni coinvolge FRAM
	 * e due blocchi NFC da 16 byte per evento. Blocchiamo il controllo neutro
	 * e scartiamo eventuali scritture NFC pendenti, per evitare che un evento
	 * vecchio venga riscritto dopo l'erase.
	 */
	inibitN = 255;
	clearNFCpending();
	
	eventoNeutro = 0;
	eventiNeutro = 0;
	saveArrayFram(&eventoNeutro,&addressFram[0],1);
	addressFram[1]++;
	saveArrayFram(&eventiNeutro,&addressFram[0],1);
	
	while(i<256){ //genero array per formattare FRAM
		formattatore[i] = 0;
		i++;
	}
	i=0;
	
	addressFram[0] = 8;	addressFram[1] = 0;
	while(i<7){
		saveArrayFram(&formattatore[0],&addressFram[0],256);
		resetWD();
		i++;
		addressFram[0]++;
	}
	
	/*
	 * NFC: 100 eventi da 32 byte = 3200 byte.
	 * Cancellazione robusta progressiva: 50 chunk da 64 byte,
	 * con verifica di lettura dopo ogni scrittura.
	 */
	formatNeutro = 200;
	inviaDebug((u8*)"erase neutral/voltage imbalance events started\n");

}


void ultimoNeutro(u8 *outBuf){
	u8 addressFram[2];
	u16 beforeAddress;
	u8 data[32];
	u32 timestamp;
	int i;
	int j;
	int found = 0;
	
	for(j=0;j<32;j++){
		outBuf[j] = 0;
	}
	outBuf[30] = 0x0d;	outBuf[31] = 0x0a;
	
	/* Non usare direttamente eventiNeutro-1: dopo erase/memoria sporca
	 * puo' puntare a 0xFFFFFFFF, cioe' 07/02/2106.
	 */
	for(i=0;i<100;i++){
		beforeAddress = 2048 + 32*i;
		u162array(&addressFram[0],beforeAddress);
		ReadArrayFram(&data[0],&addressFram[0],30);
		timestamp = array2u32(&data[0]);
		if(validNeutroTimestamp(timestamp)){
			copiaArray(&outBuf[0],&data[0],30);
			outBuf[30] = 0x0d;	outBuf[31] = 0x0a;
			found = 1;
		}
	}
	
	if(found == 0){
		for(j=0;j<30;j++){
			outBuf[j] = 0;
		}
		outBuf[30] = 0x0d;	outBuf[31] = 0x0a;
	}
		
}


void checkUnderVoltage(void){
	
	u8 sms[200];
	u8 uart[100];
	float Vlocal[3];
	static int delay = 0;
	
	//sprintf(uart,"soglia = %d, under = %d\n", underVoltageTH, underVoltageEvent);
	//HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	if(underVoltageTH == 0){
		return;
	}

	if(delay != 0){
		delay--;
		return;
	}
	
	if((V[0] <= underVoltageTH || V[1] <= underVoltageTH || V[2] <= underVoltageTH) && underVoltageEvent == 0){
		//salva evento
		
		Vlocal[0] = V[0];  Vlocal[0] /= 100;
		Vlocal[1] = V[1];  Vlocal[1] /= 100;
		Vlocal[2] = V[2];  Vlocal[2] /= 100;
		
		sprintf(sms,"Under-voltage alarm!\nUMR-BT: ----------------\nlat: %.3f  long: %.3f\nV1 = %.2fV\nV2 = %.2fV\nV3 = %.2fV",latitudineD,longitudineD,Vlocal[0],Vlocal[1],Vlocal[2]);
		copiaArray(&sms[29],&identificativo[0],16);
		//HAL_UART_Transmit(&huart1,sms,strlen(sms),100);
		inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),sms,strlen(sms));
		aggiungiUnderDB(1,&V[0]);
		underVoltageEvent = 1;
		delay = 60;
	}
	else if((V[0] > underVoltageTH && V[1] > underVoltageTH && V[2] > underVoltageTH) && underVoltageEvent == 1){
		underVoltageEvent = 0;
	}
}

void checkOverVoltage(void){
	
	u8 sms[200];
	u8 uart[100];
	float Vlocal[3];
	static int delay = 0;
	
	//sprintf(uart,"soglia = %d, under = %d\n", underVoltageTH, underVoltageEvent);
	//HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	if(overVoltageTH == 0){
		return;
	}

	if(delay != 0){
		delay--;
		return;
	}
	
	if((V[0] >= overVoltageTH || V[1] >= overVoltageTH || V[2] >= overVoltageTH) && overVoltageEvent == 0){
		//salva evento
		
		Vlocal[0] = V[0];  Vlocal[0] /= 100;
		Vlocal[1] = V[1];  Vlocal[1] /= 100;
		Vlocal[2] = V[2];  Vlocal[2] /= 100;
		
		sprintf(sms,"Over-voltage alarm!\nUMR-BT: ----------------\nlat: %.3f  long: %.3f\nV1 = %.2fV\nV2 = %.2fV\nV3 = %.2fV",latitudineD,longitudineD,Vlocal[0],Vlocal[1],Vlocal[2]);
		copiaArray(&sms[28],&identificativo[0],16);
		//HAL_UART_Transmit(&huart1,sms,strlen(sms),100);
		inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),sms,strlen(sms));
		aggiungiOverDB(1,&V[0]);
		overVoltageEvent = 1;
		delay = 60;
	}
	else if((V[0] < overVoltageTH && V[1] < overVoltageTH && V[2] < overVoltageTH) && overVoltageEvent == 1){
		overVoltageEvent = 0;
	}
}








