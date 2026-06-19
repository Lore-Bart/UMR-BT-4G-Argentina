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

//inibizione dopo formattazione
extern u8 inibizione;

//secondi persi load profile
extern u32 secondiLoad;

//variabili data e ora
extern RTC_DateTypeDef currentDate;
extern RTC_TimeTypeDef currentTime;

//salvataggio attivato
u8 salvataggioLoad = 0;
u8 salvataggioMeas = 0;

//attivazioni salvataggi
uint8_t LoadActive = 0;
uint8_t MeasActive = 0;

//indici memorizzazioni
uint32_t indiceLoad = 0;
uint32_t indiceMeas = 0;

//variabili temporali
extern uint32_t myTimeVar;

//energie
extern uint32_t E1p[3],E2p[3]; //energie attive positive
extern uint32_t E1n[3],E2n[3]; //energie attive negative
extern uint32_t R1p[3],R2p[3]; //energie reattive positive
extern uint32_t R1n[3],R2n[3]; //energie reattive negative

//variabili misurandi
extern u32 V[3]; //tensioni

extern long I1[3],I2[3]; //correnti
long I1meas[3],I2meas[3];

extern uint16_t phi1[3],phi2[3]; //phi
uint16_t phi1meas[3],phi2meas[3];

extern long P1[3],P2[3]; //potenze attive
long P1meas[3],P2meas[3]; 

extern long Q1[3],Q2[3]; //potenze reattive
long Q1meas[3],Q2meas[3];

extern int cosphi1[3],cosphi2[3]; //cosfi
int cosphi1meas[3],cosphi2meas[3];

extern u32 sniff32;

extern u16 riavvioForzato;

//formattazione Flash
extern u8 cancellaLoad;
extern u8 cancellaMeas;

//stringhe per salvataggio
u8 stringaLoad[104];
u8 stringaMeas[112];

//internet
extern u8 statoInternet;

//pagine da cancellare (sostituzione FRAM FLASH)
u8 paginaLoadSost = 255;
u8 paginaMeasSost = 255;

u16 contaLoadFake = 0;
u16 delayFake = 0;

u8 overSavings = 0;

extern u8 debugDB;

extern u8 aggiungiMeasProfileDBflag;
extern u8 aggiungiLoadProfileDBflag;


static u8 timestampProfiloValido(u32 timestamp){
	if(timestamp == 0 || timestamp == 0xffffffff){
		return 0;
	}
	/* Evita record cancellati/corrotti: 0xFFFFFFFF viene visto come 07/02/2106. */
	if(timestamp < 946684800UL || timestamp > 4102444800UL){
		return 0;
	}
	return 1;
}


void preparaLoad(void){
	long difference = 0;

	if(cancellaLoad != 0){
		return;
	}
	
	//genero la stringaLoad
	u322array(&stringaLoad[0],myTimeVar); //data ora
	u322array(&stringaLoad[4],difference); //tempo perso
	u322array(&stringaLoad[8],E1p[0]); //energie attive positive
	u322array(&stringaLoad[12],E1p[1]);
	u322array(&stringaLoad[16],E1p[2]);
	u322array(&stringaLoad[20],E2p[0]);
	u322array(&stringaLoad[24],E2p[1]);
	u322array(&stringaLoad[28],E2p[2]);
	u322array(&stringaLoad[32],E1n[0]); //energie attive negative
	u322array(&stringaLoad[36],E1n[1]);
	u322array(&stringaLoad[40],E1n[2]);
	u322array(&stringaLoad[44],E2n[0]);
	u322array(&stringaLoad[48],E2n[1]);
	u322array(&stringaLoad[52],E2n[2]);
	u322array(&stringaLoad[56],R1p[0]); //energie reattive positive
	u322array(&stringaLoad[60],R1p[1]);
	u322array(&stringaLoad[64],R1p[2]);
	u322array(&stringaLoad[68],R2p[0]);
	u322array(&stringaLoad[72],R2p[1]);
	u322array(&stringaLoad[76],R2p[2]);
	u322array(&stringaLoad[80],R1n[0]); //energie reattive negative
	u322array(&stringaLoad[84],R1n[1]);
	u322array(&stringaLoad[88],R1n[2]);
	u322array(&stringaLoad[92],R2n[0]);
	u322array(&stringaLoad[96],R2n[1]);
	u322array(&stringaLoad[100],R2n[2]);
	
	if(statoInternet == 3){
		//aggiungiLoadProfileDB(1);
		aggiungiLoadProfileDBflag = 1;
	}
	salvataggioLoad = 1;
}


//salvataggio Load Profile
void salvaLoad(void){
	uint16_t pagina = 0; //indice
	uint32_t localIndice = 0;
	uint8_t parity;
	uint8_t addressFram[2] = {0,11};
	u32 time;
	long difference;
	u8 uart[100];
	u8 addressFlash[3] = {0,0,0};
	
	
	time = saveInterval(LoadActive);
	difference = time - secondiLoad;
	if(absLong(difference) < 2){
		difference = 0;
	}
	
	
	//aggiungo +1 all'indice globale e lo salvo in FRAM
	localIndice = indiceLoad;
	if(indiceLoad < 16384){
		indiceLoad++;
	}
	else{
		indiceLoad = 0;
	}
	saveU32fram(indiceLoad,&addressFram[0]);
	
		
	//mi calcolo la pagina sulla flash e la posizione sulla fram
	while(localIndice >= 32){
		localIndice -= 32;
		pagina++;
	}
	
	//controllo la parità
	parity = localIndice/2;
	addressFram[0] = parity + 32; //aggiungo l'offset per le energie
	addressFram[1] = (localIndice - (parity * 2)) * 128;
	
	//salvo la stringa
	saveArrayFram(&stringaLoad[0],&addressFram[0],104);
		
	if(localIndice == 31){
		paginaLoadSost = pagina;
	}
	
}




//calcolo secondi intervallo salvataggio
u32 saveInterval(u8 active){
	
	u32 time = 0;
	
	switch(active){
		case 1: //15 minuti
			time = 900;
			break;
		case 2: //30 minuti
			time = 1800;
			break;
		case 3: //1 ora
			time = 3600;
			break;
		case 4: //2 ore
			time = 7200;
			break;	
		case 5: //3 ore
			time = 10800;
			break;
		case 6: //6 ore
			time = 21600;
			break;
		case 7: //12 ore
			time = 43200;
			break;
		case 8: //24 ore
			time = 86400;
			break;	
	}
	
	return time;
}



//passo dalla fram alla flash i load profile
void fram2flashLoad(uint16_t pagina, u8 riga){

	uint8_t addressfram[2] = {32,0};
	uint8_t addressflash[3] = {0,0,0};
	uint8_t array[256];
	int i = 0;
	uint8_t formattatore[256];
	
	while(i<256){
		formattatore[i] = 0;
		i++;
	}
	i = 0;
	
	pagina *= 16;
	u162array(&addressflash[0],pagina);
	
	addressflash[1] += riga;
	addressfram[0] += riga;
	
	
		ReadArrayFram(&array[0],&addressfram[0],128);
		writeArrayFlash(&array[0],&addressflash[0],128);
		addressfram[1] = 128;
		addressflash[2] = 128;
		ReadArrayFram(&array[0],&addressfram[0],128);
		writeArrayFlash(&array[0],&addressflash[0],128);
		addressfram[1] = 0;
		saveArrayFram(&formattatore[0],&addressfram[0],256);
	

}

void inviaPaginaFlash(void){
	int i = 0;
	uint8_t addressflash[3] = {0,0,0};
	uint8_t uart[256];
	
	__disable_irq();
	while(i<16){
		addressflash[1] = i;
		readArrayFlash(&uart[0],&addressflash[0],4);
		sprintf(uart,"%d %d %d %d\n",uart[0],uart[1],uart[2],uart[3]);
		HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
		addressflash[2] = 128;
		readArrayFlash(&uart[0],&addressflash[0],4);
		sprintf(uart,"%d %d %d %d\n",uart[0],uart[1],uart[2],uart[3]);
		HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
		addressflash[2] = 0;
		i++;
	}
	__enable_irq();
}


void preparaMeasDB(void){

	//preparo le variabili meas
	
	if(I1[0] > 0){
		if(phi1[0] > 90 && phi1[0] <= 270){
			I1meas[0] = -I1[0];
		}
		else{
			I1meas[0] = I1[0];
		}
		phi1meas[0] = phi1[0];
		cosphi1meas[0] = cosphi1[0];
		P1meas[0] = P1[0];
		Q1meas[0] = Q1[0];
	}
	else{
		I1meas[0] = 0;
		phi1meas[0] = 0;
		cosphi1meas[0] = 0;
		P1meas[0] = 0;
		Q1meas[0] = 0;
	}
	
	if(I1[1] > 0){
		if(phi1[1] > 90 && phi1[1] <= 270){
			I1meas[1] = -I1[1];
		}
		else{
			I1meas[1] = I1[1];
		}
		phi1meas[1] = phi1[1];
		cosphi1meas[1] = cosphi1[1];
		P1meas[1] = P1[1];
		Q1meas[1] = Q1[1];
	}
	else{
		I1meas[1] = 0;
		phi1meas[1] = 0;
		cosphi1meas[1] = 0;
		P1meas[1] = 0;
		Q1meas[1] = 0;
	}
	
	if(I1[2] > 0){
		if(phi1[2] > 90 && phi1[2] <= 270){
			I1meas[2] = -I1[2];
		}
		else{
			I1meas[2] = I1[2];
		}
		phi1meas[2] = phi1[2];
		cosphi1meas[2] = cosphi1[2];
		P1meas[2] = P1[2];
		Q1meas[2] = Q1[2];
	}
	else{
		I1meas[2] = 0;
		phi1meas[2] = 0;
		cosphi1meas[2] = 0;
		P1meas[2] = 0;
		Q1meas[2] = 0;
	}
	
	if(I2[0] > 0){
		if(phi2[0] > 90 && phi2[0] <= 270){
			I2meas[0] = -I2[0];
		}
		else{
			I2meas[0] = I2[0];
		}
		phi2meas[0] = phi2[0];
		cosphi2meas[0] = cosphi2[0];
		P2meas[0] = P2[0];
		Q2meas[0] = Q2[0];
	}
	else{
		I2meas[0] = 0;
		phi2meas[0] = 0;
		cosphi2meas[0] = 0;
		P2meas[0] = 0;
		Q2meas[0] = 0;
	}
	
	if(I2[1] > 0){
		if(phi2[1] > 90 && phi2[1] <= 270){
			I2meas[1] = -I2[1];
		}
		else{
			I2meas[1] = I2[1];
		}
		phi2meas[1] = phi2[1];
		cosphi2meas[1] = cosphi2[1];
		P2meas[1] = P2[1];
		Q2meas[1] = Q2[1];
	}
	else{
		I2meas[1] = 0;
		phi2meas[1] = 0;
		cosphi2meas[1] = 0;
		P2meas[1] = 0;
		Q2meas[1] = 0;
	}
	
	if(I2[2] > 0){
		if(phi2[2] > 90 && phi2[2] <= 270){
			I2meas[2] = -I2[2];
		}
		else{
			I2meas[2] = I2[2];
		}
		phi2meas[2] = phi2[2];
		cosphi2meas[2] = cosphi2[2];
		P2meas[2] = P2[2];
		Q2meas[2] = Q2[2];
	}
	else{
		I2meas[2] = 0;
		phi2meas[2] = 0;
		cosphi2meas[2] = 0;
		P2meas[2] = 0;
		Q2meas[2] = 0;
	}	
		
}

void preparaMeas(void){

	if(cancellaMeas != 0){
		return;
	}

	//preparo le variabili meas
	
	if(I1[0] > 0){
		if(phi1[0] > 90 && phi1[0] <= 270){
			I1meas[0] = -I1[0];
		}
		else{
			I1meas[0] = I1[0];
		}
		phi1meas[0] = phi1[0];
		cosphi1meas[0] = cosphi1[0];
		P1meas[0] = P1[0];
		Q1meas[0] = Q1[0];
	}
	else{
		I1meas[0] = 0;
		phi1meas[0] = 0;
		cosphi1meas[0] = 0;
		P1meas[0] = 0;
		Q1meas[0] = 0;
	}
	
	if(I1[1] > 0){
		if(phi1[1] > 90 && phi1[1] <= 270){
			I1meas[1] = -I1[1];
		}
		else{
			I1meas[1] = I1[1];
		}
		phi1meas[1] = phi1[1];
		cosphi1meas[1] = cosphi1[1];
		P1meas[1] = P1[1];
		Q1meas[1] = Q1[1];
	}
	else{
		I1meas[1] = 0;
		phi1meas[1] = 0;
		cosphi1meas[1] = 0;
		P1meas[1] = 0;
		Q1meas[1] = 0;
	}
	
	if(I1[2] > 0){
		if(phi1[2] > 90 && phi1[2] <= 270){
			I1meas[2] = -I1[2];
		}
		else{
			I1meas[2] = I1[2];
		}
		phi1meas[2] = phi1[2];
		cosphi1meas[2] = cosphi1[2];
		P1meas[2] = P1[2];
		Q1meas[2] = Q1[2];
	}
	else{
		I1meas[2] = 0;
		phi1meas[2] = 0;
		cosphi1meas[2] = 0;
		P1meas[2] = 0;
		Q1meas[2] = 0;
	}
	
	if(I2[0] > 0){
		if(phi2[0] > 90 && phi2[0] <= 270){
			I2meas[0] = -I2[0];
		}
		else{
			I2meas[0] = I2[0];
		}
		phi2meas[0] = phi2[0];
		cosphi2meas[0] = cosphi2[0];
		P2meas[0] = P2[0];
		Q2meas[0] = Q2[0];
	}
	else{
		I2meas[0] = 0;
		phi2meas[0] = 0;
		cosphi2meas[0] = 0;
		P2meas[0] = 0;
		Q2meas[0] = 0;
	}
	
	if(I2[1] > 0){
		if(phi2[1] > 90 && phi2[1] <= 270){
			I2meas[1] = -I2[1];
		}
		else{
			I2meas[1] = I2[1];
		}
		phi2meas[1] = phi2[1];
		cosphi2meas[1] = cosphi2[1];
		P2meas[1] = P2[1];
		Q2meas[1] = Q2[1];
	}
	else{
		I2meas[1] = 0;
		phi2meas[1] = 0;
		cosphi2meas[1] = 0;
		P2meas[1] = 0;
		Q2meas[1] = 0;
	}
	
	if(I2[2] > 0){
		if(phi2[2] > 90 && phi2[2] <= 270){
			I2meas[2] = -I2[2];
		}
		else{
			I2meas[2] = I2[2];
		}
		phi2meas[2] = phi2[2];
		cosphi2meas[2] = cosphi2[2];
		P2meas[2] = P2[2];
		Q2meas[2] = Q2[2];
	}
	else{
		I2meas[2] = 0;
		phi2meas[2] = 0;
		cosphi2meas[2] = 0;
		P2meas[2] = 0;
		Q2meas[2] = 0;
	}
	
	//genero la stringaMeas
	u322array(&stringaMeas[0],myTimeVar); //data ora
	u322array(&stringaMeas[4],V[0]); //tensioni
	u322array(&stringaMeas[8],V[1]);
	u322array(&stringaMeas[12],V[2]);
	u322array(&stringaMeas[16],I1meas[0]); //correnti
	u322array(&stringaMeas[20],I1meas[1]);
	u322array(&stringaMeas[24],I1meas[2]);
	u322array(&stringaMeas[28],I2meas[0]);
	u322array(&stringaMeas[32],I2meas[1]);
	u322array(&stringaMeas[36],I2meas[2]);
	u162array(&stringaMeas[40],phi1meas[0]); //phi
	u162array(&stringaMeas[42],phi1meas[1]);
	u162array(&stringaMeas[44],phi1meas[2]);
	u162array(&stringaMeas[46],phi2meas[0]);
	u162array(&stringaMeas[48],phi2meas[1]);
	u162array(&stringaMeas[50],phi2meas[2]);
	u162array(&stringaMeas[52],cosphi1meas[0]); //cosphi
	u162array(&stringaMeas[54],cosphi1meas[1]);
	u162array(&stringaMeas[56],cosphi1meas[2]);
	u162array(&stringaMeas[58],cosphi2meas[0]);
	u162array(&stringaMeas[60],cosphi2meas[1]);
	u162array(&stringaMeas[62],cosphi2meas[2]);
	u322array(&stringaMeas[64],P1meas[0]); //potenze attive
	u322array(&stringaMeas[68],P1meas[1]);
	u322array(&stringaMeas[72],P1meas[2]);
	u322array(&stringaMeas[76],P2meas[0]);
	u322array(&stringaMeas[80],P2meas[1]);
	u322array(&stringaMeas[84],P2meas[2]);	
	u322array(&stringaMeas[88],Q1meas[0]); //potenze reattive
	u322array(&stringaMeas[92],Q1meas[1]);
	u322array(&stringaMeas[96],Q1meas[2]);
	u322array(&stringaMeas[100],Q2meas[0]);
	u322array(&stringaMeas[104],Q2meas[1]);
	u322array(&stringaMeas[108],Q2meas[2]);
	
	if(statoInternet == 3){
		//aggiungiMeasProfileDB(1);
		aggiungiMeasProfileDBflag = 1;
	}
	
	salvataggioMeas = 1;
	
}


//salvataggio measurand profile
void salvaMeas(void){
	uint16_t pagina = 0; //indice
	uint32_t localIndice = 0;
	uint8_t parity;
	uint8_t addressFram[2] = {0,15};
	
	
	//aggiungo +1 all'indice globale e lo salvo in FRAM
	localIndice = indiceMeas;
	if(indiceMeas < 16384){
		indiceMeas++;
	}
	else{
		indiceMeas = 0;
	}
	saveU32fram(indiceMeas,&addressFram[0]);
	

	
	//mi calcolo la pagina sulla flash e la posizione sulla fram
	while(localIndice >= 32){
		localIndice -= 32;
		pagina++;
	}
	
	//controllo la parità
	parity = localIndice/2;
	addressFram[0] = parity + 48; //aggiungo l'offset per i misurandi
	addressFram[1] = (localIndice - (parity * 2)) * 128;
	
	
	//salvo la stringa
	saveArrayFram(&stringaMeas[0],&addressFram[0],112);
	
	
	if(localIndice == 31){
		paginaMeasSost = pagina;
	}
	
}

void fram2flashMeas(uint16_t pagina, u8 riga){

	uint8_t addressfram[2] = {48,0};
	uint8_t addressflash[3] = {0,0,0};
	uint8_t array[256];
	int i = 0;
	uint8_t formattatore[256];

		
	while(i<256){
		formattatore[i] = 0;
		i++;
	}
	i = 0;
	
	pagina *= 16;
	pagina += 8192;
	u162array(&addressflash[0],pagina);

	addressflash[1] += riga;
	addressfram[0] += riga;
	
		ReadArrayFram(&array[0],&addressfram[0],128);
		writeArrayFlash(&array[0],&addressflash[0],128);
		addressfram[1] = 128;
		addressflash[2] = 128;
		ReadArrayFram(&array[0],&addressfram[0],128);
		writeArrayFlash(&array[0],&addressflash[0],128);
		addressfram[1] = 0;
		saveArrayFram(&formattatore[0],&addressfram[0],256);


}



//salvataggio
void salvataggio(void){
	
	u8 salvataggioNormale = 0;

	if(cancellaLoad != 0 || cancellaMeas != 0){
		return;
	}
	
	//LOAD PROFILE	
	switch(LoadActive){
		case 1: //5 minuti
			if(currentTime.Seconds == 0  && (currentTime.Minutes == 0 || currentTime.Minutes == 15 || currentTime.Minutes == 30 || currentTime.Minutes == 45 || currentTime.Minutes == 5 || currentTime.Minutes == 20 || currentTime.Minutes == 35 || currentTime.Minutes == 50 || currentTime.Minutes == 10 || currentTime.Minutes == 25 || currentTime.Minutes == 40 || currentTime.Minutes == 55)){
					preparaLoad();
				}
			break;		
		case 2: //15 minuti
			if(currentTime.Seconds == 0 && (currentTime.Minutes == 0 || currentTime.Minutes == 15 || currentTime.Minutes == 30 || currentTime.Minutes == 45)){
					preparaLoad();
				}
			break;
		case 3: //30 minuti
			if(currentTime.Seconds == 0 && (currentTime.Minutes == 0 || currentTime.Minutes == 30)){
					preparaLoad();
			}
			break;
		case 4: //1 ora
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0){
					preparaLoad();
			}
			break;
		case 5: //2 ore
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0 && dispari(currentTime.Hours) == 0){
					preparaLoad();
			}
			break;
		case 6: //3 ore
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0 && multiplotre(currentTime.Hours) == 0){
					preparaLoad();
			}
			break;
		case 7: //6 ore
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0 && (currentTime.Hours == 0 || currentTime.Hours == 6 || currentTime.Hours == 12 || currentTime.Hours == 18)){
					preparaLoad();
			}
			break;
		case 8: //12 ore
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0 && (currentTime.Hours == 0 || currentTime.Hours == 12)){
					preparaLoad();
			}
			break;
		case 9: //24 ore
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0 && currentTime.Hours == 0){
					preparaLoad();
			}
			break;	
	}

	
	//MEASUREMENT PROFILE
	switch(MeasActive){
		case 1: //5 minuti
			if(currentTime.Seconds == 0  && (currentTime.Minutes == 0 || currentTime.Minutes == 15 || currentTime.Minutes == 30 || currentTime.Minutes == 45 || currentTime.Minutes == 5 || currentTime.Minutes == 20 || currentTime.Minutes == 35 || currentTime.Minutes == 50 || currentTime.Minutes == 10 || currentTime.Minutes == 25 || currentTime.Minutes == 40 || currentTime.Minutes == 55)){
					preparaMeas();
					salvataggioNormale = 1;
				}
		case 2: //15 minuti
			if(currentTime.Seconds == 0  && (currentTime.Minutes == 0 || currentTime.Minutes == 15 || currentTime.Minutes == 30 || currentTime.Minutes == 45)){
					preparaMeas();
					salvataggioNormale = 1;
				}
			break;
		case 3: //30 minuti
			if(currentTime.Seconds == 0 && (currentTime.Minutes == 0 || currentTime.Minutes == 30)){
					preparaMeas();
					salvataggioNormale = 1;	
			}
			break;
		case 4: //1 ora
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0){
					preparaMeas();
					salvataggioNormale = 1;
			}
			break;
		case 5: //2 ore
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0 && dispari(currentTime.Hours) == 0){
					preparaMeas();
					salvataggioNormale = 1;
			}
			break;
		case 6: //3 ore
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0 && multiplotre(currentTime.Hours) == 0){
					preparaMeas();
					salvataggioNormale = 1;
			}
			break;
		case 7: //6 ore
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0 && (currentTime.Hours == 0 || currentTime.Hours == 6 || currentTime.Hours == 12 || currentTime.Hours == 18)){
					preparaMeas();
					salvataggioNormale = 1;
			}
			break;
		case 8: //12 ore
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0 && (currentTime.Hours == 0 || currentTime.Hours == 12)){
					preparaMeas();
					salvataggioNormale = 1;
			}
			break;
		case 9: //24 ore
			if(currentTime.Seconds == 0 && currentTime.Minutes == 0 && currentTime.Hours == 0){
					preparaMeas();
					salvataggioNormale = 1;
			}                            
			break;	
	}
	
	//savings db
	if(overSavings == 1){
		if(currentTime.Seconds == 0){
			if(statoInternet == 3){
				if(salvataggioNormale == 0){
					preparaMeasDB();
				}
				aggiungiMeasProfileDB(1);				
			}
		}
	}
	else if(overSavings == 3 ){
		if(currentTime.Seconds == 0 && (currentTime.Minutes == 0 || multiplotre(currentTime.Minutes) == 1)){
			if(statoInternet == 3){
				if(salvataggioNormale == 0){
					preparaMeasDB();
				}
				aggiungiMeasProfileDB(1);				
			}
		}
	}
	if(overSavings == 5){
		if(currentTime.Seconds == 0 && (currentTime.Minutes == 0 || multiplocinque(currentTime.Minutes) == 1)){
			if(statoInternet == 3){
				if(salvataggioNormale == 0){
					preparaMeasDB();
				}
				aggiungiMeasProfileDB(1);				
			}
		}
	}
	
	if(debugDB == 1){
		if(currentTime.Seconds == 0 && (currentTime.Minutes == 0 || multiplocinque(currentTime.Minutes) == 1)){
			if(statoInternet == 3){
				aggiungiDebugDB(1);
			}	
		}
	}
	
}

void filtroLoadFake(void){
	u8 array[104];
	int i = 0;
	
	
	u162array(&array[0],contaLoadFake);
	HAL_UART_Transmit(&huart2,&array[0],2,1000);
	
	delay(500);
	
	while(i<contaLoadFake){
		HAL_UART_Transmit(&huart2,&array[0],104,100);
		delay(delayFake);
		i++;
	}
}



//estrazione load profile BT
void filtroLoad(u32 estremoA, u32 estremoB){
	u16 contaload = 0;
	u8 addressFlash[3] = {0,0,0};
	u8 addressFram[2] = {0,0};
	u16 i = 0;
	u8 array[104];
	u32 dato;
	
	u8 uart[100];
	long sniff;
	
	
	//scansione flash
	while(i<8192){
		resetWD();
		u162array(&addressFlash[0],i);
		addressFlash[2] = 0;
		//controllo il primo Load Profile della pagina
		readArrayFlash(&array[0],&addressFlash[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			contaload++;
		}
		
		addressFlash[2] = 128;
		//controllo il secondo Load Profile della pagina
		readArrayFlash(&array[0],&addressFlash[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			contaload++;
		}
		i++;
		
		riavvioForzato = timeoutModulo;
	}
	i = 0;
	
	sniff = contaload;
	
	//scansione fram
	while(i<16){
		resetWD();
		addressFram[0] = i + 32;
		addressFram[1] = 0;
		//controllo il primo Load Profile della pagina
		ReadArrayFram(&array[0],&addressFram[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			contaload++;
		}
		
		addressFram[1] = 128;
		//controllo il secondo Load Profile della pagina
		ReadArrayFram(&array[0],&addressFram[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			contaload++;
		}
		i++;
		
		riavvioForzato = timeoutModulo;
	}
	i = 0;
	
	u162array(&array[0],contaload);
	HAL_UART_Transmit(&huart2,&array[0],2,1000);
	
	delay(500);

	//sprintf(uart,"load filtrati:%d\n",contaload);
	//HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	//contaload = 0;
	//invio Load Profiles da fram
	while(i<16){
		resetWD();
		addressFram[0] = i + 32;
		addressFram[1] = 0;
		//controllo il primo Load Profile della pagina
		ReadArrayFram(&array[0],&addressFram[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			addressFram[1] += 4;
			ReadArrayFram(&array[4],&addressFram[0],100);
			HAL_UART_Transmit(&huart2,&array[0],104,1000);
			contaload++;
			delay(50);
		}
		
		addressFram[1] = 128;
		//controllo il secondo Load Profile della pagina
		ReadArrayFram(&array[0],&addressFram[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			addressFram[1] += 4;
			ReadArrayFram(&array[4],&addressFram[0],100);
			HAL_UART_Transmit(&huart2,&array[0],104,1000);	
			contaload++;	
			delay(50);			
		}
		i++;
		
		riavvioForzato = timeoutModulo;
	}
	i = 0;
	
	
	//invio Load Profiles da flash
	while(i<8192){
		resetWD();
		u162array(&addressFlash[0],i);
		addressFlash[2] = 0;
		//controllo il primo Load Profile della pagina
		readArrayFlash(&array[0],&addressFlash[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			addressFlash[2] += 4;
			readArrayFlash(&array[4],&addressFlash[0],100);
			HAL_UART_Transmit(&huart2,&array[0],104,1000);
			delay(50);
		}
		
		addressFlash[2] = 128;
		//controllo il secondo Load Profile della pagina
		readArrayFlash(&array[0],&addressFlash[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			addressFlash[2] += 4;
			readArrayFlash(&array[4],&addressFlash[0],100);
			HAL_UART_Transmit(&huart2,&array[0],104,1000);
			delay(50);
		}
		
		//sprintf(uart,"%d\n",i);
		//HAL_UART_Transmit(&huart1,&uart[0],strlen(uart),1000);
		
		i++;
		
		riavvioForzato = timeoutModulo;
	}
	//sprintf(uart,"load inviati:%d\n",contaload);
	//HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	i = 0;
	
		
}


//estrazione measurand profile BT
void filtroMeas(u32 estremoA, u32 estremoB){
	u16 contameas = 0;
	u8 addressFlash[3] = {0,0,0};
	u8 addressFram[2] = {0,0};
	u16 i = 0,pagina;
	u8 array[200];
	u32 dato;
	u32 sniff;
	u8 uart[100];
	

	//scansione flash
	while(i<8192){
		resetWD();
		pagina = i + 8192;
		u162array(&addressFlash[0],pagina);
		addressFlash[2] = 0;
		//controllo il primo Meas Profile della pagina
		readArrayFlash(&array[0],&addressFlash[0],4);
		dato = array2u32(&array[0]);
		
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			contameas++;
		}
		
		addressFlash[2] = 128;
		//controllo il secondo Meas Profile della pagina
		readArrayFlash(&array[0],&addressFlash[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			contameas++;
		}
		i++;
		
		riavvioForzato = timeoutModulo;
	}
	i = 0;
	
	sniff = contameas;
	
	
	//scansione fram
	while(i<16){
		resetWD();
		addressFram[0] = i + 48;
		addressFram[1] = 0;
		//controllo il primo Meas Profile della pagina
		ReadArrayFram(&array[0],&addressFram[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			contameas++;
		}
		
		addressFram[1] = 128;
		//controllo il secondo Meas Profile della pagina
		ReadArrayFram(&array[0],&addressFram[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			contameas++;
		}
		i++;
		
		riavvioForzato = timeoutModulo;
	}
	i = 0;
	
	
	u162array(&array[0],contameas);
	HAL_UART_Transmit(&huart2,&array[0],2,1000);

	delay(500);
	
	//invio Load Profiles da fram
	while(i<16){
		resetWD();
		addressFram[0] = i+48;
		addressFram[1] = 0;
		//controllo il primo Load Profile della pagina
		ReadArrayFram(&array[0],&addressFram[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			addressFram[1] += 4;
			ReadArrayFram(&array[4],&addressFram[0],108);
			HAL_UART_Transmit(&huart2,&array[0],112,1000);
			delay(50);
		}
		
		addressFram[1] = 128;
		//controllo il secondo Load Profile della pagina
		ReadArrayFram(&array[0],&addressFram[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			addressFram[1] += 4;
			ReadArrayFram(&array[4],&addressFram[0],108);
			HAL_UART_Transmit(&huart2,&array[0],112,1000);		
			delay(50);
		}
		i++;
		
		riavvioForzato = timeoutModulo;
	}
	i = 0;
	
		//invio Load Profiles da flash
	while(i<8192){
		resetWD();
		pagina = i + 8192;
		u162array(&addressFlash[0],pagina);
		addressFlash[2] = 0;
		//controllo il primo Load Profile della pagina
		readArrayFlash(&array[0],&addressFlash[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			addressFlash[2] += 4;
			readArrayFlash(&array[4],&addressFlash[0],108);
			HAL_UART_Transmit(&huart2,&array[0],112,1000);
			delay(50);
		}
		
		addressFlash[2] = 128;
		//controllo il secondo Load Profile della pagina
		readArrayFlash(&array[0],&addressFlash[0],4);
		dato = array2u32(&array[0]);
		
		if(timestampProfiloValido(dato) && dato >= estremoA && dato <= estremoB){
			addressFlash[2] += 4;
			readArrayFlash(&array[4],&addressFlash[0],108);
			HAL_UART_Transmit(&huart2,&array[0],112,1000);
			delay(50);
		}
		i++;
		
		//sprintf(uart,"%d\n",i);
		//HAL_UART_Transmit(&huart1,&uart[0],strlen(uart),1000);
		
		riavvioForzato = timeoutModulo;
	}
	i = 0;
		
}


//estrazione somma energie
void sommaEnergie(u8 *outBuf, u32 min, u32 max){
	u8 addressFlash[4];
	u8 addressFlash3[3] = {0,0,0};
	u8 addressFram[2] = {0,0};
	int i = 0;
	u32 beforeAddressFlash;
	u16 beforeAddressFram;
	u32 minLocal = 0;
	u32 maxLocal = 0;
	u8 minArray[200];
	u8 maxArray[200];
	u8 data[4];
	u32 local;
	u32 dato1,dato3,dato4;
	u16 c = 0;
		
	
	//azzero gli array
	for(i=0;i<100;i++){
		minArray[i] = 0;
		maxArray[i] = 0; 
	}
	

	
	//FLASH
	for(i=0;i<16384;i++){ //16384
	
		beforeAddressFlash = i*128;
		u322array(&addressFlash[0],beforeAddressFlash);
		
		//leggo l'orario
		readArrayFlash(&data[0],&addressFlash[1],4);
		local = array2u32(&data[0]);
		
	
		//prendo il minimo e il massimo salvati in flash
		if(timestampProfiloValido(local) && local >= min && local <= max){
			if(minLocal == 0){ //primo valore che viene trovato nel range
				minLocal = local;
				maxLocal = local;
				addressFlash[3] += 4;
				readArrayFlash(&minArray[4],&addressFlash[1],100);
				copiaArray(&maxArray[4],&minArray[4],100);
			}
			else if(minLocal != 0 && local < minLocal){
				minLocal = local;
				addressFlash[3] += 4;
				readArrayFlash(&minArray[4],&addressFlash[1],100);
			}
			else if(minLocal != 0 && local > maxLocal){
				maxLocal = local;
				addressFlash[3] += 4;
				readArrayFlash(&maxArray[4],&addressFlash[1],100);
			}
				
		}
			resetWD();
	}
		
	
		
		
	
	
	//FRAM
	for(i=0;i<32;i++){
		beforeAddressFram = 8192 + 128*i; //8192 -> 0x2000
		u162array(&addressFram[0],beforeAddressFram);
		
		ReadArrayFram(&data[0],&addressFram[0],4);
		local = array2u32(&data[0]);
		
		//prendo il minimo e il massimo salvati in fram
		if(timestampProfiloValido(local) && local >= min && local <= max){
			if(minLocal == 0){ //primo valore che viene trovato nel range
				minLocal = local;
				maxLocal = local;
				addressFram[1] += 4;
				ReadArrayFram(&minArray[4],&addressFram[0],100);
				copiaArray(&maxArray[4],&minArray[4],100);
			}
			else if(minLocal != 0 && local < minLocal){
				minLocal = local;
				addressFram[1] += 4;
				ReadArrayFram(&minArray[4],&addressFram[0],100);
			}
			else if(minLocal != 0 && local > maxLocal){
				maxLocal = local;
				addressFram[1] += 4;
				ReadArrayFram(&maxArray[4],&addressFram[0],100);
			}					
		}
	resetWD();
	}
	
	u322array(&minArray[0],minLocal);
	u322array(&maxArray[0],maxLocal);

	
	//ora che ho il massimo e il minimo posso calcolare la somma
	for(i=0;i<24;i++){
	
		dato1 = array2u32(&minArray[8+4*i]);
		dato3 = array2u32(&maxArray[8+4*i]);
	
		//calcolo la differenza
		if(dato3 >= dato1){
			dato4 = dato3-dato1;
		}
		else{
			dato4 = dato3 + (1000000000-dato1);
		}
		
		//metto la differenza nel buffer
		u322array(&outBuf[8+4*i],dato4);
	}
	
	minLocal = 0; maxLocal = 0;
	////metto gli orari nel buffer
	u322array(&outBuf[0],minLocal);
	u322array(&outBuf[4],maxLocal);
	
	//HAL_UART_Transmit(&huart2,&outBuf[0],104,1000);
}


//calcolo media
void mediaMisurandi(u8 *outBuf,u32 min,u32 max){
	long stampLong;
	int stampInt;
	u8 addressFlash[4];
	u8 addressFram[2] = {0,0};
	int i = 0;
	u32 beforeAddressFlash;
	u16 beforeAddressFram;
	int contatore = 0;
	u8 array[200];
	u8 data[4];
	u32 local,maxLocal;
	u8 uart[100];
	
	//da cancellare
		u16 pagina;
	
	
	double Vl[3] = {0,0,0},I1l[3] = {0,0,0},I2l[3] = {0,0,0},P1l[3] = {0,0,0},P2l[3] = {0,0,0},R1l[3] = {0,0,0},R2l[3] = {0,0,0},phi1l[3] = {0,0,0},phi2l[3] = {0,0,0},cosphi1l[3] = {0,0,0},cosphi2l[3] = {0,0,0};
	double stampella;

		
	//scansione flash
	while(i<16384){//8192
		pagina = i/2 + 8192;
		u162array(&addressFlash[0],pagina);
		addressFlash[2] = dispari(i)*128;
		//controllo il primo Meas Profile della pagina
		readArrayFlash(&data[0],&addressFlash[0],4);	
		local = array2u32(&data[0]);

		if(timestampProfiloValido(local) && local >= min && local <= max){
			contatore++;
			maxLocal = local;
			addressFlash[2] += 4;
			readArrayFlash(&array[0],&addressFlash[0],108);
			
						stampLong = array2long(&array[0]); 
			Vl[0] += stampLong;//tensioni
			stampLong = array2long(&array[4]);
			Vl[1] += stampLong;
			stampLong = array2long(&array[8]); 
			Vl[2] += stampLong;
			
			stampella = array2long(&array[12]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I1l[0] += stampella;
			stampella = array2long(&array[16]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I1l[1] += stampella;
			stampella = array2long(&array[20]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I1l[2] += stampella;
			stampella = array2long(&array[24]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I2l[0] += stampella;
			stampella = array2long(&array[28]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I2l[1] += stampella;
			stampella = array2long(&array[32]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I2l[2] += stampella;
			
			stampInt = array2int(&array[36]); 
			phi1l[0] += stampInt;//phi
			stampInt = array2int(&array[38]); 
			phi1l[1] += stampInt;
			stampInt = array2int(&array[40]); 
			phi1l[2] += stampInt;
			stampInt = array2int(&array[42]); 
			phi2l[0] += stampInt;
			stampInt = array2int(&array[44]); 
			phi2l[1] += stampInt;
			stampInt = array2int(&array[46]); 
			phi2l[2] += stampInt;
			
			stampella = array2int(&array[48]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi1l[0] += stampella;
			stampella = array2int(&array[50]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi1l[1] += stampella;
			stampella = array2int(&array[52]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi1l[2] += stampella;
			stampella = array2int(&array[54]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi2l[0] += stampella;
			stampella = array2int(&array[56]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi2l[1] += stampella;
			stampella = array2int(&array[58]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi2l[2] += stampella;
			
			stampella = array2long(&array[60]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P1l[0] += stampella;//potenze attive
			stampella = array2long(&array[64]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P1l[1] += stampella;
			stampella = array2long(&array[68]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P1l[2] += stampella;
			stampella = array2long(&array[72]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P2l[0] += stampella;//potenze attive
			stampella = array2long(&array[76]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P2l[1] += stampella;
			stampella = array2long(&array[80]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P2l[2] += stampella;
			
			stampella = array2long(&array[84]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R1l[0] += stampella;//potenze attive
			stampella = array2long(&array[88]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R1l[1] += stampella;
			stampella = array2long(&array[92]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R1l[2] += stampella;
			stampella = array2long(&array[96]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R2l[0] += stampella;//potenze reattive
			stampella = array2long(&array[100]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R2l[1] += stampella;
			stampella = array2long(&array[104]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R2l[2] += stampella;
			
			
		}
		
		i++;
		resetWD();
	}
	i = 0;
	
	
	//scansione FRAM
	while(i<32){
		addressFram[0] = i/2 + 48;
		addressFram[1] = dispari(i)*128;
		//controllo il primo Meas Profile della pagina
		ReadArrayFram(&data[0],&addressFram[0],4);
		local = array2u32(&data[0]);
		
		if(timestampProfiloValido(local) && local >= min && local <= max){
			contatore++;
			maxLocal = local;
			addressFram[1] += 4;
			ReadArrayFram(&array[0],&addressFram[0],108);
			
						stampLong = array2long(&array[0]); 
			Vl[0] += stampLong;//tensioni
			stampLong = array2long(&array[4]);
			Vl[1] += stampLong;
			stampLong = array2long(&array[8]); 
			Vl[2] += stampLong;
			
			stampella = array2long(&array[12]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I1l[0] += stampella;
			stampella = array2long(&array[16]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I1l[1] += stampella;
			stampella = array2long(&array[20]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I1l[2] += stampella;
			stampella = array2long(&array[24]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I2l[0] += stampella;
			stampella = array2long(&array[28]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I2l[1] += stampella;
			stampella = array2long(&array[32]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			I2l[2] += stampella;
			
			stampInt = array2int(&array[36]); 
			phi1l[0] += stampInt;//phi
			stampInt = array2int(&array[38]); 
			phi1l[1] += stampInt;
			stampInt = array2int(&array[40]); 
			phi1l[2] += stampInt;
			stampInt = array2int(&array[42]); 
			phi2l[0] += stampInt;
			stampInt = array2int(&array[44]); 
			phi2l[1] += stampInt;
			stampInt = array2int(&array[46]); 
			phi2l[2] += stampInt;
			
			stampella = array2int(&array[48]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi1l[0] += stampella;
			stampella = array2int(&array[50]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi1l[1] += stampella;
			stampella = array2int(&array[52]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi1l[2] += stampella;
			stampella = array2int(&array[54]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi2l[0] += stampella;
			stampella = array2int(&array[56]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi2l[1] += stampella;
			stampella = array2int(&array[58]); if(stampella > 32000){stampella = stampella - 65536;}
			cosphi2l[2] += stampella;
			
			stampella = array2long(&array[60]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P1l[0] += stampella;//potenze attive
			stampella = array2long(&array[64]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P1l[1] += stampella;
			stampella = array2long(&array[68]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P1l[2] += stampella;
			stampella = array2long(&array[72]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P2l[0] += stampella;//potenze attive
			stampella = array2long(&array[76]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P2l[1] += stampella;
			stampella = array2long(&array[80]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			P2l[2] += stampella;
			
			stampella = array2long(&array[84]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R1l[0] += stampella;//potenze attive
			stampella = array2long(&array[88]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R1l[1] += stampella;
			stampella = array2long(&array[92]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R1l[2] += stampella;
			stampella = array2long(&array[96]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R2l[0] += stampella;//potenze reattive
			stampella = array2long(&array[100]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R2l[1] += stampella;
			stampella = array2long(&array[104]); if(stampella > 2000000000){stampella = stampella - 4294967296;}
			R2l[2] += stampella;
			

		}
			resetWD();	
		i++;
	}
	i = 0;
	
	sprintf(uart,"contatore: %d\n", contatore);
	HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	//calcolo la media
	
	if(contatore != 0){
	
		for(i=0;i<3;i++){
			Vl[i] /= contatore;
			I1l[i] /= contatore;
			I2l[i] /= contatore;
			phi1l[i] /= contatore;
			phi2l[i] /= contatore;
			cosphi1l[i] /= contatore;
			cosphi2l[i] /= contatore;
			P1l[i] /= contatore;
			P2l[i] /= contatore;
			R1l[i] /= contatore;
			R2l[i] /= contatore;
		}
		
		//Vl[0] = Vl[0] + 1;
		//Vl[0] = Vl[0] / 1;
		
		
		//compilo la stringa 108 BYTE
		stampLong = Vl[0]; u322array(&outBuf[4],stampLong);
		stampLong = Vl[1]; u322array(&outBuf[8],stampLong);
		stampLong = Vl[2]; u322array(&outBuf[12],stampLong);
		
		stampLong = I1l[0]; u322array(&outBuf[16],stampLong);
		stampLong = I1l[1]; u322array(&outBuf[20],stampLong);
		stampLong = I1l[2]; u322array(&outBuf[24],stampLong);
		
		stampLong = I2l[0]; u322array(&outBuf[28],stampLong);
		stampLong = I2l[1]; u322array(&outBuf[32],stampLong);
		stampLong = I2l[2]; u322array(&outBuf[36],stampLong);

		stampInt = phi1l[0]; u162array(&outBuf[40],stampInt);
		stampInt = phi1l[1]; u162array(&outBuf[42],stampInt);
		stampInt = phi1l[2]; u162array(&outBuf[44],stampInt);
		
		stampInt = phi2l[0]; u162array(&outBuf[46],stampInt);
		stampInt = phi2l[1]; u162array(&outBuf[48],stampInt);
		stampInt = phi2l[2]; u162array(&outBuf[50],stampInt);
		
		stampInt = cosphi1l[0]; u162array(&outBuf[52],stampInt);
		stampInt = cosphi1l[1]; u162array(&outBuf[54],stampInt);
		stampInt = cosphi1l[2]; u162array(&outBuf[56],stampInt);
		
		stampInt = cosphi2l[0]; u162array(&outBuf[58],stampInt);
		stampInt = cosphi2l[1]; u162array(&outBuf[60],stampInt);
		stampInt = cosphi2l[2]; u162array(&outBuf[62],stampInt);
		
		stampLong = P1l[0]; u322array(&outBuf[64],stampLong);
		stampLong = P1l[1]; u322array(&outBuf[68],stampLong);
		stampLong = P1l[2]; u322array(&outBuf[72],stampLong);
		
		stampLong = P2l[0]; u322array(&outBuf[76],stampLong);
		stampLong = P2l[1]; u322array(&outBuf[80],stampLong);
		stampLong = P2l[2]; u322array(&outBuf[84],stampLong);
		
		stampLong = R1l[0]; u322array(&outBuf[88],stampLong);
		stampLong = R1l[1]; u322array(&outBuf[92],stampLong);
		stampLong = R1l[2]; u322array(&outBuf[96],stampLong);
		
		stampLong = R2l[0]; u322array(&outBuf[100],stampLong);
		stampLong = R2l[1]; u322array(&outBuf[104],stampLong);
		stampLong = R2l[2]; u322array(&outBuf[108],stampLong);
		
		u322array(&outBuf[0],maxLocal);
	}
	
	
	//HAL_UART_Transmit(&huart2,&outBuf[0],112,1000);
}




u16 contaLoadProfilesFramResidui(void){
	u16 i = 0;
	u16 residui = 0;
	u8 addressFram[2] = {0,0};
	u8 data[4];
	u32 timestamp;

	while(i < 16){
		resetWD();
		addressFram[0] = i + 32;
		addressFram[1] = 0;
		ReadArrayFram(&data[0],&addressFram[0],4);
		timestamp = array2u32(&data[0]);
		if(timestampProfiloValido(timestamp)){ residui++; }

		addressFram[1] = 128;
		ReadArrayFram(&data[0],&addressFram[0],4);
		timestamp = array2u32(&data[0]);
		if(timestampProfiloValido(timestamp)){ residui++; }
		i++;
	}

	return residui;
}

u16 contaMeasProfilesFramResidui(void){
	u16 i = 0;
	u16 residui = 0;
	u8 addressFram[2] = {0,0};
	u8 data[4];
	u32 timestamp;

	while(i < 16){
		resetWD();
		addressFram[0] = i + 48;
		addressFram[1] = 0;
		ReadArrayFram(&data[0],&addressFram[0],4);
		timestamp = array2u32(&data[0]);
		if(timestampProfiloValido(timestamp)){ residui++; }

		addressFram[1] = 128;
		ReadArrayFram(&data[0],&addressFram[0],4);
		timestamp = array2u32(&data[0]);
		if(timestampProfiloValido(timestamp)){ residui++; }
		i++;
	}

	return residui;
}

u16 contaLoadProfilesResidui(void){
	u16 i = 0;
	u16 residui = 0;
	u8 addressFlash[3] = {0,0,0};
	u8 data[4];
	u32 timestamp;

	while(i < 8192){
		resetWD();
		u162array(&addressFlash[0],i);
		addressFlash[2] = 0;
		readArrayFlash(&data[0],&addressFlash[0],4);
		timestamp = array2u32(&data[0]);
		if(timestampProfiloValido(timestamp)){ residui++; }

		addressFlash[2] = 128;
		readArrayFlash(&data[0],&addressFlash[0],4);
		timestamp = array2u32(&data[0]);
		if(timestampProfiloValido(timestamp)){ residui++; }
		i++;
	}

	residui += contaLoadProfilesFramResidui();
	return residui;
}

u16 contaMeasProfilesResidui(void){
	u16 i = 0;
	u16 pagina;
	u16 residui = 0;
	u8 addressFlash[3] = {0,0,0};
	u8 data[4];
	u32 timestamp;

	while(i < 8192){
		resetWD();
		pagina = i + 8192;
		u162array(&addressFlash[0],pagina);
		addressFlash[2] = 0;
		readArrayFlash(&data[0],&addressFlash[0],4);
		timestamp = array2u32(&data[0]);
		if(timestampProfiloValido(timestamp)){ residui++; }

		addressFlash[2] = 128;
		readArrayFlash(&data[0],&addressFlash[0],4);
		timestamp = array2u32(&data[0]);
		if(timestampProfiloValido(timestamp)){ residui++; }
		i++;
	}

	residui += contaMeasProfilesFramResidui();
	return residui;
}

//formatta load profile
void formattaLoad(void){
	u8 addressFram[2];
	int i = 0;
	u8 formattatore[256];
	u8 messaggio[100] = "erase load profiles requested\n";

	
	
	HAL_UART_Transmit(&huart1,&messaggio[0],strlen(messaggio),1000);
	
	inibizione = 100;
	while(i<256){ //genero array per formattare FRAM
		formattatore[i] = 0;
		i++;
	}
	i=0;
	
	addressFram[1] = 0;
	while(i<16){ //formatto FRAM
		addressFram[0] = 32+i;
		saveArrayFram(&formattatore[0],&addressFram[0],256);
		i++;
	}
	i=0;
	
	addressFram[0] = 0;
	addressFram[1] = 11;
	
	indiceLoad = 0;
	
	saveU32fram(indiceLoad,&addressFram[0]); //salvo indiceload
	
	paginaLoadSost = 255;
	salvataggioLoad = 0;
	aggiungiLoadProfileDBflag = 0;
	cancellaLoad = 32;
	sprintf((char*)messaggio,"erase load profiles FRAM residual: %u\n",contaLoadProfilesFramResidui());
	inviaDebug(&messaggio[0]);
	inviaDebug((u8*)"erase load profiles FLASH queued: 32 sectors\n");
	
	inibizione = 0;
	
}

//formatta load profile
void formattaMeas(void){
	u8 addressFram[2];
	int i = 0;
	u8 formattatore[256];
	u8 messaggio[100] = "erase measurand profiles requested\n";

	
	
	HAL_UART_Transmit(&huart1,&messaggio[0],strlen(messaggio),1000);
	
	inibizione = 100;
	while(i<256){ //genero array per formattare FRAM
		formattatore[i] = 0;
		i++;
	}
	i=0;
	
	addressFram[1] = 0;
	while(i<16){ //formatto FRAM
		addressFram[0] = 48+i;
		saveArrayFram(&formattatore[0],&addressFram[0],256);
		i++;
	}
	i=0;
	
	addressFram[0] = 0;
	addressFram[1] = 15;
	
	indiceMeas = 0;
	
	saveU32fram(indiceMeas,&addressFram[0]); //salvo indiceMeas
	
	paginaMeasSost = 255;
	salvataggioMeas = 0;
	aggiungiMeasProfileDBflag = 0;
	cancellaMeas = 32;
	sprintf((char*)messaggio,"erase meas profiles FRAM residual: %u\n",contaMeasProfilesFramResidui());
	inviaDebug(&messaggio[0]);
	inviaDebug((u8*)"erase measurand profiles FLASH queued: 32 sectors\n");
	
	inibizione = 0;
	
}

