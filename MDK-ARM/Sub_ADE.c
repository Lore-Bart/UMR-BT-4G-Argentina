#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "string.h"
#include "math.h"

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

//segnale GSM
extern u8 segnaleGSM;

//variabili data e ora
extern RTC_DateTypeDef currentDate;
extern RTC_TimeTypeDef currentTime;

//versione software
extern u16 software;

//variabili energie NON CORRETTI
long E1nc[3],E2nc[3];
long R1nc[3],R2nc[3];

//azzeramento registri
u8 azzeramento = 0;
u8 azzeramento3 = 0;

extern u32 sniffNuovo;

//localizzazione
extern long latitudine;
extern long longitudine;

//stato batteria
extern u8 batteryLevel;

//variabili V,I,P,Q NON CORRETTI
u32 Vnc[3];
u32 I1nc[3],I2nc[3];
u32 P1nc[3],P2nc[3];

//comando calibrazione
u8 calib = 0;
u8 calib3 = 0;

u32 corrEU,corrED,corrRU,corrRD;

//identificativo
extern u8 identificativo[16];

//data e ora
extern uint32_t myTimeVar;

//coefficienti correttivi
long corrV[3]; //calibrazione tensione
long corrI1[3],corrI2[3]; //calibrazione corrente
long corrE1[3],corrE2[3]; //calibrazione energia attiva
long corrR1[3],corrR2[3]; //calibrazione energia reattiva
long offI1[3],offI2[3]; //offset corrente
long offE1[3],offE2[3]; //offset energia attiva

//variabili misurandi
u32 V[3]; //tensioni
long I1[3],I2[3]; //correnti
uint16_t phi1[3],phi2[3]; //phi
long P1[3],P2[3]; //potenze attive
long Q1[3],Q2[3]; //potenze reattive
int cosphi1[3],cosphi2[3]; //cosfi

//energie
long E1[3],E2[3],R1[3],R2[3];

uint32_t E1p[3],E2p[3]; //energie attive positive
uint32_t E1n[3],E2n[3]; //energie attive negative
uint32_t R1p[3],R2p[3]; //energie reattive positive
uint32_t R1n[3],R2n[3]; //energie reattive negative

uint32_t E1pi[3],E2pi[3]; //energie attive positive BACKUP
uint32_t E1ni[3],E2ni[3]; //energie attive negative BACKUP
uint32_t R1pi[3],R2pi[3]; //energie reattive positive BACKUP
uint32_t R1ni[3],R2ni[3]; //energie reattive negative BACKUP

u32 E1pTOT,E2pTOT,E1nTOT,E2nTOT,R1pTOT,R2pTOT,R1nTOT,R2nTOT; //TOTALIZZATORI

u32 sogliaI = 50;

//azzeramento contatori
u8 azzeraLineaA = 0;
u8 azzeraLineaB = 0;

u32 Ipot = 0;

u32 lettureV[3];

u8 triangolo = 0;

double VD[3];
extern u8 XL;

//inizializzazione ADE 1
void adeinit(void){

//manca il reset
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_4);
	HAL_Delay(10);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_4);
	
HAL_Delay(700);
	
	unsigned char corr1[4] = {0xe6,0x14,0x00,0xa0}; //12a scheda 1
	unsigned char corr2[4] = {0xe6,0x15,0x00,0xa0};//125 scheda 1
	unsigned char corr3[4] = {0xe6,0x16,0x00,0xa0};//12a scheda 1
	unsigned char dicoeff[6] = {0x43,0x88,0x0f,0xff,0x80,0x00};
	unsigned char config[4] = {0xe6,0x18,0x00,0x03}; //integratore
	unsigned char vlevel[6] = {0x43,0x9f,0x00,0x6a,0x25,0xe9};
	unsigned char status0[6] = {0xe5,0x02,0xff,0xff,0xff,0xff};
	unsigned char status1[6] = {0xe5,0x03,0x3f,0xff,0xff,0xff};
	unsigned char mask1[6] = {0xe5,0x0b,0x00,0x00,0x00,0x00}; //zero crossing
	unsigned char cf1den[4] = {0xe6,0x11,0x0d,0xb3};
	unsigned char cf2den[4] = {0xe6,0x12,0x0d,0xb3};
	unsigned char cf3den[4] = {0xe6,0x13,0x0d,0xb3};
	unsigned char gain[4] = {0xe6,0x0f,0x00,0x44};
	unsigned char lcycmode[3] = {0xe7,0x02,0x38};
	unsigned char run[4] = {0xe2,0x28,0x00,0x01};
	
	if(XL != 0){
		gain[3] = 0x43;
	}

HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, corr1, 4, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, corr2, 4, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, corr3, 4, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, dicoeff, 6, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, config, 4, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, vlevel, 6, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, status0, 6, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, status1, 6, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, mask1, 6, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, cf1den, 4, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, cf2den, 4, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, cf3den, 4, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, gain, 4, 100);
HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, lcycmode, 3, 100);

HAL_Delay(500);

HAL_I2C_Master_Transmit(&hi2c2, 0x38<<1, run, 4, 100);

HAL_Delay(500);

}

//inizializzazione ADE 2
void adeinit3(void){
	
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
	HAL_Delay(10);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
	//manca il reset
	
HAL_Delay(700);
	
	unsigned char corr1[4] = {0xe6,0x14,0x00,0x9f}; //12a scheda 1
	unsigned char corr2[4] = {0xe6,0x15,0x00,0x9c};//125 scheda 1
	unsigned char corr3[4] = {0xe6,0x16,0x00,0xa0};//12a scheda 1
	unsigned char dicoeff[6] = {0x43,0x88,0x0f,0xff,0x80,0x00};
	unsigned char config[4] = {0xe6,0x18,0x00,0x03}; //integratore
	unsigned char vlevel[6] = {0x43,0x9f,0x00,0x6a,0x25,0xe9};
	unsigned char status0[6] = {0xe5,0x02,0xff,0xff,0xff,0xff};
	unsigned char status1[6] = {0xe5,0x03,0x3f,0xff,0xff,0xff};
	unsigned char mask1[6] = {0xe5,0x0b,0x00,0x00,0x00,0x00}; //zero crossing
	unsigned char cf1den[4] = {0xe6,0x11,0x0d,0xb3};
	unsigned char cf2den[4] = {0xe6,0x12,0x0d,0xb3};
	unsigned char cf3den[4] = {0xe6,0x13,0x0d,0xb3};
	unsigned char gain[4] = {0xe6,0x0f,0x00,0x44};
	unsigned char lcycmode[3] = {0xe7,0x02,0x38};
	unsigned char run[4] = {0xe2,0x28,0x00,0x01};
	
	if(XL != 0){
		gain[3] = 0x43;
	}
	
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, corr1, 4, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, corr2, 4, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, corr3, 4, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, dicoeff, 6, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, config, 4, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, vlevel, 6, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, status0, 6, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, status1, 6, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, mask1, 6, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, cf1den, 4, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, cf2den, 4, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, cf3den, 4, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, gain, 4, 100);
HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, lcycmode, 3, 100);

HAL_Delay(500);

HAL_I2C_Master_Transmit(&hi2c3, 0x38<<1, run, 4, 100);

HAL_Delay(500);

}







//acquisizioni 1
void acquisizioni(void){
	u8 registro[2];
	static long E1 = 0, E2 = 0, E3 = 0;
	static long E1old = 0, E2old = 0, E3old = 0;
	static long R1 = 0, R2 = 0, R3 = 0;
	static long R1old = 0, R2old = 0, R3old = 0;
	u8 uart[100];
	
	u8 reset[3] = {0xe7,0x02,0x78};
	u8 set[3] = {0xe7,0x02,0x38};
	
	
		  
	//lettura energie
	if(azzeramento == 0){
		E1old = E1; E2old = E2; E3old = E3;
		R1old = R1; R2old = R2; R3old = R3;
	}
	else{
		azzeramento = 0;
		E1old = 0; E2old = 0; E3old = 0;
		R1old = 0; R2old = 0; R3old = 0;
		
		HAL_I2C_Master_Transmit(&hi2c2,0x70,reset,3,1000);
		registro[0] = 0xe4;
		registro[1] = 0x03;	readRegADE1(&registro[0]);
		registro[1] = 0x04;	readRegADE1(&registro[0]);
		registro[1] = 0x05;	readRegADE1(&registro[0]);
		registro[1] = 0x09;	readRegADE1(&registro[0]);
		registro[1] = 0x0a;	readRegADE1(&registro[0]);
		registro[1] = 0x0b;	readRegADE1(&registro[0]);
		HAL_I2C_Master_Transmit(&hi2c2,0x70,set,3,1000);		
		
	}

	
	registro[0] = 0xe4;
	registro[1] = 0x03;
	E1 = readRegADE1(&registro[0]);
	
	registro[1] = 0x04;
	E2 = readRegADE1(&registro[0]);
	
	registro[1] = 0x05;
	E3 = readRegADE1(&registro[0]);
	sniffNuovo = E3;
	
	registro[1] = 0x09;
	R1 = readRegADE1(&registro[0]);
	
	registro[1] = 0x0a;
	R2 = readRegADE1(&registro[0]);
	
	registro[1] = 0x0b;
	R3 = readRegADE1(&registro[0]);
	
	E1nc[0] = E1 - E1old; E1nc[1] = E2 - E2old; E1nc[2] = E3 - E3old;
	R1nc[0] = R1 - R1old; R1nc[1] = R2 - R2old; R1nc[2] = R3 - R3old;
	
	if(triangolo == 1){
		E1nc[1] = 0;	R1nc[1] = 0;		
	}
	

	//if(azzeramento == 1){
	//	azzeramento = 0;
	//	HAL_I2C_Master_Transmit(&hi2c2,0x70,fineazzeramento,3,1000);
	//	E1 = 0; E2 = 0; E3 = 0; R1 = 0; R2 = 0; R3 = 0;
	//}
	
	//lettura tensioni e correnti
	
	registro[0] = 0x43;
	registro[1] = 0xc1;
	Vnc[0] = readRegADE1(&registro[0]);
	lettureV[0] = Vnc[0];
	
	registro[1] = 0xc3;
	Vnc[1] = readRegADE1(&registro[0]);
	lettureV[1] = Vnc[1];
	
	registro[1] = 0xc5;
	Vnc[2] = readRegADE1(&registro[0]);
	lettureV[2] = Vnc[2];
	
	registro[1] = 0xc0;
	I1nc[0] = readRegADE1(&registro[0]);
		
	registro[1] = 0xc2;
	I1nc[1] = readRegADE1(&registro[0]);

	registro[1] = 0xc4;
	I1nc[2] = readRegADE1(&registro[0]);
	
		
}

//lettura registro ADE 1
u32 readRegADE1(u8 *registro){
	u8 dato[4];
	u32 var;
	
	HAL_I2C_Master_Transmit(&hi2c2, 0x70, &registro[0], 2, 100);
	HAL_I2C_Master_Receive(&hi2c2, 0x70, &dato[0],4,100);
	var = array2u32(&dato[0]);
	
	return var;
}






//acquisizioni ADE 2
void acquisizioni3(void){
	u8 registro[2];
	static long E1 = 0, E2 = 0, E3 = 0;
	static long E1old = 0, E2old = 0, E3old = 0;
	static long R1 = 0, R2 = 0, R3 = 0;
	static long R1old = 0, R2old = 0, R3old = 0;
	u8 reset[3] = {0xe7,0x02,0x78};
	u8 set[3] = {0xe7,0x02,0x38};
	
	u8 uart[100];
	
	//lettura energie
	if(azzeramento3 == 0){
		E1old = E1; E2old = E2; E3old = E3;
		R1old = R1; R2old = R2; R3old = R3;
	}
	else{
		azzeramento3 = 0;
		E1old = 0; E2old = 0; E3old = 0;
		R1old = 0; R2old = 0; R3old = 0;
		
		HAL_I2C_Master_Transmit(&hi2c3,0x70,reset,3,1000);
		registro[0] = 0xe4;
		registro[1] = 0x03;	readRegADE3(&registro[0]);
		registro[1] = 0x04;	readRegADE3(&registro[0]);
		registro[1] = 0x05;	readRegADE3(&registro[0]);
		registro[1] = 0x09;	readRegADE3(&registro[0]);
		registro[1] = 0x0a;	readRegADE3(&registro[0]);
		registro[1] = 0x0b;	readRegADE3(&registro[0]);
		HAL_I2C_Master_Transmit(&hi2c3,0x70,set,3,1000);
		
	}

	
	registro[0] = 0xe4;
	registro[1] = 0x03;
	E1 = readRegADE3(&registro[0]);
	
	registro[1] = 0x04;
	E2 = readRegADE3(&registro[0]);
	
	registro[1] = 0x05;
	E3 = readRegADE3(&registro[0]);
	
	registro[1] = 0x09;
	R1 = readRegADE3(&registro[0]);
	
	registro[1] = 0x0a;
	R2 = readRegADE3(&registro[0]);
	
	registro[1] = 0x0b;
	R3 = readRegADE3(&registro[0]);
	
	
	
	E2nc[0] = E1 - E1old; E2nc[1] = E2 - E2old; E2nc[2] = E3 - E3old;
	R2nc[0] = R1 - R1old; R2nc[1] = R2 - R2old; R2nc[2] = R3 - R3old;
	

	
	if(triangolo == 1){
		E2nc[1] = 0;	R2nc[1] = 0;		
	}
	
	//if(azzeramento3 == 1){
	//	azzeramento3 = 0;
	//	HAL_I2C_Master_Transmit(&hi2c3,0x70,fineazzeramento,3,1000);
	//	E1 = 0; E2 = 0; E3 = 0; R1 = 0; R2 = 0; R3 = 0;
	//}
	
	//lettura correnti
	
	registro[0] = 0x43;
	registro[1] = 0xc0;
	I2nc[0] = readRegADE3(&registro[0]);

	registro[1] = 0xc2;
	I2nc[1] = readRegADE3(&registro[0]);

	registro[1] = 0xc4;
	I2nc[2] = readRegADE3(&registro[0]);
		
}



//lettura registro ADE 2
u32 readRegADE3(u8 *registro){
	u8 dato[4];
	u32 var;
	
	HAL_I2C_Master_Transmit(&hi2c3, 0x70, &registro[0], 2, 100);
	HAL_I2C_Master_Receive(&hi2c3, 0x70, &dato[0],4,100);
	var = array2u32(&dato[0]);
	
	return var;
}











//elaborazione misure ADE 1
void elabMisure(void){
	double stampella;
	double cosfi;
	u8 addressFram[2] = {0,0};
	int i;
	u8 uart[100];
	u8 data[4];
		
	switch(calib){
		case 1:
			corrV[0] = Vnc[0]*250/23000; //calibrazione tensioni
			corrV[1] = Vnc[1]*250/23000;
			corrV[2] = Vnc[2]*250/23000;
			for (i=0; i<3; i++){
				addressFram[1] = 21+i*4;
				saveU32fram(corrV[i],&addressFram[0]);
			}
			calib = 0;
			break;
		case 2:
			offI1[0] = I1nc[0]; //offset
			offI1[1] = I1nc[1];
			offI1[2] = I1nc[2];
			offE1[0] = E1nc[0];
			offE1[1] = E1nc[1];
			offE1[2] = E1nc[2];
			for (i=0; i<3; i++){
				addressFram[1] = 33+i*4;
				saveU32fram(offI1[i],&addressFram[0]);
				addressFram[1] = 45+i*4;
				saveU32fram(offE1[i],&addressFram[0]);
			}
			calib = 0;
			break;
		case 3:

			corrI1[0] = I1nc[0]*100/7500; //calibrazione correnti
			corrI1[1] = I1nc[1]*100/7500;
			corrI1[2] = I1nc[2]*100/7500;
			for (i=0; i<3; i++){
				addressFram[1] = 57+i*4;
				saveU32fram(corrI1[i],&addressFram[0]);
			}
			calib = 0;
			break;
		case 4:

			corrE1[0] = E1nc[0]*1000 / 4313; //900 = 1000 * cosfi, non è detto che il *1000 vada bene
			corrE1[1] = E1nc[1]*1000 / 4313;
			corrE1[2] = E1nc[2]*1000 / 4313;
			corrR1[0] = R1nc[0]*1000 / 2089; //436 = 1000 * senfi (0.43589)
			corrR1[1] = R1nc[1]*1000 / 2089;
			corrR1[2] = R1nc[2]*1000 / 2089;
	
		
			for (i=0; i<3; i++){
				addressFram[1] = 69+i*4;
				saveU32fram(corrE1[i],&addressFram[0]);
				addressFram[1] = 81+i*4;
				saveU32fram(corrR1[i],&addressFram[0]);
			}
			

			calib = 0;
			break;
		case 0:
			V[0] = Vnc[0]*250 / corrV[0];
			V[1] = Vnc[1]*250 / corrV[1];
			V[2] = Vnc[2]*250 / corrV[2];
		
		if (triangolo == 1){
			VD[0] = V[0];	VD[2] = V[2];
			VD[0] /= 100;	VD[2] /= 100;
			
			VD[1] = (VD[0] - VD[2]/2)*(VD[0] - VD[2]/2) + (0.866*VD[2])*(0.866*VD[2]);
			VD[1] = sqrt(VD[1]);
			VD[1] *= 100;
			V[1] = VD[1];
			
		}
		
		if (triangolo == 2){
			if(V[1] < 5000 && V[0] > 15000 && V[2] > 15000){
				triangolo = 1;
			}
			else{
				triangolo = 0;
			}
		}
		
		
			
//			//correzione energie

			//calcolo energie attive
			E1nc[0] *= 1000; E1nc[1] *= 1000; E1nc[2] *= 1000;
			R1nc[0] *= 1000; R1nc[1] *= 1000; R1nc[2] *= 1000;
			stampella = E1nc[0] / corrE1[0];	E1[0] = stampella;
			stampella = E1nc[1] / corrE1[1];	E1[1] = stampella;
			stampella = E1nc[2] / corrE1[2];	E1[2] = stampella;// stampella;
			//calcolo energie reattive
			stampella = R1nc[0] / corrR1[0];	R1[0] = stampella;
			stampella = R1nc[1] / corrR1[1];	R1[1] = stampella;
			stampella = R1nc[2] / corrR1[2];	R1[2] = stampella;
			//calcolo correnti

			I1[0] = I1nc[0]*100 / corrI1[0]; if(cosfi<0){I1[0] = -I1[0];}
			
			I1[1] = I1nc[1]*100 / corrI1[1]; if(cosfi<0){I1[1] = -I1[1];}
				
			I1[2] = I1nc[2]*100 / corrI1[2]; if(cosfi<0){I1[2] = -I1[2];}		
	}
	
}







//elaborazione misure ADE 2
void elabMisure3(void){
	double stampella;
	double cosfi;
	u8 addressFram[2] = {0,0};
	int i;
	
	switch(calib3){
		case 1:
			calib3 = 0;
			break;
		case 2:
			offI2[0] = I2nc[0]; //offset
			offI2[1] = I2nc[1];
			offI2[2] = I2nc[2];
			offE2[0] = E2nc[0];
			offE2[1] = E2nc[1];
			offE2[2] = E2nc[2];
			for (i=0; i<3; i++){
				addressFram[1] = 141+i*4;
				saveU32fram(offI2[i],&addressFram[0]);
				addressFram[1] = 153+i*4;
				saveU32fram(offE2[i],&addressFram[0]);
			}
			calib3 = 0;
			break;
		case 3:

			corrI2[0] = I2nc[0]*100/7500; //calibrazione correnti
			corrI2[1] = I2nc[1]*100/7500;
			corrI2[2] = I2nc[2]*100/7500;
			for (i=0; i<3; i++){
				addressFram[1] = 165+i*4;
				saveU32fram(corrI2[i],&addressFram[0]);
			}
			calib3 = 0;
			break;
		case 4:

			corrE2[0] = E2nc[0]*1000 / 4313; //900 = 1000 * cosfi, non è detto che il *1000 vada bene
			corrE2[1] = E2nc[1]*1000 / 4313;
			corrE2[2] = E2nc[2]*1000 / 4313;
			corrR2[0] = R2nc[0]*1000 / 2089; //436 = 1000 * senfi (0.43589)
			corrR2[1] = R2nc[1]*1000 / 2089;
			corrR2[2] = R2nc[2]*1000 / 2089;			

		
			for (i=0; i<3; i++){
				addressFram[1] = 177+i*4;
				saveU32fram(corrE2[i],&addressFram[0]);
				addressFram[1] = 189+i*4;
				saveU32fram(corrR2[i],&addressFram[0]);
			}

			
			calib3 = 0;
			break;
		case 0:

		
			E2nc[0] *= 1000; E2nc[1] *= 1000; E2nc[2] *= 1000;
			R2nc[0] *= 1000; R2nc[1] *= 1000; R2nc[2] *= 1000;
			stampella = E2nc[0] / corrE2[0];	E2[0] = stampella;
			stampella = E2nc[1] / corrE2[1];	E2[1] = stampella;
			stampella = E2nc[2] / corrE2[2];	E2[2] = stampella;
			//calcolo energie reattive
			stampella = R2nc[0] / corrR2[0];	R2[0] = stampella;
			stampella = R2nc[1] / corrR2[1];	R2[1] = stampella;
			stampella = R2nc[2] / corrR2[2];	R2[2] = stampella;
			//calcolo correnti
			I2[0] = I2nc[0] *100/ corrI2[0]; if(cosfi<0){I2[0] = -I2[0];}
			
			I2[1] = I2nc[1] *100/ corrI2[1]; if(cosfi<0){I2[1] = -I2[1];}
				
			I2[2] = I2nc[2] *100/ corrI2[2]; if(cosfi<0){I2[2] = -I2[2];}				
			
	}
	

}







//calcolatutto ADE 1
void calcolatutto(void){
static u32 Epm[3] = {0,0,0};
static u32 Emm[3] = {0,0,0};
static u32 Rpm[3] = {0,0,0};
static u32 Rmm[3] = {0,0,0};
double stampella,cosfi,sniff;
u8 azzeroregistri[3] = {0xe7,0x02,0x78};
u8 backup[100];
u8 addressFram[2] = {0,0};
int i;

u8 uart[100];


	
//FASE 1
//divido le energie
if(abs(I1[0]) > sogliaI && 25*abs(E1[0]) > abs(Q1[0]) ){
	if(E1[0] > 0){
		Epm[0] += E1[0];		
	}
	else{
		Emm[0] += abs(E1[0]);
	}
	if(R1[0] > 0){
		Rpm[0] += R1[0];		
	}
	else{
		Rmm[0] += abs(R1[0]);
	}
	//calcolo potenze
	stampella = E1[0];
	stampella *= 3.6;
	P1[0] = stampella;
	stampella = R1[0];
	stampella *= 3.6;
	Q1[0] = stampella;	
	//calcolo cosfi
	stampella = P1[0]*P1[0] + Q1[0]*Q1[0];
	cosfi = P1[0] / sqrt(stampella);
	cosfi *= 100;
	if(cosfi>0){cosphi1[0] = cosfi + 0.5;}
	else{cosphi1[0] = cosfi - 0.5;}	
	//calcolo fi
	stampella = cosfi;
	stampella /= 100;
	sniff = stampella;
	stampella = acos(stampella)*180/PI;
	if(Q1[0] < 0){stampella = 360 - stampella;}
	phi1[0] = stampella + 0.5;	
	
}
else{
	I1[0] = 0;
	P1[0] = 0;	Q1[0] = 0;
	cosphi1[0] = 0;	phi1[0] = 0;
}	
	


//FASE 2
//divido le energie
if(abs(I1[1]) > sogliaI && 25*abs(E1[1]) > abs(Q1[1])){
	if(E1[1] > 0){
		Epm[1] += E1[1];		
	}
	else{
		Emm[1] += abs(E1[1]);
	}
	if(R1[1] > 0){
		Rpm[1] += R1[1];		
	}
	else{
		Rmm[1] += abs(R1[1]);
	}
	//calcolo potenze
	stampella = E1[1];
	stampella *= 3.6;
	P1[1] = stampella;
	stampella = R1[1];
	stampella *= 3.6;
	Q1[1] = stampella;	
	//calcolo cosfi
	stampella = P1[1]*P1[1] + Q1[1]*Q1[1];
	cosfi = P1[1] / sqrt(stampella);
	cosfi *= 100;
	if(cosfi>0){cosphi1[1] = cosfi + 0.5;}
	else{cosphi1[1] = cosfi - 0.5;}
		
		//cosfi = 90;
	//calcolo fi
	stampella = cosfi;
	stampella /= 100;		
	stampella = acos(stampella)*180/PI;
	if(Q1[1] < 0){stampella = 360 - stampella;}
	phi1[1] = stampella + 0.5;
}
else{
	I1[1] = 0;
	P1[1] = 0;	Q1[1] = 0;
	cosphi1[1] = 0;	phi1[1] = 0;
}


	
//FASE 3
//divido le energie
if(abs(I1[2]) > sogliaI && 25*abs(E1[2]) > abs(Q1[2])){
	
	if(E1[2] > 0){
		Epm[2] += E1[2];		
	}
	else{
		Emm[2] += abs(E1[2]);
	}
	if(R1[2] > 0){
		Rpm[2] += R1[2];		
	}
	else{
		Rmm[2] += abs(R1[2]);
	}
	//calcolo potenze
	stampella = E1[2];
	stampella *= 3.6;
	P1[2] = stampella;
	stampella = R1[2];
	stampella *= 3.6;
	Q1[2] = stampella;	
	//calcolo cosfi
	stampella = P1[2]*P1[2] + Q1[2]*Q1[2];
	cosfi = P1[2] / sqrt(stampella);
	cosfi *= 100;
	if(cosfi>0){cosphi1[2] = cosfi + 0.5;}
	else{cosphi1[2] = cosfi - 0.5;}
	//calcolo fi
	stampella = cosfi;
	stampella /= 100;
	stampella = acos(stampella)*180/PI;
	if(Q1[2] < 0){stampella = 360 - stampella;}
	phi1[2] = stampella + 0.5;	
}
else{
	I1[2] = 0;
	P1[2] = 0;	Q1[2] = 0;
	cosphi1[2] = 0;	phi1[2] = 0;
}


	//composizione totalizzatori
	if(azzeraLineaA == 0){
		for(i=0;i<3;i++){
			E1p[i] = E1pi[i] + Epm[i]/1000;
			E1n[i] = E1ni[i] + Emm[i]/1000;
			R1p[i] = R1pi[i] + Rpm[i]/1000;
			R1n[i] = R1ni[i] + Rmm[i]/1000;
		}
	}
	else{
		azzeraLineaA = 0;
		E1p[0] = 0; E1pi[0] = 0;	Epm[0] = 0;
		E1n[0] = 0; E1ni[0] = 0;	Emm[0] = 0;
		R1p[0] = 0; R1pi[0] = 0;	Rpm[0] = 0;
		R1n[0] = 0; R1ni[0] = 0;	Rmm[0] = 0;
		
		E1p[1] = 0; E1pi[1] = 0;	Epm[1] = 0;
		E1n[1] = 0; E1ni[1] = 0;	Emm[1] = 0;
		R1p[1] = 0; R1pi[1] = 0;	Rpm[1] = 0;
		R1n[1] = 0; R1ni[1] = 0;	Rmm[1] = 0;
		
		E1p[2] = 0; E1pi[2] = 0;	Epm[2] = 0;
		E1n[2] = 0; E1ni[2] = 0;	Emm[2] = 0;
		R1p[2] = 0; R1pi[2] = 0;	Rpm[2] = 0;
		R1n[2] = 0; R1ni[2] = 0;	Rmm[2] = 0;
	}
	
	
	//rollover
	while(i<3){
		
		if(E1p[i] > 999999999){
			E1p[i] = 0;
			E1pi[i] = 0;
			Epm[i] = 0;
		}
		if(E1n[i] > 999999999){
			E1n[i] = 0;
			E1ni[i] = 0;
			Emm[i] = 0;
		}
		if(R1p[i] > 999999999){
			R1p[i] = 0;
			R1pi[i] = 0;
			Rpm[i] = 0;
		}
		if(R1n[i] > 999999999){
			R1n[i] = 0;
			R1ni[i] = 0;
			Rmm[i] = 0;
		}		
		i++;
	}
	
	
	//totalizzatori TOTALI
	
	E1pTOT = E1p[0] + E1p[1] + E1p[2];
	E1nTOT = E1n[0] + E1n[1] + E1n[2];
	R1pTOT = R1p[0] + R1p[1] + R1p[2];
	R1nTOT = R1n[0] + R1n[1] + R1n[2];

	
	
	//azzeramento orario totalizzatori milliwattora
	if(currentTime.Minutes == 0 && currentTime.Seconds == 0){
		E1pi[0] = E1p[0]; E1pi[1] = E1p[1]; E1pi[2] = E1p[2];
		E1ni[0] = E1n[0]; E1ni[1] = E1n[1]; E1ni[2] = E1n[2]; 
		R1pi[0] = R1p[0]; R1pi[1] = R1p[1]; R1pi[2] = R1p[2]; 
		R1ni[0] = R1n[0]; R1ni[1] = R1n[1]; R1ni[2] = R1n[2]; 
		
		Epm[0] = 0; Epm[1] = 0; Epm[2] = 0; 
		Emm[0] = 0; Emm[1] = 0; Emm[2] = 0;
		Rpm[0] = 0; Rpm[1] = 0; Rpm[2] = 0; 		
		Rmm[0] = 0; Rmm[1] = 0; Rmm[2] = 0;
		
		//HAL_I2C_Master_Transmit(&hi2c1,0x70,azzeroregistri,3,1000);
		//azzeramento = 1;		
		azzeraRegistriADE1();
	}
	
	//backup energie
	addressFram[1] = 93;
	u322array(&backup[0],E1p[0]); u322array(&backup[4],E1p[1]); u322array(&backup[8],E1p[2]);
	u322array(&backup[12],E1n[0]); u322array(&backup[16],E1n[1]); u322array(&backup[20],E1n[2]); 
	u322array(&backup[24],R1p[0]); u322array(&backup[28],R1p[1]); u322array(&backup[32],R1p[2]);
	u322array(&backup[36],R1n[0]); u322array(&backup[40],R1n[1]); u322array(&backup[44],R1n[2]);
	saveArrayFram(&backup[0],&addressFram[0],48);
	
		
	if(I1[0] >= sogliaI && I1[0] < Ipot && V[0] > 10000){
		cosfi = cosphi1[0];
		cosfi = cosfi/100;
		stampella = V[0];
		stampella = stampella/100;
		cosfi = stampella*cosfi;
		stampella = P1[0];
		stampella = stampella / cosfi;
		stampella = stampella * 100;
		I1[0] = stampella;	
			if(I1[0] < sogliaI){
				I1[0] = 0;
			}
	}
	
	if(I1[1] >= sogliaI && I1[1] < Ipot && V[1] > 10000){
		cosfi = cosphi1[1];
		cosfi = cosfi/100;
		stampella = V[1];
		stampella = stampella/100;
		cosfi = stampella*cosfi;
		stampella = P1[1];
		stampella = stampella / cosfi;
		stampella = stampella * 100;
		I1[1] = stampella;
			if(I1[1] < sogliaI){
				I1[1] = 0;
			}
	}
	
	if(I1[2] >= sogliaI && I1[2] < Ipot && V[2] > 10000){
		cosfi = cosphi1[2];
		cosfi = cosfi/100;
		stampella = V[2];
		stampella = stampella/100;
		cosfi = stampella*cosfi;
		stampella = P1[2];
		stampella = stampella / cosfi;
		stampella = stampella * 100;
		I1[2] = stampella;
			if(I1[2] < sogliaI){
				I1[2] = 0;
			}
	}
	
}







//calcolatutto ADE 2
void calcolatutto3(void){
static u32 Epm[3] = {0,0,0};
static u32 Emm[3] = {0,0,0};
static u32 Rpm[3] = {0,0,0};
static u32 Rmm[3] = {0,0,0};
double stampella,cosfi;
u8 azzeroregistri[3] = {0xe7,0x02,0x78};
u8 backup[100];
u8 addressFram[2] = {0,0};
int i;
	


//FASE 1
//divido le energie
if(abs(I2[0]) > sogliaI && 25*abs(E2[0]) > abs(Q2[0])){
	
	if(E2[0] > 0){
		Epm[0] += E2[0];		
	}
	else{
		Emm[0] += abs(E2[0]);
	}
	if(R2[0] > 0){
		Rpm[0] += R2[0];		
	}
	else{
		Rmm[0] += abs(R2[0]);
	}
	//calcolo potenze
	stampella = E2[0];
	stampella *= 3.6;
	P2[0] = stampella;
	stampella = R2[0];
	stampella *= 3.6;
	Q2[0] = stampella;	
	//calcolo cosfi
	stampella = P2[0]*P2[0] + Q2[0]*Q2[0];
	cosfi = P2[0] / sqrt(stampella);
	cosfi *= 100;
	if(cosfi>0){cosphi2[0] = cosfi + 0.5;}
	else{cosphi2[0] = cosfi - 0.5;}	
	//calcolo fi
	stampella = cosfi;
	stampella /= 100;
	stampella = acos(stampella)*180/PI;
	if(Q2[0] < 0){stampella = 360 - stampella;}
	phi2[0] = stampella + 0.5;	
}
	
else{
	I2[0] = 0;
	P2[0] = 0;	Q2[0] = 0;
	cosphi2[0] = 0;	phi2[0] = 0;
}

//FASE 2
//divido le energie
if(abs(I2[1]) > sogliaI && 25*abs(E2[1]) > abs(Q2[1])){
	
	if(E2[1] > 0){
		Epm[1] += E2[1];		
	}
	else{
		Emm[1] += abs(E2[1]);
	}
	if(R2[1] > 0){
		Rpm[1] += R2[1];		
	}
	else{
		Rmm[1] += abs(R2[1]);
	}
	//calcolo potenze
	stampella = E2[1];
	stampella *= 3.6;
	P2[1] = stampella;
	stampella = R2[1];
	stampella *= 3.6;
	Q2[1] = stampella;	
	//calcolo cosfi
	stampella = P2[1]*P2[1] + Q2[1]*Q2[1];
	cosfi = P2[1] / sqrt(stampella);
	cosfi *= 100;
	if(cosfi>0){cosphi2[1] = cosfi + 0.5;}
	else{cosphi2[1] = cosfi - 0.5;}
	//calcolo fi
	stampella = cosfi;
	stampella /= 100;
	stampella = acos(stampella)*180/PI;
	if(Q2[1] < 0){stampella = 360 - stampella;}
	phi2[1] = stampella + 0.5;
}

else{
	I2[1] = 0;
	P2[1] = 0;	Q2[1] = 0;
	cosphi2[1] = 0;	phi2[1] = 0;
}

//FASE 3
//divido le energie
if(abs(I2[2]) > sogliaI && 25*abs(E2[2]) > abs(Q2[2])){
	
	if(E2[2] > 0){
		Epm[2] += E2[2];		
	}
	else{
		Emm[2] += abs(E2[2]);
	}
	if(R2[2] > 0){
		Rpm[2] += R2[2];		
	}
	else{
		Rmm[2] += abs(R2[2]);
	}
	//calcolo potenze
	stampella = E2[2];
	stampella *= 3.6;
	P2[2] = stampella;
	stampella = R2[2];
	stampella *= 3.6;
	Q2[2] = stampella;	
	//calcolo cosfi
	stampella = P2[2]*P2[2] + Q2[2]*Q2[2];
	cosfi = P2[2] / sqrt(stampella);
	cosfi *= 100;
	if(cosfi>0){cosphi2[2] = cosfi + 0.5;}
	else{cosphi2[2] = cosfi - 0.5;}
	//calcolo fi
	stampella = cosfi;
	stampella /= 100;
	stampella = acos(stampella)*180/PI;
	if(Q2[2] < 0){stampella = 360 - stampella;}
	phi2[2] = stampella + 0.5;
}

else{
	I2[2] = 0;
	P2[2] = 0;	Q2[2] = 0;
	cosphi2[2] = 0;	phi2[2] = 0;
}




	//composizione totalizzatori
	if(azzeraLineaB == 0){
		for(i=0;i<3;i++){
			E2p[i] = E2pi[i] + Epm[i]/1000;
			E2n[i] = E2ni[i] + Emm[i]/1000;
			R2p[i] = R2pi[i] + Rpm[i]/1000;
			R2n[i] = R2ni[i] + Rmm[i]/1000;
		}
	}
	else{
		azzeraLineaB = 0;
		E2p[0] = 0; E2pi[0] = 0;	Epm[0] = 0;
		E2n[0] = 0; E2ni[0] = 0;	Emm[0] = 0;
		R2p[0] = 0; R2pi[0] = 0;	Rpm[0] = 0;
		R2n[0] = 0; R2ni[0] = 0;	Rmm[0] = 0;
		
		E2p[1] = 0; E2pi[1] = 0;	Epm[1] = 0;
		E2n[1] = 0; E2ni[1] = 0;	Emm[1] = 0;
		R2p[1] = 0; R2pi[1] = 0;	Rpm[1] = 0;
		R2n[1] = 0; R2ni[1] = 0;	Rmm[1] = 0;
		
		E2p[2] = 0; E2pi[2] = 0;	Epm[2] = 0;
		E2n[2] = 0; E2ni[2] = 0;	Emm[2] = 0;
		R2p[2] = 0; R2pi[2] = 0;	Rpm[2] = 0;
		R2n[2] = 0; R2ni[2] = 0;	Rmm[2] = 0;		
	}
	
		
	
	
	//rollover
	while(i<3){
		
		if(E2p[i] > 999999999){
			E2p[i] = 0;
			E2pi[i] = 0;
			Epm[i] = 0;
		}
		if(E2n[i] > 999999999){
			E2n[i] = 0;
			E2ni[i] = 0;
			Emm[i] = 0;
		}
		if(R2p[i] > 999999999){
			R2p[i] = 0;
			R2pi[i] = 0;
			Rpm[i] = 0;
		}
		if(R2n[i] > 999999999){
			R2n[i] = 0;
			R2ni[i] = 0;
			Rmm[i] = 0;
		}		
		i++;
	}
	//totalizzatori TOTALI
	E2pTOT = E2p[0] + E2p[1] + E2p[2];
	E2nTOT = E2n[0] + E2n[1] + E2n[2];
	R2pTOT = R2p[0] + R2p[1] + R2p[2];
	R2nTOT = R2n[0] + R2n[1] + R2n[2];
	
	//azzeramento orario totalizzatori milliwattora
	if(currentTime.Minutes == 0 && currentTime.Seconds == 0){
		E2pi[0] = E2p[0]; E2pi[1] = E2p[1]; E2pi[2] = E2p[2];
		E2ni[0] = E2n[0]; E2ni[1] = E2n[1]; E2ni[2] = E2n[2]; 
		R2pi[0] = R2p[0]; R2pi[1] = R2p[1]; R2pi[2] = R2p[2]; 
		R2ni[0] = R2n[0]; R2ni[1] = R2n[1]; R2ni[2] = R2n[2]; 
		
		Epm[0] = 0; Epm[1] = 0; Epm[2] = 0; 
		Emm[0] = 0; Emm[1] = 0; Emm[2] = 0;
		Rpm[0] = 0; Rpm[1] = 0; Rpm[2] = 0; 		
		Rmm[0] = 0; Rmm[1] = 0; Rmm[2] = 0;
		
		
		//HAL_I2C_Master_Transmit(&hi2c3,0x70,azzeroregistri,3,1000);
		//azzeramento3 = 1;		
		azzeraRegistriADE3();
	}
	
	
	
	//backup energie
	addressFram[1] = 201;
	u322array(&backup[0],E2p[0]); u322array(&backup[4],E2p[1]); u322array(&backup[8],E2p[2]);
	u322array(&backup[12],E2n[0]); u322array(&backup[16],E2n[1]); u322array(&backup[20],E2n[2]); 
	u322array(&backup[24],R2p[0]); u322array(&backup[28],R2p[1]); u322array(&backup[32],R2p[2]);
	u322array(&backup[36],R2n[0]); u322array(&backup[40],R2n[1]); u322array(&backup[44],R2n[2]);
	saveArrayFram(&backup[0],&addressFram[0],48);
	
	
	
	if(I2[0] >= sogliaI && I2[0] < Ipot && V[0] > 10000){
		cosfi = cosphi2[0];
		cosfi = cosfi/100;
		stampella = V[0];
		stampella = stampella/100;
		cosfi = stampella*cosfi;
		stampella = P2[0];
		stampella = stampella / cosfi;
		stampella = stampella * 100;
		I2[0] = stampella;
			if(I2[0] < sogliaI){
				I2[0] = 0;
			}
	}
	
	if(I2[1] >= sogliaI && I2[1] < Ipot && V[1] > 10000){
		cosfi = cosphi2[1];
		cosfi = cosfi/100;
		stampella = V[1];
		stampella = stampella/100;
		cosfi = stampella*cosfi;
		stampella = P2[1];
		stampella = stampella / cosfi;
		stampella = stampella * 100;
		I2[1] = stampella;
			if(I2[1] < sogliaI){
				I2[1] = 0;
			}
	}
	
	if(I2[2] >= sogliaI && I2[2] < Ipot && V[2] > 10000){
		cosfi = cosphi2[2];
		cosfi = cosfi/100;
		stampella = V[2];
		stampella = stampella/100;
		cosfi = stampella*cosfi;
		stampella = P2[2];
		stampella = stampella / cosfi;
		stampella = stampella * 100;
		I2[2] = stampella;
			if(I2[2] < sogliaI){
				I2[2] = 0;
			}
	}
	
	/*I1[0] = (currentTime.Minutes + 10)*100;
	I1[1] = I1[0]+100;
	I1[2] = I1[0]+200;
	
	I2[0] = I1[0]+300;
	I2[1] = I1[0]+400;
	I2[2] = I1[0]+500;*/
	
}

void azzeraRegistriADE1(void){
	u8 reset[3] = {0xe7,0x02,0x78};
	u8 set[3] = {0xe7,0x02,0x38};
	u8 registro[2];
	
HAL_I2C_Master_Transmit(&hi2c3,0x70,reset,3,1000);
	registro[0] = 0xe4;
	registro[1] = 0x03;	readRegADE1(&registro[0]);
	registro[1] = 0x04;	readRegADE1(&registro[0]);
	registro[1] = 0x05;	readRegADE1(&registro[0]);
	registro[1] = 0x09;	readRegADE1(&registro[0]);
	registro[1] = 0x0a;	readRegADE1(&registro[0]);
	registro[1] = 0x0b;	readRegADE1(&registro[0]);
	delay(200);
	HAL_I2C_Master_Transmit(&hi2c3,0x70,set,3,1000);
	
	azzeramento = 1;
}

void azzeraRegistriADE3(void){
	u8 reset[3] = {0xe7,0x02,0x78};
	u8 set[3] = {0xe7,0x02,0x38};
	u8 registro[2];
	
		
	
	registro[0] = 0xe4;
	registro[1] = 0x03;	readRegADE3(&registro[0]);
	registro[1] = 0x04;	readRegADE3(&registro[0]);
	registro[1] = 0x05;	readRegADE3(&registro[0]);
	registro[1] = 0x09;	readRegADE3(&registro[0]);
	registro[1] = 0x0a;	readRegADE3(&registro[0]);
	registro[1] = 0x0b;	readRegADE3(&registro[0]);
	HAL_I2C_Master_Transmit(&hi2c3,0x70,set,3,1000);
	
	azzeramento3 = 1;
}



//crea array dashboard
void dashboard(u8 *outBuf){
	int i;
	
		//fake
		u32 tensione = 22000;
		u32 corrente = 5000;
		u32 potenzaA = 18000;
		u32 potenzaR = 8718;
		u16 cosfi = 90;
		u16 fi = 2600;
		u16 localsoftware = 126;
	
	
	copiaArray(&outBuf[0],&identificativo[0],16);
	u322array(&outBuf[16],myTimeVar);
	u322array(&outBuf[20],latitudine);
	u322array(&outBuf[24],longitudine);
	outBuf[28] = batteryLevel;
	u162array(&outBuf[29],software);
	outBuf[31] = segnaleGSM;
	
	
	for (i=0;i<3;i++){
		u322array(&outBuf[32+4*i],V[i]); //25
		if(I1[i] >= 0){ //controlla che la corrente sia sopra i 4 A
				if(phi1[i] > 90 && phi1[i] <= 270){ //controlla se la corrente è positiva
					u322array(&outBuf[44+4*i],-I1[i]);
				}
				else{
				u322array(&outBuf[44+4*i],I1[i]); //37
				}
			u322array(&outBuf[56+4*i],P1[i]); //49
			u322array(&outBuf[68+4*i],Q1[i]); //61
			u162array(&outBuf[80+2*i],cosphi1[i]); //73
			u162array(&outBuf[86+2*i],phi1[i]*100); //79
		}
		else{
			u322array(&outBuf[44+4*i],0); //37
			u322array(&outBuf[56+4*i],0); //49
			u322array(&outBuf[68+4*i],0); //61
			u162array(&outBuf[80+2*i],0); //73
			u162array(&outBuf[86+2*i],0); //79
		}
		
		if(I2[i] >= 0){
			if(phi2[i] > 90 && phi2[i] <= 270){
				u322array(&outBuf[108+4*i],-I2[i]);
			}
			else{				
				u322array(&outBuf[108+4*i],I2[i]); //101
			}
			u322array(&outBuf[120+4*i],P2[i]); //113
			u322array(&outBuf[132+4*i],Q2[i]); //125
			u162array(&outBuf[144+2*i],cosphi2[i]);	//137
			u162array(&outBuf[150+2*i],phi2[i]*100); //143		
		}
		else{
			u322array(&outBuf[108+4*i],0); //101
			u322array(&outBuf[120+4*i],0); //113
			u322array(&outBuf[132+4*i],0); //125
			u162array(&outBuf[144+2*i],0);	//137
			u162array(&outBuf[150+2*i],0); //143	
		}
	}
	
	
	
	u322array(&outBuf[92],E1pTOT);
	u322array(&outBuf[96],E1nTOT);
	u322array(&outBuf[100],R1pTOT);
	u322array(&outBuf[104],R1nTOT);
	
	u322array(&outBuf[156],E2pTOT);
	u322array(&outBuf[160],E2nTOT);
	u322array(&outBuf[164],R2pTOT);
	u322array(&outBuf[168],R2nTOT);
		
}


void azzeraMisurandi(void){

	int i = 0;
	
	while(i<3){
		//V[i] = 0; le tensioni devono essere lette
		I1[i] = 0;
		I2[i] = 0;
		cosphi1[i] = 0;
		cosphi2[i] = 0;
		phi1[i] = 0;
		phi2[i] = 0;
		P1[i] = 0;
		Q1[i] = 0;
		P2[i] = 0;
		Q2[i] = 0;
		
		i++;
	}

	return;
}

void leggiTensioni(void){
	u8 registro[2];

	registro[0] = 0x43;
	
	registro[1] = 0xc1;
	Vnc[0] = readRegADE1(&registro[0]);
	lettureV[0] = Vnc[0];
	
	registro[1] = 0xc3;
	Vnc[1] = readRegADE1(&registro[0]);
	lettureV[1] = Vnc[1];
		
	registro[1] = 0xc5;
	Vnc[2] = readRegADE1(&registro[0]);
	lettureV[2] = Vnc[2];

	V[0] = Vnc[0]*250 / corrV[0];
	V[1] = Vnc[1]*250 / corrV[1];
	V[2] = Vnc[2]*250 / corrV[2];
	
	
}








