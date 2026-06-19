#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "string.h"

#define delayNFC 0x1ffff
#define NFC_MAX_DIRECT_WRITE_SIZE 240

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

//NFC
extern int sizeNFC;
extern u8 arrayNFC[16];
extern u8 offsetNFC[2];

extern u8 avviaTestMemorie;

extern u8 risultatoTestMemorie[4];

//NFC after
extern int sizeNFCafter;
extern u8 arrayNFCafter[16];
extern u8 offsetNFCafter[2];





void writeNFC(uint8_t *inBuf, uint8_t size, uint8_t *offset){
	if(size > 16){
		size = 16;
	}
	copiaArray(&arrayNFC[0],&inBuf[0],size);
	copiaArray(&offsetNFC[0],&offset[0],2);
	sizeNFC = size;

}

void writeNFCafter(uint8_t *inBuf, uint8_t size, uint8_t *offset){
	if(size > 16){
		size = 16;
	}
	copiaArray(&arrayNFCafter[0],&inBuf[0],size);
	copiaArray(&offsetNFCafter[0],&offset[0],2);
	sizeNFCafter = size;

}

void clearNFCpending(void){
	sizeNFC = 0;
	sizeNFCafter = 0;
}

//scrittura su TAG NFC
static void nfcWaitMs(u32 ms)
{
	u32 start = HAL_GetTick();
	while((u32)(HAL_GetTick() - start) < ms){
		resetWD();
		HAL_Delay(5);
	}
}


/*
 * Scrittura diretta su tag NFC.
 * Per le normali code evento restano validi i blocchi da 16 byte.
 * Per l'erase NFC usiamo blocchi medi e soprattutto verifichiamo il risultato:
 * prima la vecchia versione dichiarava "completed" anche se il tag non aveva
 * realmente scritto i dati.
 */
static u8 writeNFC16Status(uint8_t *inBuf, uint8_t size, uint8_t *offset){
	uint8_t wrArray[NFC_MAX_DIRECT_WRITE_SIZE + 8];
	u8 outBuf[5] = {0,0,0,0,0};
	HAL_StatusTypeDef txStatus;
	HAL_StatusTypeDef rxStatus;
	
	if(size > NFC_MAX_DIRECT_WRITE_SIZE){
		size = NFC_MAX_DIRECT_WRITE_SIZE;
	}
	if(size == 0){
		return 0;
	}
		
	wrArray[0] = 2; wrArray[1] = 0; wrArray[2] = 0xd6; 
	wrArray[3] = offset[0]; wrArray[4] = offset[1]; wrArray[5] = size;
	copiaArray(&wrArray[6], &inBuf[0], size);
	
	M24SR_ComputeCrc(&wrArray[0],size+6);

	txStatus = HAL_I2C_Master_Transmit(&hi2c1,0xAC,&wrArray[0],size+8,1000);
	if(txStatus != HAL_OK){
		return 0;
	}

	/*
	 * Per scritture superiori a 16 byte il M24SR64 puo' impiegare piu' tempo.
	 * Il timeout massimo indicato per scritture lunghe e' dell'ordine di decine
	 * di ms: teniamo margine e poi leggiamo la risposta APDU.
	 */
	if(size <= 16){
		nfcWaitMs(12);
	}
	else{
		nfcWaitMs(110);
	}

	rxStatus = HAL_I2C_Master_Receive(&hi2c1,0xAC,&outBuf[0],5,1000);
	if(rxStatus != HAL_OK){
		return 0;
	}

	/* Risposta attesa: PCB, SW1=0x90, SW2=0x00, CRC1, CRC2 */
	if(outBuf[1] == 0x90 && outBuf[2] == 0x00){
		return 1;
	}

	return 0;
}

void writeNFC32(uint8_t *inBuf, uint8_t size, uint8_t *offset){
	uint8_t backupOff[2];
	int a = 0;
	u8 backupIn[NFC_MAX_DIRECT_WRITE_SIZE];
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	delay(10);
	
	if(size > NFC_MAX_DIRECT_WRITE_SIZE){
		size = NFC_MAX_DIRECT_WRITE_SIZE;
	}
	while(a < size){
		backupIn[a] = inBuf[a];
		a++;
	}
	a = 0;

	backupOff[0] = offset[0]; backupOff[1] = offset[1];
	
	initNFC5();
	(void)writeNFC16Status(&inBuf[0],size,&offset[0]);
		
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
	
	while(a < size){
		inBuf[a] = backupIn[a];
		a++;
	}
	offset[0] = backupOff[0]; offset[1] = backupOff[1];
	
}

void writeNFC16(uint8_t *inBuf, uint8_t size, uint8_t *offset){
	(void)writeNFC16Status(inBuf,size,offset);
}

/*
 * Scrittura verificata: usata dalla cancellazione NFC.
 * Dopo la scrittura rilegge il blocco e confronta byte per byte.
 * Ritorna 1 solo se il tag contiene davvero i dati richiesti.
 */
u8 writeNFC32Checked(uint8_t *inBuf, uint8_t size, uint8_t *offset){
	uint8_t backupOff[2];
	int a = 0;
	u8 backupIn[NFC_MAX_DIRECT_WRITE_SIZE];
	u8 verifyBuf[NFC_MAX_DIRECT_WRITE_SIZE];
	u8 ok = 1;
	
	if(size > NFC_MAX_DIRECT_WRITE_SIZE){
		size = NFC_MAX_DIRECT_WRITE_SIZE;
	}
	if(size == 0){
		return 0;
	}

	while(a < size){
		backupIn[a] = inBuf[a];
		a++;
	}
	backupOff[0] = offset[0];
	backupOff[1] = offset[1];

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	delay(10);
	initNFC5();
	ok = writeNFC16Status(&inBuf[0],size,&offset[0]);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);

	/* Ripristino parametri prima della lettura di verifica. */
	offset[0] = backupOff[0];
	offset[1] = backupOff[1];
	for(a = 0; a < size; a++){
		inBuf[a] = backupIn[a];
	}

	if(ok == 0){
		return 0;
	}

	readNFC(&verifyBuf[0],size,&offset[0]);
	for(a = 0; a < size; a++){
		if(verifyBuf[a] != backupIn[a]){
			return 0;
		}
	}

	return 1;
}

//lettura da NFC
void readNFC(uint8_t *outBuf, uint8_t size, uint8_t *offset){
	uint8_t reArray[size+5];
	uint32_t ciao = 0x4fff;
	uint8_t command[8] = {0x02,0,0xb0,0,0,12,0,0};

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	delay(10);
	
	initNFC5();
		
	command[3] = offset[0];
	command[4] = offset[1];
	command[5] = size;
	M24SR_ComputeCrc(&command[0],6);
		
	HAL_I2C_Master_Transmit(&hi2c1,0xAC,&command[0],8,1000);
	while(ciao != 0){ciao--;}	ciao = 0x4fff;
	HAL_I2C_Master_Receive(&hi2c1,0xAC,&reArray[0],size+5,1000);
	while(ciao != 0){ciao--;}	ciao = 0x4fff;
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
	
	copiaArray(&outBuf[0],&reArray[1],size);
	
}


//inizializza NFC
void initNFC(){
	uint8_t command[16] = {0x02,0,0xa4,0x04,0,0x07,0xd2,0x76,0,0,0x85,1,1,0,0x35,0xc0};
	uint8_t outBuf[5] = {0,0,0,0,0};
	uint8_t command4[10] = {0x02,0,0xa4,0,0x0c,0x02,0,1,0,0};
	u8 result = 0;
	u8 uart[100];
	
	uint8_t scrittura1[10] = {0x02,0,0xd6,0,0,0x02,0x1f,0xef,0,0};
	uint8_t scrittura3[18] = {0x02,0,0xd6,0,0x02,0x0a,0xc1,1,0,0,0x1f,0xe8,0x54,0x02,0x65,0x6e,0,0};
	uint8_t scrittura4[12] = {0x02,0,0xd6,0,0x0c,4,0,0,0,0,0,0};
	u32 time = delayNFC;
	
	uint8_t kill = 0x52;
	uint32_t ciao = 0xfff;
	u8 outBuf1[5] = {0,0,0,0,0};
	
	M24SR_ComputeCrc(&command[0],14);
	
		
	M24SR_ComputeCrc(&command4[0],8);
	
	M24SR_ComputeCrc(&scrittura1[0],8);
	
	M24SR_ComputeCrc(&scrittura3[0],16);
	
	M24SR_ComputeCrc(&scrittura4[0],10);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	delay(100);
	
	while(time != 0){time--;}	time = delayNFC;
	HAL_I2C_Master_Transmit(&hi2c1,0xac,&kill,1,50);
	while(time != 0){time--;}	time = delayNFC;
	
	HAL_I2C_Master_Transmit(&hi2c1,0xAC,&command[0],16,100);
	while(time != 0){time--;}	time = delayNFC;
	HAL_I2C_Master_Receive(&hi2c1,0xAC,&outBuf[0],5,100);
	
	HAL_I2C_Master_Transmit(&hi2c1,0xAC,&command4[0],10,100);
	while(time != 0){time--;}	time = delayNFC;
	HAL_I2C_Master_Receive(&hi2c1,0xAC,&outBuf[0],5,100);

	HAL_I2C_Master_Transmit(&hi2c1,0xAC,&scrittura1[0],10,100);
	while(time != 0){time--;}	time = delayNFC;
	HAL_I2C_Master_Receive(&hi2c1,0xAC,&outBuf[0],5,100);

	HAL_I2C_Master_Transmit(&hi2c1,0xAC,&scrittura3[0],18,100);
	while(time != 0){time--;}	time = delayNFC;
	HAL_I2C_Master_Receive(&hi2c1,0xAC,&outBuf1[0],5,100);
	
	HAL_I2C_Master_Transmit(&hi2c1,0xAC,&scrittura4[0],12,100);
	while(time != 0){time--;}	time = delayNFC;
	HAL_I2C_Master_Receive(&hi2c1,0xAC,&outBuf[0],5,100);
	
}

void initNFC5(){
	uint8_t command[16] = {0x02,0,0xa4,0x04,0,0x07,0xd2,0x76,0,0,0x85,1,1,0,0x35,0xc0};
	uint8_t outBuf[5] = {0,0,0,0,0};
	uint8_t command4[10] = {0x02,0,0xa4,0,0x0c,0x02,0,1,0,0};
	u8 result = 0;
	u8 uart[100];
	
	uint8_t scrittura1[10] = {0x02,0,0xd6,0,0,0x02,0x1f,0xef,0,0};
	uint8_t scrittura3[18] = {0x02,0,0xd6,0,0x02,0x0a,0xc1,1,0,0,0x1f,0xe8,0x54,0x02,0x65,0x6e,0,0};
	uint8_t scrittura4[12] = {0x02,0,0xd6,0,0x0c,4,0,0,0,0,0,0};
	u32 time = 0xfff;
	
	uint8_t kill = 0x52;
	uint32_t ciao = 0xfff;
		
	M24SR_ComputeCrc(&command[0],14);
	
		
	M24SR_ComputeCrc(&command4[0],8);
	
	
	
	HAL_I2C_Master_Transmit(&hi2c1,0xac,&kill,1,50);
	while(time != 0){time--;}	time = delayNFC;
	
	HAL_I2C_Master_Transmit(&hi2c1,0xAC,&command[0],16,100);
	while(time != 0){time--;}	time = delayNFC;
	HAL_I2C_Master_Receive(&hi2c1,0xAC,&outBuf[0],5,100);
		
	HAL_I2C_Master_Transmit(&hi2c1,0xAC,&command4[0],10,100);
	while(time != 0){time--;}	time = delayNFC;
	HAL_I2C_Master_Receive(&hi2c1,0xAC,&outBuf[0],5,100);
	
}

void I2C_TransmitNFC(u8* inBuf, int size){
	__disable_irq();
	HAL_I2C_Master_Transmit(&hi2c1,0xac,&inBuf[0],size,100);
	__enable_irq();
}

void I2C_ReceiveNFC(u8* outBuf, int size){
	__disable_irq();
	HAL_I2C_Master_Receive(&hi2c1,0xac,&outBuf[0],size,100);
	__enable_irq();
}

void killRF(void){
	uint8_t kill = 0x52;
	u32 time = 0x4ffff;
	
	while(time != 0){time--;}	time = 0x4fff;
	HAL_I2C_Master_Transmit(&hi2c1,0xac,&kill,1,50);
	while(time != 0){time--;}	time = 0x4fff;
}