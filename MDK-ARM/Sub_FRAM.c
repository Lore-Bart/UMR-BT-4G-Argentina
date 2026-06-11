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
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;



void saveU32fram(uint32_t inVar,uint8_t *address){
	uint8_t save[4];
	
	u322array(&save[0],inVar);
	saveArrayFram(&save[0],&address[0],4);	
}


//salva array sulla fram
void saveArrayFram(uint8_t *data,uint8_t *address,uint16_t size){
	u16 beforeAddress;
	u8 localAddress[2];
	
	if(size <= 100){
		saveArrayFram100(&data[0],&address[0],size);
	}
	else if(size > 100 && size <= 200){
		saveArrayFram100(&data[0],&address[0],100);
		beforeAddress = array2u16(&address[0]) + 100;
		u162array(&localAddress[0],beforeAddress);
		saveArrayFram100(&data[100],&localAddress[0],size-100);			
	}
	else if(size > 200 && size <= 256){
		saveArrayFram100(&data[0],&address[0],100);
		beforeAddress = array2u16(&address[0]) + 100;
		u162array(&localAddress[0],beforeAddress);
		saveArrayFram100(&data[100],&localAddress[0],100);
		beforeAddress += 100;
		u162array(&localAddress[0],beforeAddress);
		saveArrayFram100(&data[200],&localAddress[0],size-200);	
	}
	else{
		return;
	}
		
}

//precursore salvataggio array su fram
void saveArrayFram100(uint8_t *data,uint8_t *address,uint16_t size){
	uint8_t save[256];
	int i = 0;
	
	copiaArray(&save[0],&address[0],2);
	copiaArray(&save[2],&data[0],size);
	HAL_I2C_Master_Transmit(&hi2c1,I2Cfram,&save[0],size+2,1000);

}


//lettura array salvato in FRAM
void ReadArrayFram(uint8_t *OutBuf,uint8_t *address,uint16_t size){

	HAL_I2C_Master_Transmit(&hi2c1,I2Cfram,&address[0],2,1000);
	HAL_I2C_Master_Receive(&hi2c1,I2Cfram,&OutBuf[0],size,1000);

}



//salva u16 in fram
void saveU16fram(uint16_t inVar,uint8_t *address){
	uint8_t save[2];
	
	u162array(&save[0],inVar);
	saveArrayFram(&save[0],&address[0],2);
}



















