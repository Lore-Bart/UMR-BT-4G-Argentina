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




//formatta settore
void FormatSector(uint8_t *address){
	uint8_t command = 0x06;
	uint8_t array[4] = {0x20,0,0,0};
	
	//write enable
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&command,1,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
	copiaArray(&array[1],&address[0],3);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&array[0],4,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
}



//salva array su flash
void writeArrayFlash(uint8_t *InBuf, uint8_t *address, uint16_t size){
	u32 beforeAddress;
	u8 address3[4];
	u8 address4[4] = {0,0,0};
		
	address3[0] = address[0];
	address3[1] = address[1];
	address3[2] = address[2];
	
	if(size	<= 200){
		writeArrayFlashBefore(&InBuf[0],&address3[0],size);
	}
	else if(size > 200 && size <= 256){
		writeArrayFlashBefore(&InBuf[0],&address3[0],200);
		copiaArray(&address4[1],&address3[0],3);
		beforeAddress = array2u32(&address4[0]) + 200;
		u322array(&address3[0],beforeAddress);
			
		delay(5);
		
		writeArrayFlashBefore(&InBuf[200],&address3[1],size-200);
	}

}


//precursore salva array su flash
void writeArrayFlashBefore(uint8_t *InBuf, uint8_t *address, uint16_t size){
	
	uint8_t save[200],messaggio[100];
	uint8_t command = 0x06;
	
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&command,1,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
	save[0] = 0x02;
	copiaArray(&save[1],&address[0],3);
	copiaArray(&save[4],&InBuf[0],size);

	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&save[0],size+4,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
}


//leggi array su flash
void readArrayFlash(uint8_t *OutBuf,uint8_t *address, uint16_t size){
	uint8_t command[4];
	
	command[0] = 0x03;
	command[1] = 0;
	command[2] = 0;
	command[3] = 0;
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&command[0],1,1000);
	HAL_SPI_Transmit(&hspi4,&address[0],3,1000);
	HAL_SPI_Receive(&hspi4,&OutBuf[0],size,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
}


//formatta blocco
void blockErase (int block){
	uint8_t command = 0x06;
	uint8_t array[4] = {0xd8,0,0,0};
	
			
	//write enable
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&command,1,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
	array[1] = block;
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&array[0],4,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
}



//sblocca flash
void sbloccaFlash(void){
	uint8_t command = 0x06;
	uint8_t array[4] = {1,0};
	
	
	//write enable
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&command,1,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
	delay(10);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&array[0],2,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
}




//formatta blocco
void formatFlash (void){
	uint8_t command = 0x06;
	uint8_t array[4] = {0xc7,0,0,0};
	
			
	//write enable
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&command,1,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
	delay(10);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4,&array[0],4,1000);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	
}















