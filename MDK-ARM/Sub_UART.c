#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"

//periferiche
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c2;
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;

//variabili Rx/Tx
uint8_t tx[200];

uint8_t rxBT[500];
int uartPosBT = 0;
int uartPosOldBT = 0;
uint32_t sizeBT;
uint8_t messaggioRecBT[500];

uint8_t rx4G[500];
int uartPos4G = 0;
int uartPosOld4G = 0;
uint32_t size4G;
uint8_t messaggioRec4G[500];

extern u8 timerModuloESC;



u8 programmaPacchetto = 0;

extern u8 statoModulo;


//update attivo
extern u8 updateAttivo;

//BT attivo
u8 BTattivo = 0;

u8 IDconnesso = 48;

extern u8 emergenza;

extern u16 riavvioForzato;
extern u8 password[17];

extern u8 disconnessione;

u8 uartPack[500];
extern u16 NpackRecGSM;
extern u8 updateGSMatt;

void USART2_IRQHandler(void)
{
	int i = 0;
	u8 uart[20];
	
  HAL_UART_IRQHandler(&huart2);
	
	

	

	for(i=0;i<500;i++){messaggioRecBT[i] = 0;} //azzero vettore UART
		
	
	uartPosBT = 500 - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5); // acquisizione dati in ingresso dal GSM
	sizeBT = RicMsg(&rxBT[0],&messaggioRecBT[0],uartPosBT,uartPosOldBT,500);
	uartPosOldBT = uartPosBT;
	
	sprintf(uart,"pos: %d oldPos: %d\n", uartPosBT, uartPosOldBT);
	HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	//__HAL_UART_CLEAR_IDLEFLAG(&huart2);
	//return;
	
		
	HAL_UART_Transmit(&huart1,messaggioRecBT,strlen(messaggioRecBT),100);
	HAL_UART_Transmit(&huart1,(u8*)"\n",1,100);
	
	//BTattivo = 1;
	
	/*if(comparaStringhe(&messaggioRecBT[0],(u8*)"%CONNECT",8) || comparaStringhe(&messaggioRecBT[0],(u8*)"%RFCOMM_OPEN%",13)){
		BTattivo = 1;
	}
	else if(comparaStringhe(&messaggioRecBT[0],(u8*)"%DISCONN%",9) || comparaStringhe(&messaggioRecBT[0],(u8*)"%RFCOMM_CLOSE%",14)){
		BTattivo = 0;
	}*/
	
	if(updateAttivo == 0){
		if(comparaStringhe(&messaggioRecBT[0],(u8*)"%CONNECT",8) && strlen(messaggioRecBT) > 25){
			eseguiComandoBT(&messaggioRecBT[35]);
			inviaDebug((u8*)"\ncomando\n");
		}
		else if(comparaStringhe(&messaggioRecBT[0],(u8*)"%RFCOMM_OPEN%",13) && strlen(messaggioRecBT) > 20){
			eseguiComandoBT(&messaggioRecBT[13]);
			inviaDebug((u8*)"\ncomando\n");
		}
		else if(messaggioRecBT[0] != '%'){
			eseguiComandoBT(&messaggioRecBT[0]);
		}
	}
	else{
			programmaPacchetto = 1;
	}
	
	
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
}

void USART6_IRQHandler(void)
{
	int i = 0;
	
  HAL_UART_IRQHandler(&huart6);
	
	timerModuloESC = timerModuloESCinit;

	for(i=0;i<500;i++){messaggioRec4G[i] = 0;} //azzero vettore UART
		
	uartPos4G = 500 - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1); // acquisizione dati in ingresso dal GSM
	size4G = RicMsg(&rx4G[0],&messaggioRec4G[0],uartPos4G,uartPosOld4G,500);
	uartPosOld4G = uartPos4G;
	
	inviaDebug("messaggio:\n");
		
	HAL_UART_Transmit(&huart1,messaggioRec4G,strlen(messaggioRec4G),100);
	HAL_UART_Transmit(&huart1,(u8*)"\n",1,100);
	
	//delay(200);
	
	risposteGSM(messaggioRec4G);
	
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
}


//pulisci messaggio in arrivo salvato nel DMA
/*int RicMsg(uint8_t* BuffIn, uint8_t* BuffOut, int pos, int oldpos,int sizeArray){

	int i = 0;
	
	while(oldpos != pos){
		BuffOut[i] = BuffIn[oldpos];
		i++;
		if(oldpos == (sizeArray-1)){
			oldpos = 0;
		}
		else{
			oldpos++;
		}
	}
	return i;
}*/

int RicMsg(uint8_t* BuffIn, uint8_t* BuffOut, int pos, int oldpos, int sizeArray)
{
    int i = 0;

    // Copia finché raggiungi pos, ma senza uscire da BuffOut
    while (oldpos != pos && i < (sizeArray - 1)) {   // -1 per terminatore
        BuffOut[i++] = BuffIn[oldpos];

        oldpos++;
        if (oldpos >= sizeArray) oldpos = 0;
    }

    BuffOut[i] = '\0';   // rende BuffOut una stringa C valida
    return i;
}



