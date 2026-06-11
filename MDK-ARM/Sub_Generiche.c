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
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;

extern int netCount;

extern u8 emergenza;


//conto gli impulsi del modulo 4G
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
	netCount++;
	//inviaDebug("ciao\n");
  /* USER CODE END EXTI2_IRQn 1 */
}


u8 cercaStringa(u8* stringa1, u8* stringa2, int size, u8* pointer){
	u8 esito = 0;
	u8 limite = 0;
	int i = 0;
	
	limite = strlen(stringa1)-size;
	
	while(i<limite && esito == 0){
		esito = comparaStringhe(&stringa1[i],&stringa2[0],size);
		i++;
	}
	
	if(pointer != NULL){
		*pointer = i-1;
	}
	return esito;
	
}

//compara due stringhe
uint8_t comparaStringhe(uint8_t *stringa1, uint8_t *stringa2, int size){
	uint8_t risultato = 1;
	int i = 0;
	
	while(i<size){
		if(stringa1[i] != stringa2[i]){
			risultato = 0;
			i = size;
		}
		i++;
	}
	
	return risultato;	//ritorna 1 se sono uguali, 0 se sono diverse
}

void cambioNomeBTfunction(u8* stringa){
	u8 comando[30];

	HAL_UART_Transmit(&huart2,(u8*)"$$$",3,100);
	HAL_Delay(100);
	sprintf(comando,"SN,%s\r",stringa);
	HAL_UART_Transmit(&huart2,comando,strlen(comando),100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart2,(u8*)"R,1\r",4,100);
	HAL_Delay(100);
}

//delay(200) è circa 1 secondo
void delay(long dato){
	long i = 10000;
	
	dato *= 2;
	while(dato>0){
		resetWD();
		while(i>0){
			i--;
		}
		i=10000;
		dato--;
	}
}

//copia un array in un altro
void copiaArray(uint8_t *OutBuf, uint8_t *InBuf, int size){
	int i = 0;
	
	while(i<size){
		OutBuf[i] = InBuf[i];
		i++;
	}
}


long array2u32(uint8_t *inBuf){
	uint32_t buffer[5] = {0,0,0,0,0};
	uint32_t outVar;
	
	buffer[0] = inBuf[0]; buffer[0] = buffer[0]<<24;
	buffer[1] = inBuf[1]; buffer[1] = buffer[1]<<16;
	buffer[2] = inBuf[2]; buffer[2] = buffer[2]<<8;
	buffer[3] = inBuf[3];
	
	buffer[4] = buffer[0] | buffer[1] | buffer[2] | buffer[3];
	
	outVar = buffer[4];
	
	return outVar;

}

//u32 to array
void u322array(uint8_t *outBuf,uint32_t inVar){
	uint32_t buffer;
	
	buffer = (inVar & 0xff000000)>>24;
	outBuf[0] = buffer;
	buffer = (inVar & 0x00ff0000)>>16;
	outBuf[1] = buffer;
	buffer = (inVar & 0x0000ff00)>>8;
	outBuf[2] = buffer;
	buffer = (inVar & 0x000000ff);
	outBuf[3] = buffer;
}

//u16 to array
void u162array(uint8_t *outBuf,uint16_t inVar){
	uint16_t buffer;
	
	buffer = (inVar & 0xff00)>>8;
	outBuf[0] = buffer;
	buffer = (inVar & 0x00ff);
	outBuf[1] = buffer;	
}


//array to u16
u16 array2u16(uint8_t *inBuf){
	uint16_t buffer[3] = {0,0,0};
	uint16_t outVar;
	
	buffer[0] = inBuf[0]; buffer[0] = buffer[0]<<8;
	buffer[1] = inBuf[1];

	
	buffer[2] = buffer[0] | buffer[1];
	
	outVar = buffer[2];
	
	return outVar;

}



//valore assoluto variabile long
long absLong(long value){
	
	if(value < 0){
		value = -value;
	}
	
	return value;
}


//codice comando
uint8_t codiceComando(uint8_t *inBuf){
	uint8_t comando;
	comando = ascii2byte(inBuf[0])*16 + ascii2byte(inBuf[1]);
	
	return comando;	
}

//ascii to byte
uint8_t ascii2byte(uint8_t inVar){
	if(inVar < 60){
		inVar -= 48;
	}
	else if(inVar > 60 && inVar < 95) {
		inVar -= 55;		
	}
	else{
		inVar -= 87;
	}
	return inVar;
}

//controllo dispari
u8 dispari(int dato){
	int compara;
	u8 flag = 0;
	
	compara = dato;
	dato /= 2;
	dato *= 2;
	
	if(compara != dato){
		flag = 1;
	}
	
	return flag;
}

//controllo multiplo di tre
u8 multiplotre(int dato){
	u8 flag = 0;
	int compara;
	
	compara = dato;
	dato /= 3;
	dato *= 3;
	
	if(compara == dato){
		flag = 1;
	}
	
	return flag;
}

//controllo multiplo di cinque
u8 multiplocinque(int dato){
	u8 flag = 0;
	int compara;
	
	compara = dato;
	dato /= 5;
	dato *= 5;
	
	if(dato == compara){
		flag = 1;
	}

	return flag;
}

//controllo multiplo di dieci
u8 multiplodieci(int dato){
	u8 flag = 0;
	int compara;
	
	compara = dato;
	dato /= 10;
	dato *= 10;
	
	if(dato == compara){
		flag = 1;
	}

	return flag;
}

//controllo multiplo di trenta
u8 multiplotrenta(int dato){
	u8 flag = 0;
	int compara;
	
	compara = dato;
	dato /= 30;
	dato *= 30;
	
	if(dato == compara){
		flag = 1;
	}

	return flag;
}



//array to int
int array2int(uint8_t *inBuf){
	uint16_t buffer[3] = {0,0,0};
	int outVar;
	
	buffer[0] = inBuf[0]; buffer[0] = buffer[0]<<8;
	buffer[1] = inBuf[1];

	
	buffer[2] = buffer[0] | buffer[1];
	
	outVar = buffer[2];
	
	return outVar;

}


//array to long
long array2long(uint8_t *inBuf){
	uint32_t buffer[5] = {0,0,0,0,0};
	long outVar;
	
	buffer[0] = inBuf[0]; buffer[0] = buffer[0]<<24;
	buffer[1] = inBuf[1]; buffer[1] = buffer[1]<<16;
	buffer[2] = inBuf[2]; buffer[2] = buffer[2]<<8;
	buffer[3] = inBuf[3];
	
	buffer[4] = buffer[0] | buffer[1] | buffer[2] | buffer[3];
	
	outVar = buffer[4];
	
	return outVar;

}



//da stringa di ascii ad array di byte (per GSM)
void byte2string(u8 *inByte, u8 *outString, int len){
	
	int a = 0;
	u8 hchar,lchar;	
	
	while(a	<	len*2){
		hchar = (inByte[a/2] & 0xf0) >> 4;
		lchar = inByte[a/2] & 0x0f;
		
		outString[a] = byte2char(hchar);
		outString[a+1] = byte2char(lchar);
		
		a += 2;		
	}
}



u8 byte2char(u8 byte){
	
	if(byte < 10){
		byte +=	48;
	}
	else{
		byte += 55;
	}
	
	return byte;

}



void string2byte(u8 *inString, u8* outByte, int len){
	int a = 0;
	u8 hchar,lchar;
	
	
	while(a < len){
		hchar = ascii2byte(inString[a]);
		lchar = ascii2byte(inString[a+1]);
		
		outByte[a/2] = hchar;
		outByte[a/2] = outByte[a/2] << 4;
		outByte[a/2] = outByte[a/2] | lchar;
		
		a += 2;	
	}

}



//inversione array
void invertiArray(uint8_t *outBuf, uint16_t size){
	int i = 0;
	uint8_t inBuf[size];
	
	while(i<size){
		inBuf[i] = outBuf[i];
		i++;
	}
	i = 0;	
	while(i<size){
		outBuf[i] = inBuf[size-i-1];
		i++;
	}
	
}



u32 acquisizioneADC(int channel){
	ADC_ChannelConfTypeDef sConfig;
	u32 acquisizione = 0;
	
switch(channel){
	case 0:
		sConfig.Channel = ADC_CHANNEL_0;
		break;
	case 1:
		sConfig.Channel = ADC_CHANNEL_1;
		break;
	case 2:
		sConfig.Channel = ADC_CHANNEL_2;
		break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_3;
		break;
	case 4:
		sConfig.Channel = ADC_CHANNEL_4;
		break;
	case 5:
		sConfig.Channel = ADC_CHANNEL_5;
		break;
	case 6:
		sConfig.Channel = ADC_CHANNEL_6;
		break;
	case 7:
		sConfig.Channel = ADC_CHANNEL_7;
		break;
	case 8:
		sConfig.Channel = ADC_CHANNEL_8;
		break;
	case 9:
		sConfig.Channel = ADC_CHANNEL_9;
		break;
}
	
		


					//sConfig.Channel = ADC_CHANNEL_0;
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
					
	return acquisizione;
	
}



void rebootSMS(void){
	
	inviaDebug((u8*)"riavvio");
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	inviaDebug((u8*)"riavvio effettivo");
	
	HAL_TIM_Base_Stop(&htim4);
	emergenza = 3;
}


void inviaDebug(u8* messaggio){

	//u8 uart[150];
	HAL_UART_Transmit(&huart1,messaggio,strlen(messaggio),100);
	return;
	
	
}

void invia4G(u8* messaggio){

	//u8 uart[150];
	HAL_UART_Transmit(&huart6,messaggio,strlen(messaggio),100);
	return;
	
	
}


