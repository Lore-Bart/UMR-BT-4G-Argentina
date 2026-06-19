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
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;


u8 aggiungiTensioniDBflag = 0;
u8 aggiungiMeasProfileDBflag = 0;
u8 aggiungiLoadProfileDBflag = 0;
u8 aggiungiGuastoDBflag = 0;
u8 aggiungiUnderDBflag = 0;
u8 aggiungiOverDBflag = 0;
u8 aggiungiNeutroStartDBflag = 0;
u8 aggiungiNeutroEndDBflag = 0;
u8 aggiungiIntrusioneDBflag = 0;
u8 aggiungiRebootDBflag = 0;
u8 aggiungiDebugDBflag = 0;

void clearDatabaseRequests(void){
	aggiungiTensioniDBflag = 0;
	aggiungiMeasProfileDBflag = 0;
	aggiungiLoadProfileDBflag = 0;
	aggiungiGuastoDBflag = 0;
	aggiungiUnderDBflag = 0;
	aggiungiOverDBflag = 0;
	aggiungiNeutroStartDBflag = 0;
	aggiungiNeutroEndDBflag = 0;
	aggiungiIntrusioneDBflag = 0;
	aggiungiRebootDBflag = 0;
	aggiungiDebugDBflag = 0;
}



//internet
extern u8 statoInternet;
extern u8 APN[50]; //vecchio 30
extern u8 mySQL[100]; //vecchio 50
extern u8 userSQL[30]; //vecchio 20
extern u8 pwSQL[30]; //vecchio 20
extern u8 userAPN[30];
extern u8 pwAPN[30];

extern u32 myTimeVar;
extern u32 V[3];
extern u8 identificativo[16];

extern u8 statoModulo;
extern u8 univoco[5];
extern long temperatura;
extern int tensioneBint;
extern u8 batteriaInCarica;
extern u8 batteryLevel;

//variabili misurandi
extern u32 V[3]; //tensioni
extern long I1[3],I2[3]; //correnti
extern uint16_t phi1[3],phi2[3]; //phi
extern long P1[3],P2[3]; //potenze attive
extern long Q1[3],Q2[3]; //potenze reattive
extern int cosphi1[3],cosphi2[3]; //cosfi
extern long I1meas[3],I2meas[3];
extern uint16_t phi1meas[3],phi2meas[3];
extern long P1meas[3],P2meas[3]; 
extern long Q1meas[3],Q2meas[3];
extern int cosphi1meas[3],cosphi2meas[3];

//variabili energie
extern uint32_t E1p[3],E2p[3]; //energie attive positive
extern uint32_t E1n[3],E2n[3]; //energie attive negative
extern uint32_t R1p[3],R2p[3]; //energie reattive positive
extern uint32_t R1n[3],R2n[3]; //energie reattive negative

extern u8 updateGSMatt;

/*
 * Buffer per richieste DB generate da eventi/RTC.
 * Le funzioni chiamate con passaggio != 0 non usano piu' direttamente il modem:
 * salvano i dati e accodano la richiesta. Il main loop la invia solo quando
 * internet e' connesso e statoModulo e' libero.
 */
static u32 mysqlPendingFaultCurrent[6];
static u32 mysqlPendingUnderVoltage[3];
static u32 mysqlPendingOverVoltage[3];
static u16 mysqlPendingNeutralStartDiff[3];
static u16 mysqlPendingNeutralEndMax[3];
static u16 mysqlPendingNeutralEndEnd[3];
static u32 mysqlPendingNeutralEndTimeMax = 0;

static int mysqlCanSend(void)
{
	if(updateGSMatt != 0) return 0;
	if(statoInternet != 3) return 0;
	if(statoModulo != 0) return 0;
	if(mySQL[0] == 0 || userSQL[0] == 0 || pwSQL[0] == 0) return 0;
	return 1;
}


void aggiungiIntrusioneDB(u8 passaggio){
	
	u8 stringaIntrusioneDB[250];
	
	int i = 0;
	
	if(passaggio != 0){
		aggiungiIntrusioneDBflag = 1;
		return;
	}

	if(mysqlCanSend() == 0){
		return;
	}
	
		while(i<150){
			stringaIntrusioneDB[i] = 0;
			i++;
			}
		i = 0;
		
		statoModulo++; inviaDebug("statoModulo++\n");
		sprintf(stringaIntrusioneDB,"AT+HTTPPARA=\x22URL\x22,\x22");
			
		copiaArray(&stringaIntrusioneDB[strlen(stringaIntrusioneDB)],&mySQL[0],strlen(mySQL));
		sprintf(&stringaIntrusioneDB[strlen(stringaIntrusioneDB)],"/addIntrusion.php?user=");
		copiaArray(&stringaIntrusioneDB[strlen(stringaIntrusioneDB)],&userSQL[0],strlen(userSQL));
		sprintf(&stringaIntrusioneDB[strlen(stringaIntrusioneDB)],"&psw=");
		copiaArray(&stringaIntrusioneDB[strlen(stringaIntrusioneDB)],&pwSQL[0],strlen(pwSQL));
			
		sprintf(&stringaIntrusioneDB[strlen(stringaIntrusioneDB)],"&serial=");
		copiaArray(&stringaIntrusioneDB[strlen(stringaIntrusioneDB)],&univoco[0],5);	
		
		sprintf(&stringaIntrusioneDB[strlen(stringaIntrusioneDB)],"&name=");
		while(identificativo[i] != ' ' && i < 16){
			stringaIntrusioneDB[strlen(stringaIntrusioneDB)] = identificativo[i];
			i++;
		}
		sprintf(&stringaIntrusioneDB[strlen(stringaIntrusioneDB)],"&time=%u",myTimeVar);  
		stringaIntrusioneDB[strlen(stringaIntrusioneDB)] = '\x22';
		stringaIntrusioneDB[strlen(stringaIntrusioneDB)] = '\r';
		HAL_UART_Transmit(&huart6,stringaIntrusioneDB,strlen(stringaIntrusioneDB),1000);
		delay(100);
	
		HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
			
}


void aggiungiMeasProfileDB(u8 passaggio){
	
	
	u8 stringaMeasDB[900];
	
	int i = 0;
	
	if(passaggio != 0){
		aggiungiMeasProfileDBflag = 1;
		return;
	}

	if(mysqlCanSend() == 0){
		return;
	}
	
		while(i<500){
			stringaMeasDB[i] = 0;
			i++;
		}
		i = 0;
		
		inviaDebug("entrato MEAS\n");
		
		statoModulo++; inviaDebug("statoModulo++\n");
		
		sprintf(stringaMeasDB,"AT+HTTPPARA=\x22URL\x22,\x22");
		
		copiaArray(&stringaMeasDB[strlen(stringaMeasDB)],&mySQL[0],strlen(mySQL));
		sprintf(&stringaMeasDB[strlen(stringaMeasDB)],"/addMeasProfile.php?user=");
		copiaArray(&stringaMeasDB[strlen(stringaMeasDB)],&userSQL[0],strlen(userSQL));
		sprintf(&stringaMeasDB[strlen(stringaMeasDB)],"&psw=");
		copiaArray(&stringaMeasDB[strlen(stringaMeasDB)],&pwSQL[0],strlen(pwSQL));
		
		sprintf(&stringaMeasDB[strlen(stringaMeasDB)],"&serial=");
		copiaArray(&stringaMeasDB[strlen(stringaMeasDB)],&univoco[0],5);	
		
		sprintf(&stringaMeasDB[strlen(stringaMeasDB)],"&name=");
		while(identificativo[i] != ' ' && i < 16){
			stringaMeasDB[strlen(stringaMeasDB)] = identificativo[i];
			i++;
		}
		sprintf(&stringaMeasDB[strlen(stringaMeasDB)],"&time=%u&V1=%u&V2=%u&V3=%u&I1a=%d&I2a=%d&I3a=%d&I1b=%d&I2b=%d&I3b=%d&f1a=%d&f2a=%d&f3a=%d&f1b=%d&f2b=%d&f3b=%d&pf1a=%d&pf2a=%d&pf3a=%d&pf1b=%d&pf2b=%d&pf3b=%d&P1a=%d&P2a=%d&P3a=%d&P1b=%d&P2b=%d&P3b=%d&Q1a=%d&Q2a=%d&Q3a=%d&Q1b=%d&Q2b=%d&Q3b=%d",myTimeVar,V[0],V[1],V[2],I1meas[0],I1meas[1],I1meas[2],I2meas[0],I2meas[1],I2meas[2],phi1meas[0],phi1meas[1],phi1meas[2],phi2meas[0],phi2meas[1],phi2meas[2],cosphi1meas[0],cosphi1meas[1],cosphi1meas[2],cosphi2meas[0],cosphi2meas[1],cosphi2meas[2],P1meas[0],P1meas[1],P1meas[2],P2meas[0],P2meas[1],P2meas[2],Q1meas[0],Q1meas[1],Q1meas[2],Q2meas[0],Q2meas[1],Q2meas[2]);               
		stringaMeasDB[strlen(stringaMeasDB)] = '\x22';
		stringaMeasDB[strlen(stringaMeasDB)] = '\r';
		HAL_UART_Transmit(&huart6,stringaMeasDB,strlen(stringaMeasDB),1000);
		delay(100);
		
		HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
		
}


void aggiungiLoadProfileDB(u8 passaggio){

	u8 stringaLoadDB[900];
	
	int i = 0;
	
	if(passaggio != 0){
		aggiungiLoadProfileDBflag = 1;
		return;
	}

	if(mysqlCanSend() == 0){
		return;
	}
		
		while(i<500){
			stringaLoadDB[i] = 0;
			i++;
			}
		i = 0;
			
		statoModulo++; inviaDebug("statoModulo++\n");
			
		sprintf(stringaLoadDB,"AT+HTTPPARA=\x22URL\x22,\x22");
	
		copiaArray(&stringaLoadDB[strlen(stringaLoadDB)],&mySQL[0],strlen(mySQL));
		sprintf(&stringaLoadDB[strlen(stringaLoadDB)],"/addLoadProfile.php?user=");
		copiaArray(&stringaLoadDB[strlen(stringaLoadDB)],&userSQL[0],strlen(userSQL));
		sprintf(&stringaLoadDB[strlen(stringaLoadDB)],"&psw=");
		copiaArray(&stringaLoadDB[strlen(stringaLoadDB)],&pwSQL[0],strlen(pwSQL));
			
		sprintf(&stringaLoadDB[strlen(stringaLoadDB)],"&serial=");
		copiaArray(&stringaLoadDB[strlen(stringaLoadDB)],&univoco[0],5);	
		
		sprintf(&stringaLoadDB[strlen(stringaLoadDB)],"&name=");
		while(identificativo[i] != ' ' && i < 16){
			stringaLoadDB[strlen(stringaLoadDB)] = identificativo[i];
			i++;
		}
		sprintf(&stringaLoadDB[strlen(stringaLoadDB)],"&time=%u&E1p1=%lu&E2p1=%lu&E3p1=%lu&E1p2=%lu&E2p2=%lu&E3p2=%lu&E1n1=%lu&E2n1=%lu&E3n1=%lu&E1n2=%lu&E2n2=%lu&E3n2=%lu&R1p1=%lu&R2p1=%lu&R3p1=%lu&R1p2=%lu&R2p2=%lu&R3p2=%lu&R1n1=%lu&R2n1=%lu&R3n1=%lu&R1n2=%lu&R2n2=%lu&R3n2=%lu",myTimeVar,E1p[0],E1p[1],E1p[2],E2p[0],E2p[1],E2p[2],E1n[0],E1n[1],E1n[2],E2n[0],E2n[1],E2n[2],R1p[0],R1p[1],R1p[2],R2p[0],R2p[1],R2p[2],R1n[0],R1n[1],R1n[2],R2n[0],R2n[1],R2n[2]);               
		stringaLoadDB[strlen(stringaLoadDB)] = '\x22';
		stringaLoadDB[strlen(stringaLoadDB)] = '\r';
		HAL_UART_Transmit(&huart6,stringaLoadDB,strlen(stringaLoadDB),1000);
		delay(100);
		
		HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);

}


void aggiungiGuastoDB(u8 passaggio, u32* corrente){

	u8 stringaGuastoDB[300];
	
	int i = 0;
	
	if(passaggio != 0){
		for(i=0;i<6;i++){ mysqlPendingFaultCurrent[i] = corrente[i]; }
		aggiungiGuastoDBflag = 1;
		return;
	}

	corrente = &mysqlPendingFaultCurrent[0];
	i = 0;

	if(mysqlCanSend() == 0){
		return;
	}
	
		while(i<150){
			stringaGuastoDB[i] = 0;
			i++;
			}
		i = 0;
			
		statoModulo++; inviaDebug("statoModulo++\n");
			
		sprintf(stringaGuastoDB,"AT+HTTPPARA=\x22URL\x22,\x22");
			
		copiaArray(&stringaGuastoDB[strlen(stringaGuastoDB)],&mySQL[0],strlen(mySQL));
		sprintf(&stringaGuastoDB[strlen(stringaGuastoDB)],"/addFault.php?user=");
		copiaArray(&stringaGuastoDB[strlen(stringaGuastoDB)],&userSQL[0],strlen(userSQL));
		sprintf(&stringaGuastoDB[strlen(stringaGuastoDB)],"&psw=");
		copiaArray(&stringaGuastoDB[strlen(stringaGuastoDB)],&pwSQL[0],strlen(pwSQL));
			
		sprintf(&stringaGuastoDB[strlen(stringaGuastoDB)],"&serial=");
		copiaArray(&stringaGuastoDB[strlen(stringaGuastoDB)],&univoco[0],5);	
		
		sprintf(&stringaGuastoDB[strlen(stringaGuastoDB)],"&name=");
		while(identificativo[i] != ' ' && i < 16){
			stringaGuastoDB[strlen(stringaGuastoDB)] = identificativo[i];
			i++;
		}
		
		sprintf(&stringaGuastoDB[strlen(stringaGuastoDB)],"&time=%u&I1a=%u&I2a=%u&I3a=%u&I1b=%u&I2b=%u&I3b=%u",myTimeVar,corrente[0],corrente[1],corrente[2],corrente[3],corrente[4],corrente[5]);  
		stringaGuastoDB[strlen(stringaGuastoDB)] = '\x22';
		stringaGuastoDB[strlen(stringaGuastoDB)] = '\r';
		HAL_UART_Transmit(&huart6,stringaGuastoDB,strlen(stringaGuastoDB),1000);
		delay(100);
		
		HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
		
}


void aggiungiUnderDB(u8 passaggio, u32* tensione){
	
	u8 stringaUnderDB[300];
	
	int i = 0;

	if(passaggio != 0){
		for(i=0;i<3;i++){ mysqlPendingUnderVoltage[i] = tensione[i]; }
		aggiungiUnderDBflag = 1;
		return;
	}

	tensione = &mysqlPendingUnderVoltage[0];
	i = 0;

	if(mysqlCanSend() == 0){
		return;
	}	
	
		while(i<150){
			stringaUnderDB[i] = 0;
			i++;
			}
		i = 0;
		
		statoModulo++; inviaDebug("statoModulo++\n");
			
		sprintf(stringaUnderDB,"AT+HTTPPARA=\x22URL\x22,\x22");
			
		copiaArray(&stringaUnderDB[strlen(stringaUnderDB)],&mySQL[0],strlen(mySQL));
		sprintf(&stringaUnderDB[strlen(stringaUnderDB)],"/addUnderVoltage.php?user=");
		copiaArray(&stringaUnderDB[strlen(stringaUnderDB)],&userSQL[0],strlen(userSQL));
		sprintf(&stringaUnderDB[strlen(stringaUnderDB)],"&psw=");
		copiaArray(&stringaUnderDB[strlen(stringaUnderDB)],&pwSQL[0],strlen(pwSQL));
			
		sprintf(&stringaUnderDB[strlen(stringaUnderDB)],"&serial=");
		copiaArray(&stringaUnderDB[strlen(stringaUnderDB)],&univoco[0],5);	
		
		sprintf(&stringaUnderDB[strlen(stringaUnderDB)],"&name=");
		while(identificativo[i] != ' ' && i < 16){
			stringaUnderDB[strlen(stringaUnderDB)] = identificativo[i];
			i++;
		}
		
		sprintf(&stringaUnderDB[strlen(stringaUnderDB)],"&time=%u&V1=%u&V2=%u&V3=%u",myTimeVar,tensione[0],tensione[1],tensione[2]);  
		stringaUnderDB[strlen(stringaUnderDB)] = '\x22';
		stringaUnderDB[strlen(stringaUnderDB)] = '\r';
		HAL_UART_Transmit(&huart6,stringaUnderDB,strlen(stringaUnderDB),1000);
		delay(100);
		
		HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
	
}

void aggiungiOverDB(u8 passaggio, u32* tensione){

	u8 stringaOverDB[300];
	
	int i = 0;

	if(passaggio != 0){
		for(i=0;i<3;i++){ mysqlPendingOverVoltage[i] = tensione[i]; }
		aggiungiOverDBflag = 1;
		return;
	}

	tensione = &mysqlPendingOverVoltage[0];
	i = 0;

	if(mysqlCanSend() == 0){
		return;
	}
	
		while(i<150){
			stringaOverDB[i] = 0;
			i++;
			}
		i = 0;
			
		statoModulo++; inviaDebug("statoModulo++\n");
			
		sprintf(stringaOverDB,"AT+HTTPPARA=\x22URL\x22,\x22");
			
		copiaArray(&stringaOverDB[strlen(stringaOverDB)],&mySQL[0],strlen(mySQL));
		sprintf(&stringaOverDB[strlen(stringaOverDB)],"/addOverVoltage.php?user=");
		copiaArray(&stringaOverDB[strlen(stringaOverDB)],&userSQL[0],strlen(userSQL));
		sprintf(&stringaOverDB[strlen(stringaOverDB)],"&psw=");
		copiaArray(&stringaOverDB[strlen(stringaOverDB)],&pwSQL[0],strlen(pwSQL));
			
		sprintf(&stringaOverDB[strlen(stringaOverDB)],"&serial=");
		copiaArray(&stringaOverDB[strlen(stringaOverDB)],&univoco[0],5);	
		
		sprintf(&stringaOverDB[strlen(stringaOverDB)],"&name=");
		while(identificativo[i] != ' ' && i < 16){
			stringaOverDB[strlen(stringaOverDB)] = identificativo[i];
			i++;
		}
		
		sprintf(&stringaOverDB[strlen(stringaOverDB)],"&time=%u&V1=%u&V2=%u&V3=%u",myTimeVar,tensione[0],tensione[1],tensione[2]);  
		stringaOverDB[strlen(stringaOverDB)] = '\x22';
		stringaOverDB[strlen(stringaOverDB)] = '\r';
		HAL_UART_Transmit(&huart6,stringaOverDB,strlen(stringaOverDB),1000);
		delay(100);
		
		HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
}


void aggiungiNeutroStartDB(u8 passaggio, u16* diffStart){

	u8 stringaNeutroStartDB[300];
	int i = 0;

	if(passaggio != 0){
		for(i=0;i<3;i++){ mysqlPendingNeutralStartDiff[i] = diffStart[i]; }
		aggiungiNeutroStartDBflag = 1;
		return;
	}

	diffStart = &mysqlPendingNeutralStartDiff[0];
	i = 0;

	if(mysqlCanSend() == 0){
		return;
	}
	
	
		while(i<150){
			stringaNeutroStartDB[i] = 0;
			i++;
			}
		i = 0;
		
		statoModulo++; inviaDebug("statoModulo++\n");
			
		sprintf(stringaNeutroStartDB,"AT+HTTPPARA=\x22URL\x22,\x22");
		
		copiaArray(&stringaNeutroStartDB[strlen(stringaNeutroStartDB)],&mySQL[0],strlen(mySQL));
		sprintf(&stringaNeutroStartDB[strlen(stringaNeutroStartDB)],"/addNeutralStart.php?user=");
		copiaArray(&stringaNeutroStartDB[strlen(stringaNeutroStartDB)],&userSQL[0],strlen(userSQL));
		sprintf(&stringaNeutroStartDB[strlen(stringaNeutroStartDB)],"&psw=");
		copiaArray(&stringaNeutroStartDB[strlen(stringaNeutroStartDB)],&pwSQL[0],strlen(pwSQL));
			
		sprintf(&stringaNeutroStartDB[strlen(stringaNeutroStartDB)],"&serial=");
		copiaArray(&stringaNeutroStartDB[strlen(stringaNeutroStartDB)],&univoco[0],5);	
		
		sprintf(&stringaNeutroStartDB[strlen(stringaNeutroStartDB)],"&name=");
		while(identificativo[i] != ' ' && i < 16){
			stringaNeutroStartDB[strlen(stringaNeutroStartDB)] = identificativo[i];
			i++;
		}
		
		sprintf(&stringaNeutroStartDB[strlen(stringaNeutroStartDB)],"&timeS=%u&d1s=%u&d2s=%u&d3s=%u",myTimeVar,diffStart[0],diffStart[1],diffStart[2]);  
		stringaNeutroStartDB[strlen(stringaNeutroStartDB)] = '\x22';
		stringaNeutroStartDB[strlen(stringaNeutroStartDB)] = '\r';
		HAL_UART_Transmit(&huart6,stringaNeutroStartDB,strlen(stringaNeutroStartDB),1000);
		delay(100);
		
		HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
}


void aggiungiNeutroEndDB(u8 passaggio, u32 timeMax, u16* diffMax, u16* diffEnd){

	u8 stringaNeutroEndDB[350];
	int i = 0;

	if(passaggio != 0){
		mysqlPendingNeutralEndTimeMax = timeMax;
		for(i=0;i<3;i++){
			mysqlPendingNeutralEndMax[i] = diffMax[i];
			mysqlPendingNeutralEndEnd[i] = diffEnd[i];
		}
		aggiungiNeutroEndDBflag = 1;
		return;
	}

	timeMax = mysqlPendingNeutralEndTimeMax;
	diffMax = &mysqlPendingNeutralEndMax[0];
	diffEnd = &mysqlPendingNeutralEndEnd[0];
	i = 0;

	if(mysqlCanSend() == 0){
		return;
	}
	
	
		while(i<200){
			stringaNeutroEndDB[i] = 0;
			i++;
			}
		i = 0;
			
		statoModulo++; inviaDebug("statoModulo++\n");
			
		sprintf(stringaNeutroEndDB,"AT+HTTPPARA=\x22URL\x22,\x22");
			
		copiaArray(&stringaNeutroEndDB[strlen(stringaNeutroEndDB)],&mySQL[0],strlen(mySQL));
		sprintf(&stringaNeutroEndDB[strlen(stringaNeutroEndDB)],"/addNeutralEnd.php?user=");
		copiaArray(&stringaNeutroEndDB[strlen(stringaNeutroEndDB)],&userSQL[0],strlen(userSQL));
		sprintf(&stringaNeutroEndDB[strlen(stringaNeutroEndDB)],"&psw=");
		copiaArray(&stringaNeutroEndDB[strlen(stringaNeutroEndDB)],&pwSQL[0],strlen(pwSQL));
		
		sprintf(&stringaNeutroEndDB[strlen(stringaNeutroEndDB)],"&serial=");
		copiaArray(&stringaNeutroEndDB[strlen(stringaNeutroEndDB)],&univoco[0],5);	
		
		sprintf(&stringaNeutroEndDB[strlen(stringaNeutroEndDB)],"&name=");
		while(identificativo[i] != ' ' && i < 16){
			stringaNeutroEndDB[strlen(stringaNeutroEndDB)] = identificativo[i];
			i++;
		}
		sprintf(&stringaNeutroEndDB[strlen(stringaNeutroEndDB)],"&timeM=%u&d1m=%u&d2m=%u&d3m=%u&timeE=%u&d1e=%u&d2e=%u&d3e=%u",timeMax, diffMax[0], diffMax[1], diffMax[2], myTimeVar, diffEnd[0], diffEnd[1], diffEnd[2]);  
		stringaNeutroEndDB[strlen(stringaNeutroEndDB)] = '\x22';
		stringaNeutroEndDB[strlen(stringaNeutroEndDB)] = '\r';
		HAL_UART_Transmit(&huart6,stringaNeutroEndDB,strlen(stringaNeutroEndDB),1000);
		delay(100);
		
		HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
}


void aggiungiRebootDB(u8 passaggio){

	u8 stringaRebootDB[250];
	
	int i = 0;

	if(passaggio != 0){
		aggiungiRebootDBflag = 1;
		return;
	}

	if(mysqlCanSend() == 0){
		return;
	}	

		while(i<150){
			stringaRebootDB[i] = 0;
			i++;
			}
		i = 0;
		
		statoModulo++; inviaDebug("statoModulo++\n");
			
		sprintf(stringaRebootDB,"AT+HTTPPARA=\x22URL\x22,\x22");			
			
		copiaArray(&stringaRebootDB[strlen(stringaRebootDB)],&mySQL[0],strlen(mySQL));
		sprintf(&stringaRebootDB[strlen(stringaRebootDB)],"/addReboot.php?user=");
		copiaArray(&stringaRebootDB[strlen(stringaRebootDB)],&userSQL[0],strlen(userSQL));
		sprintf(&stringaRebootDB[strlen(stringaRebootDB)],"&psw=");
		copiaArray(&stringaRebootDB[strlen(stringaRebootDB)],&pwSQL[0],strlen(pwSQL));
			
		sprintf(&stringaRebootDB[strlen(stringaRebootDB)],"&serial=");
		copiaArray(&stringaRebootDB[strlen(stringaRebootDB)],&univoco[0],5);	
		
		sprintf(&stringaRebootDB[strlen(stringaRebootDB)],"&name=");
		while(identificativo[i] != ' ' && i < 16){
			stringaRebootDB[strlen(stringaRebootDB)] = identificativo[i];
			i++;
		}
		
		sprintf(&stringaRebootDB[strlen(stringaRebootDB)],"&time=%u",myTimeVar-5);  
		stringaRebootDB[strlen(stringaRebootDB)] = '\x22';
		stringaRebootDB[strlen(stringaRebootDB)] = '\r';
		HAL_UART_Transmit(&huart6,stringaRebootDB,strlen(stringaRebootDB),1000);
		delay(100);
		
		HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
	
}



void aggiungiDebugDB(u8 passaggio){

	u8 stringaDebugDB[300];
	
	int i = 0;

	if(passaggio != 0){
		aggiungiDebugDBflag = 1;
		return;
	}

	if(mysqlCanSend() == 0){
		return;
	}
	
		while(i<150){
			stringaDebugDB[i] = 0;
			i++;
			}
		i = 0;
			
		statoModulo++; inviaDebug("statoModulo++\n");
			
		sprintf(stringaDebugDB,"AT+HTTPPARA=\x22URL\x22,\x22");	
			
		sprintf(&stringaDebugDB[strlen(stringaDebugDB)],"http://a2atestmisure.altervista.org");
		sprintf(&stringaDebugDB[strlen(stringaDebugDB)],"/addDebug.php?user=");
		copiaArray(&stringaDebugDB[strlen(stringaDebugDB)],&userSQL[0],strlen(userSQL));
		sprintf(&stringaDebugDB[strlen(stringaDebugDB)],"&psw=");
		copiaArray(&stringaDebugDB[strlen(stringaDebugDB)],&pwSQL[0],strlen(pwSQL));
			
		sprintf(&stringaDebugDB[strlen(stringaDebugDB)],"&serial=");
		copiaArray(&stringaDebugDB[strlen(stringaDebugDB)],&univoco[0],5);	
		
		sprintf(&stringaDebugDB[strlen(stringaDebugDB)],"&name=");
		while(identificativo[i] != ' ' && i < 16){
			stringaDebugDB[strlen(stringaDebugDB)] = identificativo[i];
			i++;
		}
		
		sprintf(&stringaDebugDB[strlen(stringaDebugDB)],"&time=%u&temp=%d&batt=%d&level=%d&char=%d",myTimeVar,temperatura,tensioneBint,batteryLevel,batteriaInCarica);  
		stringaDebugDB[strlen(stringaDebugDB)] = '\x22';
		stringaDebugDB[strlen(stringaDebugDB)] = '\r';
		HAL_UART_Transmit(&huart6,stringaDebugDB,strlen(stringaDebugDB),1000);
		delay(100);
		
		HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
		
}