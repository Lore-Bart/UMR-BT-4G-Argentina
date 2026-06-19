#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"

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
extern u8 inviaSMSpollFlag;
extern u8 inviaSMSpollFlagCoda;
extern u8 indicationSMSflag;

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

#define DB_TX_NONE            0
#define DB_TX_INTRUSION       1
#define DB_TX_FAULT           2
#define DB_TX_UNDER           3
#define DB_TX_OVER            4
#define DB_TX_NEUTRO_START    5
#define DB_TX_NEUTRO_END      6
#define DB_TX_REBOOT          7
#define DB_TX_DEBUG           8
#define DB_TX_MEAS            9
#define DB_TX_LOAD            10

#define DB_RETRY_DELAY_MS        30000UL
#define DB_RETRY_DELAY_LONG_MS   120000UL
#define DB_MAX_RETRY             5

static u8 databaseTxBusy = 0;
static u8 databaseTxType = DB_TX_NONE;
static u8 databaseTxRetry = 0;
static u8 databaseWaitHttpParaOk = 0;
static u32 databaseNextRetryTick = 0;

static void databaseFinalizeUrl(u8 *buf, u16 maxLen)
{
	u16 len;
	if(buf == 0 || maxLen < 4){
		return;
	}
	len = (u16)strlen((char*)buf);
	if(len > (u16)(maxLen - 3)){
		/* Tronca in modo sicuro: meglio un HTTP error con retry che inviare byte sporchi. */
		len = (u16)(maxLen - 3);
	}
	buf[len++] = '\x22';
	buf[len++] = '\r';
	buf[len] = 0;
}

static void databaseSendUrl(u8 *buf, u16 maxLen)
{
	databaseFinalizeUrl(buf,maxLen);
	HAL_UART_Transmit(&huart6,buf,strlen((char*)buf),1000);
}

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
	databaseTxBusy = 0;
	databaseTxType = DB_TX_NONE;
	databaseTxRetry = 0;
	databaseWaitHttpParaOk = 0;
	databaseNextRetryTick = 0;
}

static const char* databaseTxName(u8 type)
{
	switch(type){
		case DB_TX_INTRUSION: return "intrusion";
		case DB_TX_FAULT: return "fault";
		case DB_TX_UNDER: return "undervoltage";
		case DB_TX_OVER: return "overvoltage";
		case DB_TX_NEUTRO_START: return "neutral_start";
		case DB_TX_NEUTRO_END: return "neutral_end";
		case DB_TX_REBOOT: return "reboot";
		case DB_TX_DEBUG: return "debug";
		case DB_TX_MEAS: return "meas_profile";
		case DB_TX_LOAD: return "load_profile";
		default: return "none";
	}
}

static void databaseClearFlag(u8 type)
{
	switch(type){
		case DB_TX_INTRUSION: aggiungiIntrusioneDBflag = 0; break;
		case DB_TX_FAULT: aggiungiGuastoDBflag = 0; break;
		case DB_TX_UNDER: aggiungiUnderDBflag = 0; break;
		case DB_TX_OVER: aggiungiOverDBflag = 0; break;
		case DB_TX_NEUTRO_START: aggiungiNeutroStartDBflag = 0; break;
		case DB_TX_NEUTRO_END: aggiungiNeutroEndDBflag = 0; break;
		case DB_TX_REBOOT: aggiungiRebootDBflag = 0; break;
		case DB_TX_DEBUG: aggiungiDebugDBflag = 0; break;
		case DB_TX_MEAS: aggiungiMeasProfileDBflag = 0; break;
		case DB_TX_LOAD: aggiungiLoadProfileDBflag = 0; break;
		default: break;
	}
}


static void databaseDropPending(u8 type, const char *reason)
{
	u8 uart[140];
	sprintf((char*)uart,"DB save aborted: %s after %d retries (%s)\n",databaseTxName(type),databaseTxRetry,reason);
	inviaDebug(uart);
	databaseClearFlag(type);
	databaseTxBusy = 0;
	databaseTxType = DB_TX_NONE;
	databaseTxRetry = 0;
	databaseWaitHttpParaOk = 0;
	databaseNextRetryTick = 0;
}

static void databaseScheduleRetry(const char *reason, int status)
{
	u8 uart[150];
	u8 type = databaseTxType;
	databaseTxBusy = 0;
	databaseWaitHttpParaOk = 0;
	databaseTxRetry++;

	if(databaseTxRetry >= DB_MAX_RETRY){
		databaseDropPending(type,reason);
		return;
	}

	if(databaseTxRetry > 3){
		databaseNextRetryTick = HAL_GetTick() + DB_RETRY_DELAY_LONG_MS;
	}
	else{
		databaseNextRetryTick = HAL_GetTick() + DB_RETRY_DELAY_MS;
	}

	if(status >= 0){
		sprintf((char*)uart,"DB save retry pending: %s HTTP %d retry %d/%d\n",databaseTxName(type),status,databaseTxRetry,DB_MAX_RETRY);
	}
	else{
		sprintf((char*)uart,"DB save modem error: %s retry %d/%d\n",databaseTxName(type),databaseTxRetry,DB_MAX_RETRY);
	}
	inviaDebug(uart);
}

static int databaseParseHttpStatus(u8 *messaggio)
{
	char *p;
	if(messaggio == 0) return -1;
	p = strstr((char*)messaggio,"HTTPACTION:");
	if(p == 0) return -1;
	p = strchr(p, ',');
	if(p == 0) return -1;
	p++;
	return atoi(p);
}

static void databaseTxStart(u8 type)
{
	u8 uart[90];
	databaseTxBusy = 1;
	databaseTxType = type;
	databaseWaitHttpParaOk = 1;
	sprintf((char*)uart,"DB send started: %s\n",databaseTxName(type));
	inviaDebug(uart);
}

u8 databaseTxIsBusy(void)
{
	return databaseTxBusy;
}

void databaseHttpParaOk(void)
{
	if(databaseTxBusy == 0 || databaseWaitHttpParaOk == 0){
		return;
	}
	databaseWaitHttpParaOk = 0;
	inviaDebug("DB HTTPPARA OK, HTTPACTION start\n");
	HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
}

void databaseHttpActionResult(u8 *messaggio)
{
	int status;
	u8 uart[120];
	if(databaseTxBusy == 0){
		return;
	}

	status = databaseParseHttpStatus(messaggio);
	if(status >= 200 && status < 400){
		sprintf((char*)uart,"DB save OK: %s HTTP %d\n",databaseTxName(databaseTxType),status);
		inviaDebug(uart);
		databaseClearFlag(databaseTxType);
		databaseTxBusy = 0;
		databaseTxType = DB_TX_NONE;
		databaseTxRetry = 0;
		databaseWaitHttpParaOk = 0;
		databaseNextRetryTick = 0;
	}
	else{
		databaseScheduleRetry("HTTPACTION",status);
	}
}

void databaseHttpError(void)
{
	if(databaseTxBusy == 0){
		return;
	}
	databaseScheduleRetry("HTTPPARA/timeout",-1);
}

static int databaseRetryReady(void)
{
	if(databaseNextRetryTick == 0) return 1;
	return ((int32_t)(HAL_GetTick() - databaseNextRetryTick) >= 0);
}

static int mysqlCanSend(void)
{
	if(updateGSMatt != 0) return 0;
	if(databaseTxBusy != 0) return 0;
	if(databaseRetryReady() == 0) return 0;
	if(statoInternet != 3) return 0;
	if(statoModulo != 0) return 0;
	/* Priorita' agli SMS: evitiamo che una transazione HTTP/DB parta mentre c'e'
	 * un SMS da inviare o leggere. Questo evita incroci fra AT+CMGS/AT+CMGR e HTTPACTION. */
	if(inviaSMSpollFlag != 0 || inviaSMSpollFlagCoda != 0 || indicationSMSflag != 0) return 0;
	if(mySQL[0] == 0 || userSQL[0] == 0 || pwSQL[0] == 0) return 0;
	return 1;
}

void processDatabaseRequests(void)
{
	if(mysqlCanSend() == 0){
		return;
	}

	/* Priorita' agli eventi/allarmi rispetto ai profili periodici. */
	if(aggiungiIntrusioneDBflag == 1){ aggiungiIntrusioneDB(0); return; }
	if(aggiungiGuastoDBflag == 1){ aggiungiGuastoDB(0,&mysqlPendingFaultCurrent[0]); return; }
	if(aggiungiUnderDBflag == 1){ aggiungiUnderDB(0,&mysqlPendingUnderVoltage[0]); return; }
	if(aggiungiOverDBflag == 1){ aggiungiOverDB(0,&mysqlPendingOverVoltage[0]); return; }
	if(aggiungiNeutroStartDBflag == 1){ aggiungiNeutroStartDB(0,&mysqlPendingNeutralStartDiff[0]); return; }
	if(aggiungiNeutroEndDBflag == 1){ aggiungiNeutroEndDB(0,mysqlPendingNeutralEndTimeMax,&mysqlPendingNeutralEndMax[0],&mysqlPendingNeutralEndEnd[0]); return; }
	if(aggiungiRebootDBflag == 1){ aggiungiRebootDB(0); return; }
	if(aggiungiDebugDBflag == 1){ aggiungiDebugDB(0); return; }
	if(aggiungiMeasProfileDBflag == 1){ aggiungiMeasProfileDB(0); return; }
	if(aggiungiLoadProfileDBflag == 1){ aggiungiLoadProfileDB(0); return; }
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
	
		memset(stringaIntrusioneDB,0,sizeof(stringaIntrusioneDB));
		i = 0;
		
		statoModulo++; inviaDebug("statoModulo++\n");
		databaseTxStart(DB_TX_INTRUSION);
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
		databaseSendUrl(stringaIntrusioneDB,sizeof(stringaIntrusioneDB));
			
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
	
			memset(stringaMeasDB,0,sizeof(stringaMeasDB));
			i = 0;
		
		inviaDebug("entrato MEAS\n");
		
		statoModulo++; inviaDebug("statoModulo++\n");
		databaseTxStart(DB_TX_MEAS);
		
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
		databaseSendUrl(stringaMeasDB,sizeof(stringaMeasDB));

		
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
		
		memset(stringaLoadDB,0,sizeof(stringaLoadDB));
		i = 0;
			
		statoModulo++; inviaDebug("statoModulo++\n");
		databaseTxStart(DB_TX_LOAD);
			
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
		databaseSendUrl(stringaLoadDB,sizeof(stringaLoadDB));


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
	
		memset(stringaGuastoDB,0,sizeof(stringaGuastoDB));
		i = 0;
			
		statoModulo++; inviaDebug("statoModulo++\n");
		databaseTxStart(DB_TX_FAULT);
			
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
		databaseSendUrl(stringaGuastoDB,sizeof(stringaGuastoDB));

		
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
	
			memset(stringaUnderDB,0,sizeof(stringaUnderDB));
			i = 0;
		
		statoModulo++; inviaDebug("statoModulo++\n");
		databaseTxStart(DB_TX_UNDER);
			
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
		databaseSendUrl(stringaUnderDB,sizeof(stringaUnderDB));

	
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
	
			memset(stringaOverDB,0,sizeof(stringaOverDB));
			i = 0;
			
		statoModulo++; inviaDebug("statoModulo++\n");
		databaseTxStart(DB_TX_OVER);
			
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
		databaseSendUrl(stringaOverDB,sizeof(stringaOverDB));

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
	
	
		memset(stringaNeutroStartDB,0,sizeof(stringaNeutroStartDB));
		i = 0;
		
		statoModulo++; inviaDebug("statoModulo++\n");
		databaseTxStart(DB_TX_NEUTRO_START);
			
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
		databaseSendUrl(stringaNeutroStartDB,sizeof(stringaNeutroStartDB));

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
	
	
		memset(stringaNeutroEndDB,0,sizeof(stringaNeutroEndDB));
		i = 0;
			
		statoModulo++; inviaDebug("statoModulo++\n");
		databaseTxStart(DB_TX_NEUTRO_END);
			
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
		databaseSendUrl(stringaNeutroEndDB,sizeof(stringaNeutroEndDB));

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

		memset(stringaRebootDB,0,sizeof(stringaRebootDB));
		i = 0;
		
		statoModulo++; inviaDebug("statoModulo++\n");
		databaseTxStart(DB_TX_REBOOT);
			
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
		databaseSendUrl(stringaRebootDB,sizeof(stringaRebootDB));

	
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
	
		memset(stringaDebugDB,0,sizeof(stringaDebugDB));
		i = 0;
			
		statoModulo++; inviaDebug("statoModulo++\n");
		databaseTxStart(DB_TX_DEBUG);
			
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
		databaseSendUrl(stringaDebugDB,sizeof(stringaDebugDB));

		
}