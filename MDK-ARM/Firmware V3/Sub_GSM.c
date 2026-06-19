#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

//periferiche
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c2;
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;

u8 sniff[50];


//BT attivo
extern u8 BTattivo;

//secondi dall'avvio
extern u32 seconds;

//sincronizzazione
u8 syncActive = 0;
u32 SyncCorr = 0;

//ultimi dati SMS
u32 lastSMS;
u8 lastNumber[20];

//timer cancellaSMS
u8 cancellaSMS = 0;

//segnale GSM
u8 segnaleGSM = 0;

//stato del modulo
u8 statoModulo = 0;
extern u8 timerModuloESC;

//numeri
extern u8 numeroAllarmi[20];
extern u8 numeroDevice[20];

extern u8 inibitGuastoSMS;

extern u16 riavvioSMS;

extern u8 attivaInternetFlag;
u8 disattivaInternetFlag;

u8 indicationSMSarrivato[100];
u8 indicationSMSflag = 0;

extern long temperatura;
extern int tensioneBint;
extern u8 batteriaInCarica;


//SMS polling
u8 numeroPoll[20];
int sizeNumeroPoll;
u8 messaggioPoll[160];
int sizeMessaggioPoll;
u8 inviaSMSpollFlag = 0;

//SMS polling coda
u8 numeroPollCoda[20];
int sizeNumeroPollCoda;
u8 messaggioPollCoda[160];
int sizeMessaggioPollCoda;
u8 inviaSMSpollFlagCoda = 0;

//internet
u8 statoInternet = 0;
u8 APN[50]; //vecchio 30
u8 mySQL[100]; //vecchio 50
u8 userSQL[30]; //vecchio 20
u8 pwSQL[30]; //vecchio 20
u8 userAPN[30];
u8 pwAPN[30];



u8 retePrivata = 0;


extern u32 myTimeVar;
extern u32 V[3];
extern u8 identificativo[16];

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

extern u8 univoco[5];
extern u32 sogliaI;
extern u8 sogliaPers;
extern u32 Ipot;

//attivazione SMS
u8 smsAttivi = 0;
u8 smsTextModeReady = 0;
static u8 smsTxBusy = 0;
static int lastReadSmsIndex = 0;
u8 debugDB = 0; //salvataggio batteria su 



extern uint8_t password[4];

u8 nuovaConnessione = 0;

u8 inibitNFC = 0;

extern u16 riavvioForzato;

extern u16 packTotGSM;
extern u8 updateGSMatt;
extern u16 NpackRecGSM;
extern u8 packRecFlag;
extern u8 packGSM[500];
extern u16 crcGSM;
extern u16 updateGSMvers;
extern u8 downloadNewPackFlag;
extern u8 bytes16[16];
extern u16 software;
u8 timerUpdateGSM;
extern u8 uartPack[500];

extern u8 riavvia;

extern u16 underVoltageTH;
extern u16 overVoltageTH;

extern u8 overSavings;

extern u8 timerStatoModulo;
extern u8 timerRebootDB;

extern u8 batteryLevel;

extern u8 timerModulo4G;

extern u32 regolaOra;
extern u8 aggioraOrarioNTPflag = 0;


u16 sizePack = 0;

u8 upAddressGSM[200];

u8 tentativoUpdate = 0;

//comandi NTP
u8 NTPattivo = 0;
u8 addressNTP[50];



static int smsIsDigit(char c)
{
    return (c >= '0' && c <= '9');
}

static int smsIsHexChar(char c)
{
    if(c >= '0' && c <= '9') return 1;
    if(c >= 'A' && c <= 'F') return 1;
    if(c >= 'a' && c <= 'f') return 1;
    return 0;
}

static int smsIsHexMessage(u8 *m)
{
    int i = 0;
    int len = 0;

    if(m == 0) return 0;

    while(m[i] == ' ' || m[i] == '\r' || m[i] == '\n' || m[i] == '\t') i++;

    while(m[i] != 0){
        if(m[i] == '\r' || m[i] == '\n' || m[i] == ' ' || m[i] == '\t'){
            break;
        }
        if(smsIsHexChar(m[i]) == 0){
            return 0;
        }
        len++;
        i++;
    }

    if(len < 2) return 0;
    if((len & 1) != 0) return 0;
    return 1;
}

static void smsSafeCopy(u8 *dst, const u8 *src, int maxLen)
{
    int i = 0;

    if(dst == 0 || maxLen <= 0) return;
    if(src == 0){
        dst[0] = 0;
        return;
    }

    while(i < (maxLen - 1) && src[i] != 0){
        dst[i] = src[i];
        i++;
    }
    dst[i] = 0;
}

static int smsParseCmtiIndex(u8 *messaggio)
{
    char *p;
    int index = 0;

    if(messaggio == 0) return 0;

    p = strstr((char*)messaggio, "CMTI");
    if(p == 0) return 0;

    p = strchr(p, ',');
    if(p == 0) return 0;
    p++;

    while(*p == ' ' || *p == '\"') p++;

    while(smsIsDigit(*p)){
        index = (index * 10) + (*p - '0');
        p++;
    }

    return index;
}

static int smsParseTwoDigits(const char *p)
{
    if(p == 0) return 0;
    if(!smsIsDigit(p[0]) || !smsIsDigit(p[1])) return 0;
    return (p[0] - '0') * 10 + (p[1] - '0');
}

void initSMStextMode(void)
{
    /*
     * Forza il SIM7600 in modalita' SMS testo. Questo evita che AT+CMGR
     * restituisca PDU esadecimali, che non sono interpretabili dai comandi
     * SMS in chiaro e non vanno confusi con i comandi codificati dell'app.
     */
    if(statoModulo != 0){
        return;
    }

    if(updateGSMatt != 0){
        return;
    }

    statoModulo++;
    inviaDebug("init SMS text mode\n");

    HAL_UART_Transmit(&huart6,(u8*)"AT+CMGF=1\r",strlen("AT+CMGF=1\r"),1000);
    HAL_Delay(150);
    HAL_UART_Transmit(&huart6,(u8*)"AT+CSCS=\"GSM\"\r",strlen("AT+CSCS=\"GSM\"\r"),1000);
    HAL_Delay(150);
    HAL_UART_Transmit(&huart6,(u8*)"AT+CNMI=2,1,0,0,0\r",strlen("AT+CNMI=2,1,0,0,0\r"),1000);
    HAL_Delay(150);
    HAL_UART_Transmit(&huart6,(u8*)"AT+CPMS=\"SM\",\"SM\",\"SM\"\r",strlen("AT+CPMS=\"SM\",\"SM\",\"SM\"\r"),1000);
    HAL_Delay(150);

    smsTextModeReady = 1;

    if(statoModulo > 0){
        statoModulo--;
        inviaDebug("statoModulo--\n");
    }
}

void risposteGSM(uint8_t *messaggio){
	uint8_t signal[3] = "CSQ"; //SEGNALE
	u8 interpretaMessaggio = 0;
	u8 pointer;
	int numeroByte = 0;
	
	timerModulo4G = 30;
	
	
	
	uint8_t uart[300],uartConv[300];
	int size;
	int i = 7;
	
	riavvioForzato = timeoutModulo;
	
	if(cercaStringa(&messaggio[0],(u8*)"CMTI",4,&pointer) == 1){//lettura SMS
		/* Salvo la notifica completa e poi il main leggera' l'indice in modo robusto. */
		smsSafeCopy(&indicationSMSarrivato[0], &messaggio[0], sizeof(indicationSMSarrivato));
		indicationSMSflag = 1;
	}
	else if(cercaStringa(&messaggio[0],(u8*)"CMGR:",5,&pointer) == 1){//pulizia SMS
		size = pulisciSMS(&messaggio[0],&uart[0]);
		if(size > 0){
			interpretaMessaggio = 1;
		}
		else{
			interpretaMessaggio = 0;
		}
	}
	else if(cercaStringa(&messaggio[0],(u8*)"CSQ:",4,&pointer) == 1){//risposta livello segnale
		/*
		 * La risposta AT+CSQ puo' arrivare con echo, CR/LF o spezzata in modo
		 * diverso dal previsto. Non usiamo piu' un offset fisso come &messaggio[15].
		 */
		refreshSignal(&messaggio[0]);
		if(statoModulo > 0){
			statoModulo--; inviaDebug("statoModulo--\n");
		}
	}
	else if(cercaStringa(&messaggio[0],(u8*)"CMGS:",5,&pointer) == 1){
		smsTxBusy = 0;
		if(statoModulo > 0){
			statoModulo--; inviaDebug("statoModulo--\n");
		}
	}
	else if(cercaStringa(&messaggio[0],(u8*)"OK",2,&pointer) == 1){
		/* Durante una richiesta DB, l'OK di AT+HTTPPARA abilita solo ora HTTPACTION.
		 * Se HTTPPARA risponde ERROR non viene lanciato un HTTPACTION con URL vecchio. */
		databaseHttpParaOk();
	}
	else if(cercaStringa(&messaggio[0],(u8*)"ERROR",5,&pointer) == 1){
		/* Se un invio SMS o una richiesta DB fallisce, non lasciamo il modulo occupato per sempre. */
		if(smsTxBusy != 0){
			smsTxBusy = 0;
			if(statoModulo > 0){
				statoModulo--; inviaDebug("statoModulo--\n");
			}
		}
		else if(databaseTxIsBusy() != 0){
			databaseHttpError();
			if(statoModulo > 0){
				statoModulo--; inviaDebug("statoModulo--\n");
			}
		}
	}
	else if(cercaStringa(&messaggio[0],(u8*)"ATI",3,&pointer)){
		statoModulo = 0;
	}
	else if(cercaStringa(&messaggio[0],(u8*)"+NETOPEN:",9,&pointer)){
		/*
		 * Parsing robusto: prima veniva controllato messaggio[18], quindi bastava
		 * un echo o un CR/LF diverso per interpretare male la risposta.
		 */
		if(statoModulo > 0){
			statoModulo--; inviaDebug("statoModulo--\n");
		}
		if(strstr((char*)messaggio,"+NETOPEN: 0") != 0){
			statoInternet = 3;
			delay(200);
			HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPINIT\r",12,100);
			delay(200);
		}
		else{
			/* NETOPEN fallita: internet resta abilitato ma non connesso, cosi' il main ritenta. */
			statoInternet = 1;
		}
	}
		else if(cercaStringa(&messaggio[0],(u8*)"CNTP:",5,&pointer)){
		if(statoModulo > 0){
			statoModulo--; inviaDebug("statoModulo--\n");
		}
		if(messaggio[9] == '0'){
			aggioraOrarioNTPflag = 1;
		}
	}
	else if(cercaStringa(&messaggio[0],(u8*)"CCLK:",5,&pointer)){
		impostaOrarioNTP(&messaggio[pointer+7]);
	}
	else if(cercaStringa(&messaggio[0],(u8*)"HTTPACTION:",11,&pointer)){
		if(statoModulo > 0){
			statoModulo--; inviaDebug("statoModulo--\n");
		}
		/* Esito HTTP delle richieste MySQL. I flag DB vengono cancellati solo qui,
		 * dopo un vero +HTTPACTION con codice 2xx/3xx. In caso di errore restano pendenti. */
		databaseHttpActionResult(&messaggio[0]);
		if(updateGSMatt == 1){ //lettura pagina contenente il pacchetto
			if(messaggio[pointer+19] == 0x0d){
				numeroByte = messaggio[pointer+18]-48;
			}
			else if(messaggio[pointer+20] == 0x0d){
				numeroByte = (messaggio[pointer+18]-48)*10 + messaggio[pointer+19]-48;
			}
			else{
				numeroByte = (messaggio[pointer+18]-48)*100 + (messaggio[pointer+19]-48)*10 + messaggio[pointer+20]-48;
			}
			sprintf(uart,"AT+HTTPREAD=%d\r",numeroByte);
			invia4G(uart);
			updateGSMatt = 0;
		}
	}
	else if(cercaStringa(&messaggio[0],(u8*)"HTTPREAD: DATA",14,&pointer)){
		filtroUpdate(&messaggio[pointer]);
	}
	
	if(interpretaMessaggio == 1){
		
		interpretaMessaggio = 0;
		
		if(comparaStringhe(&uart[0],(u8*)"AlarmPhone",10)){
				impostaAlarmPhone(uart);			
			}
		else if(comparaStringhe(&uart[0],(u8*)"APNpriv",7)){
				esportaRetePrivata(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"APNpub",6)){
				disattivaRetePrivata(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"modSoglia",9)){
				modificaSogliaI(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"modIpot",7)){
				modificaIpot(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"visSoglia",9)){
				visualizzaSogliaI(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"visIpot",7)){
				visualizzaIpot(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"update",6)){
				updateSMS(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"reboot",6)){
				reboot(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"modVunder",9)){
				modificaVunder(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"visVunder",9)){
				visualizzaVunder(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"modVover",8)){
				modificaVover(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"visVover",8)){
				visualizzaVover(uart);
			}
		else if(comparaStringhe(&uart[0],(u8*)"dbSavings",8)){
				dbSavings(uart);
			}	
		else if(comparaStringhe(&uart[0],(u8*)"debugDB",7)){
				attivaDebugDB(uart);
			}	
		else if(comparaStringhe(&uart[0],(u8*)"NTPon",5)){
				NTPon(uart);
		}
		else if(comparaStringhe(&uart[0],(u8*)"NTPoff",6)){
				NTPoff(uart);
		}
		else if(comparaStringhe(&uart[0],(u8*)"NTPrefresh",10)){
				NTPrefresh(uart);
		}
		else if(comparaStringhe(&uart[0],(u8*)"SQLon",5)){
				esportaParametriInternet(uart);
		}
		else if(comparaStringhe(&uart[0],(u8*)"SQLoff",6)){
				disattivaInternet(uart);
		}
		else if(comparaStringhe(&uart[0],(u8*)"SQLstate",8)){
				stateInternet(uart);
		}
		else if(comparaStringhe(&uart[0],(u8*)"SQLrestart",10)){
				restartInternet(uart);
		}
			
		else{
			/*
			 * Comandi codificati generati dall'app: il corpo SMS e' testo HEX.
			 * Li convertiamo in byte solo se il messaggio e' effettivamente HEX;
			 * questo evita di eseguire accidentalmente PDU o testi non riconosciuti.
			 */
			if(smsIsHexMessage(&uart[0]) != 0){
				inviaDebug((u8*)"esecuzione comando codificato SMS\n");
				for(i = 0; i < 300; i++) uartConv[i] = 0;
				string2byte(&uart[0],&uartConv[0],strlen((char*)uart));
				eseguiComando4G(uartConv);
			}
			else{
				inviaDebug((u8*)"comando SMS non riconosciuto\n");
				inviaSMS(&lastNumber[0],strlen((char*)lastNumber),(u8*)"COMMAND NOT VALID",17);
			}
		}
	
	}
	sprintf(uart,"stato modulo:%d\n",statoModulo);
	inviaDebug(uart);
}

void NTPon(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 addressFram[2] = {3,0};
	u8 passwordSent[20];
	int contPW = 0;
	u8 NTPaddressLocal[50];
	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
		while(messaggio[i] != 0x0d && i < 320){
			NTPaddressLocal[a] = messaggio[i];
			a++;
			i++;
		}
		//a++;
		//APN[a] = 0;
		while(a < 50){
			NTPaddressLocal[a] = 0;
			a++;
		}
		a = 0;
		i++;
		i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
  passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	NTPattivo = 1;
	saveArrayFram(&NTPattivo,&addressFram[0],1);
	copiaArray(&addressNTP[0],&NTPaddressLocal[0],50);
	addressFram[1] = 1;
	saveArrayFram(&addressNTP[0],&addressFram[0],50);
	
	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);
	
	aggiornaOrarioNTP();
	
	
}

void NTPoff(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 addressFram[2] = {3,0};
	u8 passwordSent[20];
	int contPW = 0;
	u8 NTPaddressLocal[50];
	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
  passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	NTPattivo = 0;
	saveArrayFram(&NTPattivo,&addressFram[0],1);
	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);
}

void NTPrefresh(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 passwordSent[20];
	int contPW = 0;
	u8 NTPaddressLocal[50];
	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
  passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);
	aggiornaOrarioNTP();
}

void aggiornaOrarioNTP(void){
	
	statoModulo++; inviaDebug("statoModulo++\n");
	
	invia4G("AT+CNTP=\x22");
	invia4G(addressNTP);
	// Argentina: UTC-3. Il parametro CNTP del SIM7600 e' in quarti d'ora.
	// -12 = -3 ore. Evita correzioni da ora legale, non usata in Argentina.
	invia4G("\x22,-12\r");
	delay(100);
	invia4G("AT+CNTP\r");

}

void impostaOrarioNTP(u8* messaggio){
	RTC_DateTypeDef dateNTP;
	RTC_TimeTypeDef timeNTP;
	u8 uart[100];
	u32 timeLocal;
	
	dateNTP.Year = (messaggio[0]-48)*10+messaggio[1]-48;
	dateNTP.Month = (messaggio[3]-48)*10+messaggio[4]-48;
	dateNTP.Date = (messaggio[6]-48)*10+messaggio[7]-48;
	
	timeNTP.Hours = (messaggio[9]-48)*10+messaggio[10]-48;
	timeNTP.Minutes = (messaggio[12]-48)*10+messaggio[13]-48;
	timeNTP.Seconds = (messaggio[15]-48)*10+messaggio[16]-48;
	
	
	timeLocal = timetoposix(dateNTP,timeNTP);
	
	regolaOra = timeLocal;
			
	
	//sprintf(uart,"set: %d %d %d     %d %d %d\n%d\n\n", dateNTP.Date, dateNTP.Month, dateNTP.Year, timeNTP.Hours,timeNTP.Minutes, timeNTP.Seconds,myTimeVar);
	sprintf(uart,"time = %d\n",timeLocal);
	HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	
}


void visualizzaIpot(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 passwordSent[20];
	int contPW = 0;
	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;

	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	sprintf(uart,"Ipot:%d",Ipot);
	inviaSMS(&lastNumber[0],strlen(lastNumber),uart,strlen(uart));
	
}

void visualizzaSogliaI(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 passwordSent[20];
	int contPW = 0;
	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;

	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	sprintf(uart,"soglia:%d",sogliaI);
	inviaSMS(&lastNumber[0],strlen(lastNumber),uart,strlen(uart));
	
}

void visualizzaVunder(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 passwordSent[20];
	int contPW = 0;
	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;

	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	sprintf(uart,"under-voltage threshold: %d V",underVoltageTH/100);
	inviaSMS(&lastNumber[0],strlen(lastNumber),uart,strlen(uart));
	
}

void visualizzaVover(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 passwordSent[20];
	int contPW = 0;
	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;

	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	sprintf(uart,"over-voltage threshold: %d V",overVoltageTH/100);
	inviaSMS(&lastNumber[0],strlen(lastNumber),uart,strlen(uart));
	
}

void modificaIpot(u8* messaggio){
	int i = 0;
	int a = 0;
	int o = 0;
	u8 uart[100];
	u8 addressFram[2] = {1,182};
	u8 passwordSent[20];
	u8 sogliaChar[150];
	u32 sogliaIlocal = 0;
	int contPW = 0;

	
	while(messaggio[i] != 0x0a && i < 50){
		i++;
	}
	i++;

	while(messaggio[i] != 0x0a && i < 50){
			sogliaChar[a] = messaggio[i];
			a++;
			i++;
	}
	i++;
	
	inviaDebug("soglia ");
	inviaDebug(sogliaChar);
	inviaDebug("\n");
	

	if(a == 1){
		sogliaIlocal = (sogliaChar[0]-48);
	}
	else if(a == 2){
		sogliaIlocal = (sogliaChar[0]-48)*10 + (sogliaChar[1]-48);
	}
	else if(a == 3){
		sogliaIlocal = (sogliaChar[0]-48)*100 + (sogliaChar[1]-48)*10 + (sogliaChar[2]-48);
	}
	else if(a == 4){
		sogliaIlocal = (sogliaChar[0]-48)*1000 + (sogliaChar[1]-48)*100 + (sogliaChar[2]-48)*10 + (sogliaChar[3]-48);
	}
	else{
		sogliaIlocal = 0;
	}
		
	sprintf(uart,"soglia = %d\n",sogliaIlocal);
	inviaDebug(uart);
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
		
	Ipot = sogliaIlocal;
	addressFram[1] = 182;
	saveU32fram(Ipot,&addressFram[0]);
	sprintf(uart,"Ipot:%d",Ipot);
	inviaSMS(&lastNumber[0],strlen(lastNumber),uart,strlen(uart));
			
}

void modificaSogliaI(u8* messaggio){
	int i = 0;
	int a = 0;
	int o = 0;
	u8 uart[100];
	u8 addressFram[2] = {1,177};
	u8 passwordSent[20];
	u8 sogliaChar[10];
	u32 sogliaIlocal = 0;
	int contPW = 0;

	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;

	while(messaggio[i] != 0x0a && i < 320){
			sogliaChar[a] = messaggio[i];
			a++;
			i++;
	}
	i++;
	

	if(a == 1){
		sogliaIlocal = (sogliaChar[0]-48);
	}
	else if(a == 2){
		sogliaIlocal = (sogliaChar[0]-48)*10 + (sogliaChar[1]-48);
	}
	else if(a == 3){
		sogliaIlocal = (sogliaChar[0]-48)*100 + (sogliaChar[1]-48)*10 + (sogliaChar[2]-48);
	}
	else if(a == 4){
		sogliaIlocal = (sogliaChar[0]-48)*1000 + (sogliaChar[1]-48)*100 + (sogliaChar[2]-48)*10 + (sogliaChar[3]-48);
	}
	else{
		sogliaIlocal = 0;
	}
		
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	sogliaPers = 1;
	saveArrayFram(&sogliaPers,&addressFram[0],1);
	
	sogliaI = sogliaIlocal;
	addressFram[1] = 178;
	saveU32fram(sogliaI,&addressFram[0]);
	sprintf(uart,"soglia:%d",sogliaI);
	inviaSMS(&lastNumber[0],strlen(lastNumber),uart,strlen(uart));
			
}

void modificaVunder(u8* messaggio){
	int i = 0;
	int a = 0;
	int o = 0;
	u8 uart[100];
	u8 addressFram[2] = {1,182};
	u8 passwordSent[20];
	u8 sogliaChar[150];
	u32 sogliaVlocal = 0;
	int contPW = 0;

	
	while(messaggio[i] != 0x0a && i < 50){
		i++;
	}
	i++;

	while(messaggio[i] != 0x0a && i < 50){
			sogliaChar[a] = messaggio[i];
			a++;
			i++;
	}
	i++;
	

	if(a == 1){
		sogliaVlocal = (sogliaChar[0]-48);
	}
	else if(a == 2){
		sogliaVlocal = (sogliaChar[0]-48)*10 + (sogliaChar[1]-48);
	}
	else if(a == 3){
		sogliaVlocal = (sogliaChar[0]-48)*100 + (sogliaChar[1]-48)*10 + (sogliaChar[2]-48);
	}
	else if(a == 4){
		sogliaVlocal = (sogliaChar[0]-48)*1000 + (sogliaChar[1]-48)*100 + (sogliaChar[2]-48)*10 + (sogliaChar[3]-48);
	}
	else{
		sogliaVlocal = 0;
	}
		
	sogliaVlocal *= 100;
	
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
		
	underVoltageTH = sogliaVlocal;
	addressFram[0] = 2;
	addressFram[1] = 250;
	saveU16fram(underVoltageTH,&addressFram[0]);
	sprintf(uart,"under-voltage threshold: %d V",underVoltageTH/100);
	inviaSMS(&lastNumber[0],strlen(lastNumber),uart,strlen(uart));
			
}

void dbSavings(u8* messaggio){
	int i = 0;
	int a = 0;
	int o = 0;
	u8 uart[100];
	u8 addressFram[2] = {1,182};
	u8 passwordSent[20];
	u8 saving = 0;
	u32 sogliaVlocal = 0;
	int contPW = 0;

	
	while(messaggio[i] != 0x0a && i < 50){
		i++;
	}
	i++;

	saving = messaggio[i];
	while(messaggio[i] != 0x0a && i < 50){
			i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	if(saving == 48){
		overSavings = 0;
		sprintf(uart,"function deactived");
	}
	else if(saving == (1+48)){
		overSavings = 1;
		sprintf(uart,"saving every minute");
	}
	else if(saving == (3+48)){
		overSavings = 3;
		sprintf(uart,"saving every 3 minutes");
	}
	else if(saving == (5+48)){
		overSavings = 5;
		sprintf(uart,"saving every 5 minutes");
	}
	else{
		sprintf(uart,"time not valid");
	}
		
	addressFram[0] = 2;
	addressFram[1] = 254;
	saveArrayFram(&overSavings,&addressFram[0],1);
	inviaSMS(&lastNumber[0],strlen(lastNumber),uart,strlen(uart));
			
}


void modificaVover(u8* messaggio){
	int i = 0;
	int a = 0;
	int o = 0;
	u8 uart[100];
	u8 addressFram[2] = {1,182};
	u8 passwordSent[20];
	u8 sogliaChar[150];
	u32 sogliaVlocal = 0;
	int contPW = 0;

	
	while(messaggio[i] != 0x0a && i < 50){
		i++;
	}
	i++;

	while(messaggio[i] != 0x0a && i < 50){
			sogliaChar[a] = messaggio[i];
			a++;
			i++;
	}
	i++;
	

	if(a == 1){
		sogliaVlocal = (sogliaChar[0]-48);
	}
	else if(a == 2){
		sogliaVlocal = (sogliaChar[0]-48)*10 + (sogliaChar[1]-48);
	}
	else if(a == 3){
		sogliaVlocal = (sogliaChar[0]-48)*100 + (sogliaChar[1]-48)*10 + (sogliaChar[2]-48);
	}
	else if(a == 4){
		sogliaVlocal = (sogliaChar[0]-48)*1000 + (sogliaChar[1]-48)*100 + (sogliaChar[2]-48)*10 + (sogliaChar[3]-48);
	}
	else{
		sogliaVlocal = 0;
	}
		
	sogliaVlocal *= 100;
	
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
		
	overVoltageTH = sogliaVlocal;
	addressFram[0] = 2;
	addressFram[1] = 252;
	saveU16fram(overVoltageTH,&addressFram[0]);
	sprintf(uart,"over-voltage threshold: %d V",overVoltageTH/100);
	inviaSMS(&lastNumber[0],strlen(lastNumber),uart,strlen(uart));
			
}

void attivaDebugDB(u8* messaggio){
	int i = 0;
	u8 uart[100];
	u8 addressFram[2] = {2,255};
	u8 passwordSent[20];
	u8 sogliaChar[150];
	u32 sogliaVlocal = 0;
	int contPW = 0;
	u8 debugDBlocal;

	
	while(messaggio[i] != 0x0a && i < 50){
		i++;
	}
	i++;

	while(messaggio[i] != 0x0a && i < 50){
			debugDBlocal = messaggio[i];
			i++;
	}
	i++;
		
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	
	if(debugDBlocal == 49){
		debugDB = 1;
		sprintf(uart,"debug attivato");
		}
	else{
		debugDB = 0;
		sprintf(uart,"debug disattivato");
		}
	addressFram[0] = 2;
	addressFram[1] = 255;
	saveArrayFram(&debugDB,&addressFram[0],1);
	inviaSMS(&lastNumber[0],strlen(lastNumber),uart,strlen(uart));
			
}



void connettiInternet(void){
	
	u8 setAPN[100] = "AT+CGDCONT=1,\x22IP\x22,\x22";
	u8 setAPN3[2] = "\x22\r";
	u8 taskTCP[13] = "AT+CGACT=1,1\r";
	u8 connessione[11] = "AT+NETOPEN\r";
	
	copiaArray(&setAPN[19],&APN[0],strlen(APN));
	copiaArray(&setAPN[strlen(setAPN)],&setAPN3[0],2);
	
	statoModulo++; inviaDebug("statoModulo++\n");
	
	HAL_UART_Transmit(&huart6,&setAPN[0],strlen(setAPN),1000);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart6,&taskTCP[0],13,1000);
	HAL_Delay(100);
	statoInternet = 2;

	HAL_UART_Transmit(&huart6,&connessione[0],11,1000);
	
	
}

void connettiInternet5(void){ //con credenziali
	
	u8 setAPN[150] = "AT+QICSGP=1,\x22";
	u8 setAPN3[3] = "\x22,\x22";
	u8 setAPN4[3] = "\x22,\x22";
	u8 setAPN5[5] = "\x22,0\r\0";

	
	u8 taskTCP[12] = "AT+QIREGAPP\r";
	u8 connessione[9] = "AT+QIACT\r";
	u8 richiestaIP[11] = "AT+QILOCIP\r";
	
	copiaArray(&setAPN[13],&APN[0],strlen(APN));
	copiaArray(&setAPN[strlen(setAPN)],&setAPN3[0],3);
	copiaArray(&setAPN[strlen(setAPN)],&userAPN[0],strlen(userAPN));
	copiaArray(&setAPN[strlen(setAPN)],&setAPN4[0],3);
	copiaArray(&setAPN[strlen(setAPN)],&pwAPN[0],strlen(pwAPN));
	copiaArray(&setAPN[strlen(setAPN)],&setAPN5[0],5);

	
	HAL_UART_Transmit(&huart1,&setAPN[0],strlen(setAPN),1000);
	HAL_UART_Transmit(&huart6,&setAPN[0],strlen(setAPN),1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart1,&taskTCP[0],12,1000);
	HAL_UART_Transmit(&huart6,&taskTCP[0],12,1000);
	HAL_Delay(200);
	statoInternet = 2;
	statoModulo = 2;
	HAL_UART_Transmit(&huart1,&connessione[0],9,1000);
	HAL_UART_Transmit(&huart6,&connessione[0],9,1000);
	
}




void esportaRetePrivata(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 addressFram[2] = {2,0};
	u8 passwordSent[20];
	u8 userAPNlocal[30];
	u8 pwAPNlocal[30];
	int contPW = 0;
	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x0a && i < 320){
			userAPNlocal[a] = messaggio[i];
			a++;
			i++;
		}
		//a++;
		//APN[a] = 0;
		while(a < 30){
			userAPNlocal[a] = 0;
			a++;
		}
		a = 0;
		i++;
	
		
	while(messaggio[i] != 0x0a && i < 320){
			pwAPNlocal[a] = messaggio[i];
			a++;
			i++;
		}
		//a++;
		//mySQL[a] = 0;
		while(a < 30){
			pwAPNlocal[a] = 0;
			a++;
		}
		a = 0;
		i++;
		
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	retePrivata = 1;
	addressFram[0] = 1; addressFram[1] = 116;
	copiaArray(&userAPN[0],&userAPNlocal[0],30);
	saveArrayFram(&userAPN[0],&addressFram[0],30);
	
	addressFram[0] = 1; addressFram[1] = 146;
	copiaArray(&pwAPN[0],&pwAPNlocal[0],30);
	saveArrayFram(&pwAPN[0],&addressFram[0],30);
	
	addressFram[0] = 1; addressFram[1] = 176;
	saveArrayFram(&retePrivata,&addressFram[0],1);
	
	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);

}

void disattivaRetePrivata(u8* messaggio){
		u8 addressFram[2] = {1,176};
		u8 passwordSent[20];
		int i = 0;
		int contPW = 0;
		
		while(messaggio[i] != 0x0a && i < 320){
			i++;
		}
		i++;
		
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
		
		
	retePrivata = 0;
		
	saveArrayFram(&retePrivata,&addressFram[0],1);

	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);
}

void disattivaInternet(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 addressFram[2] = {2,0};
	u8 passwordSent[20];
	int contPW = 0;
	
		
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
  passwordSent[contPW] = 0;
	
	inviaDebug("password sent:\n");
	inviaDebug(passwordSent);
	inviaDebug("\n");
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);
	
	statoInternet = 0;
	addressFram[0] = 2;
	addressFram[1] = 0;			
	saveArrayFram(&statoInternet,&addressFram[0],1);
	clearDatabaseRequests();
	disattivaInternetFlag = 1;	
	
}

void restartInternet(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 addressFram[2] = {2,0};
	u8 passwordSent[20];
	int contPW = 0;
	
		
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
  passwordSent[contPW] = 0;
	
	inviaDebug("password sent:\n");
	inviaDebug(passwordSent);
	inviaDebug("\n");
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);
	
	statoInternet = 1;
	disattivaInternetFlag = 0;
	addressFram[0] = 2;
	addressFram[1] = 0;			
	saveArrayFram(&statoInternet,&addressFram[0],1);
	
	if(statoModulo != 0 && timerModuloESC == 0){
		inviaDebug("recupero statoModulo per riavvio internet\n");
		statoModulo = 0;
	}
	
	if(statoModulo == 0){
		connettiInternet();
	}
	
}

void stateInternet(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 addressFram[2] = {2,0};
	u8 passwordSent[20];
	int contPW = 0;
	
		
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
  passwordSent[contPW] = 0;
	
	inviaDebug("password sent:\n");
	inviaDebug(passwordSent);
	inviaDebug("\n");
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	
	
	if(statoInternet == 3){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"CONNECTED",9);
	}
	else{
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"NOT CONNECTED",13);
	}
	
}

void esportaParametriInternet(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 addressFram[2] = {2,0};
	u8 passwordSent[20];
	int contPW = 0;
	
	
	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
		/*while(messaggio[i] != 0 && i <= pwOff+52){
			APN[a] = messaggio[i];
			i++;
			a++;
		}
		while(a<50){
			APN[a] = 0;
			a++;
		}*/
	
		while(messaggio[i] != 0x0d && i < 320){
			APN[a] = messaggio[i];
			a++;
			i++;
		}
		while(a < 50){
			APN[a] = 0;
			a++;
		}
		a = 0;
		i++;
		i++;
		
		inviaDebug("APN:\n");
		inviaDebug(APN);
		inviaDebug("\n");
		
		
		while(messaggio[i] != 0x0d && i < 320){
			mySQL[a] = messaggio[i];
			a++;
			i++;
		}
		//a++;
		//mySQL[a] = 0;
		while(a < 100){
			mySQL[a] = 0;
			a++;
		}
		a = 0;
		i++;
		i++;
		
		inviaDebug("database:\n");
		inviaDebug(mySQL);
		inviaDebug("\n");
		
		while(messaggio[i] != 0x0d && i < 320){
			userSQL[a] = messaggio[i];
			a++;
			i++;
		}
		//a++;
		//userSQL[a] = 0;
		while(a < 30){
			userSQL[a] = 0;
			a++;
		}
		a = 0;
		i++;
		i++;
		
		inviaDebug("username:\n");
		inviaDebug(userSQL);
		inviaDebug("\n");
		
		while(messaggio[i] != 0x0d && i < 320 && messaggio[i] > 32){
			pwSQL[a] = messaggio[i];
			a++;
			i++;
		}
		//a++;
		//pwSQL[a] = 0;
		while(a < 30){
			pwSQL[a] = 0;
			a++;
		}
		a = 0;
		i++;
		i++;
	
		inviaDebug("password:\n");
		inviaDebug(pwSQL);
		inviaDebug("\n");
		
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
  passwordSent[contPW] = 0;
	
	inviaDebug("password sent:\n");
	inviaDebug(passwordSent);
	inviaDebug("\n");
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		
		//recupero vecchie impostazioni
		//ripristini dati internet
		addressFram[0] = 2;
		addressFram[1] = 1;
		ReadArrayFram(&APN[0],&addressFram[0],50);
		addressFram[1] = 51;
		ReadArrayFram(&mySQL[0],&addressFram[0],100);
		addressFram[1] = 151;
		ReadArrayFram(&userSQL[0],&addressFram[0],30);
		addressFram[1] = 181;
		ReadArrayFram(&pwSQL[0],&addressFram[0],30);		
		return;
	}
	

	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);
	
	statoInternet = 1;
	addressFram[0] = 2;
	addressFram[1] = 0;			
	saveArrayFram(&statoInternet,&addressFram[0],1);
	
	
	saveArrayFram(&statoInternet,&addressFram[0],1);
	addressFram[1] = 1; saveArrayFram(&APN[0],&addressFram[0],50);
	addressFram[1] = 51; saveArrayFram(&mySQL[0],&addressFram[0],100);
	addressFram[1] = 151; saveArrayFram(&userSQL[0],&addressFram[0],30);
	addressFram[1] = 181; saveArrayFram(&pwSQL[0],&addressFram[0],30);
}

void impostaAlarmPhone(u8* messaggio){
	
	int i = 0;
	int a = 0;
	u8 addressFram[2] = {1,47};
	u8 passwordSent[20];
	int contPW = 0;
	
	while(messaggio[i] != 0x0a && i < 160){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x0a && i < 160){
			numeroAllarmi[a] = messaggio[i];
			a++;
			i++;
		}
		numeroAllarmi[a] = 0;
		numeroAllarmi[a+1] = 0;
		
		i++;
		
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);		
		ReadArrayFram(&numeroAllarmi[0],&addressFram[0],20);
		return;
	}
	//salvo su Fram
	saveArrayFram(&numeroAllarmi[0],&addressFram[0],20);
	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);
	
}

void updateRemoto(void){
	
	HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPURL=60,30\r",18,1000);
	HAL_UART_Transmit(&huart1,(u8 *)"AT+QHTTPURL=60,30\r",18,1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart6,(u8 *)"http://a2atestmisure.altervista.org/aggiornamento.php?pack=1",60,1000);
	HAL_UART_Transmit(&huart1,(u8 *)"http://a2atestmisure.altervista.org/aggiornamento.php?pack=1",60,1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPGET=60\r",15,1000);
	HAL_UART_Transmit(&huart1,(u8 *)"AT+QHTTPGET=60\r",15,1000);
	statoModulo = 20;
	//HAL_Delay(5000);
	//HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPREAD=60\r",16,1000);
	//HAL_UART_Transmit(&huart1,(u8 *)"AT+QHTTPREAD=60\r",16,1000);
}

void testAPN(void){
	HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPURL=17,30\r",18,1000);
	HAL_UART_Transmit(&huart1,(u8 *)"AT+QHTTPURL=17,30\r",18,1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart6,(u8 *)"http://10.218.7.2",17,1000);
	HAL_UART_Transmit(&huart1,(u8 *)"http://10.218.7.2",17,1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPGET=60\r",15,1000);
	HAL_UART_Transmit(&huart1,(u8 *)"AT+QHTTPGET=60\r",15,1000);
	//statoModulo = 20;
	//HAL_Delay(5000);
	//HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPREAD=60\r",16,1000);
	//HAL_UART_Transmit(&huart1,(u8 *)"AT+QHTTPREAD=60\r",16,1000);
}

void downloadPacchetto(u16 pack){
	u8 uart[100];
	u8 URL[200];
	//static u8 lenght = 0;
	
	timerUpdateGSM = 30;
	
	statoModulo++; inviaDebug("statoModulo++\n");
	
	sprintf(URL,"AT+HTTPPARA=\x22URL\x22,\x22");	
	
	if(NpackRecGSM == 0){
		if(posizioneFlash()){
				sprintf(&upAddressGSM[strlen(upAddressGSM)],"?pos=up&pack=");
			}
			else{
				sprintf(&upAddressGSM[strlen(upAddressGSM)],"?pos=down&pack=");
			}			
		}
	copiaArray(&URL[strlen(URL)],upAddressGSM,strlen(upAddressGSM));
	
	URL[strlen(URL)] = '0';
	URL[strlen(URL)] = '\x22';
	URL[strlen(URL)] = '\r';
		
	HAL_UART_Transmit(&huart6,URL,strlen(URL),1000);
	delay(100);
	HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPACTION=1\r",16,1000);
	
	
	/*if(pack < 10){
		//HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPURL=60,20\r",18,1000);
		sprintf(uart,"AT+QHTTPURL=%d,30\r",strlen(upAddressGSM)+1);
	}
	else if(pack >= 10 && pack < 100){
		//HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPURL=61,20\r",18,1000);
		sprintf(uart,"AT+QHTTPURL=%d,30\r",strlen(upAddressGSM)+2);
	}
	else if(pack >= 100 && pack < 1000){
		//HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPURL=62,20\r",18,1000);
		sprintf(uart,"AT+QHTTPURL=%d,30\r",strlen(upAddressGSM)+3);
	}
	else{
		//HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPURL=63,20\r",18,1000);
		sprintf(uart,"AT+QHTTPURL=%d,30\r",strlen(upAddressGSM)+4);
	}
	HAL_UART_Transmit(&huart6,uart,strlen(uart),100);
	
	HAL_Delay(100);
	
	copiaArray(&URL[0],&upAddressGSM[0],strlen(upAddressGSM));
	sprintf(&URL[strlen(upAddressGSM)],"%d",pack);
	HAL_UART_Transmit(&huart6,URL,strlen(URL),100);
	HAL_UART_Transmit(&huart1,URL,strlen(URL),100);
	
	HAL_Delay(100);
	HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPGET=20\r",15,1000);
	HAL_UART_Transmit(&huart1,(u8 *)"\nget\n",4,1000);
	
	sprintf(uart,"\ndownload pacchetto %d di %d\n", pack, packTotGSM);
	HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	statoModulo = 20;*/
	
}

void downloadPacchettoTest(u16 pack){
	
	u8 command[100] = "http://a2atestmisure.altervista.org/aggiornamento.php?pack=";

	timerUpdateGSM = 30;
	
	
	if(pack < 10){
		HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPURL=60,20\r",18,1000);
	}
	else if(pack >= 10 && pack < 100){
		HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPURL=61,20\r",18,1000);
	}
	else if(pack >= 100 && pack < 1000){
		HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPURL=62,20\r",18,1000);
	}
	else{
		HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPURL=63,20\r",18,1000);
	}
	
	HAL_Delay(100);
	
	sprintf(&command[59],"%d",pack);
	HAL_UART_Transmit(&huart6,command,strlen(command),100);
	HAL_UART_Transmit(&huart1,command,strlen(command),100);
	
	HAL_Delay(100);
	HAL_UART_Transmit(&huart6,(u8 *)"AT+QHTTPGET=20\r",15,1000);
	HAL_UART_Transmit(&huart1,(u8 *)"get\n",4,1000);
	
	
	statoModulo = 20;		
	
}

int filtroUpdate(u8* messaggio){
	int i = 0;
	int a = 0;
	int o = 0;
	int u = 0;
	
	
	static u8 messaggioLocal[480];
	u8 bytes[500];
	u8 uart[100];
	
	tentativoUpdate = 0;
	
	if(NpackRecGSM == 0){		
		
		while(messaggio[i] != 0x0a && i < 20){
			i++;
		}
		i++;
		
		sprintf(uart,"messaggio 0: ");
		HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
		HAL_UART_Transmit(&huart1,&messaggio[i],12,100);
		HAL_UART_Transmit(&huart1,(u8*)"\n\n",2,100);
		
		while(o < 12){
			if(messaggio[i+o] < 60){
				messaggio[i+o] = messaggio[i+o] - 48;
			}
			else{
				messaggio[i+o] = messaggio[i+o] - 55;
			}
			o++;			
		}
		
		updateGSMvers = (messaggio[i] << 4) | messaggio[i+1];
		updateGSMvers = (updateGSMvers << 8) | (messaggio[i+2] << 4) | messaggio[i+3];
		
		crcGSM = (messaggio[i+4] << 4) | messaggio[i+5];
		crcGSM = (crcGSM << 8) | (messaggio[i+6] << 4) | messaggio[i+7];
		
		packTotGSM = (messaggio[i+8] << 4) | messaggio[i+9];
		packTotGSM = (packTotGSM << 8) | (messaggio[i+10] << 4) | messaggio[i+11];
		
		sprintf(uart,"dati: %d %x %d\n", updateGSMvers,crcGSM, packTotGSM);
		HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
		
		/*if(updateGSMvers <= software){
			inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"downgrade not possible",22);
			updateGSMatt = 0;
			packTotGSM = 0;
			NpackRecGSM = 0;
			statoModulo = 0;
			timerUpdateGSM = 30;
		}
		else{
			formattaFlashInterna();
			inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"download started",16);			
			//inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"downgrade not possible",22);
			//updateGSMatt = 0;
			//packTotGSM = 0;
			//NpackRecGSM = 0;
		}*/
	}
	else{
	
		
		while(messaggio[i] != 0x0a && i < 10){
			i++;
		}
		i++;
		
		while(messaggio[i+a] != 0x0d && a < 500){
			messaggioLocal[a] = messaggio[i+a];
			a++;
		}
		
	
		
		while(o < a){
			if(messaggioLocal[o] < 60){
				messaggioLocal[o] = messaggioLocal[o] - 48;
			}
			else{
				messaggioLocal[o] = messaggioLocal[o] - 55;
			}
					
			o++;			
		}
	
		o = 0;
		
		sprintf(uart,"pack:\n",packGSM[u]);
		HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
		
		while(o < a){
			packGSM[u] = (messaggioLocal[o] * 16) + messaggioLocal[o+1];
						
			
			u++;
			o++;
			o++;
		}
				
		a = a / 2;
		u = 0;
				
	}
		
	sizePack = strlen(uartPack);
	
	while(u<500){
		uartPack[u] = 0;
		u++;
	}
	
	//HAL_UART_Transmit(&huart1,&messaggio[i],a,100);
	return a;
}

u8 controlloO(u8* messaggio){
	int i = 0;
	u8 check = 0;
	
	while(i < 500){
		if(messaggio[i] == 'O' && messaggio[i+1] != 'R'){
			i = 500;
			check = 1;			
		}
		i++;
	}

	
	return check;
}

u8 controlloK(u8* messaggio){
	int i = 0;
	u8 check = 0;
	
	while(i < 500){
		if(comparaStringhe(&messaggio[i],(u8*)"K",1)){
			i = 500;
			check = 1;
		}
		i++;
	}
	
	return check;
}

void updateSMS(u8* messaggio){
	int i = 0;
	int a = 0;
	int o = 0;
	int contPW = 0;
	u8 uart[100];
	u8 passwordSent[20];
	u8 upAddressLocal[100];
	
		
	while(messaggio[i] != 0x0a && i < 50){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x0a && i < 100){
			upAddressLocal[a] = messaggio[i];
			a++;
			i++;
	}
	upAddressLocal[a] = 0;
	
	i++;
	
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;
	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	
	copiaArray(&upAddressGSM[0],&upAddressLocal[0],strlen(upAddressLocal)+1);
	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);
	
	updateGSMatt = 1;
	NpackRecGSM = 0;
	downloadNewPackFlag = 1;
		
}

u8 controlloOK(u8* messaggio){
	int i = 0;
	u8 check = 0;
	
	while(i < 500){
		if(comparaStringhe(&messaggio[i],(u8*)"OK",2)){
			i = 500;
			check = 1;
		}
		i++;
	}
	
	return check;
}

void reboot(u8* messaggio){
	int i = 0;
	int a = 0;
	u8 uart[100];
	u8 passwordSent[20];
	int contPW = 0;
	
	while(messaggio[i] != 0x0a && i < 320){
		i++;
	}
	i++;
	
	while(messaggio[i] != 0x20 && contPW < 16){
		passwordSent[contPW] = messaggio[i];
		i++;
		contPW++;
	}
	passwordSent[contPW] = 0;

	
	if(comparaStringhe(&passwordSent[0],&password[0],strlen(password)) != 1 || strlen(password)  != strlen(passwordSent)){
		inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"PASSWORD NOT VALID",18);
		return;
	}
	
	inviaSMS(&lastNumber[0],strlen(lastNumber),(u8*)"OK",2);
	
	riavvia = 1;
	

}





u8 findArrow(u8* messaggio){
	int i = 0;
	u8 check = 0;
	
	while(i < 20){
		if(messaggio[i] == '>'){
			check = 1;
			i = 20;
		}
		i++;
	}
	
	return check;
	
}

void deleteSMS(void){
	u8 data[100];
	
	sprintf(data,"AT+QMGDA= DEL ALL \r");
	data[9] = 34;
	data[17] = 34;
	HAL_UART_Transmit(&huart6,data,strlen(data),100);
}

//comando lettura SMS
void leggiSMS(uint8_t *numeroSMS){
    uint8_t command[20];
    int index;

    inviaDebug((u8*)"lettura SMS\n");

    index = smsParseCmtiIndex(numeroSMS);
    if(index <= 0){
        inviaDebug((u8*)"indice SMS non valido\n");
        return;
    }

    lastReadSmsIndex = index;
    sprintf((char *)command,"AT+CMGR=%d\r",index);
    HAL_UART_Transmit(&huart6,&command[0],strlen((char*)command),1000);
}

int pulisciSMS(uint8_t *inBuf, uint8_t *outBuf){
    char *p;
    char *q[10];
    char *body;
    char *end;
    int nq = 0;
    int i;
    int len;
    int copyLen;
    RTC_DateTypeDef DateSMS;
    RTC_TimeTypeDef TimeSMS;
    u8 command[20];

    if(outBuf == 0) return 0;
    outBuf[0] = 0;

    if(inBuf == 0) return 0;

    p = strstr((char*)inBuf,"+CMGR:");
    if(p == 0) p = strstr((char*)inBuf,"CMGR:");
    if(p == 0) return 0;

    /*
     * In text mode la riga +CMGR contiene campi tra virgolette.
     * Se non li troviamo e' quasi certamente PDU mode: non interpretiamo nulla.
     */
    for(i = 0; p[i] != 0 && nq < 10; i++){
        if(p[i] == '\"'){
            q[nq++] = &p[i];
        }
        if(p[i] == '\n'){
            break;
        }
    }

    if(nq < 4){
        inviaDebug((u8*)"SMS non in text mode: reinit CMGF\n");
        smsTextModeReady = 0;
        initSMStextMode();
        return 0;
    }

    /* Secondo campo tra virgolette = numero mittente. */
    len = (int)(q[3] - q[2] - 1);
    if(len < 0) len = 0;
    if(len > 19) len = 19;
    for(i = 0; i < len; i++){
        lastNumber[i] = (u8)q[2][i + 1];
    }
    lastNumber[len] = 0;

    /* Quarto campo tra virgolette = data/ora SMS, se presente. */
    if(nq >= 8 && (q[7] - q[6]) >= 18){
        char *t = q[6] + 1;
        DateSMS.Year = smsParseTwoDigits(t);
        DateSMS.Month = smsParseTwoDigits(t + 3);
        DateSMS.Date = smsParseTwoDigits(t + 6);
        TimeSMS.Hours = smsParseTwoDigits(t + 9);
        TimeSMS.Minutes = smsParseTwoDigits(t + 12);
        TimeSMS.Seconds = smsParseTwoDigits(t + 15);
        lastSMS = timetoposix(DateSMS,TimeSMS);
    }

    body = strchr(p,'\n');
    if(body == 0) return 0;
    body++;
    while(*body == '\r' || *body == '\n') body++;

    end = strstr(body,"\r\n\r\nOK");
    if(end == 0) end = strstr(body,"\r\nOK");
    if(end == 0) end = strstr(body,"\nOK");
    if(end == 0) end = body + strlen(body);

    while(end > body && (*(end - 1) == '\r' || *(end - 1) == '\n' || *(end - 1) == 0x1A)){
        end--;
    }

    copyLen = (int)(end - body);
    if(copyLen <= 0) return 0;
    if(copyLen > 299) copyLen = 299;

    memcpy(outBuf, body, copyLen);
    outBuf[copyLen] = 0;

    /* Cancello solo l'SMS appena letto, non tutta la memoria. */
    if(lastReadSmsIndex > 0){
        sprintf((char*)command,"AT+CMGD=%d\r",lastReadSmsIndex);
        HAL_UART_Transmit(&huart6,command,strlen((char*)command),100);
        delay(100);
    }

    return copyLen;
}




void refreshSignal(u8 *messaggio){

	int signal = 99;
	u8 uart[30];
	char *p;
	
	if(messaggio == 0){
		segnaleGSM = 0;
		return;
	}

	/*
	 * Parser robusto della risposta AT+CSQ.
	 * Accetta sia il buffer completo, per esempio:
	 *   AT+CSQ\r\r\n+CSQ: 18,99\r\n\r\nOK\r\n
	 * sia una stringa che inizia gia' dal numero.
	 */
	p = strstr((char*)messaggio,"CSQ:");
	if(p != 0){
		p += 4;
	}
	else{
		p = (char*)messaggio;
	}

	while(*p == ' ' || *p == '\r' || *p == '\n' || *p == '\t'){
		p++;
	}

	if(*p >= '0' && *p <= '9'){
		signal = atoi(p);
	}

	sprintf((char*)uart,"segnale CSQ: %d\n",signal);
	inviaDebug(uart);
	
	/* 0..31 valido, 99 non noto. Manteniamo uscita storica 0..4. */
	if(signal >= 2 && signal <= 9){
		segnaleGSM = 1;
	}
	else if(signal >= 10 && signal <= 16){
		segnaleGSM = 2;
	}
	else if(signal >= 17 && signal <= 25){
		segnaleGSM = 3;
	}
	else if(signal >= 26 && signal <= 31){
		segnaleGSM = 4;
	}
	else{
		segnaleGSM = 0;
	}
		
}

void requestSignal(void){
	/*
	 * Richiesta livello segnale gestita come transazione AT.
	 * Parte solo se il modulo e' libero; statoModulo viene liberato alla ricezione di +CSQ.
	 */
	if(statoModulo != 0){
		return;
	}

	statoModulo++;
	inviaDebug("request signal\n");
	HAL_UART_Transmit(&huart6,(u8*)"AT+CSQ\r",7,1000);
}



//invio SMS da interrupt
void inviaSMS(uint8_t *numero, int sizeNumero, uint8_t *messaggio,int sizeMessaggio){

	/*
	 * Accodamento sicuro dell'SMS. Limitiamo numero e testo per non uscire
	 * dai buffer statici e per restare entro la lunghezza SMS standard.
	 */
	if(numero == 0 || messaggio == 0) return;
	if(sizeNumero <= 0 || sizeMessaggio <= 0) return;
	if(sizeNumero > 19) sizeNumero = 19;
	if(sizeMessaggio > 159) sizeMessaggio = 159;

	if(inviaSMSpollFlag == 0){
		sizeNumeroPoll = sizeNumero;
		sizeMessaggioPoll = sizeMessaggio;
		memset(numeroPoll,0,sizeof(numeroPoll));
		memset(messaggioPoll,0,sizeof(messaggioPoll));
		copiaArray(&numeroPoll[0],&numero[0],sizeNumero);
		copiaArray(&messaggioPoll[0],&messaggio[0],sizeMessaggio);
		inviaSMSpollFlag = 1;
	}
	else{
		sizeNumeroPollCoda = sizeNumero;
		sizeMessaggioPollCoda = sizeMessaggio;
		memset(numeroPollCoda,0,sizeof(numeroPollCoda));
		memset(messaggioPollCoda,0,sizeof(messaggioPollCoda));
		copiaArray(&numeroPollCoda[0],&numero[0],sizeNumero);
		copiaArray(&messaggioPollCoda[0],&messaggio[0],sizeMessaggio);
		inviaSMSpollFlagCoda = 1;
	}
	return;
}

//invio SMS polling
void inviaSMSpoll(u8 coda){
	
	uint8_t command[20];
	uint8_t command3[20];
	
		
	if(coda == 1){
		if(smsTextModeReady == 0){ initSMStextMode(); }
		statoModulo++; inviaDebug("statoModulo++\n");
		smsTxBusy = 1;
		
		inviaDebug("poll = 1\n");
		
		sprintf((char *)command,"AT+CMGS=\x22");
		sprintf((char *)command3,"\x22\r");
		HAL_UART_Transmit(&huart6,&command[0],strlen((char *)command),1000);
		HAL_UART_Transmit(&huart6,&numeroPoll[0],sizeNumeroPoll,1000);
		HAL_UART_Transmit(&huart6,&command3[0],2,1000);
		
		HAL_Delay(200);
		sprintf((char *)command,"\x1A");
		HAL_UART_Transmit(&huart6,&messaggioPoll[0],sizeMessaggioPoll,1000);
		
		
		inibitGuastoSMS = 0;
		inviaSMSpollFlag = 0;
		
		HAL_UART_Transmit(&huart6,&command[0],1,1000);
	}
	else if(coda == 2){
		if(smsTextModeReady == 0){ initSMStextMode(); }
		statoModulo++; inviaDebug("statoModulo++\n");
		smsTxBusy = 1;
		inviaDebug("poll = 2\n");
		sprintf((char *)command,"AT+CMGS=\x22");
		sprintf((char *)command3,"\x22\r");
		HAL_UART_Transmit(&huart6,&command[0],strlen((char *)command),1000);
		HAL_UART_Transmit(&huart6,&numeroPollCoda[0],sizeNumeroPollCoda,1000);
		HAL_UART_Transmit(&huart6,&command3[0],2,1000);
		
		HAL_Delay(200);
		sprintf((char *)command,"\x1A");
		HAL_UART_Transmit(&huart6,&messaggioPollCoda[0],sizeMessaggioPollCoda,1000);
						
		
		inibitGuastoSMS = 0;
		inviaSMSpollFlagCoda = 0;
				
		HAL_UART_Transmit(&huart6,&command[0],1,1000);
		
	}
}



//auto sincronizzazione
void autoSync (void){
	u8 messaggio[18] = "54494D454366666666";
	
	inviaSMS(&numeroDevice[0],strlen(numeroAllarmi),&messaggio[0],18);
	syncActive = 1;
	
}



void inviaSMSprova(void){
	uint8_t command[200];
	uint8_t command3[20];

	invia4G("AT+CMGF=1\r");
	HAL_Delay(100);
	statoModulo++; inviaDebug("statoModulo++\n");
	sprintf((char *)command,"AT+CMGS=\x22");
	sprintf((char *)command3,"3931706199\x22\r");
	HAL_UART_Transmit(&huart6,&command[0],strlen((char *)command),1000);
	HAL_UART_Transmit(&huart6,&command3[0],strlen((char *)command3),1000);
	HAL_Delay(20);
	
	sprintf((char *)command,"CIAO\x1A");
	HAL_UART_Transmit(&huart6,&command[0],strlen((char *)command),1000);
	//HAL_Delay(10);
}













