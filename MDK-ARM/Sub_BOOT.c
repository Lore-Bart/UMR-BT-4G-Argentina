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
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;

extern u32 corrEU,corrED,corrRU,corrRD;


//flag auto sync
extern u8 autoSyncActive;

//variabili misurandi
extern u32 V[3]; //tensioni
extern long I1[3],I2[3]; //correnti
extern uint16_t phi1[3],phi2[3]; //phi
extern long P1[3],P2[3]; //potenze attive
extern long Q1[3],Q2[3]; //potenze reattive
extern int cosphi1[3],cosphi2[3]; //cosfi

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

//variabili energie
extern uint32_t E1p[3],E2p[3]; //energie attive positive
extern uint32_t E1n[3],E2n[3]; //energie attive negative
extern uint32_t R1p[3],R2p[3]; //energie reattive positive
extern uint32_t R1n[3],R2n[3]; //energie reattive negative

//indici memorizzazioni
extern uint32_t indiceLoad;
extern uint32_t indiceMeas;

//BlueTooth
extern u8 BTattivo;

//numeri
extern u8 numeroAllarmi[20];
extern u8 numeroDevice[20];

//coefficienti
extern long corrV[3]; //calibrazione tensione
extern long corrI1[3],corrI2[3]; //calibrazione corrente
extern long corrE1[3],corrE2[3]; //calibrazione energia attiva
extern long corrR1[3],corrR2[3]; //calibrazione energia reattiva
extern long offI1[3],offI2[3]; //offset corrente
extern long offE1[3],offE2[3]; //offset energia attiva

//attivazioni memorizzazioni
extern uint8_t LoadActive;
extern uint8_t MeasActive;

//avvio
extern u8 avvioConcluso;

//timer
extern u32 secondiLoad;
extern u32 secondiMeas;

//energie backup
extern uint32_t E1pi[3],E2pi[3]; //energie attive positive BACKUP
extern uint32_t E1ni[3],E2ni[3]; //energie attive negative BACKUP
extern uint32_t R1pi[3],R2pi[3]; //energie reattive positive BACKUP
extern uint32_t R1ni[3],R2ni[3]; //energie reattive negative BACKUP

//identificativo
extern u8 identificativo[16];

//soglie
extern u16 sogliaNeutro;
extern u16 sogliaCorrenteA;
extern u16 sogliaCorrenteB;

//evento neutro
extern u8 eventoNeutro;
extern u8 eventiNeutro;

//guasti
extern u8 guasti;
extern u32 calibrazioneI[6];

//coordinate GPS
extern long	latitudine;
extern long longitudine;

//ora legale universale
extern u8 DSTon;
extern u8 DSThourStart;
extern u8 DSTdayStart;
extern u8 DSTweekStart;
extern u8 DSTmonthStart;
extern u8 DSTdayStop;
extern u8 DSTweekStop;
extern u8 DSTmonthStop;
extern u8 DSThourStop;

//antifurto
extern u8 antifurtoAttivo;
extern u16 calibAntifurto;

//password
extern uint8_t password[17];

//data e ora
extern uint32_t myTimeVar;

//antifurto
extern u8 antifurtoScattato;
extern u16 nIntrusioni;

//batteria
extern u8 messaggioBatteria;

extern u8 riavvio;

extern u8 emergenza;
extern u8 IDconnesso;

extern u8 alimentatore;

//internet
extern u8 APN[30];
extern u8 mySQL[50];
extern u8 userSQL[20];
extern u8 pwSQL[20];
extern u8 statoInternet;

extern u8 spegniLed;
extern u16 timeCarica;

extern u8 userAPN[30];
extern u8 pwAPN[30];
extern u8 retePrivata;
extern u32 sogliaI;
u8 sogliaPers = 0;

u8 univoco[5] = "00001";

extern u32 Ipot;
extern u8 holdDisBT;

u16 eccezione = 0;

u8 BTon = 0;

extern u8 disconnessione;

extern u16 underVoltageTH;
extern u16 overVoltageTH;

extern u8 overSavings;

extern u8 debugDB;

extern u8 NTPattivo;
extern u8 addressNTP[50];

int netCount = 0;

extern u8 emergenzaCollaudo;

void boot(void){
	int i = 0;
	

	//attivo timer watchdog
	//HAL_TIM_Base_Init(&htim4);
	
	
	//HAL_Delay(3000);
	
	
	avvioSistema(); //ricarica le variabili salvate
	sbloccaFlash(); //sblocco la flash
	adeinit(); //avvio ade 1
	adeinit3(); //avvio ade 2
	initNFC();

	
		
	//HAL_TIM_Base_Start_IT(&htim3);
	
	
	//sethour(1570454986);
	
}

void calibrazioneTest(void){
	corrV[0] = 28691;//27314
	corrV[1] = 28702;//27324
	corrV[2] = 28700;//27322
	
	corrI1[0] = 8615;
	corrI1[1] = 8663;
	corrI1[2] = 8621;
	
	corrI2[0] = 8630;
	corrI2[1] = 8637;
	corrI2[2] = 8642;
	
	corrE1[0] = 1720;//1637
	corrE1[1] = 1731;//1648
	corrE1[2] = 1720;//1581
	
	corrE2[0] = 1721;//1638
	corrE2[1] = 1722;//1639
	corrE2[2] = 1723;//1640
	
	corrR1[0] = 1745;//1661
	corrR1[1] = 1750;//1666
	corrR1[2] = 1745;//1888
	
	corrR2[0] = 1735;//1652
	corrR2[1] = 1740;//1656
	corrR2[2] = 1742;//1658
	
	calibrazioneI[0] = 3980;
	calibrazioneI[1] = 4000;
	calibrazioneI[2] = 4070;
	calibrazioneI[3] = 4010;
	calibrazioneI[4] = 4140;
	calibrazioneI[5] = 3980;
	
	
}


void internet(void){
	
	u8 setAPN[34] = "AT+QICSGP=1,\x22ibox.tim.it\x22,\x22\x22,\x22\x22,0\r";
	u8 taskTCP[12] = "AT+QIREGAPP\r";
	u8 connessione[9] = "AT+QIACT\r";
	
	HAL_UART_Transmit(&huart2,&setAPN[0],34,1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart2,&taskTCP[0],12,1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart2,&connessione[0],9,1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart2,(u8 *)"AT+QHTTPURL=55,30\r",18,1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart2,(u8 *)"http://testmisure.altervista.org/creaTab.php?name=cazzo",56,1000);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart2,(u8 *)"AT+QHTTPGET=60\r",15,1000);
	HAL_UART_Transmit(&huart2,(u8 *)"AT+QBTNAME?\r",12,1000);
}



//attivazione bluetooth
void attivaBT(void){
	u8 command[12] = "AT+QBTPWR=1\r";
	u8 command3[3] = "+++"; 	
	u8 command4[16] = "AT+QBTDISCONN=2\r";
	u8 name[31] = "AT+QBTNAME =\x22                \x22\r";
	u8 textMode[10] = "AT+CMGF=1\r";
	u8 deleteSMS[19] = "AT+QMGDA= DEL ALL\x22\r";
	
	deleteSMS[9] = 0x22;
	
	copiaArray(&name[13],&identificativo[0],16);
	
	HAL_UART_Transmit(&huart1,(u8*)"attivazione BT\n",15,100);
	
	HAL_Delay(1300);
	HAL_UART_Transmit(&huart2,&command3[0],3,1000);
	HAL_Delay(1300);
	HAL_UART_Transmit(&huart2,&command4[0],16,1000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart2,&command[0],12,1000);
	HAL_Delay(1000);//aspetto la risposta dal modulo per l'accensione (3 secondi)
	HAL_Delay(1000);
	HAL_Delay(1000);		
	HAL_UART_Transmit(&huart2,&textMode[0],10,1000);
	HAL_Delay(500);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart2,&deleteSMS[0],19,1000);
	HAL_Delay(500);
	
	disconnessione = 0;
	BTon = 1;	
}

//disattiva BT
void disattivaBT(void){
	u8 command[12] = "AT+QBTPWR=0\r";
	u8 command3[3] = "+++"; 	
	u8 command4[16] = "AT+QBTDISCONN=2\r";
	
	HAL_UART_Transmit(&huart1,(u8*)"disattivazione BT\n",18,100);
	
	HAL_Delay(1300);
	HAL_UART_Transmit(&huart2,&command3[0],3,1000);
	HAL_Delay(1300);
	HAL_UART_Transmit(&huart2,&command4[0],16,1000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart2,&command[0],12,1000);
	HAL_Delay(1000);//aspetto la risposta dal modulo per l'accensione (3 secondi)
	HAL_Delay(1000);
	HAL_Delay(1000);	
	
	BTon = 0;
}


//riavvio BT



//inizializza GSM
void initGSM(void){
	uint8_t command[5] = "ATE0\r";
	uint8_t command3[10] = "AT+CMGF=1\r";
	
	HAL_UART_Transmit(&huart2,&command[0],5,1000);
	HAL_Delay(10);

}



//IMPOSTA LA PASSWORD DI DEFAULT E FORMATTA LA PAGINA DELLE IMPOSTAZIONI
void primoAvvio(void){
uint8_t indirizzo[2] = {0,0};
uint8_t data[100];
uint8_t defaultPassword[4] = "0000";
uint8_t formattatore[256];
int i = 0; //indice utile per tutto

while(i<256){
	formattatore[i] = 0;
	i++;
}
i = 0; //azzero l'indice, potrebbe riservire

data[0] = 1; //salvo primo avvio effettuato
saveArrayFram(&data[0],&indirizzo[0],1);
indirizzo[1] = 1;
saveArrayFram(&defaultPassword[0],&indirizzo[0],10); //salvo la password di default
copiaArray(&password[0],&defaultPassword[0],4); //imposto la password uguale alla default password
indirizzo[1] = 5; //formatto tutto quello che c'è dopo nella pagina delle impostazioni
saveArrayFram(&formattatore[0],&indirizzo[0],251);
indirizzo[0] = 1;
indirizzo[1] = 0;
saveArrayFram(&formattatore[0],&indirizzo[0],256); //formatto la seconda pagina della FRAM
saveArrayFram(&identificativo[0],&indirizzo[0],16); //imposto l'identificativo di default
data[0] = 0;
indirizzo[1] = 68;
saveArrayFram(&data[0],&indirizzo[0],1); //Argentina: DST disabilitato di default

indirizzo[0] = 2; //formatto la terza pagina della FRAM
indirizzo[1] = 0;
saveArrayFram(&formattatore[0],&indirizzo[0],256);

sprintf(&password[0],"00000000");
password[8] = 0;

recuperaSeriale();

indirizzo[0] = 1; indirizzo[1] = 182;

Ipot = 850;
saveU32fram(Ipot,&indirizzo[0]);

}


//RIPRISTINA LE VARIABILI E I BACKUP
void avvioSistema(void){
	uint8_t indirizzo[2] = {0,0};
	uint8_t data[100];
	int i;
	u8 newDefaultPassword[16] = {48,48,48,48,48,48,48,48,0,0,0,0,0,0,0,0};

	ReadArrayFram(&data[0],&indirizzo[0],1);

	if(data[0] != 1){
		primoAvvio();
	}
	else{
		//ripristino password
		indirizzo[1] = 1;
		ReadArrayFram(&password[0],&indirizzo[0],4);
		////se non sono tutti 0xff, vai a prendere la password da un altra parte //parte aggiunta a causa della modifica sulla gestione della password
		if(password[0] != 0xff || password[1] != 0xff || password[2] != 0xff || password[3] != 0xff){
			password[0] = 0xff; password[1] = 0xff; password[2] = 0xff; password[3] = 0xff;
			saveArrayFram(&password[0],&indirizzo[0],4);
			indirizzo[0] = 2; indirizzo[1] = 234;
			saveArrayFram(&newDefaultPassword[0],&indirizzo[0],16);
			indirizzo[0] = 0;
		}		
		//recupero indici
		indirizzo[1] = 11;
		ReadArrayFram(&data[0],&indirizzo[0],8);
		indiceLoad = array2u32(&data[0]);
		indiceMeas = array2u32(&data[4]);
		//attivazione salvataggi
		indirizzo[1] = 19;
		ReadArrayFram(&data[0],&indirizzo[0],2);
		LoadActive = data[0];
		MeasActive = data[1];		
		//corrV
		indirizzo[1] = 21;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		corrV[0] = array2u32(&data[0]);
		corrV[1] = array2u32(&data[4]);
		corrV[2] = array2u32(&data[8]);
		//offI linea A
		indirizzo[1] = 33;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		offI1[0] = array2u32(&data[0]);
		offI1[1] = array2u32(&data[4]);
		offI1[2] = array2u32(&data[8]);
		//offE linea A
		indirizzo[1] = 45;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		offE1[0] = array2u32(&data[0]);
		offE1[1] = array2u32(&data[4]);
		offE1[2] = array2u32(&data[8]);
		//corrI linea A
		indirizzo[1] = 57;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		corrI1[0] = array2u32(&data[0]);
		corrI1[1] = array2u32(&data[4]);
		corrI1[2] = array2u32(&data[8]);
		//corrE linea A
		indirizzo[1] = 69;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		corrE1[0] = array2u32(&data[0]);
		corrE1[1] = array2u32(&data[4]);
		corrE1[2] = array2u32(&data[8]);
		//corrR linea A
		indirizzo[1] = 81;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		corrR1[0] = array2u32(&data[0]);
		corrR1[1] = array2u32(&data[4]);
		corrR1[2] = array2u32(&data[8]);
		//ripristino energie linea A
		indirizzo[1] = 93;
		ReadArrayFram(&data[0],&indirizzo[0],48);
		i = 0;
		while(i<3){
			E1pi[i] = array2u32(&data[i*4]);
			E1ni[i] = array2u32(&data[12+i*4]);
			R1pi[i] = array2u32(&data[24+i*4]);
			R1ni[i] = array2u32(&data[36+i*4]);
			i++;
		}
		i = 0;
		
		//offI linea A
		indirizzo[1] = 141;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		offI2[0] = array2u32(&data[0]);
		offI2[1] = array2u32(&data[4]);
		offI2[2] = array2u32(&data[8]);
		//offE linea A
		indirizzo[1] = 153;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		offE2[0] = array2u32(&data[0]);
		offE2[1] = array2u32(&data[4]);
		offE2[2] = array2u32(&data[8]);
		//corrI linea A
		indirizzo[1] = 165;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		corrI2[0] = array2u32(&data[0]);
		corrI2[1] = array2u32(&data[4]);
		corrI2[2] = array2u32(&data[8]);
		//corrE linea A
		indirizzo[1] = 177;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		corrE2[0] = array2u32(&data[0]);
		corrE2[1] = array2u32(&data[4]);
		corrE2[2] = array2u32(&data[8]);
		//corrR linea A
		indirizzo[1] = 189;
		ReadArrayFram(&data[0],&indirizzo[0],12);
		corrR2[0] = array2u32(&data[0]);
		corrR2[1] = array2u32(&data[4]);
		corrR2[2] = array2u32(&data[8]);

		//ripristino energie linea B
		indirizzo[1] = 201;
		ReadArrayFram(&data[0],&indirizzo[0],48);
		for (i=0;i<3;i++){
			E2pi[i] = array2u32(&data[i*4]);
			E2ni[i] = array2u32(&data[12+i*4]);
			R2pi[i] = array2u32(&data[24+i*4]);
			R2ni[i] = array2u32(&data[36+i*4]);
		}
		
		//ripristino soglia neutro
		indirizzo[1] = 249;
		ReadArrayFram(&data[0],&indirizzo[0],2);
		sogliaNeutro = array2int(&data[0]);
		
		
		//INIZIO PAGINA 1
		indirizzo[0] = 1;
		
		//ripristino identificativo
		indirizzo[1] = 0;
		ReadArrayFram(&identificativo[0],&indirizzo[0],16);
		//ripristino flag evento neutro
		indirizzo[1] = 16;
		ReadArrayFram(&eventoNeutro,&indirizzo[0],1);
		//numero eventi neutro e guasti
		indirizzo[1] = 17;
		ReadArrayFram(&data[0],&indirizzo[0],2);
		eventiNeutro = data[0];
		guasti = data[1];
		//calibrazione I
		indirizzo[1] = 19;
		ReadArrayFram(&data[0],&indirizzo[0],24);
		for(i=0;i<6;i++){
			calibrazioneI[i] = array2u32(&data[i*4]);
		}
		//soglie corrente
		indirizzo[1] = 43;
		ReadArrayFram(&data[0],&indirizzo[0],4);
		sogliaCorrenteA = array2u16(&data[0]);
		sogliaCorrenteB = array2u16(&data[2]);
		//numero allarmi
		indirizzo[1] = 47;
		ReadArrayFram(&numeroAllarmi[0],&indirizzo[0],20);
		////numero device SPOSTATO 2 121
		//indirizzo[1] = 57;
		//ReadArrayFram(&numeroDevice[0],&indirizzo[0],10);
		//flag auto sync
		indirizzo[1] = 67;
		ReadArrayFram(&autoSyncActive,&indirizzo[0],1);
		//ora legale
		indirizzo[1] = 68;
		ReadArrayFram(&data[0],&indirizzo[0],9);
		DSTon = 0; //Argentina: nessuna ora legale
		DSThourStart = data[1]; DSTdayStart = data[2]; DSTweekStart = data[3]; DSTmonthStart = data[4];
		DSThourStop = data[5]; DSTdayStop = data[6]; DSTweekStop = data[7]; DSTmonthStop = data[8];
		//coordinate
		indirizzo[1] = 77;
		ReadArrayFram(&data[0],&indirizzo[0],8);
		latitudine = array2long(&data[0]);
		longitudine = array2long(&data[4]);
		//antifurto
		indirizzo[1] = 85;
		ReadArrayFram(&data[0],&indirizzo[0],6);
		antifurtoAttivo = data[0];
		calibAntifurto = array2u16(&data[1]);
		nIntrusioni = array2u16(&data[3]);
		antifurtoScattato = data[5];
		//batteria
		indirizzo[1] = 91;
		ReadArrayFram(&data[0],&indirizzo[0],1);
		messaggioBatteria = data[0];
		
		indirizzo[1] = 100;
		ReadArrayFram(&data[0],&indirizzo[0],8);
		corrEU = array2u32(&data[0]);
		corrRU = array2u32(&data[4]);
		
		indirizzo[1] = 108;
		ReadArrayFram(&data[0],&indirizzo[0],8);
		corrED = array2u32(&data[0]);
		corrRD = array2u32(&data[4]);
		
		//metto i dati apn privato dopo 116
		indirizzo[1] = 116;
		ReadArrayFram(&userAPN[0],&indirizzo[0],30);
		
		indirizzo[1] = 146;
		ReadArrayFram(&pwAPN[0],&indirizzo[0],30);
		
		indirizzo[1] = 176;
		ReadArrayFram(&retePrivata,&indirizzo[0],1);
		
		indirizzo[1] = 177; //soglia pers
		ReadArrayFram(&sogliaPers,&indirizzo[0],1);
		
		if(sogliaPers != 0){
			indirizzo[1] = 178;
			ReadArrayFram(&data[0],&indirizzo[0],4);
			sogliaI = array2u32(&data[0]);
		}
		
		indirizzo[1] = 182;//Ipot
		ReadArrayFram(&data[0],&indirizzo[0],4);
		Ipot = array2u32(&data[0]);
		
		//indirizzo[1] = 130;
		//ReadArrayFram(&univoco[0],&indirizzo[0],5);
		
		recuperaSeriale();
		
		//ripristini dati internet
		indirizzo[0] = 2;
		indirizzo[1] = 0;
		ReadArrayFram(&statoInternet,&indirizzo[0],1);
		indirizzo[1] = 1;
		ReadArrayFram(&APN[0],&indirizzo[0],50);
		indirizzo[1] = 51; //vecchio 31
		ReadArrayFram(&mySQL[0],&indirizzo[0],100);
		indirizzo[1] = 151; //vecchio 81
		ReadArrayFram(&userSQL[0],&indirizzo[0],30);
		indirizzo[1] = 181; //vecchio 101
		ReadArrayFram(&pwSQL[0],&indirizzo[0],30);
		
		//numero device lo sposto qui 2 121
		indirizzo[1] = 211; //vecchio 121
		ReadArrayFram(&numeroDevice[0],&indirizzo[0],20);
		
		//2 //150 time carica
		indirizzo[1] = 231; //vecchio 150
		ReadArrayFram(&data[0],&indirizzo[0],2);
		timeCarica = array2u16(&data[0]);
		
		indirizzo[1] = 233;
		ReadArrayFram(&holdDisBT,&indirizzo[0],1);
		
		indirizzo[1] = 234;
		ReadArrayFram(&password[0],&indirizzo[0],16);
		
		indirizzo[1] = 250;
		ReadArrayFram(&data[0],&indirizzo[0],2);
		underVoltageTH = array2u16(&data[0]);
		
		indirizzo[1] = 252;
		ReadArrayFram(&data[0],&indirizzo[0],2);
		overVoltageTH = array2u16(&data[0]);
		
		indirizzo[1] = 254;
		ReadArrayFram(&overSavings,&indirizzo[0],1);
		
		indirizzo[1] = 255;
		ReadArrayFram(&debugDB,&indirizzo[0],1);
		
		indirizzo[0] = 3;
		
		indirizzo[1] = 0;
		ReadArrayFram(&NTPattivo,&indirizzo[0],1);
		
		indirizzo[1] = 1;
		ReadArrayFram(&addressNTP[0],&indirizzo[0],50);
		
	}

}

void recuperaSeriale(void){

	u32 var1;
	u32 var5;
	
	var1 = *(uint32_t*)0x8007FF8;
	var5 = *(uint32_t*)0x08007FFC;

	eccezione = (var1 & 0xffff0000) >> 16;
	univoco[0] = var1 & 0x000000ff;
	univoco[1] = (var5 & 0xff000000) >> 24;
	univoco[2] = (var5 & 0x00ff0000) >> 16;
	univoco[3] = (var5 & 0x0000ff00) >> 8;
	univoco[4] = (var5 & 0x000000ff);
	
	inviaDebug("recupero seriale\n");
	
	if(univoco[0] == 0xff && univoco[1] == 0xff && univoco[2] == 0xff && univoco[3] == 0xff && univoco[4] == 0xff){
		univoco[0] = 48;
		univoco[1] = 48;
		univoco[2] = 48;
		univoco[3] = 48;
		univoco[4] = 49;
	}
}

//azzeramento watchdog
void TIM4_IRQHandler(void)
{
	//static u8 statoPrecedente = 0;
	//u8 statoAttuale = 0;
	
  HAL_TIM_IRQHandler(&htim4);
	
	/*statoAttuale = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2);
	if(statoAttuale != statoPrecedente){
		netCount++;
	}
	statoPrecedente = statoAttuale;*/
	
	//inviaDebug("ciao");
	
	resetWD();
	
}


void resetWD(void){
	u8 delay = 0xff;
	
	

//	if(emergenza == 3 || emergenzaCollaudo != 0){
//		return;
//	}
	
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_4);
	while(delay != 0){delay--;}
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_4);
	
		
	if(avvioConcluso == 1){
		if(spegniLed == 0){
			
			if(alimentatore !=0){
			
				if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1) == 0){
					BTattivo = 1;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
				}
				else{
					BTattivo = 0;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
				}
			}
			
			else{
				if(multiplocinque(myTimeVar)){
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
				}
				else{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
				}
			}
						
		}
		else{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
		}
	}


}


void reset(void){

	/*uint32_t msp_value = *(volatile uint32_t *)0x08000000;
	
	__set_MSP(msp_value);*/
	
	SCB->VTOR = 0x08000004;
	
	riavvio = 1;
	
	return;	

}


