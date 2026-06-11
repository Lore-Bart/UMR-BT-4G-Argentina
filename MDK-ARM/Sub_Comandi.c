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
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;

//comandi calibrazione
extern u8 calib;
extern u8 calib3;
extern u8 calibGuasti;

//attivazione antifurto
extern u8 antifurtoAttivo;

//antifurto scattato
extern u8 antifurtoScattato;
extern u16 nIntrusioni;

//attivazione antifurto
extern u8 antifurtoAttivo;

//stato batteria
extern u8 batteryLevel;

//soglie
extern u16 sogliaCorrenteA;
extern u16 sogliaCorrenteB;
extern u16 sogliaNeutro;

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

//azzeramento contatori
extern u8 azzeraLineaA;
extern u8 azzeraLineaB;

//flag auto sync
extern u8 autoSyncActive;

//numeri
extern u8 numeroAllarmi[20];
extern u8 numeroDevice[20];


//variabili temporali
extern u32 myTimeVar;
extern u8 DST;

//localizzazione
long latitudine = 0;
long longitudine = 0;
extern double latitudineD,longitudineD;

//identificativo
extern u8 identificativo[16];

//ultimi dati SMS
extern u32 lastSMS;
extern u8 lastNumber[10];

//sincronizzazione
extern u32 SyncCorr;
extern u32 seconds;

//password
uint8_t password[17] = {48,48,48,48,48,48,48,48,0}; //modifico con 8 caratteri, default 00000000

//BT attivo
extern u8 BTattivo;

//update attivo
extern u8 updateAttivo;

//attivazioni salvataggi
extern uint8_t LoadActive;
extern uint8_t MeasActive;

//pacchetti totali
extern u16 paccTot;

//variabili misurandi
extern u32 V[3]; //tensioni
extern long I1[3],I2[3]; //correnti
extern int cosphi1[3],cosphi2[3]; //cosfi

extern u8 univoco[5];

extern u8 smstest;

extern u16 riavvioForzato;


extern u8 inizializzaAntifurto;

//internet
extern u8 statoInternet;
extern u8 APN[30];
extern u8 mySQL[50];
extern u8 userSQL[20];
extern u8 pwSQL[20];

extern u8 disattivaInternetFlag;


extern long timerBT;
extern uint16_t phi1[3],phi2[3]; //phi

extern u16 sogliaCorrenteA;
extern u16 sogliaCorrenteB;
extern u16 sogliaNeutro;

//regola ora
u32 regolaOra = 0;

u32 sniff32 = 0;

u8 data[400],string[400];

u8 cambioNomeBT = 0;

u8 disconnessione = 0;

u8 holdDisBT = 0;
u8 forzaAtt = 0;
u8 forzaDis = 0;




void eseguiComandoBT(uint8_t *messaggio){
	u8 OK[4] = "OK\r\n";
	u8 WP[4] = "WP\r\n";
	u8 addressFram[2] = {0,0};
	u16 addressFramInt;
	u32 A,B;
	u8 offset[2];
	int i = 0; //indice
	int a = 0;
	u32 azz = 0;
	u8 uart[100];
		
	u8 pwOff = 0; //posizione del codice del comando
	u8 oldPassword[20];
	u8 newPassword[20];
	int contPW = 0;
	int contPWnew = 0;
	u8 check = 0;

	
	check = checkPasswordBT(&messaggio[0]);
	//controllo password
	if(check == 0){		
			HAL_UART_Transmit(&huart2,&WP[0],4,1000);		
			HAL_UART_Transmit(&huart1,(u8*)"password errata\n",15,100);			
			//HAL_UART_Transmit(&huart1,&messaggio[0],12,100);
			//HAL_UART_Transmit(&huart1,(u8*)"\n",1,100);				
		return;	
	}
	
	//tengo attivo il BT fisso se è attiva la disattivazione automatica. Al riavvio holdDisBT tornerà 1
	if(holdDisBT == 1 && BTattivo == 1){
		holdDisBT = 0;
	}
		
	pwOff = strlen(password)+2;

		
	
	if(updateAttivo == 0){
		
		if(codiceComando(&messaggio[pwOff]) != 0xff){
			disconnessione = 0;
		}
		
		switch(codiceComando(&messaggio[pwOff])){
			
			case 0x01://dashboard SOLO BT
				dashboard(&data[0]);
				__disable_irq();
				HAL_UART_Transmit(&huart2,&data[0],174,1000);
				__enable_irq();
				break;				
		
			case 0x02: //memorizzazione load profiles
				addressFram[1] = 19;
				LoadActive = messaggio[pwOff+2]-48;
				saveArrayFram(&LoadActive,&addressFram[0],1);
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
			
			case 0x03: //memorizzazione meas profiles
				addressFram[1] = 20;
				MeasActive = messaggio[pwOff+2]-48;
				saveArrayFram(&MeasActive,&addressFram[0],1);
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
		
			case 0x04: //download load profiles (solo BT)
				HAL_UART_Transmit(&huart2,&identificativo[0],16,1000);
				A = array2u32(&messaggio[pwOff+2]);
				B = array2u32(&messaggio[pwOff+6]);
				__disable_irq();
				filtroLoad(A,B);
				//filtroLoadFake();
				__enable_irq();
				break;
				
			case 0x05: //download measurand profiles (solo BT)
				HAL_UART_Transmit(&huart2,&identificativo[0],16,1000);
				A = array2u32(&messaggio[pwOff+2]);
				B = array2u32(&messaggio[pwOff+6]);
				__disable_irq();
				filtroMeas(A,B);
				__enable_irq();
				break;	
				
			case 0x06: //somma energie
				
				u322array(&data[0],latitudine);
				u322array(&data[4],longitudine);
				u322array(&data[8],azz);
				__disable_irq();
				sommaEnergie(&data[8],array2u32(&messaggio[pwOff+2]),array2u32(&messaggio[pwOff+6]));
				__enable_irq();
				HAL_UART_Transmit(&huart2,&identificativo[0],16,1000);
				HAL_UART_Transmit(&huart2,&data[0],112,1000);				
				break;
				
			case 0x07: //media misurandi
				u322array(&data[0],latitudine);
				u322array(&data[4],longitudine);
				__disable_irq();
				mediaMisurandi(&data[8],array2u32(&messaggio[pwOff+2]),array2u32(&messaggio[pwOff+6]));
				__enable_irq();
							
				__disable_irq();
				HAL_UART_Transmit(&huart2,&identificativo[0],16,1000);
				HAL_UART_Transmit(&huart2,&data[0],120,1000);			
				__enable_irq();					
					
				break;		
					
			case 0x08: //download eventi sovracorrente 
				HAL_UART_Transmit(&huart2,&identificativo[0],16,1000);
				u162array(&data[0],sogliaCorrenteA);
				u162array(&data[2],sogliaCorrenteB);
				HAL_UART_Transmit(&huart2,&data[0],4,1000);
				__disable_irq();
				downloadGuasti();		
				__enable_irq();					
				break;
				
				
			case 0x09: //download eventi neutro 
				HAL_UART_Transmit(&huart2,&identificativo[0],16,1000);
				u162array(&data[0],sogliaNeutro);
				HAL_UART_Transmit(&huart2,&data[0],2,1000);
				__disable_irq();
				estrazioneNeutro();		
				__enable_irq();					
				break;
												
			case 0x0a: //imposta numero a cui mandare gli allarmi
				a = 0;
				while(messaggio[pwOff+2+a] != 0x0a && a < 20){
					numeroAllarmi[a] = messaggio[pwOff+2+a];
					a++;
				}
				a--;
				while(a < 20){
					numeroAllarmi[a] = 0;
					a++;
				}
				
				addressFram[0] = 1; addressFram[1] = 47;
				saveArrayFram(&numeroAllarmi[0],&addressFram[0],20);
					
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
				
			case 0x0b: //imposta il numero della SIM
				a = 0;
				while(messaggio[pwOff+2+a] != 0x0a && a < 20){
					numeroDevice[a] = messaggio[pwOff+2+a];
					a++;
				}
				a--;
				while(a < 20){
					numeroDevice[a] = 0;
					a++;
				}
			
				addressFram[0] = 2; addressFram[1] = 211;
				saveArrayFram(&numeroDevice[0],&addressFram[0],20);
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
				
			case 0x0c: //imposta data e ora
				A = array2u32(&messaggio[pwOff+2]);
				impostaOra(A);
				sprintf(uart,"set: %d\n",A);
				HAL_UART_Transmit(&huart1,uart, strlen(uart),100);
				estremiDSTposix(A);
			
			
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
				
			case 0x0d: //attiva sincronizzazione automatica
				autoSyncActive = messaggio[pwOff+2];
				addressFram[0] = 1;	addressFram[1] = 67;
				saveArrayFram(&autoSyncActive,&addressFram[0],1);
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
					
			case 0x0e: //imposta identificativo
				if(messaggio[pwOff+2] == 0){messaggio[pwOff+2] = 0x20;}	if(messaggio[pwOff+3] == 0){messaggio[pwOff+3] = 0x20;}	if(messaggio[pwOff+4] == 0){messaggio[pwOff+4] = 0x20;}	if(messaggio[pwOff+5] == 0){messaggio[pwOff+5] = 0x20;}
				if(messaggio[pwOff+6] == 0){messaggio[pwOff+6] = 0x20;}	if(messaggio[pwOff+7] == 0){messaggio[pwOff+7] = 0x20;}	if(messaggio[pwOff+8] == 0){messaggio[pwOff+8] = 0x20;}	if(messaggio[pwOff+9] == 0){messaggio[pwOff+9] = 0x20;}
				if(messaggio[pwOff+10] == 0){messaggio[pwOff+10] = 0x20;}	if(messaggio[pwOff+11] == 0){messaggio[pwOff+11] = 0x20;}	if(messaggio[pwOff+12] == 0){messaggio[pwOff+12] = 0x20;}	if(messaggio[pwOff+13] == 0){messaggio[pwOff+13] = 0x20;}
				if(messaggio[pwOff+14] == 0){messaggio[pwOff+14] = 0x20;}	if(messaggio[pwOff+15] == 0){messaggio[pwOff+15] = 0x20;}	if(messaggio[pwOff+16] == 0){messaggio[pwOff+16] = 0x20;}	if(messaggio[pwOff+17] == 0){messaggio[pwOff+17] = 0x20;}
							
				copiaArray(&identificativo[0],&messaggio[pwOff+2],16);
				addressFram[0] = 1;	addressFram[1] = 0; //salvo in FRAM
				saveArrayFram(&identificativo[0],&addressFram[0],16);
				offset[0] = 0; offset[1] = 16; //salvo in Tag NFC
				writeNFC(&identificativo[0],16,&offset[0]);
				cambioNomeBT = 1;
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
			
			case 0x0f: //imposta soglia linea A
				modificaSogliaA(array2u16(&messaggio[pwOff+2]));
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
			
			case 0x10: //imposta soglia linea B
				modificaSogliaB(array2u16(&messaggio[pwOff+2]));
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
			
			case 0x11: //imposta soglia neutro
				modificaSogliaN(array2u16(&messaggio[pwOff+2]));
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
				
			case 0x13: //update
				paccTot = array2u16(&messaggio[pwOff+2]);
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				updateAttivo = 2;
				break;
				
			case 0x14: //ora legale;
				for(i=0;i<9;i++){ //-48 da rimuovere
					messaggio[pwOff+2+i] -= 48;
					}
				DSTon = messaggio[pwOff+2];
				DSThourStart = messaggio[pwOff+3];
				DSTdayStart = messaggio[pwOff+4];
				DSTweekStart = messaggio[pwOff+5];
				DSTmonthStart = messaggio[pwOff+6];
				DSThourStop = messaggio[pwOff+7];
				DSTdayStop = messaggio[pwOff+8];
				DSTweekStop = messaggio[pwOff+9];
				DSTmonthStop = messaggio[pwOff+10];
				//estremiDSTuniv();
				addressFram[0] = 1; addressFram[1] = 68;
				saveArrayFram(&messaggio[13-6],&addressFram[0],9);
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
				
			case 0x17: //cambio password
					
				while(messaggio[pwOff+2+contPW] != 0x20 && contPW < 17){
					oldPassword[contPW] = messaggio[pwOff+2+contPW];
					contPW++;
				}
				oldPassword[contPW] = 0;
				contPW++;
					
				if(comparaStringhe(&password[0],&oldPassword[0],strlen(password)) == 0 || strlen(password) != strlen(oldPassword)){
					if(BTattivo > 0){
						HAL_UART_Transmit(&huart2,&WP[0],4,1000);		
						HAL_UART_Transmit(&huart1,(u8*)"password errata",15,100);			
					}
				}
				else{
					while(messaggio[pwOff+2+contPW] != 0x0d && contPWnew < 17){
						newPassword[contPWnew] = messaggio[pwOff+2+contPW];
						contPW++;
						contPWnew++;
						}
					newPassword[contPWnew] = 0;
					//salvataggio su FRAM
					HAL_UART_Transmit(&huart1,(u8*)"new:",4,100);
					HAL_UART_Transmit(&huart1,&newPassword[0],strlen(newPassword),100);
					addressFram[0] = 2;	addressFram[1] = 234;
					saveArrayFram(&newPassword[0],&addressFram[0],16);
					copiaArray(&password[0],&newPassword[0],16);
					if(BTattivo == 1){
						HAL_UART_Transmit(&huart2,&OK[0],4,1000);
					}
				}
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
				
			case 0x18: //azzera contatori
				azzeraLineaA = 1;
				azzeraLineaB = 1;			
				azzeraRegistriADE1();
				azzeraRegistriADE3();
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;		
			
			case 0x19: //formatta
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
						
				__disable_irq();
				switch(messaggio[pwOff+2]-48){
						case 0:
							formattaLoad();
							break;
						case 1:
							formattaMeas();
							break;
						case 3:
							formattaGuasti();
							break;
						case 2:
							formattaNeutro();
							break;
						case 4:
							formattaTampering();
							break;
					}			
				__enable_irq();
				break;
					
			case 0x1a: //imposta coordinate GPS
				latitudine = array2u32(&messaggio[pwOff+2]);
				longitudine = array2u32(&messaggio[pwOff+6]);
				addressFram[0] = 1; addressFram[1] = 77;
				saveU32fram(latitudine,&addressFram[0]);
				addressFram[1] = 81;
				saveU32fram(longitudine,&addressFram[0]);
				
				latitudineD = latitudine;	latitudineD /= 1000000;
				longitudineD = longitudine; longitudineD /= 1000000;
							
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
						
			case 0x1c: //attivazione antifurto
				antifurtoAttivo = messaggio[pwOff+2]-48;
				addressFram[0] = 1; addressFram[1] = 85;
				saveArrayFram(&antifurtoAttivo,&addressFram[0],1);
					
				__disable_irq();
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
							
				if(antifurtoAttivo == 1){
					inizializzaAntifurto = 1;
				}
						
				if(antifurtoScattato == 1){ //azzero l'antifurto se era scattato
					antifurtoScattato = 0; //azzero antifurto
					addressFram[0] = 1; addressFram[1] = 90;
					saveArrayFram(&antifurtoScattato,&addressFram[0],1); //salvo azzeramento
					addressFramInt = 2048 + 4 + nIntrusioni*8;
					u162array(&addressFram[0],addressFramInt);
					u322array(&data[0],myTimeVar);
					saveArrayFram(&data[0],&addressFram[0],4); //salvo fine evento						
				}
				__enable_irq();
				break;

						
			case 0x1d: //download eventi intrusione
				HAL_UART_Transmit(&huart2,&identificativo[0],16,1000);
				__disable_irq();
				downloadIntrusioni();
				__enable_irq();
				break;
						
			case 0x20: //calibrazione
				calib = messaggio[pwOff+2]-48;
				calib3 = calib;
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
					
			case 0x21:	//calibrazione guasti
				calibOverI();
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
						
			case 0x25:
				coefficientiPositivi();
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;	
						
			case 0x50:	//sms di prova
				smstest = 15;
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
						
					
			//nuovi comandi aggiunti
					
			case 0x51: //set APN
				i = pwOff+2;
				a = 0;
				while(messaggio[i] != 0 && i <= pwOff+52){
					APN[a] = messaggio[i];
					i++;
					a++;
				}
				while(a<50){
					APN[a] = 0;
					a++;
				}
						
				addressFram[0] = 2; addressFram[1] = 1;
				saveArrayFram(&APN[0],&addressFram[0],50);
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
					
			case 0x52: //set mySQL parameters
				i = pwOff+2;
				a = 0;
				while(messaggio[i] != 0 && i <= pwOff+102){
					mySQL[a] = messaggio[i];
					i++;
					a++;
				}
				while(a < 100){
					mySQL[a] = 0;
					a++;
				}
				a = 0;
				i=pwOff+102;
					
				while(messaggio[i] != 0 && i <= pwOff+132){
					userSQL[a] = messaggio[i];
					i++;
					a++;
				}
				while(a < 30){
					userSQL[a] = 0;
					a++;
				}
				a = 0;
				i=pwOff+132;
						
				while(messaggio[i] != 0 && i <= pwOff+162){
					pwSQL[a] = messaggio[i];
					i++;
					a++;
				}
				while(a < 30){
					pwSQL[a] = 0;
					a++;
				}
				
				addressFram[0] = 2; addressFram[1] = 51;
				saveArrayFram(&mySQL[0],&addressFram[0],100);
						
				addressFram[0] = 2; addressFram[1] = 151;
				saveArrayFram(&userSQL[0],&addressFram[0],30);
						
				addressFram[0] = 2; addressFram[1] = 181;
				saveArrayFram(&pwSQL[0],&addressFram[0],30);
						
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
						
					
			case 0x53: //attiva internet
				
				statoInternet = messaggio[pwOff+2]-48;
				addressFram[0] = 2;
				addressFram[1] = 0;			
				saveArrayFram(&statoInternet,&addressFram[0],1);
			
				if(statoInternet == 0){
					disattivaInternetFlag = 1;
				}
			
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
					
			case 0x54: //lettura impostazioni
						
				sprintf(uart,"%d",LoadActive);
				HAL_UART_Transmit(&huart2,uart,1,100);
				sprintf(uart,"%d",MeasActive);
				HAL_UART_Transmit(&huart2,uart,1,100);
						
				sprintf(uart,"\0");
				HAL_UART_Transmit(&huart2,numeroDevice,20,100);
				HAL_UART_Transmit(&huart2,numeroAllarmi,20,100);
				HAL_UART_Transmit(&huart2,APN,50,100);
						
				HAL_UART_Transmit(&huart2,mySQL,100,100);
						
				HAL_UART_Transmit(&huart2,userSQL,30,100);
						
				HAL_UART_Transmit(&huart2,pwSQL,30,100);
						
				if(statoInternet == 3){
					sprintf(uart,"11");
					HAL_UART_Transmit(&huart2,uart,2,100);
				}
				else{
					sprintf(uart,"00");
					HAL_UART_Transmit(&huart2,uart,2,100);
				}
				break;
			case 0x55:
				u162array(&data[0],sogliaCorrenteA);
				u162array(&data[2],sogliaCorrenteB);
				u162array(&data[4],sogliaNeutro);
				data[6] = 0x0d;
				data[7] = 0x0a;
				HAL_UART_Transmit(&huart2,data,8,100);
				inviaDebug("comando 0x55\n");
				break;
						
			case 0xff: //controllo password
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				/*if(disconnessione < 7){
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
					disconnessione++;
				}				
				else{
					//HAL_UART_Transmit(&huart2,&WP[0],4,1000);
					//HAL_UART_Transmit(&huart2,(u8*)"+++",3,100);
				}*/
				//sprintf(uart,"disconnessione: %d\n",disconnessione);
				//HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
				break;								
			
		}	
				
	}
}


void eseguiComando4G(uint8_t *messaggio){
	u8 sync[9] = "TIMECff";
	u8 OK_GSM[8] = "4F4B0D0A";
	u8 addressFram[2] = {0,0};
	u16 addressFramInt;
	u32 A,B;
	u8 offset[2];
	int i = 0; //indice
	int a = 0;
	u32 azz = 0;
	u8 uart[100];
	u8 pwOff = 0; //posizione del codice del comando

	

	//autosync
	if(comparaStringhe(&messaggio[0],&sync[0],7) == 1){
		regolaOra = isDST(lastSMS + (seconds - SyncCorr));
		return;
	}
	
	inviaDebug("check passoword\n");
	//controllo password
	if(checkPasswordBT(&messaggio[0]) == 0){
		inviaDebug("negativo\n");		
		return;	
	}
	
	pwOff = strlen(password)+2;
	
	inviaDebug("positivo\ncodice comando ");	
	HAL_UART_Transmit(&huart1,&messaggio[pwOff],2,100);
	inviaDebug("\n");
	

	if(updateAttivo == 0){
		
		if(codiceComando(&messaggio[pwOff]) != 0xff){
			disconnessione = 0;
		}
		
		switch(codiceComando(&messaggio[pwOff])){
			
			case 0x02: //memorizzazione load profiles
				addressFram[1] = 19;
				LoadActive = messaggio[pwOff+2]-48;
				saveArrayFram(&LoadActive,&addressFram[0],1);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
				break;
			
			case 0x03: //memorizzazione meas profiles
				addressFram[1] = 20;
				MeasActive = messaggio[pwOff+2]-48;
				saveArrayFram(&MeasActive,&addressFram[0],1);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
				break;
		
			case 0x06: //somma energie
				
				u322array(&data[0],latitudine);
				u322array(&data[4],longitudine);
				u322array(&data[8],azz);
				__disable_irq();
				sommaEnergie(&data[8],array2u32(&messaggio[pwOff+2]),array2u32(&messaggio[pwOff+6]));
				__enable_irq();

				copiaArray(&string[0],&identificativo[0],16);
				copiaArray(&string[16],&data[0],112);
				byte2string(&string[0],&data[0],128);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&data[0],256);
				break;
				
			case 0x07: //media misurandi
				u322array(&data[0],latitudine);
				u322array(&data[4],longitudine);
				__disable_irq();
				mediaMisurandi(&data[8],array2u32(&messaggio[pwOff+2]),array2u32(&messaggio[pwOff+6]));
				__enable_irq();
				
				copiaArray(&string[0],&identificativo[0],16);
				copiaArray(&string[16],&data[0],120);
				byte2string(&string[0],&data[0],136);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&data[0],272);
				break;		
					
								
								
			case 0x0a: //imposta numero a cui mandare gli allarmi
				a = 0;
				while(messaggio[pwOff+2+a] != 0x0a && a < 20){
					numeroAllarmi[a] = messaggio[pwOff+2+a];
					a++;
				}
				a--;
				while(a < 20){
					numeroAllarmi[a] = 0;
					a++;
				}
				addressFram[0] = 1; addressFram[1] = 47;
				saveArrayFram(&numeroAllarmi[0],&addressFram[0],20);
					
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
				break;
				
			case 0x0b: //imposta il numero della SIM
				a = 0;
				while(messaggio[pwOff+2+a] != 0x0a && a < 20){
					numeroDevice[a] = messaggio[pwOff+2+a];
					a++;
				}
				a--;
				while(a < 20){
					numeroDevice[a] = 0;
					a++;
				}
			
				addressFram[0] = 2; addressFram[1] = 211;
				saveArrayFram(&numeroDevice[0],&addressFram[0],20);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
				break;
				
			case 0x0c: //imposta data e ora
				A = array2u32(&messaggio[pwOff+2]);
				//A = isDST(A);
				regolaOra = A;
				//sethour(A);
				sprintf(uart,"set A: %d\n",A);
				HAL_UART_Transmit(&huart1,uart, strlen(uart),100);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);					
				break;
				
			case 0x0d: //attiva sincronizzazione automatica
				autoSyncActive = messaggio[pwOff+2];
				addressFram[0] = 1;	addressFram[1] = 67;
				saveArrayFram(&autoSyncActive,&addressFram[0],1);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
				break;
					
			case 0x0e: //imposta identificativo
				if(messaggio[pwOff+2] == 0){messaggio[pwOff+2] = 0x20;}	if(messaggio[pwOff+3] == 0){messaggio[pwOff+3] = 0x20;}	if(messaggio[pwOff+4] == 0){messaggio[pwOff+4] = 0x20;}	if(messaggio[pwOff+5] == 0){messaggio[pwOff+5] = 0x20;}
				if(messaggio[pwOff+6] == 0){messaggio[pwOff+6] = 0x20;}	if(messaggio[pwOff+7] == 0){messaggio[pwOff+7] = 0x20;}	if(messaggio[pwOff+8] == 0){messaggio[pwOff+8] = 0x20;}	if(messaggio[pwOff+9] == 0){messaggio[pwOff+9] = 0x20;}
				if(messaggio[pwOff+10] == 0){messaggio[pwOff+10] = 0x20;}	if(messaggio[pwOff+11] == 0){messaggio[pwOff+11] = 0x20;}	if(messaggio[pwOff+12] == 0){messaggio[pwOff+12] = 0x20;}	if(messaggio[pwOff+13] == 0){messaggio[pwOff+13] = 0x20;}
				if(messaggio[pwOff+14] == 0){messaggio[pwOff+14] = 0x20;}	if(messaggio[pwOff+15] == 0){messaggio[pwOff+15] = 0x20;}	if(messaggio[pwOff+16] == 0){messaggio[pwOff+16] = 0x20;}	if(messaggio[pwOff+17] == 0){messaggio[pwOff+17] = 0x20;}
						
				copiaArray(&identificativo[0],&messaggio[pwOff+2],16);
				addressFram[0] = 1;	addressFram[1] = 0; //salvo in FRAM
				saveArrayFram(&identificativo[0],&addressFram[0],16);
				offset[0] = 0; offset[1] = 16; //salvo in Tag NFC
				writeNFC(&identificativo[0],16,&offset[0]);
				cambioNomeBT = 1;
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
				break;
				
			case 0x0f: //imposta soglia linea A
				modificaSogliaA(array2u16(&messaggio[pwOff+2]));
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
				break;
			
			case 0x10: //imposta soglia linea B
				modificaSogliaB(array2u16(&messaggio[pwOff+2]));
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
				break;
			
			case 0x11: //imposta soglia neutro
				modificaSogliaN(array2u16(&messaggio[pwOff+2]));
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
				break;
				
			case 0x12: //invia V I cosphi via GSM
				VIcosGSM(&data[0]);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&data[0],144);
				break;
								
			case 0x15: //ultimo evento sovracorrente (solo GSM)
				copiaArray(&data[0],&identificativo[0],16);
				u322array(&data[16],latitudine);
				u322array(&data[20],longitudine);
				ultimoGuasto(&data[24]);
				byte2string(&data[0],&string[0],40);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&string[0],80);
				break;
				
			case 0x16: //ultimo evento neutro (solo GSM)
				copiaArray(&data[0],&identificativo[0],16);
				u322array(&data[16],latitudine);
				u322array(&data[20],longitudine);
				ultimoNeutro(&data[24]);
				byte2string(&data[0],&string[0],56);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&string[0],112);
				break;
				
			case 0x1b: //richiesta stato batteria GSM
				copiaArray(&data[0],&identificativo[0],16);
				u322array(&data[16],latitudine);
				u322array(&data[20],longitudine);
				data[24] = batteryLevel;
				data[25] = 0x0d;
				data[26] = 0x0a;
				byte2string(&data[0],&string[0],27);
				inviaSMS(&lastNumber[0],strlen(lastNumber),&string[0],54);
				break;
					
						
			case 0x1c: //attivazione antifurto
				antifurtoAttivo = messaggio[pwOff+2]-48;
				addressFram[0] = 1; addressFram[1] = 85;
				saveArrayFram(&antifurtoAttivo,&addressFram[0],1);
			
				__disable_irq();
				inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
											
				if(antifurtoAttivo == 1){
					inizializzaAntifurto = 1;
				}
						
				if(antifurtoScattato == 1){ //azzero l'antifurto se era scattato
					antifurtoScattato = 0; //azzero antifurto
					addressFram[0] = 1; addressFram[1] = 90;
					saveArrayFram(&antifurtoScattato,&addressFram[0],1); //salvo azzeramento
					addressFramInt = 2048 + 4 + nIntrusioni*8;
					u162array(&addressFram[0],addressFramInt);
					u322array(&data[0],myTimeVar);
					saveArrayFram(&data[0],&addressFram[0],4); //salvo fine evento						
				}
				__enable_irq();
				break;

						
			case 0x60: //disattivazione BT						
				addressFram[0] = 2;	addressFram[1] = 233;
				if(messaggio[pwOff+2] == 0x30){
					holdDisBT = 1;
					inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
					saveArrayFram(&holdDisBT,&addressFram[0],1);
					timerBT = tempoDisBT;
					forzaDis = 1;
				}
				else if(messaggio[pwOff+2] == 0x31){
					holdDisBT = 0;
					inviaSMS(&lastNumber[0],strlen(lastNumber),&OK_GSM[0],8);
					saveArrayFram(&holdDisBT,&addressFram[0],1);
					forzaAtt = 1;
				}
				break;							
			
		}	
				
	}
}




u8 checkPasswordBT(u8* messaggio){
	u8 passwordLocal[20];
	int i = 0;
	u8 check = 0;
	u8 uart[20];
	
	
	while(messaggio[i] != 0x20 && i < 20){
		passwordLocal[i] = messaggio[i];
		i++;
	}
	passwordLocal[i] = 0;
	
	
	//sprintf(uart,"i:%d len:%d\n",i,strlen(password));
	//HAL_UART_Transmit(&huart1,uart,strlen(uart),100);	
	//HAL_UART_Transmit(&huart1,&messaggio[0],12,100);
	//HAL_UART_Transmit(&huart1,(u8*)"\n",1,100);
	
	if(i != strlen(password)){
		return check;
	}
	
	if(comparaStringhe(&passwordLocal[0], &password[0], i) && strlen(passwordLocal) == strlen(password)){
		check = 1;
	}
	
	
	/*HAL_UART_Transmit(&huart1,&passwordLocal[0],8,100);
	HAL_UART_Transmit(&huart1,(u8*)"\n",1,100);			
	HAL_UART_Transmit(&huart1,&password[0],8,100);
	HAL_UART_Transmit(&huart1,(u8*)"\n",1,100);*/
	//HAL_UART_Transmit(&huart1,(u8*)"CHECK\n",6,100);			
	
return check;	
}

//comando GSM tensione, corrente, fase
int VIcosGSM(u8 *outString){
	u8 array[200];
	int i = 0;
	int size;

	copiaArray(&array[0],&identificativo[0],16); //16
	u322array(&array[16],latitudine); //4
	u322array(&array[20],longitudine); //4
	
	for(i=0;i<3;i++){
		u322array(&array[24+i*4],V[i]); //12
		if(phi1[i] > 90 && phi1[i] <= 270){
			u322array(&array[24+12+i*4],-I1[i]); //12
			u162array(&array[24+36+i*2],cosphi1[i]); //6
		}
		else{
			u322array(&array[24+12+i*4],I1[i]); //12
			u162array(&array[24+36+i*2],cosphi1[i]); //6
		}
		if(phi2[i] > 90 && phi2[i] <= 270){
			u322array(&array[24+24+i*4],-I2[i]); //12
			u162array(&array[24+42+i*2],cosphi2[i]); //6
		}
		else{
			u322array(&array[24+24+i*4],I2[i]); //12
			u162array(&array[24+42+i*2],cosphi2[i]); //6
		}
	}
	
	array[72] = 0x0d;
	array[73] = 0x0a;
	
	byte2string(&array[0],&outString[0],74);
	size = 148;
	
	return size;
}

void bluetoothID(u8* ID){
	int i = 0,a = 0, contachar = 0;
	u8 stringa[50];
	
	sprintf(stringa,"AT+QBTNAME =\x22");
	
	for(i=0;i<16;i++){
		if(ID[i] != 0 && ID[i] != ' '){
			contachar++;
		}
	}
	
	
	while(a<contachar){
		stringa[13+a] = ID[a];
		a++;
	}
	

	stringa[13+contachar] = '\x22';
	stringa[14+contachar] = '\r';
	
	
}



















