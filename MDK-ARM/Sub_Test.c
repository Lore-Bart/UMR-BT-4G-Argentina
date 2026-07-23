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
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


extern uint8_t password[4];

extern u8 univoco[5];

extern u32 myTimeVar;


u8 rxTest[500];
int uartPosTest = 0;
int	uartPosOldTest = 0;
int sizeTest = 0;

u8 messaggioRecTest[100];

u8 risultatoTestMemorie[4] = {0,0,0,0x0a};

//variabili misurandi
extern u32 V[3]; //tensioni
extern long I1[3],I2[3]; //correnti
extern uint16_t phi1[3],phi2[3]; //phi
extern long P1[3],P2[3]; //potenze attive
extern long Q1[3],Q2[3]; //potenze reattive
extern int cosphi1[3],cosphi2[3]; //cosfi
extern u32 regolaOra;

//variabili energie
extern uint32_t E1p[3],E2p[3]; //energie attive positive
extern uint32_t E1n[3],E2n[3]; //energie attive negative
extern uint32_t R1p[3],R2p[3]; //energie reattive positive
extern uint32_t R1n[3],R2n[3]; //energie reattive negative

//comandi calibrazione
extern u8 calib;
extern u8 calib3;
extern u8 calibGuasti;

//coefficienti correttivi
extern long corrV[3]; //calibrazione tensione
extern long corrI1[3],corrI2[3]; //calibrazione corrente
extern long corrE1[3],corrE2[3]; //calibrazione energia attiva
extern long corrR1[3],corrR2[3]; //calibrazione energia reattiva
//sovracorrenti
extern u32 calibrazioneI[6];

extern u8 antifurtoScattato;


u8 avviaTestMemorie = 0;

u8 attivaTestMyMain = 0;

u8 RTCuartON = 0;
u8 RTCuartFREQ = 1;

extern u8 segnaleGSM;
extern u8 simulaGuasto;
extern u8 simulaNeutro;

extern uint8_t messaggioRec[100];
extern u8 antifurtoAttivo;
extern u8 inizializzaAntifurto;

u8 testSalvataggio = 0;
extern u8 batteriaInCarica;
extern u8 numeroAllarmi[20];
extern u8 salvataggioLoad;
u8 salvaSeriale = 0;
extern u8 APN[50];
extern u8 attivaInternetFlag;
extern u8 disattivaInternetFlag;
extern u8 statoModulo;
extern u8 statoInternet;
extern u16 nIntrusioni;
extern u8 lastNumber[20];
extern u8 aggiungiNeutroStartDBflag2;
extern u8 numeroAllarmi[20];

extern long corrE1[3],corrE2[3]; //calibrazione energia attiva
extern long corrR1[3],corrR2[3]; //calibrazione energia reattiva

extern u16 packTotGSM;
extern u8 updateGSMatt;
extern u16 NpackRecGSM;
extern u8 packRecFlag;
extern u8 packGSM[500];
extern u16 crcGSM;
extern u16 updateGSMvers;
extern u8 downloadNewPackFlag;
extern u8 timerUpdateGSM;

extern u8 testAPNflag;
extern u16 contaLoadFake;
extern u16 delayFake;
extern u16 underVoltageTH;
extern u8 underVoltageEvent;

extern u8 aggiungiIntrusioneDBflag;
extern u8 stato4G;

extern u8 upAddressGSM[200];

extern RTC_DateTypeDef currentDate;
extern RTC_TimeTypeDef currentTime;

extern u8 salvataggioLoad;
extern u8 salvataggioMeas;

extern u8 mySQL[100]; //vecchio 50
extern u8 userSQL[30]; //vecchio 20
extern u8 pwSQL[30]; //vecchio 20

extern u8 produzione;
extern u16 timerProduzione;
extern u8 serialeDaScrivere[4];
extern u8 timerModuloESC;

void USART1_IRQHandler(void)
{
	u8 offset[2] = {0,16};
	u8 uart[20];

  HAL_UART_IRQHandler(&huart1);
	
	uartPosTest = 500 - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);
	sizeTest = RicMsg(&rxTest[0],&messaggioRecTest[0],uartPosTest,uartPosOldTest,500);
	uartPosOldTest = uartPosTest;
	
	//HAL_UART_Transmit(&huart1,&messaggioRecTest[0],sizeTest,1000);
	eseguiComandoTest(&messaggioRecTest[0]);

	
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	
}


void eseguiComandoTest(uint8_t *messaggio){

	u8 OK[4] = "OK\r\n";
	u8 WP[4] = "WP\r\n";
	u8 risposta[110];
	u8 smsTestMessage[8] = "CIAO";
	u8 identificativoTest[16] = "TEST UMR        ";
	u8 offset[2] = {0,16};

	u16 addressFramInt;
	
	u32 ciao = 23100;
	u16 ciao3 = 200;
	u32 a,b;
	int i;
	u16 light;
	u32 A;
	u8 uart[100];
	u8 addressFram[2];
	u8 data[20];
	u32 annullo = 0;
	u16 sogliaI = 1500;
	
	RTC_DateTypeDef currentDateLocal;
	RTC_TimeTypeDef currentTimeLocal;
	
	
	//controllo password
	/*if(comparaStringhe(&messaggio[0],&password[0],4) == 0){
		HAL_UART_Transmit(&huart1,&WP[0],4,1000);
		return;	
	}
	else{
		HAL_UART_Transmit(&huart1,&OK[0],4,1000);
	}*/
	
		//HAL_UART_Transmit(&huart1,&OK[0],4,1000);
		
		//sprintf(uart,"codice comando %d\n",codiceComando(&messaggio[5]));
		//inviaDebug(uart);
	
		switch(codiceComando(&messaggio[5])){
			
			case 0x01://misurandi
							
				u322array(&risposta[0],V[0]);	u322array(&risposta[4],V[1]);	u322array(&risposta[8],V[2]);
			
				
				if(abs(I1[0]) < sogliaI){
					u322array(&risposta[12],annullo);
				}
				else{
					u322array(&risposta[12],I1[0]);	
				}
				
				if(abs(I1[1]) < sogliaI){
					u322array(&risposta[16],annullo);
				}
				else{
					u322array(&risposta[16],I1[1]);	
				}
				
				if(abs(I1[2]) < sogliaI){
					u322array(&risposta[20],annullo);
				}
				else{
					u322array(&risposta[20],I1[2]);	
				}
				
				if(abs(I2[0]) < sogliaI){
					u322array(&risposta[24],annullo);
				}
				else{
					u322array(&risposta[24],I2[0]);	
				}
				
				if(abs(I2[1]) < sogliaI){
					u322array(&risposta[28],annullo);
				}
				else{
					u322array(&risposta[28],I2[1]);	
				}
				
				if(abs(I2[2]) < sogliaI){
					u322array(&risposta[32],annullo);
				}
				else{
					u322array(&risposta[32],I2[2]);	
				}
				
				u322array(&risposta[36],P1[0]);	u322array(&risposta[40],P1[1]);	u322array(&risposta[44],P1[2]);
				u322array(&risposta[48],P2[0]);	u322array(&risposta[52],P2[1]);	u322array(&risposta[56],P2[2]);
			
				u322array(&risposta[60],Q1[0]);	u322array(&risposta[64],Q1[1]);	u322array(&risposta[68],Q1[2]);
				u322array(&risposta[72],Q2[0]);	u322array(&risposta[76],Q2[1]);	u322array(&risposta[80],Q2[2]);
			
				u162array(&risposta[84],cosphi1[0]);	u162array(&risposta[86],cosphi1[1]);	u162array(&risposta[88],cosphi1[2]);
				u162array(&risposta[90],cosphi2[0]);	u162array(&risposta[92],cosphi2[1]);	u162array(&risposta[94],cosphi2[2]);
			
				u162array(&risposta[96],phi1[0]);		u162array(&risposta[98],phi1[1]);	u162array(&risposta[100],phi1[2]);
				u162array(&risposta[102],phi2[0]);	u162array(&risposta[104],phi2[1]);	u162array(&risposta[106],phi2[2]);
			
				risposta[108] = '\r';
				risposta[109] = '\n';
			
				HAL_UART_Transmit(&huart1,&risposta[0],110,1000);
				break;	

			case 0x02:
				sprintf(uart,"4G = %d stato = %d internet = %d\n", stato4G, statoModulo, statoInternet);
				inviaDebug(uart);
				break;
			case 0x03:
				aggiungiLoadProfileDB(1);
				break;
			case 0x04:
				aggiungiIntrusioneDB(1);
				break;
			case 0x05://ora attuale
				sprintf(uart,"epoch: %d    %d/%d/%d  %d.%d.%d\n", myTimeVar, currentDate.Date,currentDate.Month,currentDate.Year, currentTime.Hours, currentTime.Minutes, currentTime.Seconds);			
				inviaDebug(uart);
				break;
			case 0x07:
				HAL_UART_Transmit(&huart6,(u8*)"AT+HTTPINIT\r",12,100);
				break;
			case 0x08:
				invia4G("AT+HTTPPARA=\x22URL\x22,\x22http://www.a2atestmisure.altervista.org\x22\r");
				delay(200);
				invia4G("AT+HTTPACTION=1\r");
				break;
			case 0x09:
				invia4G("AT+HTTPCLOSE\r");
				break;
			case 0x10:
				aggiornaOrarioNTP();
				break;
			case 0x0a:
				invia4G("AT\r");
				break;
			case 0x0b:
				invia4G("AT+SERVERSTART=8080,1\r");
				break;
			case 0x0c:
				A = array2u32(&messaggio[7]);
				A = isDST(A);
				regolaOra = A;
				//sethour(A);
				sprintf(uart,"set: %d\n",A);
				HAL_UART_Transmit(&huart1,uart, strlen(uart),100);
				break;
			
			case 0x0d:
				HAL_UART_Transmit(&huart6,(u8*)"AT+CGREG?\r", 10,100);
				break;
			
			case 0x0e:
				HAL_UART_Transmit(&huart6,(u8*)"AT+CEREG?\r", 10,100);
				break;
			
			case 0x0f:
				sprintf(uart,"stato: %d\n",statoModulo);
				HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
				break;
			
			case 0x20://calibrazione
				calib = messaggio[7]-48;
				calib3 = messaggio[7]-48;
				HAL_UART_Transmit(&huart1,&OK[0],4,1000);
				break;
			
			case 0x21://calibrazione guasti
				calibOverI();
				HAL_UART_Transmit(&huart1,&OK[0],4,1000);
				break;
			
			case 0x24:
				salvaGuastoFake();			
				break;	
			
			case 0x25:
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);				
				break;	
			
			case 0x27:
				coefficientiPositivi();
				break;
			
			case 0x28:
				sprintf(uart,"http://a2atestmisure.altervista.org/aggiornamento.php");
				uart[strlen(uart)] = 0;
				copiaArray(&upAddressGSM[0],&uart[0],strlen(uart)+1);			
				updateGSMatt = 1;
				NpackRecGSM = 0;
				downloadNewPackFlag = 1;
				break;
			
			
			case 0x30://test memorie
				//avviaTestMemorie = 1;
				testMemorie();			
				break;
			
			case 0x35:
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10);				
				break;
			
			case 0x37:
				if(testSalvataggio == 0){
					testSalvataggio = 1;
				}
				else{
					testSalvataggio = 0;
				}
				break;
				
			case 0x38:
				contaLoadFake = (messaggio[7]-48)*1000 + (messaggio[8]-48)*100 + (messaggio[9]-48)*10 + (messaggio[10]-48);
				break;
			
			case 0x39:
				delayFake = (messaggio[7]-48)*100 + (messaggio[8]-48)*10 + (messaggio[9]-48);
				break;
			
			case 0x40://test ADE
				testADE(&risposta[0]);
				risposta[2] = '\r';
				risposta[3] = '\n';
				HAL_UART_Transmit(&huart1,&risposta[0],4,1000);
				break;
						
			case 0x41:
				if(underVoltageTH == 0){
					underVoltageTH = 1000;
				}
				else{
					underVoltageTH = 0;
				}
				break;
					
			case 0x50://esporta coefficienti di calibrazione
				
				u322array(&risposta[0],corrV[0]);	u322array(&risposta[4],corrV[1]);	u322array(&risposta[8],corrV[2]);
			
				u322array(&risposta[12],corrI1[0]);	u322array(&risposta[16],corrI1[1]);	u322array(&risposta[20],corrI1[2]);
				u322array(&risposta[24],corrI2[0]);	u322array(&risposta[28],corrI2[1]);	u322array(&risposta[32],corrI2[2]);
			
				u322array(&risposta[36],corrE1[0]);	u322array(&risposta[40],corrE1[1]);	u322array(&risposta[44],corrE1[2]);
				u322array(&risposta[48],corrE2[0]);	u322array(&risposta[52],corrE2[1]);	u322array(&risposta[56],corrE2[2]);
			
				u322array(&risposta[60],corrR1[0]);	u322array(&risposta[64],corrR1[1]);	u322array(&risposta[68],corrR1[2]);
				u322array(&risposta[72],corrR2[0]);	u322array(&risposta[76],corrR2[1]);	u322array(&risposta[80],corrR2[2]);
			
				u322array(&risposta[84],calibrazioneI[0]);	u322array(&risposta[88],calibrazioneI[1]);	u322array(&risposta[92],calibrazioneI[2]);
				u322array(&risposta[96],calibrazioneI[3]);	u322array(&risposta[100],calibrazioneI[4]);	u322array(&risposta[104],calibrazioneI[5]);
				
				risposta[108] = '\r';
				risposta[109] = '\n';
			
				HAL_UART_Transmit(&huart1,&risposta[0],110,1000);
				break;
				
			case 0x51://segnale
				requestSignal();
				delay(1000);
				risposta[0] = segnaleGSM;
				risposta[1] = 0x0d;
				risposta[2] = 0x0a;
				HAL_UART_Transmit(&huart1,&risposta[0],3,1000);
				break;

			case 0x52://rilevatori di picco
				risposta[0] = testGuasti();
				risposta[1] = 0x0d;
				risposta[2] = 0x0a;
				HAL_UART_Transmit(&huart1,&risposta[0],3,1000);
				break;
			
			case 0x53://luce
				HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10);
				delay(500);
				light = acquisizioneADC(0);
				u162array(&risposta[0],light);
				risposta[2] = 0x0d;
				risposta[3] = 0x0a;
				HAL_UART_Transmit(&huart1,&risposta[0],4,1000);
				break;
			
			case 0x54: //salva Meas
				preparaLoad();
				preparaMeas();
				
				break;
			
			case 0x55:
				statoInternet = 1;
				sprintf(&APN[0],"ibox.tim.it");
				sprintf(&mySQL[0],"http://a2atestmisure.altervista.org");
				sprintf(&userSQL[0],"a2atestmisure");
				sprintf(&pwSQL[0],"REPL");
			
				HAL_UART_Transmit(&huart2,&OK[0],4,1000);
				break;
			case 0x60://invia SMS di prova
				//inviaSMS((u8*)"3931706199",10,&smsTestMessage[0],4);
				inviaSMSprova();
				HAL_UART_Transmit(&huart1,&OK[0],4,1000);			
				break;
			
			case 0x70: //scrittura NFC
				//inizializzaI2C();
				//writeNFC(&messaggio[7],4,&offset[0]);
				
				offset[1] = 0;
				ReadArrayFram(&OK[0],offset,1);
				OK[0]+=48;
				HAL_UART_Transmit(&huart1,&OK[0],4,1000);		
				break;
			
			case 0x71:
				if(messaggio[7] == 50){
					sprintf(uart,"%d\n",acquisizioneTemp());
					HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
				}
				else if(messaggio[7] == 49){
					sprintf(uart,"%f\n",controllaBatteriaProva());
					HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
				}
				else if(messaggio[7] == 51){
					HAL_UART_Transmit(&huart6,(u8*)"AT+CMGR=5\r",10,100);
				}
				else if(messaggio[7] == 52){
					HAL_UART_Transmit(&huart6,(u8*)"ATI\r",4,100);
				}
				else if(messaggio[7] == 53){
					connettiInternet();					
				}
				else if(messaggio[7] == 55){//7
					aggiungiIntrusioneDBflag = 1;
				}
				else if(messaggio[7] == 56){
					HAL_UART_Transmit(&huart6,(u8*)"AT+NETCLOSE\r\r",12,100);					
				}
				else{
					sprintf(uart,"%d\n",controllaBatteriaProva5());
					HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
				}
				
				break;
			
			case 0x72:
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
				break;
			
			case 0x73:
				inviaDebug("ciao\n");
				controllaBatteriaProva2();
				break;
			case 0x74:
				sprintf(APN,"ibox.tim.it");
				sprintf(mySQL,"http://a2atestmisure.altervista.org");
				sprintf(userSQL,"a2atestmisure");
				sprintf(pwSQL,"REPL");
			
				statoInternet = 1;
				disattivaInternetFlag = 0;					
					
					
					/* Se statoModulo e' rimasto bloccato da una risposta persa,
					 * dopo il timeout lo libero per permettere la riattivazione manuale.
					 */
					if(statoModulo != 0 && timerModuloESC == 0){
						inviaDebug("recupero statoModulo per riavvio internet\n");
						statoModulo = 0;
					}
					
					if(statoModulo == 0){
						connettiInternet();
					}
					
				break;
			
			
			case 0x75:
				preparaLoad();
				preparaMeas();
				break;
			
			case 0x76: //ping
				HAL_UART_Transmit(&huart2,(u8*)"AT+QPING=\x22",10,100);
				HAL_UART_Transmit(&huart2,(u8*)"10.218.7.2",10,100);
				HAL_UART_Transmit(&huart2,(u8*)"\x22,4,4\r",6,100);
				break;
			
			case 0x77:
				calibraTensioneBatteria(1, 6800);
				break;
			case 0x78:
				calibraTensioneBatteria(2, 8100);
				break;
			
			case 0x80: //lettura NFC
				readNFC(&risposta[0],8,&offset[0]);
				if(risposta[5] == 'T'){
					risposta[0] = 1;										
				}
				else{
					risposta[0] = 0;
				}
				
				risposta[1] = 0x0d;
				risposta[2] = 0x0a;
				HAL_UART_Transmit(&huart1,&risposta[0],3,1000);					
				break;
				
			case 0x85: //setAPN
					i = 7;
					a = 0;
					while(messaggio[i] != 0 && i <= 57){
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
					
			case 0x86:
				HAL_UART_Transmit(&huart2,(u8 *)"AT+QBTNAME?\r",12,1000);
				break;
			
			case 0x87:
				recuperaSeriale();
				inviaDebug("seriale da flash: ");
				HAL_UART_Transmit(&huart1,&univoco[0],5,100);
				inviaDebug("\n");
				break;
			
			case 0x88:
				connettiInternet5();
				break;
					
			case 0x89:
				HAL_UART_Transmit(&huart2,(u8 *)"AT+QBTNAME=\x22_ciao\x22\r",19,1000);
				break;
						
			case 0x90: //attiva RTC uart con una certa frequenza
				RTCuartON = messaggio[7] - 48;
				RTCuartFREQ = (messaggio[8]-48)*10 + (messaggio[9]-48);
				break;
			
			case 0x92:
				updateRemoto();
				break;	
			
			case 0x95: //attivazione antifurto
				antifurtoAttivo = messaggio[7]-48;
				addressFram[0] = 1; addressFram[1] = 85;
				saveArrayFram(&antifurtoAttivo,&addressFram[0],1);
													
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
					HAL_UART_Transmit(&huart2,&OK[0],4,1000);						
				break;
						
			case 0x98:
				updateGSMatt = 1;
				NpackRecGSM = 0;
				downloadNewPackFlag = 1;
				//testFlash();
			  //CRC_flash(0x08040200,960);
				break;
			
			case 0x99:
				formattaFlashInterna();
				break;
			
			case 0xbb:
				sprintf(uart,"%d %d %d\n",statoModulo, updateGSMatt, timerUpdateGSM);
				HAL_UART_Transmit(&huart1,uart, strlen(uart),100);
				break;		
			
			case 0xa0:
				simulaGuasto = 1;
				HAL_UART_Transmit(&huart1,&OK[0],4,100);
				break;
			
			case 0xa1:
				simulaNeutro = 1;
				HAL_UART_Transmit(&huart1,&OK[0],4,100);
				break;
			
			case 0xa5:
				sprintf(uart,"Alarm!\nthis cabinet could be open: ----------------\nlat:   long: ");
				inviaSMS(&numeroAllarmi[0],strlen(numeroAllarmi),&messaggio[0],strlen(uart));
				aggiungiIntrusioneDB(1);
				break;
			
			case 0xa9:
				corrE1[0] = absLong(corrE1[0]);	corrE1[1] = absLong(corrE1[1]);	corrE1[2] = absLong(corrE1[2]);
				corrE2[0] = absLong(corrE2[0]);	corrE2[1] = absLong(corrE2[1]);	corrE2[2] = absLong(corrE2[2]);
				corrR1[0] = absLong(corrR1[0]);	corrR1[1] = absLong(corrR1[1]);	corrR1[2] = absLong(corrR1[2]);
				corrR2[0] = absLong(corrR2[0]);	corrR2[1] = absLong(corrR2[1]);	corrR2[2] = absLong(corrR2[2]);
			
				addressFram[0] = 0;
			
				for (i=0; i<3; i++){
					addressFram[1] = 69+i*4;
					saveU32fram(corrE1[i],&addressFram[0]);
					addressFram[1] = 81+i*4;
					saveU32fram(corrR1[i],&addressFram[0]);
				}
				
				for (i=0; i<3; i++){
					addressFram[1] = 177+i*4;
					saveU32fram(corrE2[i],&addressFram[0]);
					addressFram[1] = 189+i*4;
					saveU32fram(corrR2[i],&addressFram[0]);
				}
			
				break;
			case 0xaa: //accuracy
				sprintf(uart,"voltage: %d %d %d\n",corrV[0],corrV[1],corrV[2]);
				inviaDebug(uart);
				sprintf(uart,"current: %d %d %d %d %d %d\n",corrI1[0],corrI1[1],corrI1[2],corrI2[0],corrI2[1],corrI2[2]);
				inviaDebug(uart);
				sprintf(uart,"active energy: %d %d %d %d %d %d\n",corrE1[0],corrE1[1],corrE1[2],corrE2[0],corrE2[1],corrE2[2]);
				inviaDebug(uart);
				sprintf(uart,"reactive energy: %d %d %d %d %d %d\n",corrR1[0],corrR1[1],corrR1[2],corrR2[0],corrR2[1],corrR2[2]);
				inviaDebug(uart);
				sprintf(uart,"fault: %d %d %d %d %d %d\n",calibrazioneI[0],calibrazioneI[1],calibrazioneI[2],calibrazioneI[3],calibrazioneI[4],calibrazioneI[5]);
				inviaDebug(uart);
				break;
						
			case 0xad:
				requestSignal();
				break;

			case 0xaf:
				produzione = 2;
				if(messaggio[7] >= 48 && messaggio[8] >= 48 && messaggio[9] >= 48 && messaggio[10] >= 48 && messaggio[7] <= 57 && messaggio[8] <= 57 && messaggio[9] <= 57 && messaggio[10] <= 57){
					copiaArray(&serialeDaScrivere[0],&messaggio[7],4);
				}
				inviaDebug("seriale inserito: ");
				inviaDebug(serialeDaScrivere);
				inviaDebug("\n");
				break;
			
			}	
			
		
}




void coefficientiPositivi(void){
	
	int i = 0;
	u8 addressFram[2] = {0,0};
	
	corrE1[0] = absLong(corrE1[0]);	corrE1[1] = absLong(corrE1[1]);	corrE1[2] = absLong(corrE1[2]);
	corrE2[0] = absLong(corrE2[0]);	corrE2[1] = absLong(corrE2[1]);	corrE2[2] = absLong(corrE2[2]);
	corrR1[0] = absLong(corrR1[0]);	corrR1[1] = absLong(corrR1[1]);	corrR1[2] = absLong(corrR1[2]);
	corrR2[0] = absLong(corrR2[0]);	corrR2[1] = absLong(corrR2[1]);	corrR2[2] = absLong(corrR2[2]);
	
	addressFram[0] = 0;
			
	for (i=0; i<3; i++){
		addressFram[1] = 69+i*4;
		saveU32fram(corrE1[i],&addressFram[0]);
		addressFram[1] = 81+i*4;
		saveU32fram(corrR1[i],&addressFram[0]);
	}
	
	for (i=0; i<3; i++){
		addressFram[1] = 177+i*4;
		saveU32fram(corrE2[i],&addressFram[0]);
		addressFram[1] = 189+i*4;
		saveU32fram(corrR2[i],&addressFram[0]);
	}
	
}


void testADE(u8 *risultato){
	
	u8 registro[2] = {0x43,0x88};
	u32 lettura = 0;
	u8 uart[100];
	
	risultato[0] = 0;
	risultato[1] = 0;
	
	lettura = readRegADE1(&registro[0]);
	
	if(lettura == 0x0fff8000){
		risultato[0] = 1;	
	}
	
	lettura = 0;
	lettura = readRegADE3(&registro[0]);
	

	if(lettura == 0x0fff8000){
		risultato[1] = 1;	
	}
	
	return;
}



void testMemorie(void){

	u8 addressFram[2] = {0,0};
	u8 lettura = 0;
	u8 addressFlash[3] = {0x3f,0,0};
	u8 scrittura = 6;
	u8 risultato[5] = {0,0,0x0d,0x0a};
	u8 identificativoTest[16] = "UMRtest         ";
	u8 offset[2] = {0,16};
	u8 letturaNFC[4] = "0000";
	
	ReadArrayFram(&risultato[0],&addressFram[0],1);
	
	if(risultato[0] == 1){
		risultatoTestMemorie[0] = 1;
	}
	else{
		risultatoTestMemorie[0] = 0;
	}

	
	lettura = 0;
	writeArrayFlash(&scrittura,&addressFlash[0],1);
	delay(200);
	readArrayFlash(&lettura,&addressFlash[0],1);
	blockErase(63);
	writeNFC(&identificativoTest[0],16,&offset[0]);
	delay(1000);
	readNFC(&letturaNFC[0],7,&offset[0]);
	
	
	if(lettura == 6){
		risultatoTestMemorie[1] = 1;
	}
	else{
		risultatoTestMemorie[1] = 0;
	}

	if(comparaStringhe(&letturaNFC[0],(u8*)"UMRtest",7)){
		risultatoTestMemorie[2] = 1;
	}
	else{
		risultatoTestMemorie[2] = 0;
	}

	//avviaTestMemorie = 2;
	
	
	HAL_UART_Transmit(&huart1,&risultatoTestMemorie[0],4,1000);	
	
}


u8 testGuasti(void){
	
	u8 esito = 1;
	u32 corr[6];
	int i = 0;

	
	while(i<6){
		corr[i] = acquisizioneADC(i+1)/calibrazioneI[i];
		if(corr[i] < 90 || corr[i] > 110){
			esito = 0;
		}
		i++;
	}
	i = 0;

	return esito;
}

void ripetizione(void){
	u8 stringa1[300];
	u8 stringa3[100];
	
	sprintf(stringa1,"http://testmisure.altervista.org/salvaMeas.php?time=%d&V1=%d&V2=%d&V3=%d&I1a=%ld&I2a=%ld&I3a=%ld&I1b=%ld&I2b=%ld&I3b=%d&f1a=%d&f2a=%d&f3a=%d&f1b=%d&f2b=%d&f3b=%d&cf1a=%d&cf2a=%d&cf3a=%d&cf1b=%d&cf2b=%d&cf3b=%d&P1a=%ld&P2a=%ld&P3a=%ld&P1b=%ld&P2b=%ld&P3b=%ld&Q1a=%ld&Q2a=%ld&Q3a=%ld&Q1b=%ld&Q2b=%ld&Q3b=%d\n\n",myTimeVar, V[0],V[1],V[2],I1[0],I1[1],I1[2],I2[0],I2[1],I2[2],phi1[0],phi1[1],phi1[2],phi2[0],phi2[1],phi2[2],cosphi1[0],cosphi1[1],cosphi1[2],cosphi2[0],cosphi2[1],cosphi2[2],P1[0],P1[1],P1[2],P2[0],P2[1],P2[2],Q1[0],Q1[1],Q1[2],Q2[0],Q2[1],Q2[2]);   
	sprintf(stringa3,"http://testmisure.altervista.org/salvaLoad.php?time=%d&E1pa=%d&E2pa=%d&E3pa=%d&E1pb=%d&E2pb=%d&E3pb=%d&E1na=%d&E2na=%d&E3na=%d&E1nb=%d&E2nb=%d&E3nb=%d&R1pa=%d&R2pa=%d&R3pa=%d&R1pb=%d&R2pb=%d&R3pb=%d&R1na=%d&R2na=%d&R3na=%d&R1nb=%d&R2nb=%d&R3nb=%d\n\n",myTimeVar,E1p[0],E1p[1],E1p[2],E2p[0],E2p[1],E2p[2],E1n[0],E1n[1],E1n[2],E2n[0],E2n[1],E2n[2],R1p[0],R1p[1],R1p[2],R2p[0],R2p[1],R2p[2],R1n[0],R1n[1],R1n[2],R2n[0],R2n[1],R2n[2]);


	
	HAL_UART_Transmit(&huart1,stringa3,strlen(stringa3),100);
		
	
}




