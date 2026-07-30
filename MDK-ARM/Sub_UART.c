#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "string.h"
#include "stdio.h"

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

#define BT_UPDATE_PACKET_TOTAL_SIZE 74U
uint8_t updatePacketBT[BT_UPDATE_PACKET_TOTAL_SIZE];
u32 updatePacketBTsize = 0;
u8 updatePacketReady = 0;

uint8_t rx4G[500];
int uartPos4G = 0;
int uartPosOld4G = 0;
uint32_t size4G;
uint8_t messaggioRec4G[500];

/*
 * La ricezione del SIM7600 usa una DMA circolare. L'interrupt IDLE si limita
 * ad accodare i dati ricevuti; stampe e parsing vengono eseguiti dal main.
 * In questo modo l'ISR resta breve e non puo bloccare le altre UART.
 *
 * La coda contiene al massimo UART6_RX_QUEUE_DEPTH - 1 messaggi: uno slot
 * rimane libero per distinguere in modo atomico coda piena e coda vuota.
 */
#define UART6_RX_BUFFER_SIZE       500U
#define UART6_RX_QUEUE_DEPTH       16U
#define UART6_RECOVERY_ERROR       0x01U
#define UART6_RECOVERY_TIMEOUT     0x02U

static uint8_t uart6RxQueue[UART6_RX_QUEUE_DEPTH][UART6_RX_BUFFER_SIZE];
static volatile uint16_t uart6RxQueueLength[UART6_RX_QUEUE_DEPTH];
static volatile uint8_t uart6RxQueueHead = 0U;
static volatile uint8_t uart6RxQueueTail = 0U;
static volatile uint8_t uart6RecoveryPending = 0U;
static volatile uint8_t uart6RecoveryReason = 0U;
static volatile uint8_t uart6QueueOverflowPending = 0U;
static volatile uint32_t uart6LastError = HAL_UART_ERROR_NONE;
static volatile uint32_t uart6ErrorCount = 0U;
static volatile uint32_t uart6QueueOverflowCount = 0U;

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
extern u8 cambioNomeBT;
extern u8 identificativo[16];

/*
 * Cambio nome RN4678
 *
 * Il manuale prescrive di attendere la risposta di ogni comando prima di
 * inviare il successivo. La procedura e quindi gestita come macchina a stati
 * non bloccante e le risposte del modulo vengono intercettate prima che
 * possano essere interpretate come comandi provenienti dall'app.
 */
#define BT_NAME_RESPONSE_TIMEOUT_MS  2000UL
#define BT_NAME_REBOOT_TIMEOUT_MS    6000UL
#define BT_NAME_RECOVERY_TIMEOUT_MS  1000UL
#define BT_NAME_MAX_ATTEMPTS         3U
#define BT_NAME_RX_BUFFER_SIZE       128U

#define BT_NAME_EVENT_NONE           0x00U
#define BT_NAME_EVENT_CMD            0x01U
#define BT_NAME_EVENT_AOK            0x02U
#define BT_NAME_EVENT_ERR            0x04U
#define BT_NAME_EVENT_VERIFIED       0x08U
#define BT_NAME_EVENT_REBOOT         0x10U

typedef enum
{
	BT_NAME_STATE_IDLE = 0,
	BT_NAME_STATE_WAIT_CMD,
	BT_NAME_STATE_WAIT_SN_AOK,
	BT_NAME_STATE_WAIT_VERIFY,
	BT_NAME_STATE_WAIT_REBOOT,
	BT_NAME_STATE_WAIT_RECOVERY_CMD
} BtNameState_t;

static volatile BtNameState_t btNameState = BT_NAME_STATE_IDLE;
static volatile u8 btNameEvents = BT_NAME_EVENT_NONE;
static volatile u16 btNameRxLength = 0U;
static u8 btNameRxBuffer[BT_NAME_RX_BUFFER_SIZE];
static u8 btNameRequested[17];
static u8 btNameRequestedLength = 0U;
static u8 btNameAttempt = 0U;
static u32 btNameDeadline = 0UL;

static u8 acquisisciRispostaCambioNomeBT(const u8 *messaggio, u32 lunghezza);

void USART2_IRQHandler(void)
{
	int i = 0;
	u8 uart[80];
	
  HAL_UART_IRQHandler(&huart2);

	for(i=0;i<500;i++){messaggioRecBT[i] = 0;} //azzero vettore UART
		
	uartPosBT = 500 - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5); // acquisizione dati in ingresso dal BT
	sizeBT = RicMsg(&rxBT[0],&messaggioRecBT[0],uartPosBT,uartPosOldBT,500);
	uartPosOldBT = uartPosBT;
	
	sprintf(uart,"BT rx bytes: %u\n", (unsigned int)sizeBT);
	HAL_UART_Transmit(&huart1,uart,strlen(uart),100);
	
	if(updateAttivo == 0){
		/* Fuori dall'aggiornamento il payload e' testo, quindi si puo' usare strlen. */
		HAL_UART_Transmit(&huart1,messaggioRecBT,strlen((char*)messaggioRecBT),100);
		HAL_UART_Transmit(&huart1,(u8*)"\n",1,100);

		/*
		 * Durante la configurazione del nome, CMD/AOK/ERR e REBOOT sono
		 * risposte locali del RN4678, non pacchetti inviati dall'app.
		 */
		if(acquisisciRispostaCambioNomeBT(&messaggioRecBT[0],sizeBT) != 0U){
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);
			return;
		}
		
		if(comparaStringhe(&messaggioRecBT[0],(u8*)"%CONNECT",8) && strlen((char*)messaggioRecBT) > 25){
			eseguiComandoBT(&messaggioRecBT[35]);
			inviaDebug((u8*)"\ncomando\n");
		}
		else if(comparaStringhe(&messaggioRecBT[0],(u8*)"%RFCOMM_OPEN%",13) && strlen((char*)messaggioRecBT) > 20){
			eseguiComandoBT(&messaggioRecBT[13]);
			inviaDebug((u8*)"\ncomando\n");
		}
		else if(messaggioRecBT[0] != '%'){
			eseguiComandoBT(&messaggioRecBT[0]);
		}
	}
	else{
		/* Durante l'update il pacchetto e' binario: non usare strlen e non interpretarlo come testo.
		   Accumulo eventuali frammenti fino ai 74 byte attesi: 2 byte numero pacchetto + 72 byte dati. */
		if(updatePacketReady == 0 && programmaPacchetto == 0){
			for(i=0; i<(int)sizeBT && updatePacketBTsize < BT_UPDATE_PACKET_TOTAL_SIZE; i++){
				updatePacketBT[updatePacketBTsize] = messaggioRecBT[i];
				updatePacketBTsize++;
			}
			
			if(updatePacketBTsize >= BT_UPDATE_PACKET_TOTAL_SIZE){
				updatePacketReady = 1;
				programmaPacchetto = 1;
				updateAttivo = 15;
			}
			else{
				sprintf(uart,"BT update partial packet: %u/74\n", (unsigned int)updatePacketBTsize);
				inviaDebug(uart);
			}
		}
		else{
			inviaDebug("BT update packet ignored: previous packet still pending\n");
		}
	}
	
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
}

void USART6_IRQHandler(void)
{
	uint32_t statusRegister;
	uint32_t controlRegister1;
	uint32_t errorFlags;
	uint8_t nextHead;
	uint32_t receivedSize;

	statusRegister = READ_REG(huart6.Instance->SR);
	controlRegister1 = READ_REG(huart6.Instance->CR1);
	errorFlags = statusRegister &
		(uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE);

	/*
	 * Gli errori devono essere affidati alla HAL prima di pulire IDLE:
	 * la macro di pulizia legge SR e DR e cancellerebbe anche FE/NE/ORE,
	 * impedendo alla HAL di registrare correttamente la causa.
	 */
	if(errorFlags != 0U){
		HAL_UART_IRQHandler(&huart6);
		return;
	}

	if(((statusRegister & USART_SR_IDLE) != 0U) &&
	   ((controlRegister1 & USART_CR1_IDLEIE) != 0U)){
		/* Pulizia immediata: evita che lo stesso IDLE generi altri interrupt. */
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);

		uartPos4G = (int)(UART6_RX_BUFFER_SIZE -
			LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1));

		if(uartPos4G != uartPosOld4G){
			nextHead = (uint8_t)((uart6RxQueueHead + 1U) %
				UART6_RX_QUEUE_DEPTH);

			if(nextHead != uart6RxQueueTail){
				receivedSize = (uint32_t)RicMsg(
					&rx4G[0],
					&uart6RxQueue[uart6RxQueueHead][0],
					uartPos4G,
					uartPosOld4G,
					(int)UART6_RX_BUFFER_SIZE
				);

				if(receivedSize > 0U){
					uart6RxQueueLength[uart6RxQueueHead] =
						(uint16_t)receivedSize;
					uart6RxQueueHead = nextHead;

					/*
					 * Il timeout viene ricaricato soltanto per dati reali.
					 * Un interrupt vuoto non puo quindi mantenere occupata
					 * indefinitamente la macchina a stati del modem.
					 */
					timerModuloESC = timerModuloESCinit;
				}
			}
			else{
				/*
				 * Il main e temporaneamente piu lento del modem. Scartiamo
				 * questo frammento, ma segnaliamo l'evento fuori dall'ISR.
				 */
				uart6QueueOverflowCount++;
				uart6QueueOverflowPending = 1U;
				timerModuloESC = timerModuloESCinit;
			}

			/* I byte sono stati accodati oppure scartati consapevolmente. */
			uartPosOld4G = uartPos4G;
		}
		return;
	}

	/* Gestione HAL degli eventuali interrupt UART standard (TXE, TC, RXNE). */
	HAL_UART_IRQHandler(&huart6);
}

/*
 * Callback invocata dalla HAL dopo l'arresto della DMA causato da un errore
 * UART. Qui non si effettuano operazioni bloccanti: il riavvio vero e proprio
 * viene demandato al ciclo principale.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart6){
		uart6LastError |= huart->ErrorCode;
		uart6ErrorCount++;
		uart6RecoveryReason |= UART6_RECOVERY_ERROR;
		uart6RecoveryPending = 1U;

		/* Evita nuovi interrupt IDLE mentre la DMA e ferma. */
		__HAL_UART_DISABLE_IT(&huart6, UART_IT_IDLE);
	}
}

/*
 * Richiede un riallineamento completo della ricezione UART6. Viene usata
 * anche dal timeout della macchina a stati, per recuperare un'eventuale DMA
 * rimasta attiva ma non piu coerente con gli indici software.
 */
void richiediRecuperoUART6(void)
{
	uart6RecoveryReason |= UART6_RECOVERY_TIMEOUT;
	uart6RecoveryPending = 1U;
}

/*
 * Esegue in contesto main il recupero della DMA e processa al massimo un
 * messaggio per giro. Nessuna stampa e nessun parser vengono piu eseguiti
 * dentro USART6_IRQHandler().
 */
void gestisciRicezioneUART6(void)
{
	HAL_StatusTypeDef restartStatus;
	uint8_t localTail;
	uint16_t localLength;
	uint8_t recoveryReason;
	uint32_t recoveryError;
	u8 uart[100];

	if(uart6RecoveryPending != 0U){
		/*
		 * Salviamo la diagnostica prima che HAL_UART_Receive_DMA azzeri
		 * ErrorCode. IDLE resta disabilitato per tutta la riallineazione.
		 */
		recoveryReason = uart6RecoveryReason;
		recoveryError = uart6LastError;
		uart6RecoveryReason = 0U;
		uart6LastError = HAL_UART_ERROR_NONE;
		uart6RecoveryPending = 0U;

		__HAL_UART_DISABLE_IT(&huart6, UART_IT_IDLE);
		HAL_UART_DMAStop(&huart6);
		__HAL_UART_CLEAR_OREFLAG(&huart6);

		uartPos4G = 0;
		uartPosOld4G = 0;

		restartStatus = HAL_UART_Receive_DMA(
			&huart6,
			&rx4G[0],
			(uint16_t)UART6_RX_BUFFER_SIZE
		);

		if(restartStatus == HAL_OK){
			__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

			if((recoveryReason & UART6_RECOVERY_ERROR) != 0U){
				sprintf(
					(char*)uart,
					"[UART6] DMA RX riavviata: error=0x%08lX count=%lu\n",
					(unsigned long)recoveryError,
					(unsigned long)uart6ErrorCount
				);
			}
			else{
				sprintf((char*)uart,"[UART6] DMA RX riallineata dopo timeout\n");
			}
			inviaDebug(uart);
		}
		else{
			/*
			 * Una DMA ancora occupata verra ritentata al giro successivo.
			 * IDLE resta spento per evitare una nuova raffica di interrupt.
			 */
			uart6RecoveryReason |= recoveryReason;
			uart6LastError |= recoveryError;
			uart6RecoveryPending = 1U;
		}
	}

	if(uart6QueueOverflowPending != 0U){
		uart6QueueOverflowPending = 0U;
		sprintf(
			(char*)uart,
			"[UART6] coda RX piena: scarti=%lu\n",
			(unsigned long)uart6QueueOverflowCount
		);
		inviaDebug(uart);
	}

	/* Estrae un solo messaggio per non monopolizzare il ciclo principale. */
	localTail = uart6RxQueueTail;
	if(localTail == uart6RxQueueHead){
		return;
	}

	localLength = uart6RxQueueLength[localTail];
	if(localLength >= UART6_RX_BUFFER_SIZE){
		localLength = UART6_RX_BUFFER_SIZE - 1U;
	}

	memcpy(
		&messaggioRec4G[0],
		&uart6RxQueue[localTail][0],
		localLength
	);
	messaggioRec4G[localLength] = '\0';
	size4G = localLength;

	/*
	 * Lo slot viene liberato soltanto dopo averne completato la copia.
	 * L'ISR puo quindi continuare ad accodare senza sovrascrivere il dato
	 * che il main sta per interpretare.
	 */
	uart6RxQueueTail = (uint8_t)((localTail + 1U) %
		UART6_RX_QUEUE_DEPTH);

	inviaDebug("messaggio:\n");
	HAL_UART_Transmit(&huart1,messaggioRec4G,size4G,100);
	HAL_UART_Transmit(&huart1,(u8*)"\n",1,100);
	risposteGSM(messaggioRec4G);
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

static u8 btNameIsAlphanumeric(u8 carattere)
{
	if(carattere >= (u8)'0' && carattere <= (u8)'9') return 1U;
	if(carattere >= (u8)'A' && carattere <= (u8)'Z') return 1U;
	if(carattere >= (u8)'a' && carattere <= (u8)'z') return 1U;
	return 0U;
}

/*
 * Converte l'identificativo interno, lungo sempre 16 byte e riempito con
 * spazi, nel parametro SN ammesso dal RN4678: da 1 a 16 caratteri
 * alfanumerici, senza gli spazi di riempimento finali.
 */
static u8 preparaNomeRN4678(void)
{
	int ultimo = 15;
	int i;

	while(ultimo >= 0 &&
	      (identificativo[ultimo] == 0U || identificativo[ultimo] == (u8)' ')){
		ultimo--;
	}

	if(ultimo < 0){
		return 0U;
	}

	btNameRequestedLength = (u8)(ultimo + 1);
	for(i = 0; i <= ultimo; i++){
		if(btNameIsAlphanumeric(identificativo[i]) == 0U){
			return 0U;
		}
		btNameRequested[i] = identificativo[i];
	}
	btNameRequested[btNameRequestedLength] = 0U;
	return 1U;
}

static u8 btNameBufferContains(const u8 *buffer, u16 bufferLength, const u8 *token)
{
	u16 tokenLength;
	u16 i;

	tokenLength = (u16)strlen((const char*)token);
	if(tokenLength == 0U || bufferLength < tokenLength){
		return 0U;
	}

	for(i = 0U; i <= (u16)(bufferLength - tokenLength); i++){
		if(memcmp(&buffer[i],token,tokenLength) == 0){
			return 1U;
		}
	}
	return 0U;
}

/*
 * Azzera in modo atomico l'accumulatore RX e pubblica il nuovo stato prima
 * dell'invio del comando. In questo modo una risposta molto rapida non puo
 * essere persa o attribuita allo stato precedente.
 */
static void preparaAttesaNomeBT(BtNameState_t nuovoStato, u32 timeoutMs)
{
	u32 primask;

	primask = __get_PRIMASK();
	__disable_irq();
	btNameRxLength = 0U;
	btNameRxBuffer[0] = 0U;
	btNameEvents = BT_NAME_EVENT_NONE;
	btNameState = nuovoStato;
	if(primask == 0U){
		__enable_irq();
	}

	btNameDeadline = HAL_GetTick() + timeoutMs;
}

static void inviaIngressoCommandModeBT(void)
{
	preparaAttesaNomeBT(BT_NAME_STATE_WAIT_CMD,BT_NAME_RESPONSE_TIMEOUT_MS);
	HAL_UART_Transmit(&huart2,(u8*)"$$$",3U,100U);
}

static void inviaComandoNomeBT(void)
{
	u8 comando[21];
	u8 lunghezza;

	comando[0] = (u8)'S';
	comando[1] = (u8)'N';
	comando[2] = (u8)',';
	memcpy(&comando[3],&btNameRequested[0],btNameRequestedLength);
	lunghezza = (u8)(3U + btNameRequestedLength);
	comando[lunghezza] = (u8)'\r';
	lunghezza++;

	preparaAttesaNomeBT(BT_NAME_STATE_WAIT_SN_AOK,BT_NAME_RESPONSE_TIMEOUT_MS);
	HAL_UART_Transmit(&huart2,&comando[0],lunghezza,100U);
}

static void inviaVerificaNomeBT(void)
{
	/*
	 * G<char> legge il valore memorizzato dal relativo comando Set:
	 * GN restituisce quindi il nome salvato da SN.
	 */
	preparaAttesaNomeBT(BT_NAME_STATE_WAIT_VERIFY,BT_NAME_RESPONSE_TIMEOUT_MS);
	HAL_UART_Transmit(&huart2,(u8*)"GN\r",3U,100U);
}

static void inviaRiavvioBT(void)
{
	/*
	 * SN modifica la NVM ma diventa effettivo soltanto dopo R,1, come
	 * specificato dal manuale RN4678.
	 */
	preparaAttesaNomeBT(BT_NAME_STATE_WAIT_REBOOT,BT_NAME_REBOOT_TIMEOUT_MS);
	HAL_UART_Transmit(&huart2,(u8*)"R,1\r",4U,100U);
}

static void terminaCambioNomeBT(u8 esitoPositivo)
{
	if(esitoPositivo != 0U){
		inviaDebug((u8*)"[BT-NAME] nome verificato e modulo riavviato\n");
	}
	else{
		/*
		 * Uscita best-effort dal Command mode. Se il modulo fosse gia in
		 * Data mode e disconnesso, la stringa verrebbe semplicemente ignorata.
		 */
		HAL_UART_Transmit(&huart2,(u8*)"---\r",4U,100U);
		inviaDebug((u8*)"[BT-NAME] cambio nome fallito dopo 3 tentativi\n");
	}

	btNameState = BT_NAME_STATE_IDLE;
	btNameEvents = BT_NAME_EVENT_NONE;
	btNameRxLength = 0U;
	btNameAttempt = 0U;
	cambioNomeBT = 0U;
}

static void recuperaCambioNomeBT(void)
{
	u8 escape = 0x1BU;
	u8 uart[100];

	if(btNameAttempt >= BT_NAME_MAX_ATTEMPTS){
		terminaCambioNomeBT(0U);
		return;
	}

	btNameAttempt++;
	sprintf(
		(char*)uart,
		"[BT-NAME] nuovo tentativo %u/%u\n",
		(unsigned int)btNameAttempt,
		(unsigned int)BT_NAME_MAX_ATTEMPTS
	);
	inviaDebug(uart);

	/*
	 * Se siamo ancora in Command mode, ESC scarta eventuali caratteri
	 * incompleti e il modulo risponde con un nuovo prompt CMD>. Se invece
	 * siamo in Data mode non arrivera risposta e il timeout ritentera $$$.
	 */
	preparaAttesaNomeBT(
		BT_NAME_STATE_WAIT_RECOVERY_CMD,
		BT_NAME_RECOVERY_TIMEOUT_MS
	);
	HAL_UART_Transmit(&huart2,&escape,1U,100U);
}

/*
 * Chiamata dall'ISR USART2. Accumula anche risposte frammentate, ma non
 * effettua trasmissioni, ritardi o log in interrupt.
 */
static u8 acquisisciRispostaCambioNomeBT(const u8 *messaggio, u32 lunghezza)
{
	u32 i;
	u16 spazio;

	if(btNameState == BT_NAME_STATE_IDLE){
		return 0U;
	}

	if(lunghezza > 0U){
		spazio = (u16)(BT_NAME_RX_BUFFER_SIZE - 1U - btNameRxLength);
		if(lunghezza > spazio){
			lunghezza = spazio;
		}

		for(i = 0U; i < lunghezza; i++){
			btNameRxBuffer[btNameRxLength] = messaggio[i];
			btNameRxLength++;
		}
		btNameRxBuffer[btNameRxLength] = 0U;
	}

	if(btNameBufferContains(
		   &btNameRxBuffer[0],
		   btNameRxLength,
		   (const u8*)"ERR") != 0U){
		btNameEvents |= BT_NAME_EVENT_ERR;
	}

	switch(btNameState){
		case BT_NAME_STATE_WAIT_CMD:
		case BT_NAME_STATE_WAIT_RECOVERY_CMD:
			if(btNameBufferContains(
				   &btNameRxBuffer[0],
				   btNameRxLength,
				   (const u8*)"CMD") != 0U){
				btNameEvents |= BT_NAME_EVENT_CMD;
			}
			break;

		case BT_NAME_STATE_WAIT_SN_AOK:
			if(btNameBufferContains(
				   &btNameRxBuffer[0],
				   btNameRxLength,
				   (const u8*)"AOK") != 0U){
				btNameEvents |= BT_NAME_EVENT_AOK;
			}
			break;

		case BT_NAME_STATE_WAIT_VERIFY:
			if(btNameBufferContains(
				   &btNameRxBuffer[0],
				   btNameRxLength,
				   &btNameRequested[0]) != 0U){
				btNameEvents |= BT_NAME_EVENT_VERIFIED;
			}
			break;

		case BT_NAME_STATE_WAIT_REBOOT:
			if(btNameBufferContains(
				   &btNameRxBuffer[0],
				   btNameRxLength,
				   (const u8*)"REBOOT") != 0U){
				btNameEvents |= BT_NAME_EVENT_REBOOT;
			}
			break;

		default:
			break;
	}

	return 1U;
}

void gestisciCambioNomeBT(void)
{
	u8 eventi;
	u8 uart[100];

	if(btNameState == BT_NAME_STATE_IDLE){
		if(cambioNomeBT == 0U || BTattivo != 0U || updateAttivo != 0U){
			return;
		}

		if(preparaNomeRN4678() == 0U){
			inviaDebug(
				(u8*)"[BT-NAME] identificativo non valido: usare 1-16 caratteri alfanumerici\n"
			);
			cambioNomeBT = 0U;
			return;
		}

		btNameAttempt = 1U;
		sprintf(
			(char*)uart,
			"[BT-NAME] avvio nome=%s tentativo 1/%u\n",
			(char*)btNameRequested,
			(unsigned int)BT_NAME_MAX_ATTEMPTS
		);
		inviaDebug(uart);
		inviaIngressoCommandModeBT();
		return;
	}

	/*
	 * Una nuova connessione o un aggiornamento firmware hanno priorita.
	 * Il flag resta attivo e la procedura ripartira dopo la disconnessione.
	 */
	if(BTattivo != 0U || updateAttivo != 0U){
		btNameState = BT_NAME_STATE_IDLE;
		btNameEvents = BT_NAME_EVENT_NONE;
		btNameRxLength = 0U;
		btNameAttempt = 0U;
		inviaDebug((u8*)"[BT-NAME] procedura sospesa\n");
		return;
	}

	eventi = btNameEvents;
	if((eventi & BT_NAME_EVENT_ERR) != 0U){
		inviaDebug((u8*)"[BT-NAME] risposta ERR dal modulo\n");
		recuperaCambioNomeBT();
		return;
	}

	switch(btNameState){
		case BT_NAME_STATE_WAIT_CMD:
		case BT_NAME_STATE_WAIT_RECOVERY_CMD:
			if((eventi & BT_NAME_EVENT_CMD) != 0U){
				inviaDebug((u8*)"[BT-NAME] Command mode confermato\n");
				inviaComandoNomeBT();
				return;
			}
			break;

		case BT_NAME_STATE_WAIT_SN_AOK:
			if((eventi & BT_NAME_EVENT_AOK) != 0U){
				inviaDebug((u8*)"[BT-NAME] comando SN accettato\n");
				inviaVerificaNomeBT();
				return;
			}
			break;

		case BT_NAME_STATE_WAIT_VERIFY:
			if((eventi & BT_NAME_EVENT_VERIFIED) != 0U){
				inviaDebug((u8*)"[BT-NAME] valore NVM verificato\n");
				inviaRiavvioBT();
				return;
			}
			break;

		case BT_NAME_STATE_WAIT_REBOOT:
			if((eventi & BT_NAME_EVENT_REBOOT) != 0U){
				terminaCambioNomeBT(1U);
				return;
			}
			break;

		default:
			break;
	}

	if((int32_t)(HAL_GetTick() - btNameDeadline) >= 0){
		if(btNameState == BT_NAME_STATE_WAIT_RECOVERY_CMD){
			/* ESC senza risposta: probabilmente eravamo in Data mode. */
			inviaIngressoCommandModeBT();
		}
		else{
			inviaDebug((u8*)"[BT-NAME] timeout risposta RN4678\n");
			recuperaCambioNomeBT();
		}
	}
}



