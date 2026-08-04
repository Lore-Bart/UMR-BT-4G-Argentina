
#include "main.h"
#include "stm32f4xx_hal.h"
#define DmaBufSize 20
#define	I2Cfram 0xae
#define PI 3.14159
#define timeoutModulo2 5
#define timeoutModulo 64000
#define timeoutSMS 60
#define timeoutSMS2 5
#define	VAR 0
#define tempoDisBT 300
#define timerModuloESCinit	15
//#define sogliaI 500

#define SMS_ALARM_FRAM_PAGE           3U
#define SMS_ALARM_FRAM_OFFSET         51U
#define SMS_ALARM_FRAM_MARKER         0xA5U
#define AUTO_THRESHOLD_FRAM_PAGE      3U
#define AUTO_THRESHOLD_FRAM_OFFSET    53U
#define AUTO_THRESHOLD_FRAM_MARKER    0xA6U
#define OVERCURRENT_INTERVAL_FRAM_PAGE   3U
#define OVERCURRENT_INTERVAL_FRAM_OFFSET 55U
#define OVERCURRENT_INTERVAL_FRAM_MARKER 0xA7U
#define BAT_BACKUP_TIME_FRAM_PAGE         3U
#define BAT_BACKUP_TIME_FRAM_OFFSET       57U
#define BAT_BACKUP_TIME_FRAM_MARKER       0xA8U

/*
 * Configurazione dell'autenticazione PDP dell'APN per il SIM7600.
 * I 62 byte occupati, dalla posizione 61 alla 122 della pagina 3, sono:
 * marcatore, tipo autenticazione, username (30 byte), password (30 byte).
 * Il marcatore permette di riconoscere una FRAM proveniente da firmware
 * precedenti e di importare una sola volta le vecchie credenziali.
 */
#define APN_AUTH_FRAM_PAGE                3U
#define APN_AUTH_FRAM_OFFSET              61U
#define APN_AUTH_FRAM_MARKER              0xAAU
#define APN_AUTH_NONE                     0U
#define APN_AUTH_PAP                      1U
#define APN_AUTH_CHAP                     2U
#define APN_AUTH_PAP_OR_CHAP              3U
#define APN_AUTH_USER_SIZE                30U
#define APN_AUTH_PASSWORD_SIZE            30U
#define APN_AUTH_TEXT_MAX_LENGTH           (APN_AUTH_USER_SIZE - 1U)

/*
 * Gestione automatica dell'ora legale europea per RTC mantenuto in ora
 * locale (CET/CEST):
 * - 0 mantiene il comportamento storico del firmware Argentina e non
 *   modifica mai l'RTC per l'ora legale;
 * - 1 porta l'orologio dalle 02:00 alle 03:00 l'ultima domenica di marzo
 *   e dalle 03:00 alle 02:00 l'ultima domenica di ottobre.
 *
 * Il timestamp ricevuto dai comandi continua a essere interpretato come
 * ora locale codificata in formato epoch: quando viene impostato non viene
 * quindi applicata alcuna correzione aggiuntiva.
 *
 * Sono ammessi esclusivamente i valori 0 e 1.
 */
#define EUROPEAN_DST_ENABLED               0U

#if (EUROPEAN_DST_ENABLED != 0U) && (EUROPEAN_DST_ENABLED != 1U)
#error "EUROPEAN_DST_ENABLED deve valere 0 oppure 1"
#endif

/*
 * Stato persistente dell'ora legale. Il marcatore consente di riconoscere
 * dispositivi aggiornati da firmware precedenti senza spostare l'orologio
 * alla prima accensione. Gli offset 59 e 60 della pagina 3 sono successivi
 * alle altre impostazioni attualmente presenti nella stessa pagina.
 */
#define EUROPEAN_DST_FRAM_PAGE             3U
#define EUROPEAN_DST_FRAM_OFFSET           59U
#define EUROPEAN_DST_FRAM_MARKER           0xA9U
#define EUROPEAN_DST_STANDARD              0U
#define EUROPEAN_DST_SUMMER                1U

/*
 * Tempo di funzionamento massimo senza rete:
 * - valore memorizzato e usato in minuti;
 * - 0 disabilita lo spegnimento automatico temporizzato;
 * - 1...60 imposta il timeout;
 * - 5 minuti e il valore applicato alla prima installazione del firmware
 *   o quando in FRAM non e presente una configurazione valida.
 */
#define BAT_BACKUP_TIME_DEFAULT_MIN       2U
#define BAT_BACKUP_TIME_MAX_MIN           60U

/*
 * Ritardo fisso fra il riconoscimento della mancanza rete e l'ingresso
 * effettivo in modalita low power.
 *
 * Il valore e espresso in secondi e puo essere modificato soltanto
 * ricompilando il firmware. Durante questo intervallo gli ADE e i rivelatori
 * di picco restano attivi, ma i profili load e meas sono gia sospesi perche
 * alimentatore vale 0.
 *
 * 0 mantiene il comportamento immediato precedente.
 */
#define LOWPOWER_ENTRY_DELAY_SEC           3U

/*
 * Intervalli fra i controlli periodici della tensione batteria:
 * - con alimentatore presente resta il controllo ogni 15 secondi;
 * - con alimentatore assente il controllo viene diradato a 30 secondi
 *   per ridurre il consumo introdotto dal circuito di misura.
 *
 * I valori sono espressi in secondi e devono essere maggiori di zero.
 */
#define BAT_CHECK_AC_INTERVAL_SEC          15U
#define BAT_CHECK_BACKUP_INTERVAL_SEC      30U

/*
 * Ritardo fra l'accodamento immediato dell'evento di sovracorrente al
 * database e l'accodamento del relativo SMS.
 *
 * Il valore e espresso in millisecondi. Impostando 0 l'SMS viene accodato
 * immediatamente dopo la richiesta database. Per mantenere affidabile il
 * confronto con HAL_GetTick(), usare valori inferiori a 2^31 ms.
 */
#define OVERCURRENT_SMS_DELAY_MS           5000UL

#define GUASTO_INIBIZIONE_FORMAT         0xFFFFFFFFUL

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

struct dstTime{
	u32 A;
	u32 B;
};

typedef struct
{
    u8 punto1Valido;
    u8 punto2Valido;

    u32 adcPunto1;
    u32 tensionePunto1_mV;

    u32 adcPunto2;
    u32 tensionePunto2_mV;

    double coefficiente_mV_count;
    double offset_mV;
} BatteryCalibration_t;

typedef enum
{
    BAT_STATE_NO_SUPPLY = 0,
    BAT_STATE_WAIT_START,
    BAT_STATE_BULK,
    BAT_STATE_TOP_50,
    BAT_STATE_TOP_25,
    BAT_STATE_STOP_CONFIRM,
    BAT_STATE_FULL,
    BAT_STATE_WAIT_TEMPERATURE,
    BAT_STATE_TOO_LOW,
    BAT_STATE_WAKEUP,
    BAT_STATE_FAULT,
    BAT_STATE_PLATEAU_REST
} BatteryState_t;


typedef enum
{
    BAT_FAULT_NONE = 0,
    BAT_FAULT_ADC,
    BAT_FAULT_TEMPERATURE_SENSOR,
    BAT_FAULT_BATTERY_TOO_LOW,
    BAT_FAULT_OVERVOLTAGE,
    BAT_FAULT_TOTAL_TIMEOUT,
    BAT_FAULT_TOP_TIMEOUT,
    BAT_FAULT_CHARGE_STALLED
} BatteryFault_t;

static BatteryCalibration_t calibrazioneBatteria;


void adeinit(void);
int RicMsg(uint8_t* BuffIn, uint8_t* BuffOut, int pos, int oldpos,int sizeArray);
void gestisciRicezioneUART6(void);
void richiediRecuperoUART6(void);
int mymain(void);
uint32_t timetoposix(RTC_DateTypeDef data, RTC_TimeTypeDef ora);
uint8_t WhatWeekDay(RTC_DateTypeDef data);
void sethour(uint32_t A);
void estremiDST(void);
void UpdateTime(void);
void inizializzaGestioneOraLegaleEuropea(void);
u8 gestisciOraLegaleEuropea(void);
u8 cambioOraLegaleEuropeaInAttesa(void);
void copiaArray(uint8_t *OutBuf, uint8_t *InBuf, int size);
void saveArrayFram(uint8_t *data,uint8_t *address,uint16_t size);
void ReadArrayFram(uint8_t *OutBuf,uint8_t *address,uint16_t size);
void writeArrayFlash(uint8_t *InBuf, uint8_t *address, uint16_t size);
void readArrayFlash(uint8_t *OutBuf,uint8_t *address, uint16_t size);
void readFactory(uint8_t *OutBuf);
void saveU32fram(uint32_t inVar,uint8_t *address);
void invertiArray(uint8_t *outBuf, uint16_t size);
void u322array(uint8_t *outBuf,uint32_t inVar);
void saveU16fram(uint16_t inVar,uint8_t *address);
void u162array(uint8_t *outBuf,uint16_t inVar);
long array2long(uint8_t *inBuf);
uint16_t M24SR_UpdateCrc(uint8_t ch, uint16_t *pwCrc);
void M24SR_ComputeCrc(uint8_t *Data, uint8_t Lenght);
void initNFC(void);
void initNFC5(void);
void firstNFCread(uint8_t *outBuf);
void writeNFC(uint8_t *inBuf, uint8_t size, uint8_t *offset);
void readNFC(uint8_t *outBuf, uint8_t size, uint8_t *offset);
void initGSM(void);
void inviaSMSprova(void);
void inviaStringa(uint8_t seriale, uint8_t *messaggio, int size);
void primoAvvio(void);
uint8_t comparaStringhe(uint8_t *stringa1, uint8_t *stringa2, int size);
uint8_t ascii2byte(uint8_t inVar);
uint8_t codiceComando(uint8_t *inBuf);
void eseguiComandoBT(uint8_t *messaggio);
void inviaSMS(uint8_t *numero, int sizeNumero, uint8_t *messaggio,int sizeMessaggio);
void leggiSMS(uint8_t *numeroSMS);
int pulisciSMS(uint8_t *inBuf, uint8_t *outBuf);
void risposteGSM(uint8_t *messaggio);
void delay(long dato);
long array2u32(uint8_t *inBuf);
void avvioSistema(void);
void salvaLoad(void);
void FormatSector(uint8_t *address);
void fram2flashLoad(uint16_t pagina, u8 riga);
void salvaMeas(void);
void fram2flashMeas(uint16_t pagina, u8 riga);
void ultimoLoad(uint8_t *outBuf);
void ultimoMeas(uint8_t *outBuf);
void misuratempo(void);
void filtroLoad(u32 estremoA, u32 estremoB);
void filtroMeas(u32 estremoA, u32 estremoB);
u32 readRegADE1(u8 *registro);
void autoSync (void);
void requestSignal(void);
void initSMStextMode(void);
void refreshSignal(u8 *messaggio);
void adeinit3(void);
void acquisizioni(void);
void acquisizioni3(void);
u32 readRegADE3(u8 *registro);
void elabMisure(void);
void elabMisure3(void);
void calcolatutto(void);
void calcolatutto3(void);
void dashboard(u8 *outBuf);
void salvataggio(void);
u8 dispari(int dato);
u8 multiplotre(int dato);
u32 saveInterval(u8 active);
long absLong(long value);
u16 array2u16(uint8_t *inBuf);
int array2int(uint8_t *inBuf);
void attivaBT(void);
void estrazioneNeutro(void);
void sommaEnergie(u8 *outBuf, u32 min, u32 max);
void mediaMisurandi(u8 *outBuf,u32 min,u32 max);
void downloadNeutro(void);
u8 progPacchetto(u8 *inBuf, u16 nPacchetto, u16 paccTot);
void MX_IWDG_Init(void);
void acquisisciCorrenti(void);
void calibOverI(void);
void calcolaCorrenti(void);
void controllaGuasto(void);
void downloadGuasti(void);
void checkNeutro(void);
void oraEstesa(uint32_t A);
u8 bisestile(u8 anno);
void estremiDSTuniv(void);
void saveArrayFram100(uint8_t *data,uint8_t *address,uint16_t size);
void saveArrayFram(uint8_t *data,uint8_t *address,uint16_t size);
void formattaMemoria(void);
void writeArrayFlashBefore(uint8_t *InBuf, uint8_t *address, uint16_t size);
void blockErase (int block);
u8 blockEraseAndWait (int block);
u8 readFlashStatus(void);
u8 waitFlashReady(u16 timeout10ms);
void formattaLoad(void);
void formattaMeas(void);
u16 contaLoadProfilesResidui(void);
u16 contaMeasProfilesResidui(void);
u16 contaLoadProfilesFramResidui(void);
u16 contaMeasProfilesFramResidui(void);
void formattaGuasti(void);
void formattaNeutro(void);
void byte2string(u8 *inByte, u8 *outBuf, int len);
u8 byte2char(u8 byte);
void string2byte(u8 *inString, u8* outByte, int len);
int VIcosGSM(u8 *outString);
void attivazioneAntifurto(void);
void controlloAntifurto(void);
void boot(void);
void sbloccaFlash(void);
void modificaSogliaA(int value);
void modificaSogliaB(int value);
void modificaSogliaN(u16 value);
void modificaSogliaUnderVoltage(u16 value);
void modificaSogliaOverVoltage(u16 value);
void ultimoNeutro(u8 *outBuf);
void ultimoGuasto(u8 *outBuf);
void downloadIntrusioni(void);
u8 multiplocinque(int dato);
u8 controllaBatteria(void);
void gestisciInterruzioneReteCarica(void);
void gestisciTimeoutFunzionamentoBatteria(void);
u32 acquisizioneADC(int channel);
void controlloAntifurtoProva(void);
void writeNFC16(uint8_t *inBuf, uint8_t size, uint8_t *offset);
void writeNFC32(uint8_t *inBuf, uint8_t size, uint8_t *offset);
u8 writeNFC32Checked(uint8_t *inBuf, uint8_t size, uint8_t *offset);
void killRF(void);
void salvaGuastoFake(void);
void generaEventoSovracorrenteTest(void);
void gestisciInvioSMSGuastoRitardato(void);
u32 isDST(u32 A);
void salvaNeutroFake(void);
void resetWD(void);
void writeNFCafter(uint8_t *inBuf, uint8_t size, uint8_t *offset);
void clearNFCpending(void);
void afterNeutro(void);
void ricalcolaSoglie(void);
void formattaFlashInterna(void);
void reset(void);
u8 progPacchetto3(u8 *inBuf, u16 nPacchetto, u16 paccTot);
void formattaTampering(void);
u8 multiplodieci(int dato);
u8 multiplotrenta(int dato);
void salvaMeasFake(void);
void salvaLoadFake(void);
double controllaBatteriaProva(void);
void azzeraRegistriADE1(void);
void azzeraRegistriADE3(void);
void exTimerGuasti(void);
void inizializzaI2C(void);
void internet(void);
void eseguiComandoTest(uint8_t *messaggio);
void testADE(u8 *risultato);
void testMemorie(void);
u8 testGuasti(void);
void JumpToApp(int app);
void riavvioUMR(void);
void ripetizione(void);
void bluetoothID(u8* ID);
void calibrazioneTest(void);
u32 giornoSettimana(RTC_DateTypeDef data);
void riavvioBT(void);
void inviaSMSpoll(u8 coda);
void esportaParametriInternet(u8* messaggio);
void connettiInternet(void);
void clearDatabaseRequests(void);
void clearDatabaseProfileRequests(void);
void processDatabaseRequests(void);
void gestisciIngressoLowPower(void);
u8 databaseTxIsBusy(void);
void databaseHttpActionResult(u8 *messaggio);
void databaseHttpError(void);
void databaseHttpParaOk(void);
void aggiungiTensioniDB(void);
void aggiungiMeasProfileDB(u8 passaggio);
void aggiungiLoadProfileDB(u8 passaggio);
void formatFlash (void);
void inviaPaginaFlash(void);
void I2C_TransmitNFC(u8* inBuf, int size);
void I2C_ReceiveNFC(u8* outBuf, int size);
void deleteSMS(void);
void preparaLoad(void);
void preparaMeas(void);
void aggiungiGuastoDB(u8 passaggio, u32* corrente);
void aggiungiNeutroStartDB(u8 passaggio, u16* diffStart);
void aggiungiNeutroEndDB(u8 passaggio, u32 timeMax, u16* diffMax, u16* diffEnd);
void initWDgpio(void);
long acquisizioneTemp(void);
void azzeraMisurandi(void);
void recuperaSeriale(void);
void aggiungiIntrusioneDB(u8 passaggio);
void connettiInternet5(void);
void esportaRetePrivata(u8* messaggio);
void disattivaRetePrivata(u8* messaggio);
void modificaSogliaI(u8* messaggio);
void visualizzaSogliaI(u8* messaggio);
void modificaIpot(u8* messaggio);
void visualizzaIpot(u8* messaggio);
void impostaAlarmPhone(u8* messaggio);
void modificaVover(u8* messaggio);
//cybersecurity ACEA
u8 checkPasswordBT(u8* messaggio);
void disattivaBT(void);
u8 findArrow(u8* messaggio);
u32 controllaBatteriaProva5(void);

//aggiornamento automatico
void updateRemoto(void); //
int filtroUpdate(u8* messaggio); //
void downloadPacchetto(u16 pack); //
void installaPacchettoGSM(void); //
void testFlash(void); //
u16 CRC_flash(u32 addStart, u32 lenght); //
u8 controlloOK(u8* messaggio); //
void testAPN(void); //
u8 controlloO(u8* messaggio); //
u8 controlloK(u8* messaggio); //
void downloadPacchettoTest(u16 pack); //
u8 posizioneFlash(void); //
void updateSMS(u8* messaggio); //
void validazioneUpdate(void); //

void reboot(u8* messaggio);
void rebootSMS(void);
void coefficientiPositivi(void);
void filtroLoadFake(void);
void checkUnderVoltage(void);
void modificaVunder(u8* messaggio);
void visualizzaVunder(u8* messaggio);
void checkOverVoltage(void);
void visualizzaVover(u8* messaggio);
void aggiungiUnderDB(u8 passaggio, u32* tensione);
void aggiungiOverDB(u8 passaggio, u32* tensione);
void dbSavings(u8* messaggio);
void aggiungiRebootDB(u8 passaggio);
void inviaDebug(u8* messaggio);
void leggiTensioni(void);
void preparaMeasDB(void);
void aggiungiDebugDB(u8 passaggio);
void attivaDebugDB(u8* messaggio);
void eseguiComando4G(uint8_t *messaggio);
void invia4G(u8* messaggio);
void contaNet(void);
void aggiornaOrarioNTP(void);
void impostaOrarioNTP(u8* messaggio);
RTC_DateTypeDef posix2date(u32 A);
struct dstTime estremiDSTposix(u32 posix);
void impostaOra(u32 A);
void NTPon(u8* messaggio);
void NTPoff(u8* messaggio);
void NTPrefresh(u8* messaggio);
u8 cercaStringa(u8* stringa1, u8* stringa2, int size, u8* pointer);

void Print_ResetFlags(UART_HandleTypeDef *huart);
void cambioNomeBTfunction(u8* stringa);
void gestisciCambioNomeBT(void);
void RTCpolling(void);
void print_reset_cause(void);
void disattivaInternet(u8* messaggio);
void stateInternet(u8* messaggio);
void restartInternet(u8* messaggio);
void scriviSeriale(void);
void produzioneFun(void);

void controllaBatteriaProva2(void);
void calibraTensioneBatteria(u8 numeroPunto, u32 tensioneMultimetro_mV);
void gestisciCalibrazioneBatteria(void);
void resetFaultCaricaBatteria(void);
static u32 convertiAdcBatteria_mV(u32 adc);
static u8 calcolaLivelloBatteria(u32 tensione_mV);
static const char *nomeStatoBatteria(BatteryState_t stato);
static const char *nomeFaultBatteria(BatteryFault_t fault);
void resetFaultCaricaBatteria(void);

	
// Erase internal flash sector with external watchdog service
u8 internalFlashEraseSectorWD(uint32_t sector);
