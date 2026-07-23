#include "main.h"
#include "stm32f4xx_hal.h"
#include "prototipi.h"
#include "string.h"
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
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;


//numeri
extern u8 lastNumber[10];
extern u8 numeroAllarmi[20];

extern u32 V[3];

extern u32 tempo;

//identificativo
extern u8 identificativo[16];

extern double latitudineD,longitudineD;

u8 batteryLevel = 5;
u8 batteriaInCarica = 0;
u8 messaggioBatteria = 2;

u16 timeCarica = 0;

extern u8 alimentatore;

extern u8 BTattivo;

extern u32 sniff32;
u8 mettoilmeno = 0;

double tensioneB;
int tensioneBint;
long temperatura = 0;

/* ================================================================
 * CALIBRAZIONE BATTERIA
 * ================================================================
 *
 * Dati ottenuti dalla calibrazione a due punti:
 *
 * Punto 1: ADC = 2415, tensione = 6800 mV
 * Punto 2: ADC = 2880, tensione = 8100 mV
 */
#define VBAT_CAL_ADC_1                   2415L
#define VBAT_CAL_MV_1                    6800L

#define VBAT_CAL_ADC_2                   2880L
#define VBAT_CAL_MV_2                    8100L


/*
 * Correzione empirica applicata esclusivamente quando l'alimentatore
 * principale e assente (alimentatore == 0).
 *
 * In questa condizione la tensione letta risulta circa 345 mV piu bassa,
 * verosimilmente per la caduta su un diodo presente nel percorso
 * di alimentazione da batteria.
 *
 * La correzione NON viene applicata durante la carica o con AC presente,
 * perche la calibrazione principale e gia stata verificata in quel caso.
 */
#define VBAT_DISCHARGE_OFFSET_MV          345U


/* ================================================================
 * PARAMETRI DELLA MISURA
 * ================================================================ */

#define BAT_ADC_SAMPLES                  16U
#define BAT_MEASURE_DELAY               400L

/*
 * Massima dispersione ammessa fra minimo e massimo ADC.
 *
 * 80 count equivalgono a circa 224 mV.
 *  un limite volutamente largo, utile per rilevare misure
 * chiaramente instabili.
 */
#define BAT_ADC_MAX_SPREAD               80U

/*
 * Limiti di plausibilit del valore ADC.
 */
#define BAT_ADC_MIN_VALID                1000U
#define BAT_ADC_MAX_VALID                3500U


/* ================================================================
 * SOGLIE DI TENSIONE
 * ================================================================ */

/*
 * Sotto questa tensione la carica automatica non viene avviata.
 * La batteria viene anche scollegata dal carico.
 */
#define VBAT_MIN_AUTOMATIC_CHARGE_MV     6000U

/*
 * Quando manca l'alimentatore, sotto questa tensione PE15
 * scollega la batteria.
 */
#define VBAT_DISCONNECT_MV               6800U

/*
 * Una nuova sessione di carica pu iniziare soltanto sotto 7,75 V.
 */
#define VBAT_RECHARGE_MV                 7750U

/*
 * Inizio della carica pulsata al 50%.
 */
#define VBAT_TOP_50_MV                   7900U

/*
 * Inizio della carica pulsata al 25%.
 */
#define VBAT_TOP_25_MV                   8000U

/*
 * Arresto normale della carica.
 */
#define VBAT_CHARGE_STOP_MV              8070U

/*
 * Fault di sovratensione.
 */
#define VBAT_OVERVOLTAGE_FAULT_MV        8200U


/* ================================================================
 * TEMPERATURA INTERNA DEL MICROCONTROLLore
 * ================================================================
 *
 * acquisizioneTemp() legge ADC_CHANNEL_TEMPSENSOR, quindi misura
 * la temperatura interna dello STM32 e NON quella della batteria.
 */
#define TEMP_MCU_CHARGE_MAX_C             60L
#define TEMP_MCU_CHARGE_RESTART_C         50L
#define TEMP_MCU_EMERGENCY_C             	80L

/* Limiti di plausibilita della misura del sensore interno. */
#define TEMP_SENSOR_MIN_VALID_C          (-40L)
#define TEMP_SENSOR_MAX_VALID_C           125L


/* ================================================================
 * CONTATORI E TIMEOUT
 * ================================================================ */

/*
 * La partenza della carica richiede tre misure consecutive
 * sotto la soglia di riavvio.
 */
#define BAT_START_CONFIRM_COUNT          3U

/*
 * La fine della carica richiede due misure consecutive
 * sopra la soglia di arresto.
 */
#define BAT_STOP_CONFIRM_COUNT           2U

/*
 * Periodo usato alla prima esecuzione, prima che HAL_GetTick()
 * possa calcolare il periodo reale.
 */
#define BAT_DEFAULT_PERIOD_S             30UL

/*
 * Massimo incremento ammesso dei timer in una singola chiamata.
 * Evita che una pausa di debug produca immediatamente un timeout.
 */
#define BAT_MAX_PERIOD_S                 300UL

/*
 * Massimo tempo effettivo con PC1 alto.
 */
#define BAT_MAX_TOTAL_ON_TIME_S          (12UL * 60UL * 60UL)

/*
 * Massimo tempo effettivo con PC1 alto nella parte finale
 * della carica.
 */
#define BAT_MAX_TOP_ON_TIME_S            (2UL * 60UL * 60UL)





/*
 * Variabili mantenute fra una chiamata e la successiva.
 */
static BatteryState_t statoCaricaBatteria = BAT_STATE_NO_SUPPLY;
static BatteryFault_t faultCaricaLatched = BAT_FAULT_NONE;

static u8 sessioneCaricaAttiva = 0;
static u8 caricaCompleta = 0;
static u8 bloccoTemperatura = 0;

static u8 contatoreAvvio = 0;
static u8 contatoreArresto = 0;
static u8 contatoreDutyCycle = 0;

static u32 tempoTotaleCaricaOn_s = 0;
static u32 tempoTopCaricaOn_s = 0;

static u32 ultimoTickBatteria = 0;

/*
 * Converte il valore ADC in millivolt tramite i due punti
 * ottenuti durante la calibrazione.
 *
 * Formula:
 *
 * V = V1 + (ADC - ADC1) * (V2 - V1) / (ADC2 - ADC1)
 *
 * Viene utilizzata aritmetica signed a 64 bit per poter gestire
 * correttamente anche valori ADC inferiori al primo punto.
 */
static u32 convertiAdcBatteria_mV(u32 adc)
{
    int64_t differenzaADC;
    int64_t numeratore;
    int64_t denominatore;
    int64_t tensione_mV;

    differenzaADC =
        (int64_t)adc -
        (int64_t)VBAT_CAL_ADC_1;

    numeratore =
        differenzaADC *
        (
            (int64_t)VBAT_CAL_MV_2 -
            (int64_t)VBAT_CAL_MV_1
        );

    denominatore =
        (int64_t)VBAT_CAL_ADC_2 -
        (int64_t)VBAT_CAL_ADC_1;

    /*
     * Arrotondamento all'intero pi vicino.
     */
    if(numeratore >= 0)
    {
        numeratore += denominatore / 2;
    }
    else
    {
        numeratore -= denominatore / 2;
    }

    tensione_mV =
        (int64_t)VBAT_CAL_MV_1 +
        (numeratore / denominatore);

    /*
     * Protezione da risultati fuori scala.
     */
    if(tensione_mV < 0)
    {
        tensione_mV = 0;
    }

    if(tensione_mV > 20000)
    {
        tensione_mV = 20000;
    }

    return (u32)tensione_mV;
}


/*
 * Calcola la percentuale mostrata all'utente.
 *
 * Il 100% corrisponde al livello operativo scelto di 8,07 V,
 * non alla tensione massima elettrochimica di 8,4 V.
 */
static u8 calcolaLivelloBatteria(u32 tensione_mV)
{
    if(tensione_mV >= VBAT_CHARGE_STOP_MV)
    {
        return 100;
    }
    else if(tensione_mV >= 7950U)
    {
        return 90;
    }
    else if(tensione_mV >= 7780U)
    {
        return 80;
    }
    else if(tensione_mV >= 7650U)
    {
        return 70;
    }
    else if(tensione_mV >= 7570U)
    {
        return 60;
    }
    else if(tensione_mV >= 7500U)
    {
        return 50;
    }
    else if(tensione_mV >= 7450U)
    {
        return 40;
    }
    else if(tensione_mV >= 7410U)
    {
        return 30;
    }
    else if(tensione_mV >= 7300U)
    {
        return 20;
    }
    else if(tensione_mV >= 7050U)
    {
        return 10;
    }

    return 5;
}


static const char *nomeStatoBatteria(BatteryState_t stato)
{
    switch(stato)
    {
        case BAT_STATE_NO_SUPPLY:
            return "NO_SUPPLY";

        case BAT_STATE_WAIT_START:
            return "WAIT_START";

        case BAT_STATE_BULK:
            return "BULK";

        case BAT_STATE_TOP_50:
            return "TOP_50";

        case BAT_STATE_TOP_25:
            return "TOP_25";

        case BAT_STATE_STOP_CONFIRM:
            return "STOP_CONFIRM";

        case BAT_STATE_FULL:
            return "FULL";

        case BAT_STATE_WAIT_TEMPERATURE:
            return "WAIT_TEMP";

        case BAT_STATE_TOO_LOW:
            return "TOO_LOW";

        case BAT_STATE_FAULT:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}


static const char *nomeFaultBatteria(BatteryFault_t fault)
{
    switch(fault)
    {
        case BAT_FAULT_NONE:
            return "NONE";

        case BAT_FAULT_ADC:
            return "ADC";

        case BAT_FAULT_TEMPERATURE_SENSOR:
            return "TEMP_SENSOR";

        case BAT_FAULT_BATTERY_TOO_LOW:
            return "BAT_TOO_LOW";

        case BAT_FAULT_OVERVOLTAGE:
            return "OVERVOLTAGE";

        case BAT_FAULT_TOTAL_TIMEOUT:
            return "TOTAL_TIMEOUT";

        case BAT_FAULT_TOP_TIMEOUT:
            return "TOP_TIMEOUT";

        default:
            return "UNKNOWN";
    }
}


/*
 * Cancella un fault permanente e azzera lo stato di carica.
 *
 * Pu essere richiamata tramite un comando di manutenzione oppure
 * dopo aver verificato la causa del fault.
 */
void resetFaultCaricaBatteria(void)
{
    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_1,
        GPIO_PIN_RESET
    );

    faultCaricaLatched = BAT_FAULT_NONE;
    statoCaricaBatteria = BAT_STATE_NO_SUPPLY;

    sessioneCaricaAttiva = 0;
    caricaCompleta = 0;
    bloccoTemperatura = 0;

    contatoreAvvio = 0;
    contatoreArresto = 0;
    contatoreDutyCycle = 0;

    tempoTotaleCaricaOn_s = 0;
    tempoTopCaricaOn_s = 0;

    batteriaInCarica = 0;
}


static u32 acquisisciADCcalibrazioneBatteria(void)
{
    u32 sommaADC = 0;
    u32 valoreADC;
    u32 valoreMinimo = 0xFFFFFFFF;
    u32 valoreMassimo = 0;
    u32 mediaADC;

    u16 i;

    /*
     * Durante la calibrazione il caricatore deve restare sempre spento.
     */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    batteriaInCarica = 0;

    /*
     * Attiva il carico da 100 ohm e il partitore resistivo.
     *
     * In questo modo la calibrazione viene eseguita nelle stesse
     * condizioni della normale misura della batteria.
     */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

    /*
     * Tempo di stabilizzazione.
     */
    delay(BAT_CAL_SETTLING_TIME_MS);

    /*
     * Acquisisce pi campioni per ridurre rumore e oscillazioni.
     */
    for(i = 0; i < BAT_CAL_ADC_SAMPLES; i++)
    {
        valoreADC = acquisizioneADC(9);

        sommaADC += valoreADC;

        if(valoreADC < valoreMinimo)
        {
            valoreMinimo = valoreADC;
        }

        if(valoreADC > valoreMassimo)
        {
            valoreMassimo = valoreADC;
        }
    }

    /*
     * Disattiva immediatamente il circuito di misura.
     */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

    /*
     * Elimina il campione minimo e il campione massimo.
     */
    sommaADC -= valoreMinimo;
    sommaADC -= valoreMassimo;

    mediaADC = sommaADC / (BAT_CAL_ADC_SAMPLES - 2U);

    return mediaADC;
}

void calibraTensioneBatteria(u8 numeroPunto, u32 tensioneMultimetro_mV)
{
    u32 adcMedio;
    double differenzaTensione;
    double differenzaADC;
    double tensioneVerifica1;
    double tensioneVerifica2;

    char uart[350];

    /*
     * Controllo degli argomenti.
     */
    if(numeroPunto != 1U && numeroPunto != 2U)
    {
        snprintf(
            uart,
            sizeof(uart),
            "[BAT-CAL] ERRORE: il punto deve essere 1 oppure 2\r\n"
        );

        HAL_UART_Transmit(
            &huart1,
            (u8 *)uart,
            strlen(uart),
            200
        );

        return;
    }

    /*
     * Intervallo minimo di plausibilit della tensione impostata.
     */
    if(tensioneMultimetro_mV < 5000U ||
       tensioneMultimetro_mV > 8500U)
    {
        snprintf(
            uart,
            sizeof(uart),
            "[BAT-CAL] ERRORE: tensione multimetro non valida: "
            "%lu mV\r\n",
            (unsigned long)tensioneMultimetro_mV
        );

        HAL_UART_Transmit(
            &huart1,
            (u8 *)uart,
            strlen(uart),
            200
        );

        return;
    }

    /*
     * Acquisizione nelle normali condizioni di misura:
     * PC1 spento, PC0 acceso, carico da 100 ohm inserito.
     */
    adcMedio = acquisisciADCcalibrazioneBatteria();

    /*
     * Controllo di plausibilit dell'ADC.
     */
    if(adcMedio == 0U || adcMedio >= 4095U)
    {
        snprintf(
            uart,
            sizeof(uart),
            "[BAT-CAL] ERRORE ADC: valore non valido: %lu\r\n",
            (unsigned long)adcMedio
        );

        HAL_UART_Transmit(
            &huart1,
            (u8 *)uart,
            strlen(uart),
            200
        );

        return;
    }

    /*
     * Salvataggio del punto acquisito.
     */
    if(numeroPunto == 1U)
    {
        calibrazioneBatteria.adcPunto1 = adcMedio;
        calibrazioneBatteria.tensionePunto1_mV =
            tensioneMultimetro_mV;

        calibrazioneBatteria.punto1Valido = 1;
    }
    else
    {
        calibrazioneBatteria.adcPunto2 = adcMedio;
        calibrazioneBatteria.tensionePunto2_mV =
            tensioneMultimetro_mV;

        calibrazioneBatteria.punto2Valido = 1;
    }

    /*
     * Stampa del punto appena acquisito.
     */
    snprintf(
        uart,
        sizeof(uart),
        "[BAT-CAL] Punto %u acquisito: ADC=%lu, "
        "Vmultimetro=%lu.%03lu V\r\n",
        (unsigned int)numeroPunto,
        (unsigned long)adcMedio,
        (unsigned long)(tensioneMultimetro_mV / 1000U),
        (unsigned long)(tensioneMultimetro_mV % 1000U)
    );

    HAL_UART_Transmit(
        &huart1,
        (u8 *)uart,
        strlen(uart),
        200
    );

    /*
     * Il calcolo pu essere eseguito soltanto quando sono disponibili
     * entrambi i punti.
     */
    if(calibrazioneBatteria.punto1Valido == 0 ||
       calibrazioneBatteria.punto2Valido == 0)
    {
        snprintf(
            uart,
            sizeof(uart),
            "[BAT-CAL] In attesa dell'altro punto di calibrazione\r\n"
        );

        HAL_UART_Transmit(
            &huart1,
            (u8 *)uart,
            strlen(uart),
            200
        );

        return;
    }

    /*
     * Verifica che il secondo punto abbia tensione e ADC maggiori
     * rispetto al primo.
     */
    if(calibrazioneBatteria.adcPunto2 <=
       calibrazioneBatteria.adcPunto1 ||
       calibrazioneBatteria.tensionePunto2_mV <=
       calibrazioneBatteria.tensionePunto1_mV)
    {
        snprintf(
            uart,
            sizeof(uart),
            "[BAT-CAL] ERRORE: il punto 2 deve essere maggiore "
            "del punto 1\r\n"
        );

        HAL_UART_Transmit(
            &huart1,
            (u8 *)uart,
            strlen(uart),
            200
        );

        return;
    }

    differenzaTensione =
        (double)(
            calibrazioneBatteria.tensionePunto2_mV -
            calibrazioneBatteria.tensionePunto1_mV
        );

    differenzaADC =
        (double)(
            calibrazioneBatteria.adcPunto2 -
            calibrazioneBatteria.adcPunto1
        );

    /*
     * Calcolo del coefficiente angolare:
     *
     * coefficiente = variazione tensione / variazione ADC
     *
     * Unit: mV per conteggio ADC.
     */
    calibrazioneBatteria.coefficiente_mV_count =
        differenzaTensione / differenzaADC;

    /*
     * Calcolo dell'offset:
     *
     * offset = V1 - ADC1 * coefficiente
     */
    calibrazioneBatteria.offset_mV =
        (double)calibrazioneBatteria.tensionePunto1_mV -
        (
            (double)calibrazioneBatteria.adcPunto1 *
            calibrazioneBatteria.coefficiente_mV_count
        );

    /*
     * Verifica dei due punti usando i coefficienti appena calcolati.
     */
    tensioneVerifica1 =
        (
            (double)calibrazioneBatteria.adcPunto1 *
            calibrazioneBatteria.coefficiente_mV_count
        ) +
        calibrazioneBatteria.offset_mV;

    tensioneVerifica2 =
        (
            (double)calibrazioneBatteria.adcPunto2 *
            calibrazioneBatteria.coefficiente_mV_count
        ) +
        calibrazioneBatteria.offset_mV;

    /*
     * Stampa del risultato.
     *
     * Il firmware pu essere compilato con i due valori stampati
     * come costanti di calibrazione.
     */
    snprintf(
        uart,
        sizeof(uart),
        "\r\n"
        "[BAT-CAL] CALIBRAZIONE COMPLETATA\r\n"
        "[BAT-CAL] Punto 1: ADC=%lu, V=%lu mV\r\n"
        "[BAT-CAL] Punto 2: ADC=%lu, V=%lu mV\r\n"
        "[BAT-CAL] Coefficiente: %.9f mV/count\r\n"
        "[BAT-CAL] Offset: %.3f mV\r\n"
        "[BAT-CAL] Verifica: V1=%.3f mV, V2=%.3f mV\r\n"
        "\r\n"
        "#define VBAT_ADC_GAIN_MV       %.9f\r\n"
        "#define VBAT_ADC_OFFSET_MV     %.3f\r\n"
        "\r\n",
        (unsigned long)calibrazioneBatteria.adcPunto1,
        (unsigned long)calibrazioneBatteria.tensionePunto1_mV,

        (unsigned long)calibrazioneBatteria.adcPunto2,
        (unsigned long)calibrazioneBatteria.tensionePunto2_mV,

        calibrazioneBatteria.coefficiente_mV_count,
        calibrazioneBatteria.offset_mV,

        tensioneVerifica1,
        tensioneVerifica2,

        calibrazioneBatteria.coefficiente_mV_count,
        calibrazioneBatteria.offset_mV
    );

    HAL_UART_Transmit(
        &huart1,
        (u8 *)uart,
        strlen(uart),
        1000
    );
}


double controllaBatteriaProva(void){
	u8 rimettiInCarica = 0;
	u32 acquisizione;
	double tensione;
	
	//tolgo dalla carica
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1) != 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		rimettiInCarica = 1;
		delay(50);
	}
	
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	delay(50);
	acquisizione = acquisizioneADC(9);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);

	if(rimettiInCarica == 1){
		delay(50);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		rimettiInCarica = 0;
	}
	
	tensione = acquisizione;
	tensione *= 0.003071;
	
	
	
	return tensione;
	
}

void controllaBatteriaProva2(void){
	u32 acquisizione;
	double tensione;
	u8 uart[35];
	
	
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	delay(50);
	acquisizione = acquisizioneADC(9);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	
	tensione = acquisizione;
	tensione *= 0.003071;

	sprintf(uart,"tensione1: %f\n",tensione);
	inviaDebug(uart);
	
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	
	HAL_Delay(1000);
	
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	delay(50);
	acquisizione = acquisizioneADC(9);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	
	tensione = acquisizione;
	tensione *= 0.003071;

	sprintf(uart,"tensione2: %f\n",tensione);
	inviaDebug(uart);
	
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	
	HAL_Delay(1000);
	
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	delay(50);
	acquisizione = acquisizioneADC(9);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	
	tensione = acquisizione;
	tensione *= 0.003071;

	sprintf(uart,"tensione1: %f\n",tensione);
	inviaDebug(uart);
	
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	
	HAL_Delay(1000);
	
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	delay(50);
	acquisizione = acquisizioneADC(9);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	
	tensione = acquisizione;
	tensione *= 0.003071;

	sprintf(uart,"tensione2: %f\n",tensione);
	inviaDebug(uart);
	
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	
}

u32 controllaBatteriaProva5(void){
	u8 rimettiInCarica = 0;
	u32 acquisizione;
	
	//tolgo dalla carica
	/*if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1) != 0){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		rimettiInCarica = 1;
		delay(50);
	}*/
	
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	delay(50);
	acquisizione = acquisizioneADC(9);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);

	/*if(rimettiInCarica == 1){
		delay(50);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		rimettiInCarica = 0;
	}*/
	
	return acquisizione;
	
}


/* Risultato completo della misura della batteria. */
typedef struct
{
    u32 media;
    u32 minimo;
    u32 massimo;
    u32 dispersione;
    u8 valida;
    GPIO_PinState statoPC0;
    GPIO_PinState statoPE15;
} BatteryAdcMeasurement_t;

/*
 * Misura la batteria nelle stesse condizioni usate per la calibrazione.
 *
 * Sequenza:
 * 1. spegne la carica (PC1 = 0);
 * 2. con alimentatore presente richiude PE15, cosi un precedente errore
 *    non puo lasciare il ramo di misura isolato in modo permanente;
 * 3. attiva PC0, attende la stabilizzazione e scarta la prima conversione;
 * 4. acquisisce piu campioni, elimina minimo e massimo e calcola la media;
 * 5. disattiva PC0.
 */
static BatteryAdcMeasurement_t misuraAdcBatteria(void)
{
    BatteryAdcMeasurement_t risultato;
    u32 somma = 0;
    u32 campione;
    u8 i;

    risultato.media = 0;
    risultato.minimo = 0xFFFFFFFFU;
    risultato.massimo = 0;
    risultato.dispersione = 0;
    risultato.valida = 0;
    risultato.statoPC0 = GPIO_PIN_RESET;
    risultato.statoPE15 = GPIO_PIN_RESET;

    /* La carica deve essere disabilitata durante tutta la misura. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    batteriaInCarica = 0;

    /*
     * Con alimentatore presente richiudiamo PE15 prima della misura.
     * Questo permette il recupero automatico dopo un precedente fault ADC.
     */
    if(alimentatore != 0)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
        delay(50);
    }

    risultato.statoPE15 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);

    /* Attiva il carico da 100 ohm e il partitore di misura. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    risultato.statoPC0 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
    delay(BAT_MEASURE_DELAY);

    /* Prima conversione di scarto dopo la selezione del canale ADC. */
    (void)acquisizioneADC(9);

    for(i = 0; i < BAT_ADC_SAMPLES; i++)
    {
        campione = acquisizioneADC(9);
        somma += campione;

        if(campione < risultato.minimo)
        {
            risultato.minimo = campione;
        }

        if(campione > risultato.massimo)
        {
            risultato.massimo = campione;
        }
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

    risultato.dispersione = risultato.massimo - risultato.minimo;

    somma -= risultato.minimo;
    somma -= risultato.massimo;
    risultato.media = somma / (BAT_ADC_SAMPLES - 2U);

    if(risultato.media >= BAT_ADC_MIN_VALID &&
       risultato.media <= BAT_ADC_MAX_VALID &&
       risultato.dispersione <= BAT_ADC_MAX_SPREAD)
    {
        risultato.valida = 1;
    }

    return risultato;
}

void controllaBatteria(void)
{
    BatteryAdcMeasurement_t misuraADC;
    u32 acquisizione;
    u32 tensioneMisurata_mV = 0;
    u32 tickAttuale;
    u32 tempoTrascorso_s;
    long temperaturaMCU;

    u8 adcValido;
    u8 temperaturaValida = 1;
    u8 comandoCarica = 0;
    u8 collegaBatteria = 1;
    u8 dutyPercentuale = 0;

    BatteryState_t statoPrecedente;
    BatteryFault_t faultAttuale;

    u8 sms[200];
    u8 addressFram[2] = {1, 91};
    char uart[360];

    /* =============================================================
     * 1. TEMPO TRASCORSO FRA DUE ESECUZIONI
     * ============================================================= */
    tickAttuale = HAL_GetTick();

    if(ultimoTickBatteria == 0U)
    {
        tempoTrascorso_s = BAT_DEFAULT_PERIOD_S;
    }
    else
    {
        tempoTrascorso_s = (tickAttuale - ultimoTickBatteria) / 1000UL;

        if(tempoTrascorso_s == 0U)
        {
            tempoTrascorso_s = 1U;
        }

        if(tempoTrascorso_s > BAT_MAX_PERIOD_S)
        {
            tempoTrascorso_s = BAT_MAX_PERIOD_S;
        }
    }

    ultimoTickBatteria = tickAttuale;

    /* =============================================================
     * 2. MISURA DELLA BATTERIA
     * ============================================================= */
    misuraADC = misuraAdcBatteria();
    acquisizione = misuraADC.media;
    adcValido = misuraADC.valida;

    /*
     * La conversione viene eseguita soltanto quando l'ADC e valido.
     * Con ADC=0 non viene quindi mostrata la falsa tensione di 48 mV
     * dovuta all'offset della retta di calibrazione.
     */
    if(adcValido != 0)
    {
        /*
         * Conversione tramite la calibrazione a due punti.
         */
        tensioneMisurata_mV = convertiAdcBatteria_mV(acquisizione);

        /*
         * In funzionamento da batteria il nodo misurato e a valle
         * di una caduta quasi costante, attribuita al diodo presente
         * nel percorso di scarica.
         *
         * L'offset viene quindi aggiunto solo con alimentatore assente.
         */
        if(alimentatore == 0)
        {
            tensioneMisurata_mV += VBAT_DISCHARGE_OFFSET_MV;
        }

        /*
         * Limite difensivo contro overflow o valori non plausibili.
         */
        if(tensioneMisurata_mV > 20000U)
        {
            tensioneMisurata_mV = 20000U;
        }

        tensioneB = ((double)tensioneMisurata_mV) / 1000.0;
        tensioneBint = (int)((tensioneMisurata_mV + 50U) / 100U);
        batteryLevel = calcolaLivelloBatteria(tensioneMisurata_mV);
    }
    else
    {
        tensioneMisurata_mV = 0;
        tensioneB = 0.0;
        tensioneBint = 0;
        batteryLevel = 5;
    }

    /* =============================================================
     * 3. TEMPERATURA INTERNA DELLO STM32
     * =============================================================
     * Non e la temperatura della batteria. Viene usata soltanto come
     * protezione molto larga della scheda elettronica.
     */
    temperaturaMCU = acquisizioneTemp();

    if(temperaturaMCU < TEMP_SENSOR_MIN_VALID_C ||
       temperaturaMCU > TEMP_SENSOR_MAX_VALID_C)
    {
        temperaturaValida = 0;
    }

    /* Isteresi del blocco termico del microcontrollore. */
    if(temperaturaValida == 0)
    {
        bloccoTemperatura = 1;
    }
    else if(bloccoTemperatura != 0)
    {
        if(temperaturaMCU <= TEMP_MCU_CHARGE_RESTART_C)
        {
            bloccoTemperatura = 0;
        }
    }
    else if(temperaturaMCU >= TEMP_MCU_CHARGE_MAX_C)
    {
        bloccoTemperatura = 1;
    }

    statoPrecedente = statoCaricaBatteria;
    faultAttuale = faultCaricaLatched;

    /* =============================================================
     * 4. FAULT DI SOVRATENSIONE
     * ============================================================= */
    if(adcValido != 0 &&
       tensioneMisurata_mV >= VBAT_OVERVOLTAGE_FAULT_MV)
    {
        faultCaricaLatched = BAT_FAULT_OVERVOLTAGE;
        faultAttuale = BAT_FAULT_OVERVOLTAGE;
        sessioneCaricaAttiva = 0;
        caricaCompleta = 0;
    }

    /* =============================================================
     * 5. MACCHINA A STATI DELLA CARICA
     * ============================================================= */
    if(faultCaricaLatched != BAT_FAULT_NONE)
    {
        statoCaricaBatteria = BAT_STATE_FAULT;
        comandoCarica = 0;
        dutyPercentuale = 0;
    }
    else if(adcValido == 0)
    {
        /* Fault transitorio: al prossimo ciclo la misura viene riprovata. */
        statoCaricaBatteria = BAT_STATE_FAULT;
        faultAttuale = BAT_FAULT_ADC;
        comandoCarica = 0;
        dutyPercentuale = 0;
    }
    else if(temperaturaValida == 0)
    {
        statoCaricaBatteria = BAT_STATE_FAULT;
        faultAttuale = BAT_FAULT_TEMPERATURE_SENSOR;
        comandoCarica = 0;
        dutyPercentuale = 0;
    }
    else if(alimentatore == 0)
    {
        statoCaricaBatteria = BAT_STATE_NO_SUPPLY;
        comandoCarica = 0;
        dutyPercentuale = 0;
    }
    else if(tensioneMisurata_mV < VBAT_MIN_AUTOMATIC_CHARGE_MV)
    {
        statoCaricaBatteria = BAT_STATE_TOO_LOW;
        faultAttuale = BAT_FAULT_BATTERY_TOO_LOW;
        sessioneCaricaAttiva = 0;
        caricaCompleta = 0;
        contatoreAvvio = 0;
        comandoCarica = 0;
        dutyPercentuale = 0;
    }
    else if(bloccoTemperatura != 0)
    {
        statoCaricaBatteria = BAT_STATE_WAIT_TEMPERATURE;
        comandoCarica = 0;
        dutyPercentuale = 0;
    }
    else
    {
        /* Dopo FULL si riparte solo sotto 7,75 V per tre misure. */
        if(caricaCompleta != 0)
        {
            statoCaricaBatteria = BAT_STATE_FULL;
            comandoCarica = 0;
            dutyPercentuale = 0;

            if(tensioneMisurata_mV <= VBAT_RECHARGE_MV)
            {
                if(contatoreAvvio < BAT_START_CONFIRM_COUNT)
                {
                    contatoreAvvio++;
                }

                if(contatoreAvvio >= BAT_START_CONFIRM_COUNT)
                {
                    caricaCompleta = 0;
                    sessioneCaricaAttiva = 1;
                    contatoreAvvio = 0;
                    contatoreArresto = 0;
                    contatoreDutyCycle = 0;
                    tempoTotaleCaricaOn_s = 0;
                    tempoTopCaricaOn_s = 0;
                }
            }
            else
            {
                contatoreAvvio = 0;
            }
        }

        /* Avvio di una nuova sessione. */
        if(caricaCompleta == 0 && sessioneCaricaAttiva == 0)
        {
            if(tensioneMisurata_mV <= VBAT_RECHARGE_MV)
            {
                if(contatoreAvvio < BAT_START_CONFIRM_COUNT)
                {
                    contatoreAvvio++;
                }

                if(contatoreAvvio >= BAT_START_CONFIRM_COUNT)
                {
                    sessioneCaricaAttiva = 1;
                    contatoreAvvio = 0;
                    contatoreArresto = 0;
                    contatoreDutyCycle = 0;
                    tempoTotaleCaricaOn_s = 0;
                    tempoTopCaricaOn_s = 0;
                }
                else
                {
                    statoCaricaBatteria = BAT_STATE_WAIT_START;
                    comandoCarica = 0;
                    dutyPercentuale = 0;
                }
            }
            else
            {
                contatoreAvvio = 0;
                statoCaricaBatteria = BAT_STATE_WAIT_START;
                comandoCarica = 0;
                dutyPercentuale = 0;
            }
        }

        /* Sessione di carica attiva. */
        if(sessioneCaricaAttiva != 0 && caricaCompleta == 0)
        {
            if(tensioneMisurata_mV >= VBAT_CHARGE_STOP_MV)
            {
                statoCaricaBatteria = BAT_STATE_STOP_CONFIRM;
                comandoCarica = 0;
                dutyPercentuale = 0;

                if(contatoreArresto < BAT_STOP_CONFIRM_COUNT)
                {
                    contatoreArresto++;
                }

                if(contatoreArresto >= BAT_STOP_CONFIRM_COUNT)
                {
                    caricaCompleta = 1;
                    sessioneCaricaAttiva = 0;
                    contatoreArresto = 0;
                    contatoreAvvio = 0;
                    contatoreDutyCycle = 0;
                    statoCaricaBatteria = BAT_STATE_FULL;
                }
            }
            else
            {
                contatoreArresto = 0;

                if(tensioneMisurata_mV < VBAT_TOP_50_MV)
                {
                    statoCaricaBatteria = BAT_STATE_BULK;
                    comandoCarica = 1;
                    dutyPercentuale = 100;
                    contatoreDutyCycle = 0;
                }
                else if(tensioneMisurata_mV < VBAT_TOP_25_MV)
                {
                    if(statoPrecedente != BAT_STATE_TOP_50)
                    {
                        contatoreDutyCycle = 0;
                    }

                    statoCaricaBatteria = BAT_STATE_TOP_50;
                    dutyPercentuale = 50;
                    comandoCarica = (contatoreDutyCycle == 0U) ? 1U : 0U;
                    contatoreDutyCycle++;

                    if(contatoreDutyCycle >= 2U)
                    {
                        contatoreDutyCycle = 0;
                    }
                }
                else
                {
                    if(statoPrecedente != BAT_STATE_TOP_25)
                    {
                        contatoreDutyCycle = 0;
                    }

                    statoCaricaBatteria = BAT_STATE_TOP_25;
                    dutyPercentuale = 25;
                    comandoCarica = (contatoreDutyCycle == 0U) ? 1U : 0U;
                    contatoreDutyCycle++;

                    if(contatoreDutyCycle >= 4U)
                    {
                        contatoreDutyCycle = 0;
                    }
                }
            }
        }
    }

    /* =============================================================
     * 6. TIMER DI SICUREZZA
     * ============================================================= */
    if(comandoCarica != 0)
    {
        tempoTotaleCaricaOn_s += tempoTrascorso_s;

        if(statoCaricaBatteria == BAT_STATE_TOP_50 ||
           statoCaricaBatteria == BAT_STATE_TOP_25)
        {
            tempoTopCaricaOn_s += tempoTrascorso_s;
        }
    }

    if(tempoTotaleCaricaOn_s >= BAT_MAX_TOTAL_ON_TIME_S)
    {
        faultCaricaLatched = BAT_FAULT_TOTAL_TIMEOUT;
        faultAttuale = BAT_FAULT_TOTAL_TIMEOUT;
        statoCaricaBatteria = BAT_STATE_FAULT;
        sessioneCaricaAttiva = 0;
        comandoCarica = 0;
        dutyPercentuale = 0;
    }

    if(tempoTopCaricaOn_s >= BAT_MAX_TOP_ON_TIME_S)
    {
        faultCaricaLatched = BAT_FAULT_TOP_TIMEOUT;
        faultAttuale = BAT_FAULT_TOP_TIMEOUT;
        statoCaricaBatteria = BAT_STATE_FAULT;
        sessioneCaricaAttiva = 0;
        comandoCarica = 0;
        dutyPercentuale = 0;
    }

    /* =============================================================
     * 7. DECISIONE UNICA SU PE15
     * =============================================================
     * Con alimentatore presente e ADC non valido PE15 resta collegato,
     * cosi la misura puo recuperare al ciclo successivo.
     */
    collegaBatteria = 1;

    if(adcValido != 0 &&
       alimentatore == 0 &&
       tensioneMisurata_mV < VBAT_DISCONNECT_MV)
    {
        collegaBatteria = 0;
    }

    if(adcValido == 0 && alimentatore == 0)
    {
        collegaBatteria = 0;
    }

    if(temperaturaValida == 0)
    {
        if(alimentatore == 0)
        {
            collegaBatteria = 0;
        }
    }
    else if(temperaturaMCU >= TEMP_MCU_EMERGENCY_C)
    {
        collegaBatteria = 0;
    }

    if(faultCaricaLatched == BAT_FAULT_OVERVOLTAGE)
    {
        collegaBatteria = 0;
    }

    /* =============================================================
     * 8. COMANDO FINALE DEI GPIO
     * ============================================================= */
    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_1,
        comandoCarica ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        GPIOE,
        GPIO_PIN_15,
        collegaBatteria ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    batteriaInCarica = comandoCarica;

    /* =============================================================
     * 9. DIAGNOSTICA SERIALE
     * ============================================================= */
    if(adcValido != 0)
    {
        snprintf(
            uart,
            sizeof(uart),
            "[BAT] PC0=%u PE15pre=%u ADC=%lu min=%lu max=%lu "
            "spread=%lu V=%lu.%03luV AC=%u Offset=%umV Tmcu=%ldC "
            "Level=%u%% dt=%lus\r\n",
            (unsigned int)misuraADC.statoPC0,
            (unsigned int)misuraADC.statoPE15,
            (unsigned long)acquisizione,
            (unsigned long)misuraADC.minimo,
            (unsigned long)misuraADC.massimo,
            (unsigned long)misuraADC.dispersione,
            (unsigned long)(tensioneMisurata_mV / 1000U),
            (unsigned long)(tensioneMisurata_mV % 1000U),
            (unsigned int)alimentatore,
            (unsigned int)((alimentatore == 0) ? VBAT_DISCHARGE_OFFSET_MV : 0U),
            temperaturaMCU,
            (unsigned int)batteryLevel,
            (unsigned long)tempoTrascorso_s
        );
    }
    else
    {
        snprintf(
            uart,
            sizeof(uart),
            "[BAT] PC0=%u PE15pre=%u ADC=%lu min=%lu max=%lu "
            "spread=%lu V=INVALID AC=%u Tmcu=%ldC dt=%lus\r\n",
            (unsigned int)misuraADC.statoPC0,
            (unsigned int)misuraADC.statoPE15,
            (unsigned long)acquisizione,
            (unsigned long)misuraADC.minimo,
            (unsigned long)misuraADC.massimo,
            (unsigned long)misuraADC.dispersione,
            (unsigned int)alimentatore,
            temperaturaMCU,
            (unsigned long)tempoTrascorso_s
        );
    }

    HAL_UART_Transmit(&huart1, (u8 *)uart, strlen(uart), 300);

    snprintf(
        uart,
        sizeof(uart),
        "[BAT] State=%s Fault=%s Session=%u Full=%u "
        "PC1=%u PE15=%u Duty=%u%% StartCnt=%u StopCnt=%u "
        "Ton=%lus Ttop=%lus\r\n",
        nomeStatoBatteria(statoCaricaBatteria),
        nomeFaultBatteria(faultAttuale),
        (unsigned int)sessioneCaricaAttiva,
        (unsigned int)caricaCompleta,
        (unsigned int)comandoCarica,
        (unsigned int)collegaBatteria,
        (unsigned int)dutyPercentuale,
        (unsigned int)contatoreAvvio,
        (unsigned int)contatoreArresto,
        (unsigned long)tempoTotaleCaricaOn_s,
        (unsigned long)tempoTopCaricaOn_s
    );

    HAL_UART_Transmit(&huart1, (u8 *)uart, strlen(uart), 300);

    /* =============================================================
     * 10. GESTIONE DEGLI ALLARMI BATTERIA
     * ============================================================= */
    if(batteryLevel >= 40 && messaggioBatteria != 0)
    {
        messaggioBatteria = 0;
        saveArrayFram(&messaggioBatteria, &addressFram[0], 1);
    }

    if(batteryLevel <= 30 &&
       batteryLevel > 10 &&
       messaggioBatteria == 0 &&
       BTattivo == 0 &&
       alimentatore == 0 &&
       V[0] < (20000 - VAR * 10000))
    {
        snprintf(
            (char *)sms,
            sizeof(sms),
            "battery alarm!\n"
            "device: ----------------\n"
            "lat: %.3f  long: %.3f\n"
            "residual charge: 30-",
            latitudineD,
            longitudineD
        );

        copiaArray(&sms[23], &identificativo[0], 16);
        sms[strlen((char *)sms) - 1U] = 37;

        /*
        inviaSMS(
            &numeroAllarmi[0],
            strlen(numeroAllarmi),
            &sms[0],
            strlen((char *)sms)
        );
        */

        messaggioBatteria = 1;
        saveArrayFram(&messaggioBatteria, &addressFram[0], 1);
    }

    if(batteryLevel <= 10 &&
       messaggioBatteria != 2 &&
       BTattivo == 0 &&
       alimentatore == 0 &&
       V[0] < (20000 - VAR * 10000))
    {
        snprintf(
            (char *)sms,
            sizeof(sms),
            "battery alarm!\n"
            "device: ----------------\n"
            "lat: %.3f  long: %.3f\n"
            "residual charge: 10-",
            latitudineD,
            longitudineD
        );

        copiaArray(&sms[23], &identificativo[0], 16);
        sms[strlen((char *)sms) - 1U] = 37;

        /*
        inviaSMS(
            &numeroAllarmi[0],
            strlen(numeroAllarmi),
            &sms[0],
            strlen((char *)sms)
        );
        */

        messaggioBatteria = 2;
        saveArrayFram(&messaggioBatteria, &addressFram[0], 1);
    }
}

long acquisizioneTemp(void)
{
    ADC_ChannelConfTypeDef sConfig;

    u32 sommaVref = 0;
    u32 sommaTemp = 0;
    u32 adcVref;
    u32 adcTemp;
    u32 adcTempRiferito3V3;

    u16 vrefCal;
    u16 tsCal1;
    u16 tsCal2;

    int64_t numeratore;
    int64_t denominatore;
    int32_t temperatura_dC;
    long temperaturaIntera;

    u8 i;

    /*
     * Valori di calibrazione individuali memorizzati da ST:
     *
     * VREFINT_CAL: VREFINT acquisito a 30 C con VDDA = 3,3 V
     * TS_CAL1:     sensore temperatura a 30 C con VDDA = 3,3 V
     * TS_CAL2:     sensore temperatura a 110 C con VDDA = 3,3 V
     */
    vrefCal = *((volatile const u16 *)0x1FFF7A2AUL);
    tsCal1  = *((volatile const u16 *)0x1FFF7A2CUL);
    tsCal2  = *((volatile const u16 *)0x1FFF7A2EUL);

    /*
     * Abilita il sensore di temperatura interno e VREFINT.
     */
    ADC->CCR |= ADC_CCR_TSVREFE;

    /*
     * =============================================================
     * 1. MISURA DI VREFINT
     * =============================================================
     *
     * Il datasheet richiede almeno 10 us di sampling time.
     * Con ADC clock a 20 MHz, 480 cicli corrispondono a circa 24 us.
     */
    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /*
     * Prima conversione scartata:
     * serve sia per il cambio canale sia per lasciare stabilizzare
     * il circuito interno, senza utilizzare delay() o HAL_GetTick().
     */
    if(HAL_ADC_Start(&hadc1) == HAL_OK)
    {
        if(HAL_ADC_PollForConversion(&hadc1, 5U) == HAL_OK)
        {
            (void)HAL_ADC_GetValue(&hadc1);
        }

        HAL_ADC_Stop(&hadc1);
    }

    /*
     * Media di 8 conversioni.
     */
    for(i = 0U; i < 8U; i++)
    {
        if(HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            temperatura = 999L;
            return temperatura;
        }

        if(HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            temperatura = 999L;
            return temperatura;
        }

        sommaVref += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    adcVref = sommaVref / 8U;

    /*
     * =============================================================
     * 2. MISURA DEL SENSORE DI TEMPERATURA
     * =============================================================
     */
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /*
     * Prima conversione scartata dopo il cambio di canale.
     */
    if(HAL_ADC_Start(&hadc1) == HAL_OK)
    {
        if(HAL_ADC_PollForConversion(&hadc1, 5U) == HAL_OK)
        {
            (void)HAL_ADC_GetValue(&hadc1);
        }

        HAL_ADC_Stop(&hadc1);
    }

    for(i = 0U; i < 8U; i++)
    {
        if(HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            temperatura = 999L;
            return temperatura;
        }

        if(HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            temperatura = 999L;
            return temperatura;
        }

        sommaTemp += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    adcTemp = sommaTemp / 8U;

    /*
     * Controlli minimi di plausibilita.
     */
    if(adcVref == 0U ||
       adcVref >= 4095U ||
       adcTemp == 0U ||
       adcTemp >= 4095U ||
       vrefCal == 0U ||
       tsCal1 == 0U ||
       tsCal2 == 0U ||
       tsCal1 == tsCal2)
    {
        temperatura = 999L;
        return temperatura;
    }

    /*
     * I valori TS_CAL1 e TS_CAL2 sono riferiti a VDDA = 3,3 V.
     *
     * Compensazione delle variazioni di VDDA tramite VREFINT:
     *
     * ADC_TEMP_3V3 = ADC_TEMP * VREFINT_CAL / ADC_VREFINT
     */
    adcTempRiferito3V3 =
        (u32)
        (
            (
                (uint64_t)adcTemp *
                (uint64_t)vrefCal +
                ((uint64_t)adcVref / 2ULL)
            ) /
            (uint64_t)adcVref
        );

    /*
     * Interpolazione lineare fra i due punti di calibrazione:
     *
     * TS_CAL1 = 30 C
     * TS_CAL2 = 110 C
     *
     * Il calcolo viene eseguito in decimi di grado.
     */
    numeratore =
        (
            (int64_t)adcTempRiferito3V3 -
            (int64_t)tsCal1
        ) *
        800LL; /* 1100 dC - 300 dC */

    denominatore =
        (int64_t)tsCal2 -
        (int64_t)tsCal1;

    temperatura_dC =
        (int32_t)
        (
            300LL +
            (numeratore / denominatore)
        );

    /*
     * Arrotondamento al grado piu vicino.
     */
    if(temperatura_dC >= 0)
    {
        temperaturaIntera =
            (long)((temperatura_dC + 5) / 10);
    }
    else
    {
        temperaturaIntera =
            (long)((temperatura_dC - 5) / 10);
    }

    /*
     * Ulteriore controllo di plausibilita.
     */
    if(temperaturaIntera < -40L ||
       temperaturaIntera > 125L)
    {
        temperatura = 999L;
        return temperatura;
    }

    temperatura = temperaturaIntera;

    return temperaturaIntera;
}






