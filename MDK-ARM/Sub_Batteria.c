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
extern u8 tempoSpegnimentoBatteriaMinuti;

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
 * La conversione usa una retta passante per due punti:
 *
 *   (VBAT_CAL_ADC_NORM_1, VBAT_CAL_MV_1)
 *   (VBAT_CAL_ADC_NORM_2, VBAT_CAL_MV_2)
 *
 * ADC_NORM e il conteggio ADC gia compensato tramite VREFINT; MV e la
 * tensione reale applicata all'ingresso batteria, espressa in millivolt.
 *
 * Vincoli da rispettare:
 * - i due ADC_NORM devono essere diversi, altrimenti si divide per zero;
 * - per una caratteristica crescente usare ADC_NORM_1 < ADC_NORM_2 e
 *   MV_1 < MV_2;
 * - usare valori positivi ottenuti con la stessa scheda e con la stessa
 *   sequenza di misura usata dal firmware;
 * - i punti devono racchiudere, per quanto possibile, il campo di lavoro.
 *
 * Non modificare un solo valore: ogni punto e sempre una coppia ADC/mV.
 * Queste costanti sono signed long, ma tensioni o conteggi negativi non
 * hanno significato fisico in questa applicazione.
 */
#define VBAT_CAL_ADC_NORM_1    1917L
#define VBAT_CAL_MV_1          6000L
#define VBAT_CAL_ADC_NORM_2    2587L
#define VBAT_CAL_MV_2          8100L


/*
 * Offset, in millivolt, sommato esclusivamente quando l'alimentatore
 * principale e assente (alimentatore == 0).
 *
 * Compensa la caduta del percorso di alimentazione da batteria e NON
 * viene applicato durante la carica o con rete presente.
 *
 * Valori ammessi dal tipo: 0 ... 4294967295 mV; intervallo pratico:
 * poche centinaia di mV. Un valore troppo alto falsifica percentuale,
 * allarmi e protezione di sottotensione durante il funzionamento a
 * batteria. Tararlo confrontando la misura con e senza rete a parita
 * di tensione reale e percorso elettrico.
 */
#define VBAT_DISCHARGE_OFFSET_MV          445U


/* ================================================================
 * PARAMETRI DELLA MISURA
 * ================================================================
 *
 * Numero di conversioni rapide del canale batteria. Il minimo e 3,
 * perche firmware elimina il campione minimo e massimo e divide per
 * BAT_ADC_SAMPLES - 2. Il contatore e u8: non superare 255.
 * Aumentarlo migliora la media ma allunga la fase ADC.
 */
#define BAT_ADC_SAMPLES                  16U

/*
 * Tempo, in millisecondi, durante il quale PC0 e il circuito di misura
 * restano inseriti prima di leggere l'ADC. L'attesa e non bloccante.
 *
 * Valore tecnico: 0 ... 4294967295 ms; usare un valore positivo e,
 * normalmente, molto inferiore al periodo di controllo di 15 s.
 * Un valore troppo basso non consente l'assestamento; uno troppo alto
 * scarica inutilmente la batteria sul carico di misura da circa 100 ohm.
 * Se cambia, verificare che la calibrazione resti valida.
 */
#define BAT_MEASURE_DELAY               250L

/*
 * Massima dispersione ammessa fra minimo e massimo dei campioni ADC
 * grezzi acquisiti nella stessa misura.
 *
 * 80 count equivalgono a circa 224 mV.
 * Intervallo utile: 0 ... 4095 count. Ridurlo rende il controllo piu
 * severo ma puo produrre fault ADC durante disturbi o picchi di carico;
 * aumentarlo accetta misure meno stabili.
 */
#define BAT_ADC_MAX_SPREAD               80U

/*
 * Limiti inclusivi di plausibilita del valore ADC normalizzato VREFINT.
 * Devono rispettare:
 *
 *   0 <= BAT_ADC_MIN_VALID < BAT_ADC_MAX_VALID
 *
 * Il limite hardware dell'ADC a 12 bit e 4095; il valore normalizzato
 * puo differire leggermente dal grezzo, quindi lasciare margine attorno
 * al campo prodotto dalla calibrazione. Se MIN e troppo basso, una
 * batteria assente/in protezione puo sembrare valida; se e troppo alto,
 * il WAKEUP puo essere avviato piu spesso. Un MAX troppo basso rifiuta
 * batterie cariche, uno troppo alto riduce la capacita diagnostica.
 */
#define BAT_ADC_MIN_VALID                1000U
#define BAT_ADC_MAX_VALID                3500U


/* ================================================================
 * SOGLIE DI TENSIONE
 * ================================================================
 *
 * Tutte le tensioni sono espresse in millivolt e confrontate con la
 * tensione calcolata dopo calibrazione (e, senza rete, dopo l'offset).
 *
 * Ordinamento raccomandato per la configurazione attuale:
 *
 *   VBAT_MIN_AUTOMATIC_CHARGE_MV
 *      <= VBAT_DISCONNECT_MV
 *      < VBAT_RECHARGE_MV
 *      <= BAT_PLATEAU_MIN_MV
 *      < VBAT_CHARGE_STOP_MV
 *      < VBAT_OVERVOLTAGE_FAULT_MV
 *
 * MIN e DISCONNECT possono essere uguali, come ora. Se DISCONNECT viene
 * alzata, il dispositivo si spegnera prima durante il funzionamento a
 * batteria; se viene abbassata sotto MIN, si ammette una scarica piu
 * profonda di quella da cui parte il WAKEUP.
 */

/*
 * Soglia, in mV, sotto la quale con rete presente viene usata la carica
 * di recupero WAKEUP (30 s ON e 15 s OFF). Deve essere inferiore a
 * VBAT_RECHARGE_MV. Non impostarla sotto il limite sicuro della batteria.
 */
#define VBAT_MIN_AUTOMATIC_CHARGE_MV     6000U

/*
 * Soglia, in mV, sotto la quale, senza rete, PE15 scollega la batteria
 * dopo la conferma definita da BAT_UNDERVOLTAGE_CONFIRM_DELAY_MS.
 *
 * Con rete presente questa soglia non apre PE15: sotto
 * VBAT_MIN_AUTOMATIC_CHARGE_MV deve infatti essere possibile eseguire
 * il WAKEUP. Valore pratico: compreso nel campo affidabile della retta
 * di calibrazione e coerente con il limite di sovrascarica della batteria.
 */
#define VBAT_DISCONNECT_MV               6650U

/*
 * Soglia, in mV, sotto la quale puo iniziare una nuova sessione normale.
 * Sono richieste BAT_START_CONFIRM_COUNT misure consecutive.
 *
 * Deve essere maggiore di VBAT_MIN_AUTOMATIC_CHARGE_MV e minore di
 * VBAT_CHARGE_STOP_MV. La differenza STOP - RECHARGE e l'isteresi che
 * impedisce continue ripartenze vicino alla tensione finale.
 */
#define VBAT_RECHARGE_MV                 7900U

/*
 * Abilita la riduzione della corrente media nelle fasce finali:
 *   0 = carica continua fino alla soglia di arresto;
 *   1 = carica pulsata al 50% e al 25% alle soglie configurate.
 *
 * Usare esclusivamente 0 oppure 1. Con valore 0, VBAT_TOP_50_MV,
 * VBAT_TOP_25_MV e BAT_MAX_TOP_ON_TIME_S non influenzano la carica.
 * Inoltre il riconoscimento del plateau e attivo soltanto con carica
 * pulsata disabilitata.
 */
#define BAT_PULSED_CHARGE_ENABLED        0U

/*
 * Soglia, in mV, da cui inizia la carica pulsata al 50%.
 * Usata solo se BAT_PULSED_CHARGE_ENABLED == 1.
 */
#define VBAT_TOP_50_MV                   7950U

/*
 * Soglia, in mV, da cui inizia la carica pulsata al 25%.
 * Usata solo se BAT_PULSED_CHARGE_ENABLED == 1.
 *
 * Se si abilita la carica pulsata deve valere:
 *
 *   VBAT_RECHARGE_MV < VBAT_TOP_50_MV
 *                    < VBAT_TOP_25_MV
 *                    < VBAT_CHARGE_STOP_MV
 *
 * ATTENZIONE: con i valori attuali TOP_25 (8050 mV) e maggiore di STOP
 * (8000 mV), quindi lo stato TOP_25 non sarebbe raggiungibile. Non e un
 * problema finche BAT_PULSED_CHARGE_ENABLED resta a 0; prima di porlo
 * a 1 occorre abbassare TOP_25 o alzare, con adeguata verifica hardware,
 * la soglia STOP.
 */
#define VBAT_TOP_25_MV                   8050U

/*
 * Soglia, in mV, di arresto normale della carica. Sono richieste
 * BAT_STOP_CONFIRM_COUNT misure consecutive a questa tensione o oltre.
 *
 * Deve essere maggiore di VBAT_RECHARGE_MV e minore della soglia di
 * overvoltage. Se viene posta sotto BAT_PLATEAU_MIN_MV, il controllo
 * plateau non potra mai accumulare campioni.
 */
#define VBAT_CHARGE_STOP_MV              8000U

/*
 * Arresto alternativo per plateau della tensione.
 *
 * BAT_PLATEAU_TERMINATION_ENABLED:
 *   0 = controllo completamente disabilitato;
 *   1 = controllo abilitato.
 * Usare solo 0 o 1. Il controllo viene comunque escluso automaticamente
 * quando BAT_PULSED_CHARGE_ENABLED == 1.
 *
 * BAT_PLATEAU_MIN_MV:
 *   limite inferiore della finestra di tensione; deve essere minore di
 *   VBAT_CHARGE_STOP_MV e, normalmente, non inferiore a
 *   VBAT_RECHARGE_MV. Il controllo opera da MIN incluso a STOP escluso.
 *
 * BAT_PLATEAU_SAMPLES:
 *   numero totale di misure nella finestra mobile. Con una misura ogni
 *   15 s, 80 campioni corrispondono a circa 20 minuti. Deve essere
 *   compreso fra 1 e 255 perche indici e contatori sono u8.
 *
 * BAT_PLATEAU_AVERAGE_SAMPLES:
 *   campioni usati per ciascuna media, all'inizio e alla fine della
 *   finestra. Deve essere almeno 1 e non maggiore di PLATEAU_SAMPLES.
 *   E raccomandato: 2 * AVERAGE_SAMPLES <= PLATEAU_SAMPLES, per evitare
 *   la sovrapposizione delle due medie.
 *
 * BAT_PLATEAU_MAX_RISE_MV:
 *   massimo aumento ammesso fra media iniziale e finale. Un valore piu
 *   basso rende piu difficile riconoscere il plateau.
 *
 * BAT_PLATEAU_MAX_FALL_MV:
 *   massima diminuzione ammessa fra media iniziale e finale. Senza questo
 *   limite una tensione in discesa rispetterebbe sempre il solo controllo
 *   MAX_RISE. Deve essere abbastanza grande da tollerare il rumore ma
 *   molto piu piccolo di una reale caduta della batteria.
 *
 * BAT_PLATEAU_MAX_SPAN_MV:
 *   massima differenza fra il minimo e il massimo dell'intera finestra.
 *   Deve essere almeno pari al rumore reale della misura. Se e troppo
 *   basso il plateau non viene mai riconosciuto; se e troppo alto si
 *   rischia un arresto prematuro.
 *
 * BAT_PLATEAU_CONFIRM_COUNT:
 *   numero di finestre mobili consecutive che devono rispettare tutti i
 *   criteri prima di iniziare la verifica a riposo. Intervallo 1...255;
 *   con misure ogni 15 s, 8 conferme aggiungono circa 2 minuti. Il valore
 *   0 non deve essere usato.
 *
 * VERIFICA A RIPOSO:
 *   dopo la conferma del plateau PC1 resta spento per
 *   BAT_PLATEAU_REST_TIME_MS. Al termine la carica e dichiarata completa
 *   soltanto se:
 *   - la tensione e almeno BAT_PLATEAU_REST_MIN_MV;
 *   - la caduta rispetto all'inizio del riposo non supera
 *     BAT_PLATEAU_REST_MAX_DROP_MV.
 *
 * BAT_PLATEAU_REST_TIME_MS deve essere > 0 e molto inferiore a 2^31 ms.
 * REST_MIN deve essere inferiore a VBAT_CHARGE_STOP_MV e, normalmente,
 * non superiore a BAT_PLATEAU_MIN_MV. REST_MAX_DROP deve essere maggiore
 * del rumore reale ma abbastanza piccolo da respingere un falso plateau.
 *
 * Se la verifica fallisce, la sessione resta attiva, le finestre vengono
 * azzerate e la carica riparte automaticamente.
 */
#define BAT_PLATEAU_TERMINATION_ENABLED  1U
#define BAT_PLATEAU_MIN_MV               7920U
#define BAT_PLATEAU_SAMPLES              80U
#define BAT_PLATEAU_AVERAGE_SAMPLES      8U
#define BAT_PLATEAU_MAX_RISE_MV          5U
#define BAT_PLATEAU_MAX_FALL_MV          5U
#define BAT_PLATEAU_MAX_SPAN_MV          20U
#define BAT_PLATEAU_CONFIRM_COUNT        8U
#define BAT_PLATEAU_REST_TIME_MS         (2UL * 60UL * 1000UL)
#define BAT_PLATEAU_REST_MIN_MV          7900U
#define BAT_PLATEAU_REST_MAX_DROP_MV     50U

/*
 * Watchdog di avanzamento della carica nella fascia precedente al
 * plateau finale.
 *
 * BAT_STALL_CHECK_ENABLED:
 *   0 = protezione disabilitata;
 *   1 = protezione abilitata. Usare soltanto 0 o 1.
 *
 * BAT_STALL_MIN_MV / BAT_STALL_MAX_MV:
 *   fascia [MIN, MAX) nella quale viene verificato l'aumento di tensione.
 *   Deve valere MIN < MAX. La configurazione raccomandata collega le
 *   fasce senza vuoti:
 *
 *     STALL_MIN = VBAT_MIN_AUTOMATIC_CHARGE_MV
 *     STALL_MAX = BAT_PLATEAU_MIN_MV
 *
 *   Se STALL_MAX supera PLATEAU_MIN, stallo e plateau possono lavorare
 *   contemporaneamente. Se e inferiore, resta una fascia non controllata.
 *
 * BAT_STALL_CHECK_TIME_S:
 *   secondi effettivi con PC1 alto disponibili per ottenere l'incremento
 *   minimo. Deve essere maggiore di zero. E opportuno che sia minore di
 *   BAT_MAX_TOTAL_ON_TIME_S, altrimenti il timeout totale puo intervenire
 *   prima che questo watchdog completi la verifica.
 *
 * BAT_STALL_MIN_RISE_MV:
 *   incremento minimo richiesto nel tempo precedente. Aumentarlo rende
 *   il controllo piu severo; deve restare ben sopra il rumore medio ma
 *   compatibile con la reale curva di carica.
 *
 * BAT_STALL_AVERAGE_SAMPLES:
 *   numero di campioni delle medie iniziale e finale. Intervallo 1...255
 *   per i contatori u8. Valori alti filtrano meglio ma ritardano l'avvio
 *   e aumentano l'uso di RAM.
 *
 * Il fault CHARGE_STALLED e latched: non usa il riarmo automatico dei
 * fault TOTAL_TIMEOUT e TOP_TIMEOUT.
 */
#define BAT_STALL_CHECK_ENABLED          1U
#define BAT_STALL_MIN_MV                 VBAT_MIN_AUTOMATIC_CHARGE_MV
#define BAT_STALL_MAX_MV                 BAT_PLATEAU_MIN_MV
#define BAT_STALL_CHECK_TIME_S           (2UL * 60UL * 60UL)
#define BAT_STALL_MIN_RISE_MV            20U
#define BAT_STALL_AVERAGE_SAMPLES        8U

/*
 * Filtro della tensione visualizzata durante il funzionamento a batteria.
 * Le protezioni continuano a usare la misura grezza.
 *
 * BAT_SOC_FILTER_SAMPLES:
 *   dimensione della finestra della mediana. Deve essere 1...255 per gli
 *   indici u8 e preferibilmente dispari; con un numero pari viene scelto
 *   il campione superiore dei due centrali. Aumentarlo rende percentuale
 *   e log piu stabili ma piu lenti nel seguire variazioni reali.
 *
 * BAT_LEVEL_HYSTERESIS_MV:
 *   isteresi applicata solo quando la percentuale scende. Valore 0
 *   disabilita di fatto l'isteresi. Con le soglie percentuali attuali e
 *   raccomandato 0...39 mV: 40 mV e la distanza minima fra due soglie.
 *   Valori maggiori possono trattenere un livello oltre quello adiacente.
 *
 * Questi parametri non ritardano PE15, overvoltage, carica o fault:
 * modificano soltanto tensione/percentuale mostrate e allarme percentuale.
 */
#define BAT_SOC_FILTER_SAMPLES           5U
#define BAT_LEVEL_HYSTERESIS_MV          25U

/*
 * Ritardo, in millisecondi, fra la prima misura sotto
 * VBAT_DISCONNECT_MV e la misura di conferma prima di aprire PE15.
 *
 * 0 richiede comunque un secondo passaggio della funzione ma senza attesa
 * temporale. Un valore troppo breve puo reagire ai picchi LTE; uno troppo
 * lungo mantiene il carico collegato piu a lungo a una batteria scarica.
 * Il calcolo usa HAL_GetTick() a 32 bit; restare molto sotto 2^31 ms.
 */
#define BAT_UNDERVOLTAGE_CONFIRM_DELAY_MS 2000UL

/*
 * Soglia, in mV, del fault latched di sovratensione. Il fault spegne PC1
 * ma non apre PE15. Deve essere maggiore di VBAT_CHARGE_STOP_MV, con un
 * margine superiore al rumore e all'overshoot della misura; in caso
 * contrario il fault puo precedere il normale arresto della carica.
 */
#define VBAT_OVERVOLTAGE_FAULT_MV        8200U


/* ================================================================
 * TEMPERATURA INTERNA DEL MICROCONTROLLORE
 * ================================================================
 *
 * acquisizioneTemp() legge ADC_CHANNEL_TEMPSENSOR, quindi misura
 * la temperatura interna dello STM32 e NON quella della batteria.
 * Tutti i valori sono espressi in gradi Celsius interi signed long.
 */

/*
 * Limite inferiore per il funzionamento del dispositivo:
 *
 * - sotto TEMP_MCU_OPERATION_MIN_C PE15 viene aperto;
 * - PE15 viene ricollegato soltanto a
 *   TEMP_MCU_OPERATION_RESTART_C o oltre.
 *
 * Deve valere MIN < RESTART. La differenza e l'isteresi che evita
 * aperture e chiusure ripetute vicino alla soglia. Entrambi i valori
 * devono essere compresi fra TEMP_SENSOR_MIN_VALID_C e
 * TEMP_SENSOR_MAX_VALID_C.
 *
 * ATTENZIONE: questa protezione usa la temperatura interna dello STM32.
 * La temperatura reale della batteria puo essere diversa.
 */
#define TEMP_MCU_OPERATION_MIN_C          (-10L)
#define TEMP_MCU_OPERATION_RESTART_C      (-5L)

/*
 * Limite inferiore per la carica:
 *
 * - sotto TEMP_MCU_CHARGE_MIN_C PC1 resta spento;
 * - dopo un blocco per temperatura bassa, la carica puo riprendere
 *   soltanto a TEMP_MCU_CHARGE_MIN_RESTART_C o oltre.
 *
 * Deve valere MIN < MIN_RESTART. Le soglie devono essere superiori
 * alle soglie minime di funzionamento e inferiori alle soglie massime
 * di carica. A esattamente TEMP_MCU_CHARGE_MIN_C la carica e ammessa
 * se il blocco per bassa temperatura non era gia attivo.
 */
#define TEMP_MCU_CHARGE_MIN_C              7L
#define TEMP_MCU_CHARGE_MIN_RESTART_C     10L

/*
 * A TEMP_MCU_CHARGE_MAX_C o oltre viene sospesa la sola carica.
 * La carica puo riprendere soltanto a TEMP_MCU_CHARGE_RESTART_C o meno.
 *
 * Deve valere RESTART < MAX. La loro differenza e l'isteresi termica;
 * se sono uguali il comando puo oscillare vicino alla soglia.
 * Entrambe devono stare dentro i limiti TEMP_SENSOR_MIN/MAX_VALID.
 */
#define TEMP_MCU_CHARGE_MAX_C             52L
#define TEMP_MCU_CHARGE_RESTART_C         50L

/*
 * Protezione termica estrema con isteresi. A EMERGENCY o oltre PE15
 * viene aperto; viene richiuso soltanto a EMERGENCY_RESTART o meno.
 *
 * Ordinamento complessivo raccomandato:
 *
 *   TEMP_MCU_OPERATION_MIN_C
 *       < TEMP_MCU_OPERATION_RESTART_C
 *       < TEMP_MCU_CHARGE_MIN_C
 *       < TEMP_MCU_CHARGE_MIN_RESTART_C
 *       < TEMP_MCU_CHARGE_RESTART_C
 *       < TEMP_MCU_CHARGE_MAX_C
 *       < TEMP_MCU_EMERGENCY_RESTART_C
 *       < TEMP_MCU_EMERGENCY_C
 *
 * Cosi la carica e gia ferma prima di scollegare la batteria. Se le fasce
 * vengono sovrapposte il firmware resta comunque deterministico, ma PE15
 * potrebbe ricollegarsi mentre la sola carica e ancora bloccata.
 * Le soglie devono stare dentro TEMP_SENSOR_MIN/MAX_VALID.
 */
#define TEMP_MCU_EMERGENCY_C              67L
#define TEMP_MCU_EMERGENCY_RESTART_C      64L

/*
 * Limiti inclusivi di plausibilita del sensore interno STM32.
 * Deve valere MIN_VALID < MAX_VALID e tutte le soglie termiche devono
 * trovarsi al loro interno. Restringerli aumenta la diagnostica ma puo
 * produrre fault sensore; allargarli puo accettare letture non realistiche.
 *
 * Se la temperatura non e valida, la carica viene fermata. Se PE15 era
 * gia aperto per sovratemperatura, resta aperto finche una misura valida
 * non conferma il raffreddamento.
 */
#define TEMP_SENSOR_MIN_VALID_C          (-40L)
#define TEMP_SENSOR_MAX_VALID_C           125L


/* ================================================================
 * CONTATORI E TIMEOUT
 * ================================================================ */

/*
 * La partenza della carica richiede tre misure consecutive
 * sotto la soglia di riavvio.
 *
 * Intervallo ammesso: 1...255 (contatore u8). Con controlli ogni 15 s,
 * 3 significa decisione alla terza misura. 0 causerebbe una conferma
 * immediata e non deve essere usato.
 */
#define BAT_START_CONFIRM_COUNT          3U

/*
 * La fine della carica richiede due misure consecutive
 * sopra la soglia di arresto.
 *
 * Intervallo ammesso: 1...255 (contatore u8). Aumentarlo rende l'arresto
 * piu robusto ma prolunga la permanenza vicino alla tensione finale.
 * 0 non deve essere usato.
 */
#define BAT_STOP_CONFIRM_COUNT           2U

/*
 * Secondi attribuiti al primo ciclo, quando non esiste ancora un tick
 * precedente da cui ricavare il tempo trascorso. Deve essere maggiore
 * di zero e non superiore a BAT_MAX_PERIOD_S. Influenza i timer di
 * sicurezza solo al primo controllo di una sessione dopo l'avvio.
 */
#define BAT_DEFAULT_PERIOD_S             30UL

/*
 * Massimo numero di secondi aggiungibile ai timer in un singolo controllo.
 * Evita che una pausa di debug o un ritardo del main producano un grande
 * salto. Deve essere > 0 e >= BAT_DEFAULT_PERIOD_S.
 *
 * Non e il periodo della misura: il controllo resta schedulato dal main.
 * Un valore troppo basso sottostima il tempo reale di carica se il main
 * rimane bloccato piu a lungo.
 */
#define BAT_MAX_PERIOD_S                 300UL

/*
 * Massimo tempo totale, in secondi, con PC1 alto durante una sessione
 * normale. Il WAKEUP e escluso da questo conteggio. Al raggiungimento
 * viene latched TOTAL_TIMEOUT e la carica si ferma.
 *
 * Valore ammesso dal contatore u32: 1...4294967295 s. Usare il suffisso
 * UL nelle espressioni. Deve essere maggiore di BAT_STALL_CHECK_TIME_S
 * se si vuole dare al watchdog di stallo il tempo di intervenire prima.
 */
#define BAT_MAX_TOTAL_ON_TIME_S          (24UL * 60UL * 60UL)

/*
 * Massimo tempo effettivo, in secondi, con PC1 alto negli stati TOP_50
 * e TOP_25. Al raggiungimento viene latched TOP_TIMEOUT.
 *
 * Ha effetto solo con BAT_PULSED_CHARGE_ENABLED == 1. Deve essere > 0
 * e normalmente <= BAT_MAX_TOTAL_ON_TIME_S; se e maggiore, il timeout
 * totale interverra sempre prima.
 */
#define BAT_MAX_TOP_ON_TIME_S            (2UL * 60UL * 60UL)

/*
 * Riarmo automatico esclusivo dei fault TOTAL_TIMEOUT e TOP_TIMEOUT.
 *
 * BAT_TIMEOUT_RESTART_DROP_MV:
 *   caduta minima, in mV, rispetto alla tensione memorizzata al timeout.
 *   0 consentirebbe il riarmo senza una vera scarica e non va usato.
 *   Deve essere maggiore di rumore/errore di misura, ma non cosi grande
 *   da richiedere una discesa sotto VBAT_DISCONNECT_MV. Se la batteria
 *   entra in protezione e non viene rilevata, il WAKEUP resta comunque
 *   autorizzato anche senza una tensione numerica valida.
 *
 * BAT_TIMEOUT_RESTART_CONFIRM_COUNT:
 *   numero di misure consecutive che devono confermare la caduta, con
 *   rete e temperatura valida. Intervallo 1...255 (contatore u8);
 *   0 non deve essere usato.
 */
#define BAT_TIMEOUT_RESTART_DROP_MV       200U
#define BAT_TIMEOUT_RESTART_CONFIRM_COUNT 3U

/*
 * Il timeout del funzionamento a batteria non e una costante locale:
 * tempoSpegnimentoBatteriaMinuti viene caricato dalla FRAM all'avvio e
 * puo essere aggiornato con il comando Bluetooth 0x63.
 *
 * 0 disabilita esclusivamente questo spegnimento temporizzato.
 * I valori 1...60 indicano i minuti continuativi senza rete; il ritorno
 * della rete azzera il conteggio e la successiva interruzione riparte da
 * zero. Le aperture di PE15 per sottotensione o temperatura rimangono
 * sempre indipendenti da questa impostazione.
 */

/*
 * Durata minima continuativa senza rete necessaria per considerare
 * interrotta una sessione di carica.
 *
 * Se la rete ritorna dopo almeno questo tempo e, al momento della sua
 * scomparsa, sessioneCaricaAttiva era diversa da zero, vengono azzerati:
 * - timer totale e timer TOP della carica;
 * - contatori di avvio, arresto e duty cycle;
 * - finestre del plateau e del watchdog di stallo.
 *
 * La sessione non viene forzata e gli eventuali fault latched non vengono
 * cancellati. Un valore 0 qualificherebbe qualsiasi interruzione, anche
 * un disturbo molto breve. Usare un valore > 0 e molto inferiore a
 * 2^31 ms per mantenere non ambiguo il confronto con HAL_GetTick().
 *
 * Esempi:
 *    1 minuto  = (60UL * 1000UL)
 *    5 minuti = (5UL * 60UL * 1000UL)
 */
#define BAT_CHARGE_SESSION_RESET_OFF_TIME_MS (60UL * 1000UL)

/*
 * Intervallo minimo, in millisecondi, fra due log riepilogativi [BAT].
 * Non cambia la frequenza delle misure, dei fault o della macchina a
 * stati. 0 stampa a ogni misura completata. Valore u32; restare molto
 * sotto 2^31 ms per un confronto temporale semplice e leggibile.
 */
#define BAT_LOG_INTERVAL_MS               60000UL




/*
 * Variabili mantenute fra una chiamata e la successiva.
 */
static BatteryState_t statoCaricaBatteria = BAT_STATE_NO_SUPPLY;
static BatteryFault_t faultCaricaLatched = BAT_FAULT_NONE;

static u8 sessioneCaricaAttiva = 0;
static u8 caricaCompleta = 0;
static u8 bloccoTemperatura = 0;
static u8 bloccoTemperaturaBassa = 0;

static u8 contatoreAvvio = 0;
static u8 contatoreArresto = 0;
static u8 contatoreDutyCycle = 0;
static u8 contatoreWakeup = 0;

static u32 tempoTotaleCaricaOn_s = 0;
static u32 tempoTopCaricaOn_s = 0;

static u32 tensioneArrestoTimeout_mV = 0;
static u8 contatoreRiarmoTimeout = 0;

static u32 ultimoTickBatteria = 0;
static u32 ultimoTickLogBatteria = 0;
static u8 logBatteriaInizializzato = 0;

static u16 campioniPlateau[BAT_PLATEAU_SAMPLES];
static u8 indiceCampionePlateau = 0;
static u8 numeroCampioniPlateau = 0;
static u8 contatoreConfermaPlateau = 0;
static u8 verificaRiposoPlateauAttiva = 0;
static u32 tickInizioRiposoPlateau = 0;
static u32 tensioneInizioRiposoPlateau_mV = 0;

static u32 sommaRiferimentoStallo = 0;
static u8 campioniRiferimentoStallo = 0;
static u32 riferimentoStallo_mV = 0;
static u8 riferimentoStalloValido = 0;
static u16 campioniRecentiStallo[BAT_STALL_AVERAGE_SAMPLES];
static u32 sommaCampioniRecentiStallo = 0;
static u8 indiceCampioneRecenteStallo = 0;
static u8 numeroCampioniRecentiStallo = 0;
static u32 tempoCaricaStallo_s = 0;

static u16 campioniTensioneScarica[BAT_SOC_FILTER_SAMPLES];
static u8 indiceTensioneScarica = 0;
static u8 filtroTensioneScaricaInizializzato = 0;
static u8 livelloBatteriaInizializzato = 0;

static u8 confermaSottotensioneAttiva = 0;
static u32 tickConfermaSottotensione = 0;
static u32 primaTensioneSottotensione_mV = 0;
static u8 disconnessioneSottotensioneAttiva = 0;
static u8 disconnessioneTermicaAttiva = 0;
static u8 disconnessioneTemperaturaBassaAttiva = 0;

static u8 timerFunzionamentoBatteriaAttivo = 0;
static u8 spegnimentoTempoBatteriaAttivo = 0;
static u32 tickInizioFunzionamentoBatteria = 0;

static u8 statoReteBatteriaInizializzato = 0;
static u8 alimentatorePrecedenteBatteria = 0;
static u8 sessioneCaricaInterrottaDaRete = 0;
static u32 tickInizioInterruzioneRete = 0;

u8 calibrazioneBatteriaRichiesta = 0;
u8 calibrazioneBatteriaAttiva = 0;

static u8 puntoCalibrazioneRichiesto = 0;
static u32 tensioneCalibrazioneRichiesta_mV = 0;
static u32 adcCalibrazioneRaw = 0;
static u32 adcCalibrazioneVref = 0;
static u32 adcCalibrazioneNormalizzato = 0;

/*
 * Converte il valore ADC normalizzato tramite VREFINT in millivolt
 * usando i due punti ottenuti durante la calibrazione universale.
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
        (int64_t)VBAT_CAL_ADC_NORM_1;

    numeratore =
        differenzaADC *
        (
            (int64_t)VBAT_CAL_MV_2 -
            (int64_t)VBAT_CAL_MV_1
        );

    denominatore =
        (int64_t)VBAT_CAL_ADC_NORM_2 -
        (int64_t)VBAT_CAL_ADC_NORM_1;

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
 * Il 100% corrisponde al livello operativo scelto di 8,00 V,
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


/*
 * Azzera il filtro usato soltanto per la tensione visualizzata durante
 * il funzionamento senza alimentazione di rete.
 */
static void resetFiltroTensioneScarica(void)
{
    indiceTensioneScarica = 0U;
    filtroTensioneScaricaInizializzato = 0U;
}


/*
 * Restituisce la mediana delle ultime BAT_SOC_FILTER_SAMPLES misure valide.
 *
 * Alla prima misura la finestra viene inizializzata con copie dello stesso
 * valore. Con una finestra di almeno tre elementi, un singolo campione
 * successivo anomalo non puo spostare immediatamente la mediana.
 */
static u32 filtraTensioneScarica_mV(u32 tensione_mV)
{
    u16 ordinati[BAT_SOC_FILTER_SAMPLES];
    u16 temporaneo;
    u8 i;
    u8 j;

    if(tensione_mV > 0xFFFFU)
    {
        return tensione_mV;
    }

    if(filtroTensioneScaricaInizializzato == 0U)
    {
        for(i = 0U; i < BAT_SOC_FILTER_SAMPLES; i++)
        {
            campioniTensioneScarica[i] = (u16)tensione_mV;
        }

        indiceTensioneScarica = 0U;
        filtroTensioneScaricaInizializzato = 1U;
        return tensione_mV;
    }

    campioniTensioneScarica[indiceTensioneScarica] =
        (u16)tensione_mV;
    indiceTensioneScarica++;

    if(indiceTensioneScarica >= BAT_SOC_FILTER_SAMPLES)
    {
        indiceTensioneScarica = 0U;
    }

    for(i = 0U; i < BAT_SOC_FILTER_SAMPLES; i++)
    {
        ordinati[i] = campioniTensioneScarica[i];
    }

    for(i = 1U; i < BAT_SOC_FILTER_SAMPLES; i++)
    {
        temporaneo = ordinati[i];
        j = i;

        while(j > 0U && ordinati[j - 1U] > temporaneo)
        {
            ordinati[j] = ordinati[j - 1U];
            j--;
        }

        ordinati[j] = temporaneo;
    }

    return (u32)ordinati[BAT_SOC_FILTER_SAMPLES / 2U];
}


/*
 * Soglia di ingresso associata a ciascun livello visualizzato.
 */
static u32 sogliaLivelloBatteria_mV(u8 livello)
{
    switch(livello)
    {
        case 100:
            return VBAT_CHARGE_STOP_MV;

        case 90:
            return 7950U;

        case 80:
            return 7780U;

        case 70:
            return 7650U;

        case 60:
            return 7570U;

        case 50:
            return 7500U;

        case 40:
            return 7450U;

        case 30:
            return 7410U;

        case 20:
            return 7300U;

        case 10:
            return 7050U;

        default:
            return 0U;
    }
}


/*
 * Applica l'isteresi alle sole transizioni verso il basso.
 *
 * La mediana elimina i singoli picchi di carico; l'isteresi richiede
 * inoltre che la tensione scenda realmente sotto la soglia. Le risalite
 * usano invece le soglie nominali, cosi il livello corretto viene
 * ripristinato senza richiedere BAT_LEVEL_HYSTERESIS_MV aggiuntivi.
 */
static u8 calcolaLivelloBatteriaConIsteresi(u32 tensione_mV)
{
    u8 nuovoLivello;
    u32 soglia;

    nuovoLivello = calcolaLivelloBatteria(tensione_mV);

    if(livelloBatteriaInizializzato == 0U)
    {
        livelloBatteriaInizializzato = 1U;
        return nuovoLivello;
    }

    if(nuovoLivello < batteryLevel)
    {
        soglia = sogliaLivelloBatteria_mV(batteryLevel);

        if(soglia > BAT_LEVEL_HYSTERESIS_MV &&
           tensione_mV >= (soglia - BAT_LEVEL_HYSTERESIS_MV))
        {
            return batteryLevel;
        }
    }

    return nuovoLivello;
}


/*
 * Azzera la finestra usata per riconoscere il plateau di tensione.
 */
static void resetControlloPlateau(void)
{
    indiceCampionePlateau = 0U;
    numeroCampioniPlateau = 0U;
    contatoreConfermaPlateau = 0U;
}


/*
 * Termina un'eventuale verifica a riposo senza modificare lo stato
 * generale della sessione di carica.
 */
static void resetVerificaRiposoPlateau(void)
{
    verificaRiposoPlateauAttiva = 0U;
    tickInizioRiposoPlateau = 0U;
    tensioneInizioRiposoPlateau_mV = 0U;
}


/*
 * Inserisce una misura nella finestra mobile e verifica che:
 * - la differenza fra media finale e iniziale resti compresa fra
 *   -BAT_PLATEAU_MAX_FALL_MV e BAT_PLATEAU_MAX_RISE_MV;
 * - l'escursione totale resti entro BAT_PLATEAU_MAX_SPAN_MV.
 */
static u8 aggiornaControlloPlateau(
    u32 tensione_mV,
    int32_t *incremento_mV,
    u32 *escursione_mV
)
{
    u32 sommaIniziale = 0U;
    u32 sommaFinale = 0U;
    u32 mediaIniziale;
    u32 mediaFinale;
    u16 minimo = 0xFFFFU;
    u16 massimo = 0U;
    u16 campione;
    u8 indiceLettura;
    u8 i;

    *incremento_mV = 0;
    *escursione_mV = 0U;

    if(tensione_mV > 0xFFFFU)
    {
        resetControlloPlateau();
        return 0U;
    }

    campioniPlateau[indiceCampionePlateau] = (u16)tensione_mV;
    indiceCampionePlateau++;

    if(indiceCampionePlateau >= BAT_PLATEAU_SAMPLES)
    {
        indiceCampionePlateau = 0U;
    }

    if(numeroCampioniPlateau < BAT_PLATEAU_SAMPLES)
    {
        numeroCampioniPlateau++;
    }

    if(numeroCampioniPlateau < BAT_PLATEAU_SAMPLES)
    {
        return 0U;
    }

    /*
     * Quando la finestra e piena, indiceCampionePlateau indica il
     * campione piu vecchio.
     */
    indiceLettura = indiceCampionePlateau;

    for(i = 0U; i < BAT_PLATEAU_SAMPLES; i++)
    {
        campione = campioniPlateau[indiceLettura];

        if(campione < minimo)
        {
            minimo = campione;
        }

        if(campione > massimo)
        {
            massimo = campione;
        }

        if(i < BAT_PLATEAU_AVERAGE_SAMPLES)
        {
            sommaIniziale += campione;
        }

        if(i >= (BAT_PLATEAU_SAMPLES - BAT_PLATEAU_AVERAGE_SAMPLES))
        {
            sommaFinale += campione;
        }

        indiceLettura++;

        if(indiceLettura >= BAT_PLATEAU_SAMPLES)
        {
            indiceLettura = 0U;
        }
    }

    mediaIniziale =
        sommaIniziale / BAT_PLATEAU_AVERAGE_SAMPLES;
    mediaFinale =
        sommaFinale / BAT_PLATEAU_AVERAGE_SAMPLES;

    *incremento_mV =
        (int32_t)mediaFinale - (int32_t)mediaIniziale;
    *escursione_mV = (u32)massimo - (u32)minimo;

    if(*incremento_mV >= -(int32_t)BAT_PLATEAU_MAX_FALL_MV &&
       *incremento_mV <= (int32_t)BAT_PLATEAU_MAX_RISE_MV &&
       *escursione_mV <= BAT_PLATEAU_MAX_SPAN_MV)
    {
        return 1U;
    }

    return 0U;
}


/*
 * Azzera il watchdog che verifica l'avanzamento della carica.
 */
static void resetControlloStalloCarica(void)
{
    sommaRiferimentoStallo = 0U;
    campioniRiferimentoStallo = 0U;
    riferimentoStallo_mV = 0U;
    riferimentoStalloValido = 0U;

    sommaCampioniRecentiStallo = 0U;
    indiceCampioneRecenteStallo = 0U;
    numeroCampioniRecentiStallo = 0U;
    tempoCaricaStallo_s = 0U;
}


/*
 * Verifica che la tensione aumenti di almeno BAT_STALL_MIN_RISE_MV
 * durante BAT_STALL_CHECK_TIME_S secondi effettivi di carica.
 *
 * Il riferimento iniziale e il valore finale sono medie di piu misure.
 * Se la carica avanza regolarmente, il valore finale diventa il nuovo
 * riferimento per la finestra successiva.
 */
static u8 aggiornaControlloStalloCarica(
    u32 tensione_mV,
    u32 tempoTrascorso_s,
    u8 comandoCarica,
    u32 *tensioneIniziale_mV,
    u32 *tensioneFinale_mV,
    int32_t *incremento_mV,
    u32 *tempoCaricaVerificato_s
)
{
    u32 mediaRecente;

    *tensioneIniziale_mV = 0U;
    *tensioneFinale_mV = 0U;
    *incremento_mV = 0;
    *tempoCaricaVerificato_s = 0U;

    if(tensione_mV > 0xFFFFU)
    {
        resetControlloStalloCarica();
        return 0U;
    }

    if(riferimentoStalloValido == 0U)
    {
        sommaRiferimentoStallo += tensione_mV;
        campioniRiferimentoStallo++;

        if(campioniRiferimentoStallo >=
           BAT_STALL_AVERAGE_SAMPLES)
        {
            riferimentoStallo_mV =
                sommaRiferimentoStallo /
                BAT_STALL_AVERAGE_SAMPLES;
            riferimentoStalloValido = 1U;
            tempoCaricaStallo_s = 0U;
        }

        return 0U;
    }

    if(numeroCampioniRecentiStallo <
       BAT_STALL_AVERAGE_SAMPLES)
    {
        campioniRecentiStallo[indiceCampioneRecenteStallo] =
            (u16)tensione_mV;
        sommaCampioniRecentiStallo += tensione_mV;
        numeroCampioniRecentiStallo++;
    }
    else
    {
        sommaCampioniRecentiStallo -=
            campioniRecentiStallo[indiceCampioneRecenteStallo];
        campioniRecentiStallo[indiceCampioneRecenteStallo] =
            (u16)tensione_mV;
        sommaCampioniRecentiStallo += tensione_mV;
    }

    indiceCampioneRecenteStallo++;

    if(indiceCampioneRecenteStallo >=
       BAT_STALL_AVERAGE_SAMPLES)
    {
        indiceCampioneRecenteStallo = 0U;
    }

    if(comandoCarica != 0U)
    {
        tempoCaricaStallo_s += tempoTrascorso_s;

        if(tempoCaricaStallo_s >
           BAT_STALL_CHECK_TIME_S)
        {
            tempoCaricaStallo_s =
                BAT_STALL_CHECK_TIME_S;
        }
    }

    if(tempoCaricaStallo_s < BAT_STALL_CHECK_TIME_S ||
       numeroCampioniRecentiStallo <
       BAT_STALL_AVERAGE_SAMPLES)
    {
        return 0U;
    }

    mediaRecente =
        sommaCampioniRecentiStallo /
        BAT_STALL_AVERAGE_SAMPLES;

    *tensioneIniziale_mV = riferimentoStallo_mV;
    *tensioneFinale_mV = mediaRecente;
    *incremento_mV =
        (int32_t)mediaRecente -
        (int32_t)riferimentoStallo_mV;
    *tempoCaricaVerificato_s = tempoCaricaStallo_s;

    if(*incremento_mV < (int32_t)BAT_STALL_MIN_RISE_MV)
    {
        return 1U;
    }

    riferimentoStallo_mV = mediaRecente;
    tempoCaricaStallo_s = 0U;

    return 0U;
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

        case BAT_STATE_WAKEUP:
            return "WAKEUP";

        case BAT_STATE_FAULT:
            return "FAULT";

        case BAT_STATE_PLATEAU_REST:
            return "PLATEAU_REST";

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

        case BAT_FAULT_CHARGE_STALLED:
            return "CHARGE_STALLED";

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
    bloccoTemperaturaBassa = 0;

    contatoreAvvio = 0;
    contatoreArresto = 0;
    contatoreDutyCycle = 0;
    contatoreWakeup = 0;

    tempoTotaleCaricaOn_s = 0;
    tempoTopCaricaOn_s = 0;
    tensioneArrestoTimeout_mV = 0;
    contatoreRiarmoTimeout = 0;
    sessioneCaricaInterrottaDaRete = 0U;

    resetControlloPlateau();
    resetVerificaRiposoPlateau();
    resetControlloStalloCarica();
    resetFiltroTensioneScarica();
    livelloBatteriaInizializzato = 0U;

    confermaSottotensioneAttiva = 0U;
    tickConfermaSottotensione = 0U;
    primaTensioneSottotensione_mV = 0U;
    disconnessioneSottotensioneAttiva = 0U;

    batteriaInCarica = 0;
}


void calibraTensioneBatteria(u8 numeroPunto, u32 tensioneMultimetro_mV)
{
    char uart[150];

    if((numeroPunto != 1U && numeroPunto != 2U) ||
       tensioneMultimetro_mV < 5000U ||
       tensioneMultimetro_mV > 8500U)
    {
        snprintf(
            uart,
            sizeof(uart),
            "[BAT-CAL] ERRORE richiesta: punto=%u tensione=%lu mV\r\n",
            (unsigned int)numeroPunto,
            (unsigned long)tensioneMultimetro_mV
        );
        HAL_UART_Transmit(&huart1, (u8 *)uart, strlen(uart), 200);
        return;
    }

    if(calibrazioneBatteriaRichiesta != 0U ||
       calibrazioneBatteriaAttiva != 0U)
    {
        HAL_UART_Transmit(
            &huart1,
            (u8 *)"[BAT-CAL] ERRORE: calibrazione gia in corso\r\n",
            strlen("[BAT-CAL] ERRORE: calibrazione gia in corso\r\n"),
            200
        );
        return;
    }

    puntoCalibrazioneRichiesto = numeroPunto;
    tensioneCalibrazioneRichiesta_mV = tensioneMultimetro_mV;
    calibrazioneBatteriaRichiesta = 1U;

    snprintf(
        uart,
        sizeof(uart),
        "[BAT-CAL] Richiesto punto %u a %lu.%03lu V\r\n",
        (unsigned int)numeroPunto,
        (unsigned long)(tensioneMultimetro_mV / 1000U),
        (unsigned long)(tensioneMultimetro_mV % 1000U)
    );
    HAL_UART_Transmit(&huart1, (u8 *)uart, strlen(uart), 200);
}

static void elaboraCalibrazioneBatteria(
    u8 numeroPunto,
    u32 tensioneMultimetro_mV
)
{
    u32 adcMedio;
    double differenzaTensione;
    double differenzaADC;
    double tensioneVerifica1;
    double tensioneVerifica2;

    char uart[600];

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

    adcMedio = adcCalibrazioneNormalizzato;

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
        "[BAT-CAL] Punto %u: ADC_RAW=%lu ADC_VREF=%lu "
        "VREF_CAL=%u ADC_NORM=%lu, "
        "Vmultimetro=%lu.%03lu V\r\n",
        (unsigned int)numeroPunto,
        (unsigned long)adcCalibrazioneRaw,
        (unsigned long)adcCalibrazioneVref,
        (unsigned int)(*((volatile const u16 *)0x1FFF7A2AUL)),
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
        "[BAT-CAL] Punto 1: ADC_NORM=%lu, V=%lu mV\r\n"
        "[BAT-CAL] Punto 2: ADC_NORM=%lu, V=%lu mV\r\n"
        "[BAT-CAL] Coefficiente: %.9f mV/count\r\n"
        "[BAT-CAL] Offset: %.3f mV\r\n"
        "[BAT-CAL] Verifica: V1=%.3f mV, V2=%.3f mV\r\n"
        "\r\n"
        "#define VBAT_CAL_ADC_NORM_1    %luL\r\n"
        "#define VBAT_CAL_MV_1          %luL\r\n"
        "#define VBAT_CAL_ADC_NORM_2    %luL\r\n"
        "#define VBAT_CAL_MV_2          %luL\r\n"
        "\r\n",
        (unsigned long)calibrazioneBatteria.adcPunto1,
        (unsigned long)calibrazioneBatteria.tensionePunto1_mV,

        (unsigned long)calibrazioneBatteria.adcPunto2,
        (unsigned long)calibrazioneBatteria.tensionePunto2_mV,

        calibrazioneBatteria.coefficiente_mV_count,
        calibrazioneBatteria.offset_mV,

        tensioneVerifica1,
        tensioneVerifica2,

        (unsigned long)calibrazioneBatteria.adcPunto1,
        (unsigned long)calibrazioneBatteria.tensionePunto1_mV,
        (unsigned long)calibrazioneBatteria.adcPunto2,
        (unsigned long)calibrazioneBatteria.tensionePunto2_mV
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
    u32 mediaRaw;
    u32 adcVref;
    u32 minimo;
    u32 massimo;
    u32 dispersione;
    u8 valida;
    u8 pronta;
    GPIO_PinState statoPC0;
    GPIO_PinState statoPE15;
} BatteryAdcMeasurement_t;

/*
 * Acquisisce VREFINT nella stessa sessione della batteria.
 * Il valore viene usato insieme alla calibrazione individuale ST
 * per riferire il conteggio della batteria a VDDA = 3,3 V.
 */
static u8 acquisisciVrefBatteria(u32 *adcVref)
{
    ADC_ChannelConfTypeDef sConfig;
    u32 somma = 0U;
    u8 i;

    ADC->CCR |= ADC_CCR_TSVREFE;

    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0U;
    }

    /* Prima conversione scartata dopo il cambio canale. */
    if(HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0U;
    }

    if(HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0U;
    }

    (void)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    for(i = 0U; i < 8U; i++)
    {
        if(HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            return 0U;
        }

        if(HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            return 0U;
        }

        somma += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    *adcVref = somma / 8U;

    if(*adcVref == 0U || *adcVref >= 4095U)
    {
        return 0U;
    }

    return 1U;
}

/*
 * Attesa non bloccante basata su HAL_GetTick().
 *
 * Alla prima chiamata memorizza l'istante iniziale e restituisce 0.
 * Le chiamate successive restituiscono 0 fino alla scadenza e 1 quando
 * il tempo richiesto e trascorso. La sottrazione unsigned gestisce
 * correttamente anche il rollover di HAL_GetTick().
 */
static u8 delayNonBloccante(
    u32 durata_ms,
    u32 *tickInizio,
    u8 *attesaAttiva
)
{
    if(*attesaAttiva == 0U)
    {
        *tickInizio = HAL_GetTick();
        *attesaAttiva = 1U;
        return 0U;
    }

    if((HAL_GetTick() - *tickInizio) < durata_ms)
    {
        return 0U;
    }

    *attesaAttiva = 0U;
    return 1U;
}

/*
 * Misura la batteria nelle stesse condizioni usate per la calibrazione.
 *
 * Sequenza:
 * 1. spegne la carica (PC1 = 0);
 * 2. con alimentatore presente richiude PE15, cosi un precedente errore
 *    non puo lasciare il ramo di misura isolato in modo permanente;
 * 3. attiva PC0 e restituisce subito il controllo al ciclo principale;
 * 4. dopo BAT_MEASURE_DELAY millisecondi scarta la prima conversione;
 * 5. acquisisce piu campioni, elimina minimo e massimo e calcola la media;
 * 6. disattiva PC0.
 */
static BatteryAdcMeasurement_t misuraAdcBatteria(void)
{
    static u8 misuraInCorso = 0U;
    static u8 attesaMisuraAttiva = 0U;
    static u32 tickInizioMisura = 0U;

    BatteryAdcMeasurement_t risultato;
    u32 somma = 0;
    u32 campione;
    u8 i;

    risultato.media = 0;
    risultato.mediaRaw = 0;
    risultato.adcVref = 0;
    risultato.minimo = 0xFFFFFFFFU;
    risultato.massimo = 0;
    risultato.dispersione = 0;
    risultato.valida = 0;
    risultato.pronta = 0;
    risultato.statoPC0 = GPIO_PIN_RESET;
    risultato.statoPE15 = GPIO_PIN_RESET;

    if(misuraInCorso == 0U)
    {
        /* La carica deve essere disabilitata durante tutta la misura. */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        batteriaInCarica = 0;

        /*
         * Con alimentatore presente richiudiamo PE15 prima della misura.
         * L'intera stabilizzazione di PE15 e PC0 avviene nella finestra
         * non bloccante BAT_MEASURE_DELAY.
         */
        if(alimentatore != 0 &&
           disconnessioneTermicaAttiva == 0U &&
           disconnessioneTemperaturaBassaAttiva == 0U)
        {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
        }

        /* Attiva il carico da 100 ohm e il partitore di misura. */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
        misuraInCorso = 1U;
    }

    risultato.statoPE15 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
    risultato.statoPC0 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

    if(delayNonBloccante(
           (u32)BAT_MEASURE_DELAY,
           &tickInizioMisura,
           &attesaMisuraAttiva
       ) == 0U)
    {
        return risultato;
    }

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

    risultato.dispersione = risultato.massimo - risultato.minimo;

    somma -= risultato.minimo;
    somma -= risultato.massimo;
    risultato.media = somma / (BAT_ADC_SAMPLES - 2U);
    risultato.mediaRaw = risultato.media;

    if(acquisisciVrefBatteria(&risultato.adcVref) != 0U)
    {
        u16 vrefCal;

        vrefCal = *((volatile const u16 *)0x1FFF7A2AUL);

        if(vrefCal != 0U)
        {
            risultato.media =
                (u32)
                (
                    (
                        (uint64_t)risultato.mediaRaw *
                        (uint64_t)vrefCal +
                        ((uint64_t)risultato.adcVref / 2ULL)
                    ) /
                    (uint64_t)risultato.adcVref
                );
        }
        else
        {
            risultato.adcVref = 0U;
        }
    }

    /*
     * VREFINT viene acquisita prima di rimuovere il carico, cosi la
     * compensazione rappresenta le stesse condizioni elettriche della
     * misura batteria. PC0 viene poi disattivato immediatamente.
     */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    misuraInCorso = 0U;
    risultato.pronta = 1U;

    if(risultato.adcVref != 0U &&
       risultato.media >= BAT_ADC_MIN_VALID &&
       risultato.media <= BAT_ADC_MAX_VALID &&
       risultato.dispersione <= BAT_ADC_MAX_SPREAD)
    {
        risultato.valida = 1;
    }

    return risultato;
}

void gestisciCalibrazioneBatteria(void)
{
    BatteryAdcMeasurement_t misura;

    if(calibrazioneBatteriaRichiesta == 0U &&
       calibrazioneBatteriaAttiva == 0U)
    {
        return;
    }

    if(calibrazioneBatteriaAttiva == 0U)
    {
        calibrazioneBatteriaRichiesta = 0U;
        calibrazioneBatteriaAttiva = 1U;

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        batteriaInCarica = 0U;
    }

    misura = misuraAdcBatteria();

    if(misura.pronta == 0U)
    {
        return;
    }

    adcCalibrazioneRaw = misura.mediaRaw;
    adcCalibrazioneVref = misura.adcVref;
    adcCalibrazioneNormalizzato = misura.media;

    if(misura.adcVref == 0U ||
       misura.media == 0U ||
       misura.media >= 4095U)
    {
        HAL_UART_Transmit(
            &huart1,
            (u8 *)"[BAT-CAL] ERRORE acquisizione ADC/VREF\r\n",
            strlen("[BAT-CAL] ERRORE acquisizione ADC/VREF\r\n"),
            200
        );
    }
    else
    {
        elaboraCalibrazioneBatteria(
            puntoCalibrazioneRichiesto,
            tensioneCalibrazioneRichiesta_mV
        );
    }

    calibrazioneBatteriaAttiva = 0U;
}

u8 controllaBatteria(void)
{
    BatteryAdcMeasurement_t misuraADC;
    u32 acquisizione;
    u32 tensioneMisurata_mV = 0;
    u32 tensioneVisualizzata_mV = 0;
    u32 tickAttuale;
    u32 tempoTrascorso_s;
    u32 escursionePlateau_mV = 0;
    u32 tensioneInizialeStallo_mV = 0;
    u32 tensioneFinaleStallo_mV = 0;
    u32 tempoCaricaVerificatoStallo_s = 0;
    int32_t incrementoPlateau_mV = 0;
    int32_t incrementoStallo_mV = 0;
    long temperaturaMCU;
    long temperaturaRapida;

    u8 adcValido;
    u8 batteriaNonRilevata = 0;
    u8 recuperoNecessario = 0;
    u8 temperaturaValida = 1;
    u8 comandoCarica = 0;
    u8 collegaBatteria = 1;
    u8 dutyPercentuale = 0;
    u8 misuraConfermaSottotensione = 0;
    u8 riarmoTimeoutRichiesto = 0;

    BatteryState_t statoPrecedente;
    BatteryFault_t faultAttuale;

    u8 sms[200];
    u8 addressFram[2] = {1, 91};
    char uart[360];

    /* =============================================================
     * 1. TEMPO TRASCORSO FRA DUE ESECUZIONI
     * ============================================================= */
    tickAttuale = HAL_GetTick();

    /*
     * Rileva i fronti dell'alimentatore per distinguere una breve
     * interruzione da una vera fase di funzionamento a batteria.
     *
     * Viene memorizzato se la sessione era attiva sul fronte di discesa.
     * Al ritorno della rete, dopo il tempo minimo configurato, si assegna
     * alla sessione un nuovo budget temporale senza alterare fault o stato.
     */
    if(statoReteBatteriaInizializzato == 0U)
    {
        alimentatorePrecedenteBatteria =
            (alimentatore != 0) ? 1U : 0U;
        statoReteBatteriaInizializzato = 1U;

        if(alimentatorePrecedenteBatteria == 0U)
        {
            tickInizioInterruzioneRete = tickAttuale;
            sessioneCaricaInterrottaDaRete =
                (sessioneCaricaAttiva != 0U) ? 1U : 0U;
        }
    }
    else if(alimentatorePrecedenteBatteria !=
            ((alimentatore != 0) ? 1U : 0U))
    {
        if(alimentatore == 0)
        {
            /* Fronte rete presente -> rete assente. */
            tickInizioInterruzioneRete = tickAttuale;
            sessioneCaricaInterrottaDaRete =
                (sessioneCaricaAttiva != 0U) ? 1U : 0U;
        }
        else
        {
            /* Fronte rete assente -> rete presente. */
            u32 durataInterruzioneRete_ms =
                tickAttuale - tickInizioInterruzioneRete;

            if(sessioneCaricaInterrottaDaRete != 0U &&
               durataInterruzioneRete_ms >=
                   BAT_CHARGE_SESSION_RESET_OFF_TIME_MS)
            {
                tempoTotaleCaricaOn_s = 0U;
                tempoTopCaricaOn_s = 0U;
                contatoreAvvio = 0U;
                contatoreArresto = 0U;
                contatoreDutyCycle = 0U;
                resetControlloPlateau();
                resetVerificaRiposoPlateau();
                resetControlloStalloCarica();

                snprintf(
                    uart,
                    sizeof(uart),
                    "[BAT] Charge timers reset: AC off for %lus\r\n",
                    (unsigned long)(durataInterruzioneRete_ms / 1000UL)
                );

                HAL_UART_Transmit(
                    &huart1,
                    (u8 *)uart,
                    strlen(uart),
                    300
                );
            }

            sessioneCaricaInterrottaDaRete = 0U;
            tickInizioInterruzioneRete = 0U;
        }

        alimentatorePrecedenteBatteria =
            (alimentatore != 0) ? 1U : 0U;
    }

    /*
     * Il ritorno dell'alimentatore:
     * - rimuove il latch di sottotensione;
     * - annulla il timer del funzionamento a batteria;
     * - consente a PE15 di essere ricollegato, salvo protezioni termiche.
     */
    if(alimentatore != 0)
    {
        disconnessioneSottotensioneAttiva = 0U;
        timerFunzionamentoBatteriaAttivo = 0U;
        tickInizioFunzionamentoBatteria = 0U;

        if(spegnimentoTempoBatteriaAttivo != 0U)
        {
            /*
             * Evita che il lungo intervallo di spegnimento venga
             * conteggiato nei timer della nuova sessione di carica.
             */
            ultimoTickBatteria = tickAttuale;
            spegnimentoTempoBatteriaAttivo = 0U;
        }
    }
    else
    {
        if(timerFunzionamentoBatteriaAttivo == 0U)
        {
            tickInizioFunzionamentoBatteria = tickAttuale;
            timerFunzionamentoBatteriaAttivo = 1U;
        }
        else if(tempoSpegnimentoBatteriaMinuti != 0U &&
                spegnimentoTempoBatteriaAttivo == 0U &&
                (tickAttuale - tickInizioFunzionamentoBatteria) >=
                    ((u32)tempoSpegnimentoBatteriaMinuti *
                     60UL * 1000UL))
        {
            spegnimentoTempoBatteriaAttivo = 1U;

            snprintf(
                uart,
                sizeof(uart),
                "[BAT] Backup time expired: %lus, disconnecting PE15\r\n",
                (unsigned long)tempoSpegnimentoBatteriaMinuti * 60UL
            );

            HAL_UART_Transmit(
                &huart1,
                (u8 *)uart,
                strlen(uart),
                300
            );
        }
    }

    /*
     * Dopo la scadenza si comandano immediatamente spenti carica,
     * circuito di misura e collegamento batteria. Se il micro resta
     * temporaneamente alimentato da capacita residue, il latch impedisce
     * che PE15 venga richiusto prima del ritorno della rete.
     */
    if(spegnimentoTempoBatteriaAttivo != 0U)
    {
        statoCaricaBatteria = BAT_STATE_NO_SUPPLY;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
        batteriaInCarica = 0U;
        return 1U;
    }

    /*
     * La conferma di una sottotensione e non bloccante. Durante
     * l'attesa il flag periodico resta attivo e il main continua a
     * richiamare questa funzione.
     */
    if(confermaSottotensioneAttiva != 0U)
    {
        if(alimentatore != 0)
        {
            confermaSottotensioneAttiva = 0U;
            primaTensioneSottotensione_mV = 0U;
        }
        else if((tickAttuale - tickConfermaSottotensione) <
                BAT_UNDERVOLTAGE_CONFIRM_DELAY_MS)
        {
            return 0U;
        }
        else
        {
            misuraConfermaSottotensione = 1U;
        }
    }

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

    /*
     * Durante il recupero la batteria viene misurata una sola volta
     * ogni 45 s, al termine dei 30 s di carica.
     *
     * contatoreWakeup:
     *   0 = trascorsi 15 s senza carica: avvia i 30 s di carica;
     *   1 = trascorsi i primi 15 s di carica: prosegue senza inserire PC0;
     *   2 = trascorsi 30 s di carica: esegue la misura completa qui sotto.
     *
     * Nei passaggi senza misura batteria viene comunque controllato l'ADC
     * tramite VREFINT e sensore di temperatura interno.
     */
    if(statoCaricaBatteria == BAT_STATE_WAKEUP &&
       faultCaricaLatched == BAT_FAULT_NONE &&
       alimentatore != 0 &&
       contatoreWakeup < 2U)
    {
        temperaturaRapida = acquisizioneTemp();

        if(temperaturaRapida >= TEMP_SENSOR_MIN_VALID_C &&
           temperaturaRapida <= TEMP_SENSOR_MAX_VALID_C &&
           temperaturaRapida >= TEMP_MCU_CHARGE_MIN_C &&
           bloccoTemperaturaBassa == 0U &&
           disconnessioneTemperaturaBassaAttiva == 0U &&
           disconnessioneTermicaAttiva == 0U &&
           temperaturaRapida < TEMP_MCU_CHARGE_MAX_C)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

            batteriaInCarica = 1;
            contatoreWakeup++;
            ultimoTickBatteria = tickAttuale;
            return 1U;
        }

        /*
         * Diagnostica ADC o temperatura non valida: interrompe subito
         * la carica e prosegue con il controllo completo.
         */
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        batteriaInCarica = 0;
    }

    /* =============================================================
     * 2. MISURA DELLA BATTERIA
     * ============================================================= */
    misuraADC = misuraAdcBatteria();

    if(misuraADC.pronta == 0U)
    {
        return 0U;
    }

    ultimoTickBatteria = tickAttuale;
    acquisizione = misuraADC.media;
    adcValido = misuraADC.valida;

    if(misuraConfermaSottotensione != 0U &&
       adcValido == 0U)
    {
        confermaSottotensioneAttiva = 0U;
        primaTensioneSottotensione_mV = 0U;
    }

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

        /*
         * Prima di scollegare la batteria per una singola misura sotto
         * soglia, viene richiesta una seconda misura dopo 2 secondi.
         * Durante l'attesa restano spenti carica e circuito di misura,
         * mentre PE15 mantiene collegata la batteria.
         */
        if(alimentatore == 0 &&
           tensioneMisurata_mV < VBAT_DISCONNECT_MV &&
           misuraConfermaSottotensione == 0U)
        {
            confermaSottotensioneAttiva = 1U;
            tickConfermaSottotensione = HAL_GetTick();
            primaTensioneSottotensione_mV = tensioneMisurata_mV;

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
            batteriaInCarica = 0U;
            return 0U;
        }

        if(misuraConfermaSottotensione != 0U)
        {
            confermaSottotensioneAttiva = 0U;

            if(tensioneMisurata_mV >= VBAT_DISCONNECT_MV)
            {
                snprintf(
                    uart,
                    sizeof(uart),
                    "[BAT] Load transient ignored: First=%lu.%03luV "
                    "Confirm=%lu.%03luV\r\n",
                    (unsigned long)
                        (primaTensioneSottotensione_mV / 1000U),
                    (unsigned long)
                        (primaTensioneSottotensione_mV % 1000U),
                    (unsigned long)(tensioneMisurata_mV / 1000U),
                    (unsigned long)(tensioneMisurata_mV % 1000U)
                );

                HAL_UART_Transmit(
                    &huart1,
                    (u8 *)uart,
                    strlen(uart),
                    300
                );
            }

            primaTensioneSottotensione_mV = 0U;
        }

        /*
         * Durante il funzionamento a batteria soltanto la tensione
         * visualizzata e la percentuale vengono filtrate. La misura
         * grezza resta disponibile per protezioni e macchina di carica.
         */
        if(alimentatore == 0)
        {
            tensioneVisualizzata_mV =
                filtraTensioneScarica_mV(tensioneMisurata_mV);
            batteryLevel =
                calcolaLivelloBatteriaConIsteresi(
                    tensioneVisualizzata_mV
                );
        }
        else
        {
            resetFiltroTensioneScarica();
            tensioneVisualizzata_mV = tensioneMisurata_mV;
            batteryLevel =
                calcolaLivelloBatteria(tensioneVisualizzata_mV);
            livelloBatteriaInizializzato = 1U;
        }

        tensioneB =
            ((double)tensioneVisualizzata_mV) / 1000.0;
        tensioneBint =
            (int)((tensioneVisualizzata_mV + 50U) / 100U);
    }
    else
    {
        tensioneMisurata_mV = 0;
        tensioneVisualizzata_mV = 0;
        tensioneB = 0.0;
        tensioneBint = 0;
        batteryLevel = 5;
        resetFiltroTensioneScarica();
        livelloBatteriaInizializzato = 0U;
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

    /*
     * Una misura bassa ma stabile non indica necessariamente un guasto ADC:
     * puo essere una batteria assente oppure una batteria in protezione per
     * sovrascarica. La misura interna di VREFINT e temperatura, eseguita da
     * acquisizioneTemp(), fornisce una verifica indipendente dell'ADC.
     */
    if(adcValido == 0 &&
       acquisizione < BAT_ADC_MIN_VALID &&
       misuraADC.dispersione <= BAT_ADC_MAX_SPREAD &&
       temperaturaValida != 0)
    {
        batteriaNonRilevata = 1;
    }

    /* Isteresi del blocco di carica per temperatura troppo alta. */
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

    /* Isteresi del blocco di carica per temperatura troppo bassa. */
    if(temperaturaValida == 0)
    {
        bloccoTemperaturaBassa = 1U;
    }
    else if(bloccoTemperaturaBassa != 0U)
    {
        if(temperaturaMCU >= TEMP_MCU_CHARGE_MIN_RESTART_C)
        {
            bloccoTemperaturaBassa = 0U;
        }
    }
    else if(temperaturaMCU < TEMP_MCU_CHARGE_MIN_C)
    {
        bloccoTemperaturaBassa = 1U;
    }

    /*
     * La protezione termica estrema e indipendente dal blocco della
     * sola carica. Una volta aperto PE15, viene richiesta una discesa
     * fino a TEMP_MCU_EMERGENCY_RESTART_C prima di ricollegare
     * la batteria.
     *
     * Con misura di temperatura non valida il latch conserva il proprio
     * stato: una batteria gia scollegata non viene ricollegata senza una
     * conferma valida del raffreddamento.
     */
    if(temperaturaValida != 0)
    {
        if(disconnessioneTermicaAttiva != 0U)
        {
            if(temperaturaMCU <= TEMP_MCU_EMERGENCY_RESTART_C)
            {
                disconnessioneTermicaAttiva = 0U;
            }
        }
        else if(temperaturaMCU >= TEMP_MCU_EMERGENCY_C)
        {
            disconnessioneTermicaAttiva = 1U;
        }
    }

    /*
     * Protezione della temperatura minima di funzionamento. A differenza
     * del limite minimo di carica, questa apre PE15. Il latch viene
     * cancellato soltanto dopo una misura valida che conferma il ritorno
     * a TEMP_MCU_OPERATION_RESTART_C o oltre.
     */
    if(temperaturaValida != 0)
    {
        if(disconnessioneTemperaturaBassaAttiva != 0U)
        {
            if(temperaturaMCU >= TEMP_MCU_OPERATION_RESTART_C)
            {
                disconnessioneTemperaturaBassaAttiva = 0U;
            }
        }
        else if(temperaturaMCU < TEMP_MCU_OPERATION_MIN_C)
        {
            disconnessioneTemperaturaBassaAttiva = 1U;
        }
    }

    if(alimentatore != 0 &&
       temperaturaValida != 0 &&
       bloccoTemperatura == 0 &&
       bloccoTemperaturaBassa == 0 &&
       (batteriaNonRilevata != 0 ||
        (adcValido != 0 &&
         tensioneMisurata_mV < VBAT_MIN_AUTOMATIC_CHARGE_MV)))
    {
        recuperoNecessario = 1;
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
        tensioneArrestoTimeout_mV = 0U;
        contatoreRiarmoTimeout = 0U;
    }

    /*
     * I fault di timeout si riarmano automaticamente soltanto dopo una
     * perdita di tensione significativa rispetto al valore registrato
     * all'arresto. La verifica viene fatta con alimentatore presente:
     * in questo modo la nuova sessione puo iniziare immediatamente e la
     * misura non e influenzata dai picchi di carico del funzionamento
     * a batteria.
     */
    if((faultCaricaLatched == BAT_FAULT_TOTAL_TIMEOUT ||
        faultCaricaLatched == BAT_FAULT_TOP_TIMEOUT) &&
       alimentatore != 0 &&
       temperaturaValida != 0)
    {
        if(batteriaNonRilevata != 0)
        {
            /*
             * Una batteria in protezione per sovrascarica puo fornire
             * ADC prossimo a zero: e comunque una condizione valida per
             * rimuovere il timeout e consentire il WAKEUP.
             */
            riarmoTimeoutRichiesto = 1U;
        }
        else if(adcValido != 0 &&
                tensioneArrestoTimeout_mV >=
                    BAT_TIMEOUT_RESTART_DROP_MV &&
                tensioneMisurata_mV <=
                    (tensioneArrestoTimeout_mV -
                     BAT_TIMEOUT_RESTART_DROP_MV))
        {
            riarmoTimeoutRichiesto = 1U;
        }
    }

    if(riarmoTimeoutRichiesto != 0U)
    {
        if(contatoreRiarmoTimeout < BAT_TIMEOUT_RESTART_CONFIRM_COUNT)
        {
            contatoreRiarmoTimeout++;
        }

        if(contatoreRiarmoTimeout >= BAT_TIMEOUT_RESTART_CONFIRM_COUNT)
        {
            if(adcValido != 0)
            {
                u32 cadutaTimeout_mV =
                    tensioneArrestoTimeout_mV - tensioneMisurata_mV;

                snprintf(
                    uart,
                    sizeof(uart),
                    "[BAT] Timeout cleared: Ref=%lu.%03luV "
                    "Now=%lu.%03luV Drop=%lumV\r\n",
                    (unsigned long)(tensioneArrestoTimeout_mV / 1000U),
                    (unsigned long)(tensioneArrestoTimeout_mV % 1000U),
                    (unsigned long)(tensioneMisurata_mV / 1000U),
                    (unsigned long)(tensioneMisurata_mV % 1000U),
                    (unsigned long)cadutaTimeout_mV
                );
            }
            else
            {
                snprintf(
                    uart,
                    sizeof(uart),
                    "[BAT] Timeout cleared: Ref=%lu.%03luV "
                    "Now=NOT_DETECTED\r\n",
                    (unsigned long)(tensioneArrestoTimeout_mV / 1000U),
                    (unsigned long)(tensioneArrestoTimeout_mV % 1000U)
                );
            }

            HAL_UART_Transmit(
                &huart1,
                (u8 *)uart,
                strlen(uart),
                300
            );

            faultCaricaLatched = BAT_FAULT_NONE;
            faultAttuale = BAT_FAULT_NONE;
            sessioneCaricaAttiva = 1U;
            caricaCompleta = 0U;
            contatoreAvvio = 0U;
            contatoreArresto = 0U;
            contatoreDutyCycle = 0U;
            tempoTotaleCaricaOn_s = 0U;
            tempoTopCaricaOn_s = 0U;
            tensioneArrestoTimeout_mV = 0U;
            contatoreRiarmoTimeout = 0U;
            resetControlloPlateau();
            resetVerificaRiposoPlateau();
            resetControlloStalloCarica();
        }
    }
    else
    {
        contatoreRiarmoTimeout = 0U;
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
    else if(recuperoNecessario != 0)
    {
        /*
         * Recupero senza limite complessivo:
         * due intervalli da 15 s con carica attiva, seguiti da un
         * intervallo da 15 s senza carica. La misura mantiene PC0
         * collegato soltanto per BAT_MEASURE_DELAY.
         */
        statoCaricaBatteria = BAT_STATE_WAKEUP;
        faultAttuale = BAT_FAULT_NONE;
        sessioneCaricaAttiva = 0;
        caricaCompleta = 0;
        contatoreAvvio = 0;
        contatoreArresto = 0;
        dutyPercentuale = 67;

        comandoCarica = (contatoreWakeup < 2U) ? 1U : 0U;
        contatoreWakeup++;

        if(contatoreWakeup >= 3U)
        {
            contatoreWakeup = 0;
        }
    }
    else if(adcValido == 0)
    {
        /* Fault transitorio: al prossimo ciclo la misura viene riprovata. */
        statoCaricaBatteria = BAT_STATE_FAULT;
        faultAttuale = BAT_FAULT_ADC;
        comandoCarica = 0;
        dutyPercentuale = 0;
        contatoreWakeup = 0;
    }
    else if(temperaturaValida == 0)
    {
        statoCaricaBatteria = BAT_STATE_FAULT;
        faultAttuale = BAT_FAULT_TEMPERATURE_SENSOR;
        comandoCarica = 0;
        dutyPercentuale = 0;
        contatoreWakeup = 0;
    }
    else if(alimentatore == 0)
    {
        statoCaricaBatteria = BAT_STATE_NO_SUPPLY;
        comandoCarica = 0;
        dutyPercentuale = 0;
        contatoreWakeup = 0;
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
        contatoreWakeup = 0;
    }
    else if(bloccoTemperatura != 0 ||
            bloccoTemperaturaBassa != 0)
    {
        statoCaricaBatteria = BAT_STATE_WAIT_TEMPERATURE;
        comandoCarica = 0;
        dutyPercentuale = 0;
    }
    else
    {
        contatoreWakeup = 0;

        /*
         * Dopo FULL si riparte sotto VBAT_RECHARGE_MV per
         * BAT_START_CONFIRM_COUNT misure consecutive.
         */
        if(caricaCompleta != 0)
        {
            statoCaricaBatteria = BAT_STATE_FULL;
            comandoCarica = 0;
            dutyPercentuale = 0;

            if(tensioneMisurata_mV < VBAT_RECHARGE_MV)
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
            if(tensioneMisurata_mV < VBAT_RECHARGE_MV)
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
            if(verificaRiposoPlateauAttiva != 0U)
            {
                /*
                 * Il candidato plateau e gia stato riconosciuto:
                 * PC1 resta spento fino al termine della verifica.
                 */
                statoCaricaBatteria = BAT_STATE_PLATEAU_REST;
                comandoCarica = 0U;
                dutyPercentuale = 0U;
                contatoreArresto = 0U;
            }
            else if(tensioneMisurata_mV >= VBAT_CHARGE_STOP_MV)
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

                if(BAT_PULSED_CHARGE_ENABLED == 0U)
                {
                    statoCaricaBatteria = BAT_STATE_BULK;
                    comandoCarica = 1;
                    dutyPercentuale = 100;
                    contatoreDutyCycle = 0;
                }
                else if(tensioneMisurata_mV < VBAT_TOP_50_MV)
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
     * 6. WATCHDOG DI AVANZAMENTO DELLA CARICA
     * =============================================================
     * Nella fascia precedente al plateau finale la tensione deve
     * aumentare di almeno BAT_STALL_MIN_RISE_MV durante il tempo
     * effettivo di carica configurato. In caso contrario il fault viene
     * latched e PC1 resta spento fino al reset esplicito del fault o al
     * riavvio del microcontrollore.
     */
    if(BAT_STALL_CHECK_ENABLED != 0U &&
       alimentatore != 0 &&
       adcValido != 0 &&
       temperaturaValida != 0 &&
       faultCaricaLatched == BAT_FAULT_NONE &&
       sessioneCaricaAttiva != 0 &&
       statoCaricaBatteria != BAT_STATE_WAKEUP)
    {
        if(tensioneMisurata_mV >= BAT_STALL_MIN_MV &&
           tensioneMisurata_mV < BAT_STALL_MAX_MV)
        {
            if(aggiornaControlloStalloCarica(
                   tensioneMisurata_mV,
                   tempoTrascorso_s,
                   comandoCarica,
                   &tensioneInizialeStallo_mV,
                   &tensioneFinaleStallo_mV,
                   &incrementoStallo_mV,
                   &tempoCaricaVerificatoStallo_s
               ) != 0U)
            {
                faultCaricaLatched = BAT_FAULT_CHARGE_STALLED;
                faultAttuale = BAT_FAULT_CHARGE_STALLED;
                statoCaricaBatteria = BAT_STATE_FAULT;
                sessioneCaricaAttiva = 0;
                caricaCompleta = 0;
                comandoCarica = 0;
                dutyPercentuale = 0;
                contatoreAvvio = 0;
                contatoreArresto = 0;
                contatoreDutyCycle = 0;

                snprintf(
                    uart,
                    sizeof(uart),
                    "[BAT] Charge stopped: STALLED Start=%lu.%03luV "
                    "End=%lu.%03luV Rise=%ldmV OnTime=%lus\r\n",
                    (unsigned long)
                        (tensioneInizialeStallo_mV / 1000U),
                    (unsigned long)
                        (tensioneInizialeStallo_mV % 1000U),
                    (unsigned long)
                        (tensioneFinaleStallo_mV / 1000U),
                    (unsigned long)
                        (tensioneFinaleStallo_mV % 1000U),
                    (long)incrementoStallo_mV,
                    (unsigned long)tempoCaricaVerificatoStallo_s
                );

                HAL_UART_Transmit(
                    &huart1,
                    (u8 *)uart,
                    strlen(uart),
                    300
                );

                resetControlloStalloCarica();
            }
        }
        else if(tensioneMisurata_mV < BAT_STALL_MIN_MV)
        {
            resetControlloStalloCarica();
        }
    }
    else
    {
        resetControlloStalloCarica();
    }

    /* =============================================================
     * 7. PLATEAU E VERIFICA A RIPOSO
     * =============================================================
     * Prima fase:
     * - finestra mobile di BAT_PLATEAU_SAMPLES;
     * - pendenza limitata sia in salita sia in discesa;
     * - escursione complessiva limitata;
     * - BAT_PLATEAU_CONFIRM_COUNT finestre valide consecutive.
     *
     * Seconda fase:
     * - PC1 spento per BAT_PLATEAU_REST_TIME_MS;
     * - controllo della tensione rilassata e della caduta.
     *
     * Soltanto la seconda fase puo dichiarare la carica completa.
     */
    if(verificaRiposoPlateauAttiva != 0U)
    {
        if(BAT_PLATEAU_TERMINATION_ENABLED != 0U &&
           BAT_PULSED_CHARGE_ENABLED == 0U &&
           alimentatore != 0 &&
           adcValido != 0 &&
           temperaturaValida != 0 &&
           bloccoTemperatura == 0U &&
           bloccoTemperaturaBassa == 0U &&
           faultCaricaLatched == BAT_FAULT_NONE &&
           sessioneCaricaAttiva != 0U)
        {
            u32 durataRiposoPlateau_ms =
                tickAttuale - tickInizioRiposoPlateau;

            statoCaricaBatteria = BAT_STATE_PLATEAU_REST;
            comandoCarica = 0U;
            dutyPercentuale = 0U;

            if(durataRiposoPlateau_ms >= BAT_PLATEAU_REST_TIME_MS)
            {
                u32 cadutaRiposoPlateau_mV = 0U;
                u8 riposoConfermato = 0U;

                if(tensioneInizioRiposoPlateau_mV >
                   tensioneMisurata_mV)
                {
                    cadutaRiposoPlateau_mV =
                        tensioneInizioRiposoPlateau_mV -
                        tensioneMisurata_mV;
                }

                if(tensioneMisurata_mV >=
                       BAT_PLATEAU_REST_MIN_MV &&
                   cadutaRiposoPlateau_mV <=
                       BAT_PLATEAU_REST_MAX_DROP_MV)
                {
                    riposoConfermato = 1U;
                }

                if(riposoConfermato != 0U)
                {
                    caricaCompleta = 1U;
                    sessioneCaricaAttiva = 0U;
                    comandoCarica = 0U;
                    dutyPercentuale = 0U;
                    contatoreAvvio = 0U;
                    contatoreArresto = 0U;
                    contatoreDutyCycle = 0U;
                    statoCaricaBatteria = BAT_STATE_FULL;

                    snprintf(
                        uart,
                        sizeof(uart),
                        "[BAT] Charge complete: PLATEAU_REST "
                        "Start=%lu.%03luV Rest=%lu.%03luV "
                        "Drop=%lumV Time=%lus\r\n",
                        (unsigned long)
                            (tensioneInizioRiposoPlateau_mV / 1000U),
                        (unsigned long)
                            (tensioneInizioRiposoPlateau_mV % 1000U),
                        (unsigned long)
                            (tensioneMisurata_mV / 1000U),
                        (unsigned long)
                            (tensioneMisurata_mV % 1000U),
                        (unsigned long)cadutaRiposoPlateau_mV,
                        (unsigned long)
                            (durataRiposoPlateau_ms / 1000UL)
                    );
                }
                else
                {
                    /*
                     * Il plateau non ha retto a riposo. La sessione
                     * rimane attiva e riparte da BULK con una finestra
                     * completamente nuova.
                     */
                    statoCaricaBatteria = BAT_STATE_BULK;
                    comandoCarica = 1U;
                    dutyPercentuale = 100U;

                    snprintf(
                        uart,
                        sizeof(uart),
                        "[BAT] Plateau rejected after rest: "
                        "Start=%lu.%03luV Rest=%lu.%03luV "
                        "Drop=%lumV Time=%lus; charge resumed\r\n",
                        (unsigned long)
                            (tensioneInizioRiposoPlateau_mV / 1000U),
                        (unsigned long)
                            (tensioneInizioRiposoPlateau_mV % 1000U),
                        (unsigned long)
                            (tensioneMisurata_mV / 1000U),
                        (unsigned long)
                            (tensioneMisurata_mV % 1000U),
                        (unsigned long)cadutaRiposoPlateau_mV,
                        (unsigned long)
                            (durataRiposoPlateau_ms / 1000UL)
                    );
                }

                HAL_UART_Transmit(
                    &huart1,
                    (u8 *)uart,
                    strlen(uart),
                    300
                );

                resetControlloPlateau();
                resetVerificaRiposoPlateau();
                resetControlloStalloCarica();
            }
        }
        else
        {
            /*
             * Rete, ADC, temperatura, sessione o fault non consentono
             * di proseguire una verifica attendibile.
             */
            resetControlloPlateau();
            resetVerificaRiposoPlateau();
        }
    }
    else if(BAT_PLATEAU_TERMINATION_ENABLED != 0U &&
            BAT_PULSED_CHARGE_ENABLED == 0U &&
            alimentatore != 0 &&
            adcValido != 0 &&
            temperaturaValida != 0 &&
            faultCaricaLatched == BAT_FAULT_NONE &&
            sessioneCaricaAttiva != 0U &&
            comandoCarica != 0U &&
            tensioneMisurata_mV >= BAT_PLATEAU_MIN_MV &&
            tensioneMisurata_mV < VBAT_CHARGE_STOP_MV)
    {
        if(aggiornaControlloPlateau(
               tensioneMisurata_mV,
               &incrementoPlateau_mV,
               &escursionePlateau_mV
           ) != 0U)
        {
            if(contatoreConfermaPlateau <
               BAT_PLATEAU_CONFIRM_COUNT)
            {
                contatoreConfermaPlateau++;
            }

            if(contatoreConfermaPlateau >=
               BAT_PLATEAU_CONFIRM_COUNT)
            {
                verificaRiposoPlateauAttiva = 1U;
                tickInizioRiposoPlateau = tickAttuale;
                tensioneInizioRiposoPlateau_mV =
                    tensioneMisurata_mV;

                statoCaricaBatteria = BAT_STATE_PLATEAU_REST;
                comandoCarica = 0U;
                dutyPercentuale = 0U;
                contatoreArresto = 0U;
                contatoreDutyCycle = 0U;

                snprintf(
                    uart,
                    sizeof(uart),
                    "[BAT] Plateau candidate: V=%lu.%03luV "
                    "Rise=%ldmV Span=%lumV Samples=%u "
                    "Confirm=%u; rest started for %lus\r\n",
                    (unsigned long)(tensioneMisurata_mV / 1000U),
                    (unsigned long)(tensioneMisurata_mV % 1000U),
                    (long)incrementoPlateau_mV,
                    (unsigned long)escursionePlateau_mV,
                    (unsigned int)BAT_PLATEAU_SAMPLES,
                    (unsigned int)BAT_PLATEAU_CONFIRM_COUNT,
                    (unsigned long)
                        (BAT_PLATEAU_REST_TIME_MS / 1000UL)
                );

                HAL_UART_Transmit(
                    &huart1,
                    (u8 *)uart,
                    strlen(uart),
                    300
                );

                resetControlloPlateau();
                resetControlloStalloCarica();
            }
        }
        else
        {
            contatoreConfermaPlateau = 0U;
        }
    }
    else
    {
        resetControlloPlateau();
    }

    /* =============================================================
     * 8. TIMER DI SICUREZZA
     * ============================================================= */
    if(comandoCarica != 0 &&
       statoCaricaBatteria != BAT_STATE_WAKEUP)
    {
        tempoTotaleCaricaOn_s += tempoTrascorso_s;

        if(statoCaricaBatteria == BAT_STATE_TOP_50 ||
           statoCaricaBatteria == BAT_STATE_TOP_25)
        {
            tempoTopCaricaOn_s += tempoTrascorso_s;
        }
    }

    if(faultCaricaLatched == BAT_FAULT_NONE &&
       tempoTotaleCaricaOn_s >= BAT_MAX_TOTAL_ON_TIME_S)
    {
        faultCaricaLatched = BAT_FAULT_TOTAL_TIMEOUT;
        faultAttuale = BAT_FAULT_TOTAL_TIMEOUT;
        statoCaricaBatteria = BAT_STATE_FAULT;
        sessioneCaricaAttiva = 0;
        comandoCarica = 0;
        dutyPercentuale = 0;
        tensioneArrestoTimeout_mV = tensioneMisurata_mV;
        contatoreRiarmoTimeout = 0U;
    }

    if(faultCaricaLatched == BAT_FAULT_NONE &&
       tempoTopCaricaOn_s >= BAT_MAX_TOP_ON_TIME_S)
    {
        faultCaricaLatched = BAT_FAULT_TOP_TIMEOUT;
        faultAttuale = BAT_FAULT_TOP_TIMEOUT;
        statoCaricaBatteria = BAT_STATE_FAULT;
        sessioneCaricaAttiva = 0;
        comandoCarica = 0;
        dutyPercentuale = 0;
        tensioneArrestoTimeout_mV = tensioneMisurata_mV;
        contatoreRiarmoTimeout = 0U;
    }

    /* =============================================================
     * 9. DECISIONE UNICA SU PE15
     * =============================================================
     * PE15 viene aperto:
     * - dopo una sottotensione confermata durante il funzionamento
     *   senza alimentatore;
     * - per temperatura MCU eccessivamente alta o bassa;
     * - alla scadenza del tempo massimo di funzionamento a batteria
     *   (quest'ultimo caso viene gestito prima della misura).
     *
     * Il latch di sottotensione mantiene la batteria isolata anche quando,
     * dopo l'apertura, l'ADC non puo piu leggerla; viene cancellato al
     * ritorno della rete. I latch termici vengono invece cancellati solo
     * quando la temperatura rientra nelle rispettive soglie di isteresi.
     */
    collegaBatteria = 1;

    if(adcValido != 0 &&
       alimentatore == 0 &&
       tensioneMisurata_mV < VBAT_DISCONNECT_MV)
    {
        disconnessioneSottotensioneAttiva = 1U;
    }

    if(alimentatore == 0 &&
       disconnessioneSottotensioneAttiva != 0U)
    {
        collegaBatteria = 0;
    }

    if(disconnessioneTermicaAttiva != 0U)
    {
        collegaBatteria = 0;
    }

    if(disconnessioneTemperaturaBassaAttiva != 0U)
    {
        collegaBatteria = 0;
    }

    /* =============================================================
     * 10. COMANDO FINALE DEI GPIO
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
     * 11. DIAGNOSTICA SERIALE
     * ============================================================= */
    if(logBatteriaInizializzato == 0U ||
       (tickAttuale - ultimoTickLogBatteria) >= BAT_LOG_INTERVAL_MS)
    {
        ultimoTickLogBatteria = tickAttuale;
        logBatteriaInizializzato = 1U;

        if(adcValido != 0)
        {
            snprintf(
                uart,
                sizeof(uart),
                "[BAT] V=%lu.%03luV Level=%u%% AC=%u T=%ldC "
                "State=%s Fault=%s Charge=%u Connected=%u\r\n",
                (unsigned long)(tensioneVisualizzata_mV / 1000U),
                (unsigned long)(tensioneVisualizzata_mV % 1000U),
                (unsigned int)batteryLevel,
                (unsigned int)alimentatore,
                temperaturaMCU,
                nomeStatoBatteria(statoCaricaBatteria),
                nomeFaultBatteria(faultAttuale),
                (unsigned int)comandoCarica,
                (unsigned int)collegaBatteria
            );
        }
        else
        {
            snprintf(
                uart,
                sizeof(uart),
                "[BAT] V=INVALID AC=%u T=%ldC State=%s Fault=%s "
                "Charge=%u Connected=%u\r\n",
                (unsigned int)alimentatore,
                temperaturaMCU,
                nomeStatoBatteria(statoCaricaBatteria),
                nomeFaultBatteria(faultAttuale),
                (unsigned int)comandoCarica,
                (unsigned int)collegaBatteria
            );
        }

        HAL_UART_Transmit(&huart1, (u8 *)uart, strlen(uart), 300);
    }

    /* =============================================================
     * 12. GESTIONE DEGLI ALLARMI BATTERIA
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

    return 1U;
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






