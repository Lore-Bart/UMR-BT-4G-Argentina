LITE V7 - Bluetooth firmware update robustness check (v11)

File modificati:
- Sub_UART.c
- mymain.c
- Sub_UPDATE.c
- Sub_Comandi.c
- Sub_RTC.c

Protocollo app Android NON modificato:
- comando 0x13 invariato: avvio update e numero pacchetti totali
- pacchetti firmware invariati: 2 byte numero pacchetto + 72 byte payload
- risposta OK\r\n mantenuta dopo pacchetto ricevuto/programmazione corretta

Correzioni e miglioramenti:
1) Pacchetti firmware BT trattati come binari
   - durante update non viene piu' usato strlen() sul pacchetto binario
   - il pacchetto viene copiato subito in un buffer dedicato da 74 byte
   - il main programma il buffer dedicato, evitando sovrascritture da successivi interrupt UART

2) Validazione minima del pacchetto
   - in update il firmware accumula eventuali frammenti fino a 74 byte
   - i dati vengono programmati solo quando il pacchetto completo e' disponibile

3) Programmazione flash verificata
   - ogni word scritta viene riletta
   - OK viene mandato solo se il pacchetto e' stato scritto/verificato
   - in caso di errore vengono fatti retry e poi inviato ER\r\n + emergenza

4) Timeout update piu' tollerante
   - updateAttivo portato a 15 tick per evitare abort troppo aggressivi tra pacchetti

5) Correzione dichiarazioni buffer
   - messaggioRecBT dichiarato coerentemente a 500 byte anche in mymain.c e Sub_RTC.c

Log utili da verificare:
- BT update start, packets: N
- BT rx bytes: 74
- BT update target slot base: 0x08008200 oppure 0x08040200
- BT update first image half programmed and verified / second image half programmed and verified
- BT update completed, reboot requested

Note:
- Non e' stato cambiato il protocollo Android.
- Se compare BT update partial packet, significa che il modulo ha consegnato il pacchetto in piu' frammenti; la programmazione parte solo al raggiungimento di 74 byte.
- Se compare BT update flash verify failed, la flash non ha accettato la scrittura o l'area target non era correttamente cancellata.
