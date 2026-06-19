UMR-BT LITE - Creazione file .bin unico per update Bluetooth
==============================================================

Il firmware BT update attuale usa pacchetti da 72 byte e divide il file ricevuto in due metà con paccTot/2.
Quindi il file unico da caricare nell'app Android deve essere:

  [APP linkata a 0x08008200, padded] + [APP linkata a 0x08040200, padded]

Le due metà devono avere la stessa dimensione e la dimensione di ogni metà deve essere multipla di 72 byte.
Il padding viene fatto con 0xFF.

1) In Keil/uVision crea due target
----------------------------------
Duplica il target esistente e crea:

Target APP_08008200:
  IROM start: 0x08008200
  IROM size:  0x00037E00

Target APP_08040200:
  IROM start: 0x08040200
  IROM size:  0x0003FE00

Nota: il file finale resta limitato dalla dimensione sicura dello slot APP1.
Lo script blocca la generazione se la metà supera 228816 byte, cioè 3178 pacchetti da 72 byte.

2) Genera il bin dal file AXF con fromelf
----------------------------------------
Nel post-build di APP_08008200 usa, adattando i percorsi:

  "C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe" --bin --output=".\bin_update\app_08008200.bin" ".\Firmware V3\Firmware V3.axf"

Nel post-build di APP_08040200 usa:

  "C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe" --bin --output=".\bin_update\app_08040200.bin" ".\Firmware V3\Firmware V3.axf"

Se fromelf è già nel PATH puoi scrivere semplicemente:

  fromelf --bin --output=".\bin_update\app_08008200.bin" ".\Firmware V3\Firmware V3.axf"

3) Unisci e allinea i due bin
-----------------------------
Copia nella cartella bin_update:

  make_umrbt_lite_update_bin.py
  merge_umrbt_lite_update.bat

Poi esegui:

  merge_umrbt_lite_update.bat

Output:

  UMRBT_LITE_BT_UPDATE.bin

Questo è il file da caricare nell'app Android.

4) Controllo fondamentale
-------------------------
Lo script crea anche:

  UMRBT_LITE_BT_UPDATE.txt

Controlla che indichi:

  half_size multiplo di 72: True
  output multiplo di 144:   True

