@echo off
setlocal

REM Metti questo .bat nella stessa cartella di:
REM - make_umrbt_lite_update_bin.py
REM - Firmware_APP1.bin
REM - Firmware_APP2.bin

python "%~dp0make_umrbt_lite_update_bin.py" "%~dp0app_08008200.bin" "%~dp0app_08040200.bin" "%~dp0UMRBT_LITE_BT_UPDATE.bin"

if errorlevel 1 (
  echo.
  echo ERRORE nella creazione del file update.
  pause
  exit /b 1
)

echo.
echo File pronto: UMRBT_LITE_BT_UPDATE.bin
echo Carica questo file nell'app Android.
pause
