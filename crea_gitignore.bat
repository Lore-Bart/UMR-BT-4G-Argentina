@echo off

(
echo # Cartelle generate da Keil
echo Objects/
echo Listings/
echo.
echo # File compilati/generati
echo *.axf
echo *.hex
echo *.bin
echo *.elf
echo *.map
echo *.lst
echo *.o
echo *.d
echo *.crf
echo *.htm
echo *.lnp
echo.
echo # File utente/sessione Keil
echo *.uvguix.*
echo *.uvgui.*
echo *.uvoptx
echo.
echo # File temporanei
echo *.bak
echo *.orig
echo *.tmp
echo ~*
echo.
echo # Windows
echo Thumbs.db
echo desktop.ini
) > .gitignore

echo File .gitignore creato.
pause