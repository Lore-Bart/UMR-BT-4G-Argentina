LITE V8 - DB URL / retry fix v14

Files to replace:
- Sub_MySQL.c
- Sub_GSM.c
- mymain.c
- Inc/prototipi.h

Changes:
1) Fixed DB URL transmission for undervoltage/overvoltage and all other DB saves.
   The URL AT command is now finalized with exactly: closing quote + CR + NUL, and it is transmitted with strlen() only.
   This avoids dirty trailing bytes such as the extra character seen after V3.

2) HTTPACTION is no longer sent immediately after AT+HTTPPARA.
   It is sent only after the modem replies OK to AT+HTTPPARA.
   If HTTPPARA replies ERROR, no stale HTTPACTION is launched.

3) Added maximum DB retry count.
   After DB_MAX_RETRY attempts the single pending DB request is discarded and the firmware logs:
   DB save aborted: <type> after <n> retries (...)
   This avoids infinite retry loops.

4) DB scheduler/SMS coexistence preserved.
   DB transactions do not start while SMS send/read is pending.

Expected logs for a good undervoltage save:
DB send started: undervoltage
DB HTTPPARA OK, HTTPACTION start
DB save OK: undervoltage HTTP 200

Expected logs for repeated failure:
DB save modem error: undervoltage retry 1/5
...
DB save aborted: undervoltage after 5 retries (HTTPPARA/timeout)
