cd bin_files
copy ..\Firmware V3\Firmware V3.hex .
srec_cat.exe DemoBoard_debug.hex -intel -crop 0x8002008 0x801FFFF -offset -0x8002008 -o DemoBoard_debug_v1.bin -binary
MaMbinGEN\MaMbinGen.exe -i DemoBoard_debug_v1.bin -o DemoBoard_debug_v1_dwld.bin

