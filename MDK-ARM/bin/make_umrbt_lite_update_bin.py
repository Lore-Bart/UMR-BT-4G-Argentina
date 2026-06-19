#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Crea il file .bin unico per aggiornamento firmware Bluetooth UMR-BT LITE.

Protocollo firmware attuale:
- l'app Android invia pacchetti da 72 byte
- il file unico è composto da due immagini:
    1) firmware linkato a 0x08008200
    2) firmware linkato a 0x08040200
- il firmware divide il file in due metà usando paccTot/2

Per questo motivo le due metà DEVONO avere lo stesso numero di pacchetti da 72 byte.
Questo script:
- riempie ogni .bin con 0xFF fino a multiplo di 72
- riempie il .bin più corto con 0xFF fino alla stessa dimensione dell'altro
- verifica che la metà non superi lo spazio sicuro dello slot APP1
- unisce i due file in un unico .bin da caricare nell'app Android
"""

import argparse
from pathlib import Path

PACKET_SIZE = 72
FILL_BYTE = 0xFF

APP1_BASE = 0x08008200
APP1_MARKER_NEXT = 0x08040000
APP1_SAFE_BYTES = APP1_MARKER_NEXT - APP1_BASE
APP1_SAFE_PACKETS = APP1_SAFE_BYTES // PACKET_SIZE
APP_SAFE_PADDED_BYTES = APP1_SAFE_PACKETS * PACKET_SIZE


def pad_to_multiple(data: bytes, multiple: int, fill: int = FILL_BYTE) -> bytes:
    rem = len(data) % multiple
    if rem == 0:
        return data
    return data + bytes([fill]) * (multiple - rem)


def pad_to_size(data: bytes, size: int, fill: int = FILL_BYTE) -> bytes:
    if len(data) > size:
        raise ValueError(f"internal error: data length {len(data)} > target size {size}")
    return data + bytes([fill]) * (size - len(data))


def main() -> int:
    parser = argparse.ArgumentParser(description="Crea file .bin unico per update BT UMR-BT LITE")
    parser.add_argument("app1_bin", nargs="?", default="app_08008200.bin", help="Bin linkato a 0x08008200")
    parser.add_argument("app3_bin", nargs="?", default="app_08040200.bin", help="Bin linkato a 0x08040200")
    parser.add_argument("output_bin", nargs="?", default="UMRBT_LITE_BT_UPDATE.bin", help="Bin unico da caricare nell'app Android")
    args = parser.parse_args()

    app1_path = Path(args.app1_bin)
    app3_path = Path(args.app3_bin)
    out_path = Path(args.output_bin)

    if not app1_path.exists():
        raise FileNotFoundError(f"Non trovo {app1_path}")
    if not app3_path.exists():
        raise FileNotFoundError(f"Non trovo {app3_path}")

    app1_raw = app1_path.read_bytes()
    app3_raw = app3_path.read_bytes()

    app1_72 = pad_to_multiple(app1_raw, PACKET_SIZE)
    app3_72 = pad_to_multiple(app3_raw, PACKET_SIZE)

    half_size = max(len(app1_72), len(app3_72))

    if half_size > APP_SAFE_PADDED_BYTES:
        raise RuntimeError(
            "ERRORE: immagine troppo grande per il protocollo attuale.\n"
            f"Dimensione metà richiesta dopo padding: {half_size} byte\n"
            f"Massimo sicuro per APP1: {APP_SAFE_PADDED_BYTES} byte "
            f"({APP1_SAFE_PACKETS} pacchetti da {PACKET_SIZE} byte)\n"
            "Riduci il firmware oppure rivedi la mappa flash/protocollo update."
        )

    app1_final = pad_to_size(app1_72, half_size)
    app3_final = pad_to_size(app3_72, half_size)
    merged = app1_final + app3_final

    out_path.write_bytes(merged)

    info = []
    info.append("UMR-BT LITE Bluetooth update image")
    info.append("")
    info.append(f"Input APP1 0x08008200: {app1_path}")
    info.append(f"Input APP3 0x08040200: {app3_path}")
    info.append(f"Output unico app Android: {out_path}")
    info.append("")
    info.append(f"APP1 raw bytes:        {len(app1_raw)}")
    info.append(f"APP1 padded 72 bytes:  {len(app1_72)}")
    info.append(f"APP3 raw bytes:        {len(app3_raw)}")
    info.append(f"APP3 padded 72 bytes:  {len(app3_72)}")
    info.append(f"Half size finale:      {half_size}")
    info.append(f"Pacchetti per metà:    {half_size // PACKET_SIZE}")
    info.append(f"Pacchetti totali:      {len(merged) // PACKET_SIZE}")
    info.append(f"Output bytes:          {len(merged)}")
    info.append("")
    info.append("Controlli:")
    info.append(f"- half_size multiplo di 72: {half_size % PACKET_SIZE == 0}")
    info.append(f"- output multiplo di 144:   {len(merged) % (2 * PACKET_SIZE) == 0}")
    info.append(f"- limite APP1 sicuro:       {APP_SAFE_PADDED_BYTES} byte")
    info.append("")
    info.append("Nota:")
    info.append("Il firmware divide il file a paccTot/2; quindi le due metà devono avere lo stesso numero di pacchetti.")
    info.append("Lo spazio mancante viene riempito con 0xFF.")

    info_path = out_path.with_suffix(".txt")
    info_path.write_text("\n".join(info), encoding="utf-8")

    print("Creato:", out_path)
    print("Creato:", info_path)
    print(f"Pacchetti per metà: {half_size // PACKET_SIZE}")
    print(f"Pacchetti totali:   {len(merged) // PACKET_SIZE}")
    print(f"Dimensione output:  {len(merged)} byte")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
