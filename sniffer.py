#!/usr/bin/env python3
"""
3‑wire (S/+/-) UART Sniffer
-----------------------------------
Receive‑only sniffer for the single‑wire "S" line (TTL level) using a USB‑TTL adapter.

Quick wiring (receive‑only):
  • Adapter GND  →  "-" (GND)
  • Adapter RX   →  "S" (through ~1–2 kΩ series resistor recommended)
  • Do NOT connect adapter VCC to "+"

Common bauds to try: 9600, 19200, 38400, 57600, 115200 (8N1).
If you see nothing or only garbage at all speeds, the line might be "inverted TTL";
use a small transistor inverter or an FTDI adapter configured for RXD inversion.

Usage examples:
  python sniffer.py --list
  python sniffer.py --port /dev/ttyUSB0 --baud 19200
  python sniffer.py --port COM5 --scan
  python sniffer.py --port /dev/ttyUSB0 --baud 38400 --log raw.bin --text-log frames.txt

  #### USAGE ####
  python sniffersniffer.py --list
  python sniffersniffer.py --port /dev/ttyUSB0 --scan
  
  python sniffer.py --port /dev/ttyUSB0 --baud 19200 --gap-ms 10 --min-frame 6 --text-log frames.txt --log raw.bin
    # Start
        --gap-ms 10 --min-frame 6 
    # If it’s messy:
        --gap-ms 12 --min-frame 8
    # If frames look glued together:
        --gap-ms 5 --min-frame 4



Hotkeys while running:
  CTRL+C  → quit cleanly

Author: ChatGPT
License: MIT
"""

import argparse
import sys
import time
from datetime import datetime
from typing import Optional, Iterable, Tuple

try:
    import serial
    from serial.tools import list_ports
except Exception as e:
    print("This script requires the 'pyserial' package.\nInstall with: pip install pyserial")
    raise

BAUD_CANDIDATES = [9600, 19200, 38400, 57600, 115200]

def list_serial_ports() -> None:
    print("Available serial ports:")
    for p in list_ports.comports():
        print(f"  - {p.device}  ({p.description})")

def hexdump_line(b: bytes) -> Tuple[str, str]:
    # Hex bytes
    hx = ' '.join(f"{x:02X}" for x in b)
    # ASCII with dots for non-printables
    def to_chr(x: int) -> str:
        return chr(x) if 32 <= x <= 126 else '.'
    ascii_repr = ''.join(to_chr(x) for x in b)
    return hx, ascii_repr

def open_port(port: str, baud: int, bytesize: int=8, parity: str='N', stopbits: float=1.0, timeout: float=0.1) -> serial.Serial:
    bs_map = {5: serial.FIVEBITS, 6: serial.SIXBITS, 7: serial.SEVENBITS, 8: serial.EIGHTBITS}
    pb_map = {'N': serial.PARITY_NONE, 'E': serial.PARITY_EVEN, 'O': serial.PARITY_ODD, 'M': serial.PARITY_MARK, 'S': serial.PARITY_SPACE}
    sb_map = {1: serial.STOPBITS_ONE, 1.5: serial.STOPBITS_ONE_POINT_FIVE, 2: serial.STOPBITS_TWO}
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=bs_map.get(bytesize, serial.EIGHTBITS),
        parity=pb_map.get(parity.upper(), serial.PARITY_NONE),
        stopbits=sb_map.get(stopbits, serial.STOPBITS_ONE),
        timeout=timeout
    )
    return ser

def scan_bauds(port: str, dwell: float=2.0, gap_ms: int=10) -> None:
    print(f"Scanning bauds on {port} (dwell {dwell:.1f}s each)...")
    for b in BAUD_CANDIDATES:
        try:
            with open_port(port, b) as ser:
                start = time.time()
                total = 0
                last = time.time()
                while time.time() - start < dwell:
                    chunk = ser.read(4096)
                    if chunk:
                        total += len(chunk)
                        last = time.time()
                # heuristic: show bytes per second
                rate = total / dwell
                print(f"  {b:6d} baud → {total} bytes ({rate:.1f} B/s)")
        except Exception as e:
            print(f"  {b:6d} baud → error: {e}")

def run_sniffer(port: str, baud: int, gap_ms: int, raw_path: Optional[str], text_log_path: Optional[str],
                bytesize: int=8, parity: str='N', stopbits: float=1.0) -> None:
    gap_s = gap_ms / 1000.0
    raw_file = open(raw_path, "ab") if raw_path else None
    text_log = open(text_log_path, "a", encoding="utf-8") if text_log_path else None

    try:
        with open_port(port, baud, bytesize=bytesize, parity=parity, stopbits=stopbits, timeout=0.05) as ser:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Listening on {port} @ {baud} baud (8{parity}1), frame gap ≥ {gap_ms} ms")
            print("Press CTRL+C to stop.\n")
            buf = bytearray()
            last_rx = None

            while True:
                data = ser.read(4096)
                now = time.time()

                if data:
                    # Frame split by idle gap
                    if last_rx is not None and (now - last_rx) >= gap_s and buf:
                        # Flush previous frame
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        hx, ascii_repr = hexdump_line(bytes(buf))
                        line = f"[{ts}] len={len(buf):4d} | {hx}\n           | {ascii_repr}\n"
                        print(line, end="")
                        if text_log:
                            text_log.write(line)
                            text_log.flush()
                        if raw_file:
                            raw_file.write(bytes(buf))
                            raw_file.flush()
                        buf.clear()

                    buf.extend(data)
                    last_rx = now
                else:
                    # If no data and we have a buffered frame lingering past gap, flush it
                    if last_rx is not None and buf and (time.time() - last_rx) >= gap_s:
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        hx, ascii_repr = hexdump_line(bytes(buf))
                        line = f"[{ts}] len={len(buf):4d} | {hx}\n           | {ascii_repr}\n"
                        print(line, end="")
                        if text_log:
                            text_log.write(line)
                            text_log.flush()
                        if raw_file:
                            raw_file.write(bytes(buf))
                            raw_file.flush()
                        buf.clear()
                        last_rx = None

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if raw_file:
            raw_file.close()
        if text_log:
            text_log.close()

def main():
    p = argparse.ArgumentParser(description="Simple UART sniffer for single-wire signal")
    p.add_argument("--list", action="store_true", help="List available serial ports and exit")
    p.add_argument("--port", help="Serial port (e.g., COM5 or /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=None, help="Baud rate (e.g., 19200). If omitted, --scan is required.")
    p.add_argument("--scan", action="store_true", help="Scan common bauds to see which yields traffic")
    p.add_argument("--gap-ms", type=int, default=10, help="Inter-byte gap (ms) to split frames (default: 10 ms)")
    p.add_argument("--log", dest="raw_path", help="Append raw bytes to this file (binary)")
    p.add_argument("--text-log", dest="text_log_path", help="Append hex/ascii frames to this text file")
    p.add_argument("--bytesize", type=int, default=8, choices=[5,6,7,8], help="Data bits (default 8)")
    p.add_argument("--parity", type=str, default='N', choices=list('NEOMS'), help="Parity (N/E/O/M/S), default N")
    p.add_argument("--stopbits", type=float, default=1.0, choices=[1.0,1.5,2.0], help="Stop bits (default 1)")
    args = p.parse_args()

    if args.list:
        list_serial_ports()
        return

    if not args.port:
        print("Error: --port is required (e.g., --port /dev/ttyUSB0). Use --list to see ports.")
        sys.exit(2)

    if args.scan:
        scan_bauds(args.port)
        if args.baud is None:
            return  # scan only

    if args.baud is None:
        print("Error: provide --baud or use --scan to discover one.")
        sys.exit(2)

    run_sniffer(
        port=args.port,
        baud=args.baud,
        gap_ms=args.gap_ms,
        raw_path=args.raw_path,
        text_log_path=args.text_log_path,
        bytesize=args.bytesize,
        parity=args.parity,
        stopbits=args.stopbits
    )

if __name__ == "__main__":
    main()
