#!/usr/bin/env python3
"""
Teensy USB Serial Monitor
-------------------------
A small, dependency-light serial monitor for Teensy (or any USB CDC ACM device).

Features
- Auto-detects Teensy port on Linux (matching 'Teensy' in port description or VID 0x16C0)
- Fallback to first /dev/ttyACM* if no explicit match
- Optional --port to override; --list to list ports
- Prints incoming data to stdout; supports logging to a file
- Optional interactive input to send lines to Teensy (on by default)

Usage examples
  python3 teensy_serial.py                # auto-detect and monitor
  python3 teensy_serial.py --list         # list ports and exit
  python3 teensy_serial.py --port /dev/ttyACM0 --log logs/session.txt

Note: Requires pyserial. Install via:
  pip install pyserial
"""
import argparse
import sys
import os
import time
import threading
import queue

try:
    import serial
    from serial.tools import list_ports
except Exception as exc:
    print("[ERR] pyserial not installed. Install with: pip install pyserial", file=sys.stderr)
    print(f"[ERR] import error: {exc}", file=sys.stderr)
    sys.exit(1)

TEENSY_VID = 0x16C0  # PJRC vendor id
# Teensy products have multiple PIDs depending on USB type. We'll just check VID and description.


def list_all_ports() -> None:
    ports = list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        desc = p.description or ""
        manu = getattr(p, 'manufacturer', '') or ""
        hwid = p.hwid
        print(f"- {p.device} :: {desc} :: {manu} :: {hwid}")


def find_teensy_port(preferred: str | None = None) -> str | None:
    if preferred:
        return preferred
    ports = list_ports.comports()
    # 1) Prefer ports with 'Teensy' in description/manufacturer
    # for p in ports:
    #     desc = (p.description or '').lower()
    #     manu = (getattr(p, 'manufacturer', '') or '').lower()
    #     if 'teensy' in desc or 'teensy' in manu:
    #         return p.device
    # # 2) Prefer PJRC VID matches
    # for p in ports:
    #     try:
    #         if p.vid == TEENSY_VID:
    #             return p.device
    #     except Exception:
    #         pass
    # 3) Fallback to first ttyACM*
    for p in ports:
        if p.device.startswith('/dev/ttyACM'):
            return p.device
    # 4) Last resort: first available
    return None #ports[0].device if ports else None


def open_serial(port: str, baud: int, timeout: float) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.write_timeout = 0.5
    ser.dsrdtr = False
    ser.rtscts = False
    ser.xonxoff = False
    ser.open()
    return ser


def reader_thread(ser: serial.Serial, out_q: queue.Queue, stop_evt: threading.Event, raw: bool) -> None:
    """Read bytes from serial and push decoded lines/bytes to a queue."""
    buf = bytearray()
    while not stop_evt.is_set():
        try:
            data = ser.read(1024)
            if not data:
                continue
            if raw:
                out_q.put(data)
                continue
            buf.extend(data)
            while b'\n' in buf:
                line, _, rest = buf.partition(b'\n')
                buf = bytearray(rest)
                # Trim CR and decode
                if line.endswith(b'\r'):
                    line = line[:-1]
                text = line.decode('utf-8', errors='replace')
                out_q.put(text)
        except serial.SerialException as e:
            out_q.put(f"[SERIAL] error: {e}")
            break
        except Exception as e:
            out_q.put(f"[SERIAL] unexpected error: {e}")
            break


def stdin_thread(in_q: queue.Queue, stop_evt: threading.Event) -> None:
    """Read lines from stdin and push to a queue for sending."""
    try:
        for line in sys.stdin:
            if stop_evt.is_set():
                break
            in_q.put(line.rstrip('\r\n'))
    except Exception:
        pass


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="Teensy USB Serial Monitor")
    ap.add_argument('--port', '-p', help="Serial port (e.g., /dev/ttyACM0). If omitted, auto-detect.")
    ap.add_argument('--baud', '-b', type=int, default=115200, help="Baud rate (ignored for USB CDC, but required by pyserial)")
    ap.add_argument('--timeout', type=float, default=0.1, help="Read timeout in seconds")
    ap.add_argument('--list', action='store_true', help="List serial ports and exit")
    ap.add_argument('--log', help="Log incoming data to a file")
    ap.add_argument('--no-input', action='store_true', help="Disable interactive input -> Teensy")
    ap.add_argument('--raw', action='store_true', help="Raw mode: dump bytes as they arrive (no line decoding)")
    ap.add_argument('--crlf', action='store_true', help="When sending, use CRLF instead of LF")
    args = ap.parse_args(argv)

    if args.list:
        list_all_ports()
        return 0

    for c in range(99999):
        port = find_teensy_port(args.port)
        if port:
            break
        else:
            time.sleep(0.1)
    if not port:    
        print("[ERR] Could not find a serial port. Use --list to see candidates.", file=sys.stderr)
        return 2

    print(f"[INFO] Opening {port} @ {args.baud} ...")
    try:
        ser = open_serial(port, args.baud, args.timeout)
    except Exception as e:
        print(f"[ERR] Failed to open {port}: {e}", file=sys.stderr)
        return 3

    log_fp = None
    if args.log:
        os.makedirs(os.path.dirname(args.log) or '.', exist_ok=True)
        log_fp = open(args.log, 'ab', buffering=0)
        print(f"[INFO] Logging to {args.log}")

    out_q: queue.Queue = queue.Queue()
    in_q: queue.Queue = queue.Queue()
    stop_evt = threading.Event()

    rt = threading.Thread(target=reader_thread, args=(ser, out_q, stop_evt, args.raw), daemon=True)
    rt.start()

    st = None
    if not args.no_input:
        st = threading.Thread(target=stdin_thread, args=(in_q, stop_evt), daemon=True)
        st.start()
        print("[INFO] Type commands and press Enter to send. Ctrl+C to exit.")

    try:
        while True:
            # Print data from serial
            try:
                item = out_q.get(timeout=0.05)
            except queue.Empty:
                item = None
            if item is not None:
                if isinstance(item, bytes):
                    sys.stdout.buffer.write(item)
                    sys.stdout.flush()
                    if log_fp:
                        log_fp.write(item)
                else:
                    print(item)
                    if log_fp:
                        log_fp.write((item + '\n').encode('utf-8', errors='replace'))
            # Send user input to serial
            if st is not None:
                try:
                    line = in_q.get_nowait()
                except queue.Empty:
                    line = None
                if line is not None:
                    tx = (line + ('\r\n' if args.crlf else '\n')).encode('utf-8', errors='replace')
                    try:
                        ser.write(tx)
                    except Exception as e:
                        print(f"[ERR] write failed: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
    finally:
        stop_evt.set()
        try:
            ser.close()
        except Exception:
            pass
        if log_fp:
            try:
                log_fp.flush()
                log_fp.close()
            except Exception:
                pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
