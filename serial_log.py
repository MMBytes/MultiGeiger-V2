#!/usr/bin/env python3
# Simple serial logger: reads COM3 @ 115200 and writes to logfile + stdout.
import sys, time
import serial

if len(sys.argv) < 3:
    print("usage: serial_log.py <port> <outfile>", file=sys.stderr)
    sys.exit(2)

port = sys.argv[1]
out  = sys.argv[2]

ser = serial.Serial(port, 115200, timeout=1)
with open(out, "wb", buffering=0) as f:
    while True:
        data = ser.read(4096)
        if data:
            f.write(data)
            sys.stdout.buffer.write(data)
            sys.stdout.flush()
