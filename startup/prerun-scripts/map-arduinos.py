#!/usr/bin/env python3

import sys
from serial.tools.list_ports import comports

MAPPINGS = {
    'sensors': '7583435383035151F192',
    'motors': '95038303531351C05022'
}

def parse_serial_number(hwid: str) -> str:
    return hwid.split(' ')[2].split('=')[1]

def find_port(FTDI_SN: str) -> str:
    ports = comports()
    for port in ports:
        sn = parse_serial_number(port.hwid)
        if sn == FTDI_SN:
            return port.device
    raise Exception(f'Port not found for {FTDI_SN}')

def print_usage() -> None:
    print('USAGE: python3 map-arduinos.py [sensors | motors]')
    sys.exit()

def main(mapping):
    if mapping not in MAPPINGS.keys():
        print_usage()
    port = find_port(MAPPINGS[mapping])
    print(port)
    return

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print_usage()
    main(sys.argv[1])
