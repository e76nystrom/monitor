#!/bin/bash

usbasp=0

if [ $usbasp -eq 1 ]
then
 /cygdrive/c/DevSoftware/avrdude-v6.3.1.1-windows/avrdude \
  -u \
  -v \
  -C "C:\DevSoftware\avrdude-v6.3.1.1-windows\avrdude.conf" \
  -p m328p \
  -c usbasp-clone \
  -U flash:w:"c:\Development\Arduino\monitor\.pio\build\pro16MHzatmega328\firmware.hex":a
fi

if [ $usbasp -eq 0 ]
then
 COMM=COM9

 /cygdrive/c/DevSoftware/avrdude-v6.3.1.1-windows/avrdude \
  -v \
  -C "C:\DevSoftware\avrdude-v6.3.1.1-windows\avrdude.conf" \
  -p atmega2560 \
  -c wiring \
  -b 115200 \
  -P $COMM \
  -U flash:w:.pio/build/mega2560/firmware.hex:i

 "putty" -load ${COMM}-19200
fi
