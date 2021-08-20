#!/bin/bash

if [ 1 -eq 1 ]
then
 /cygdrive/c/Users/Eric/.platformio/packages/tool-avrdude/avrdude \
  -v \
  -p atmega328p \
  -c usbasp \
  -P usb \
  -C "C:\Users\Eric\.platformio\packages\tool-avrdude\avrdude.conf" \
  -U flash:w:.pio/build/pro16MHzatmega328/firmware.hex:i
fi

if [ 1 -eq 0 ]
then
 COMM=COM6

 /cygdrive/c/Users/Eric/.platformio/packages/tool-avrdude/avrdude \
  -D -v \
  -p atmega2560 \
  -C "c:\Users\Eric\.platformio\packages\tool-avrdude\avrdude.conf" \
  -c wiring \
  -b 115200 \
  -P $COMM \
  -U flash:w:.pio/build/megaatmega2560/firmware.hex:i

 /cygdrive/c/DevSoftware/Putty/putty.exe -load ${COMM}-19200
fi
