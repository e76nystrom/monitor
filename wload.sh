#!/bin/bash

~/.platformio/packages/tool-avrdude/avrdude \
-v \
-p atmega328p \
-c usbasp \
-P usb \
-C "C:\cygwin\home\Eric\.platformio\packages\tool-avrdude\avrdude.conf" \
-U flash:w:.pioenvs/pro16MHzatmega328/firmware.hex:i

#/cygdrive/c/DevSoftware/Putty/putty.exe -load $port
