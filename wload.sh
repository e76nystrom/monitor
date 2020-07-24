#!/bin/bash

AVRDUDE="/cygdrive/c/DevSoftware/avrdude-6.3-mingw32/avrdude.exe"
CONF="C:/DevSoftware/avrdude-6.3-mingw32/avrdude.conf"
TARGET="c:/Development/Arduino/monitor/.pio/build/pro16MHzatmega328/firmware.hex"

$AVRDUDE \
-v \
-p atmega328p \
-c usbasp \
-P usb \
-C $CONF \
-U flash:w:$TARGET:i

#/cygdrive/c/DevSoftware/Putty/putty.exe -load $port
