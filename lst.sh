#!/bin/bash

avr-objdump.exe -S -D .pio/build/mega2560/firmware.elf >firmware.lst
avr-nm -n .pio/build/mega2560/firmware.elf >symbols.txt
