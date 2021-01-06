#!/bin/bash

/cygdrive/c/Users/Eric/.platformio/packages/tool-openocd/bin/openocd -d2 -s C:\Users\Eric\.platformio\packages\tool-openocd/scripts -f interface/stlink.cfg -c "set CPUTAPID 0x2ba01477; transport select hla_swd"  -f target/stm32f1x.cfg -c "reset_config none" -c "program {.pio\build\bluepill_f103c8\firmware.elf}  verify reset; shutdown;"
