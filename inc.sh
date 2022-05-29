#!/bin/bash

if [[ "$OSTYPE" == "cygwin" ]]; then
 INC="./inc.py"
else
 INC="python inc.py"
fi

$INC src/wifi.cpp include/wifi.h
$INC src/timer3.cpp include/timer3.h
$INC src/dns.cpp include/dns.h
$INC src/ntp.cpp include/ntp.h
$INC src/current.cpp include/current.h
$INC src/max31856.cpp src/max31856.h
$INC src/max31865.cpp src/max31865.h
$INC src/i2cx.cpp src/i2cx.h
$INC src/lcd.cpp src/lcd.h
$INC src/spix.cpp src/spix.h
