#!/bin/bash

python ./inc.py src/wifi.cpp include/wifi.h
python ./inc.py src/dns.cpp include/dns.h
python ./inc.py src/ntp.cpp include/ntp.h
python ./inc.py src/current.cpp include/current.h
python ./inc.py src/max31856.cpp src/max31856.h
python ./inc.py src/i2cx.cpp src/i2cx.h
python ./inc.py src/lcd.cpp src/lcd.h
python ./inc.py src/spix.cpp src/spix.h
