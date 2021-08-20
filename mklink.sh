#!/bin/bash


rm src/stm32f1adc.c 
rm src/stm32f1dma.c 
rm src/stm32f1gpio.c
rm src/stm32f1i2c.c 
rm src/stm32f1spi.c 
rm src/stm32f1tim.c 
rm src/current.cpp
rm src/i2cx.cpp
rm src/i2cx.h
rm src/spix.cpp
rm src/spix.h
rm src/lcd.cpp
rm src/lcd.h
rm src/stm32Info.cpp

rm include/adc.h
rm include/current.h
rm include/dma.h
rm include/gpio.h
rm include/i2c.h
rm include/main.h
rm include/spi.h
rm include/tim.h
rm include/cyclectr.h
rm include/stm32Info.h

cd src
ln -s ../../../EclipseCPP/MonitorCPP/src/adc.c	           stm32f1adc.c  
ln -s ../../../EclipseCPP/MonitorCPP/src/dma.c	           stm32f1dma.c  
ln -s ../../../EclipseCPP/MonitorCPP/src/gpio.c	           stm32f1gpio.c 
ln -s ../../../EclipseCPP/MonitorCPP/src/i2c.c	           stm32f1i2c.c  
ln -s ../../../EclipseCPP/MonitorCPP/src/spi.c	           stm32f1spi.c  
ln -s ../../../EclipseCPP/MonitorCPP/src/tim.c	           stm32f1tim.c  
ln -s ../../../EclipseCPP/MonitorCPP/src/current.cpp	   current.cpp   
ln -s ../../../EclipseCPP/LatheCPP/lathe_src/i2cx.cpp      i2cx.cpp      
ln -s ../../../EclipseCPP/LatheCPP/include/i2cx.h	   i2cx.h        
ln -s ../../../EclipseCPP/LatheCPP/lathe_src/spix.cpp      spix.cpp      
ln -s ../../../EclipseCPP/LatheCPP/include/spix.h	   spix.h        
ln -s ../../../EclipseCPP/LatheCPP/lathe_src/lcd.cpp	   lcd.cpp       
ln -s ../../../EclipseCPP/LatheCPP/include/lcd.h	   lcd.h         
ln -s ../../../EclipseCPP/LatheCPP/lathe_src/stm32Info.cpp stm32Info.cpp 
cd ..

cd include
ln -s ../../../EclipseCPP/MonitorCPP/Inc/adc.h	        adc.h       
ln -s ../../../EclipseCPP/MonitorCPP/include/current.h  current.h   
ln -s ../../../EclipseCPP/MonitorCPP/Inc/dma.h	        dma.h       
ln -s ../../../EclipseCPP/MonitorCPP/Inc/gpio.h	        gpio.h      
ln -s ../../../EclipseCPP/MonitorCPP/Inc/i2c.h	        i2c.h       
ln -s ../../../EclipseCPP/MonitorCPP/Inc/main.h	        main.h      
ln -s ../../../EclipseCPP/MonitorCPP/Inc/spi.h	        spi.h       
ln -s ../../../EclipseCPP/MonitorCPP/Inc/tim.h	        tim.h       
ln -s ../../../EclipseCPP/MonitorCPP/include/cyclectr.h cyclectr.h  
ln -s ../../../EclipseCPP/LatheCPP/include/stm32Info.h  stm32Info.h 
cd ..
