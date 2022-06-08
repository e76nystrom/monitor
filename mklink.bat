del src\stm32f1adc.c 
del src\stm32f1dma.c 
del src\stm32f1gpio.c
del src\stm32f1i2c.c 
del src\stm32f1spi.c 
del src\stm32f1tim.c 
del src\current.cpp
del src\i2cx.cpp
del src\i2cx.h
del src\spix.cpp
del src\spix.h
del src\lcd.cpp
del src\lcd.h
del src\stm32Info.cpp

del src\stm32f1Nadc.c 
del src\stm32f1Ngpio.c

del include\adc.h
del include\current.h
del include\dma.h
del include\gpio.h
del include\i2c.h
del include\main.h
rem del include\spi.h
del include\tim.h
del include\cyclectr.h
del include\stm32Info.h

del include\mainN.h

mklink src\stm32f1adc.c  ..\..\..\EclipseCPP\MonitorCPP\src\adc.c
mklink src\stm32f1dma.c  ..\..\..\EclipseCPP\MonitorCPP\src\dma.c
mklink src\stm32f1gpio.c ..\..\..\EclipseCPP\MonitorCPP\src\gpio.c
mklink src\stm32f1i2c.c  ..\..\..\EclipseCPP\MonitorCPP\src\i2c.c
mklink src\stm32f1spi.c  ..\..\..\EclipseCPP\MonitorCPP\src\spi.c
mklink src\stm32f1tim.c  ..\..\..\EclipseCPP\MonitorCPP\src\tim.c
mklink src\current.cpp  ..\..\..\EclipseCPP\MonitorCPP\src\current.cpp
mklink src\i2cx.cpp ..\..\..\EclipseCPP\LatheCPP\lathe_src\i2cx.cpp
mklink src\i2cx.h ..\..\..\EclipseCPP\LatheCPP\include\i2cx.h
mklink src\spix.cpp ..\..\..\EclipseCPP\LatheCPP\lathe_src\spix.cpp
mklink src\spix.h ..\..\..\EclipseCPP\LatheCPP\include\spix.h
mklink src\lcd.cpp  ..\..\..\EclipseCPP\LatheCPP\lathe_src\lcd.cpp
mklink src\lcd.h ..\..\..\EclipseCPP\LatheCPP\include\lcd.h
mklink src\stm32Info.cpp  ..\..\..\EclipseCPP\LatheCPP\lathe_src\stm32Info.cpp

mklink src\stm32f1Nadc.c  ..\..\..\EclipseCPP\MonitorCPPN\src\adc.c
mklink src\stm32f1Ngpio.c ..\..\..\EclipseCPP\MonitorCPPN\src\gpio.c

mklink include\adc.h ..\..\..\EclipseCPP\MonitorCPP\Inc\adc.h
mklink include\current.h ..\..\..\EclipseCPP\MonitorCPP\include\current.h
mklink include\dma.h ..\..\..\EclipseCPP\MonitorCPP\Inc\dma.h
mklink include\gpio.h ..\..\..\EclipseCPP\MonitorCPP\Inc\gpio.h
mklink include\i2c.h ..\..\..\EclipseCPP\MonitorCPP\Inc\i2c.h
mklink include\main.h ..\..\..\EclipseCPP\MonitorCPP\Inc\main.h
rem mklink include\spi.h ..\..\..\EclipseCPP\MonitorCPP\Inc\spi.h
mklink include\tim.h ..\..\..\EclipseCPP\MonitorCPP\Inc\tim.h
mklink include\cyclectr.h  ..\..\..\EclipseCPP\MonitorCPP\include\cyclectr.h
mklink include\stm32Info.h  ..\..\..\EclipseCPP\LatheCPP\include\stm32Info.h

mklink include\mainN.h ..\..\..\EclipseCPP\MonitorCPPN\Inc\main.h
