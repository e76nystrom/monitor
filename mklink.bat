rm src\stm32f1adc.c 
rm src\stm32f1dma.c 
rm src\stm32f1gpio.c
rm src\stm32f1i2c.c 
rm src\stm32f1spi.c 
rm src\stm32f1tim.c 
rm src\current.cpp
rm src\i2cx.cpp
rm src\spix.cpp
rm src\lcd.cpp
rm src\stm32Info.cpp

rm include\adc.h
rm include\current.h
rm include\dma.h
rm include\gpio.h
rm include\i2c.h
rm include\main.h
rm include\spi.h
rm include\tim.h
rm include\cyclectr.h
rm include\stm32Info.h

mklink src\stm32f1adc.c  ..\..\..\EclipseCPP\MonitorCPP\src\adc.c
mklink src\stm32f1dma.c  ..\..\..\EclipseCPP\MonitorCPP\src\dma.c
mklink src\stm32f1gpio.c ..\..\..\EclipseCPP\MonitorCPP\src\gpio.c
mklink src\stm32f1i2c.c  ..\..\..\EclipseCPP\MonitorCPP\src\i2c.c
mklink src\stm32f1spi.c  ..\..\..\EclipseCPP\MonitorCPP\src\spi.c
mklink src\stm32f1tim.c  ..\..\..\EclipseCPP\MonitorCPP\src\tim.c
mklink src\current.cpp  ..\..\..\EclipseCPP\MonitorCPP\src\current.cpp
mklink src\i2cx.cpp ..\..\..\EclipseCPP\LatheCPP\lathe_src\i2cx.cpp
mklink src\spix.cpp ..\..\..\EclipseCPP\LatheCPP\lathe_src\spix.cpp
mklink src\lcd.cpp  ..\..\..\EclipseCPP\LatheCPP\lathe_src\lcd.cpp
mklink src\stm32Info.cpp  ..\..\..\EclipseCPP\LatheCPP\lathe_src\stm32Info.cpp

mklink include\adc.h ..\..\..\EclipseCPP\MonitorCPP\Inc\adc.h
mklink include\current.h ..\..\..\EclipseCPP\MonitorCPP\include\current.h
mklink include\dma.h ..\..\..\EclipseCPP\MonitorCPP\Inc\dma.h
mklink include\gpio.h ..\..\..\EclipseCPP\MonitorCPP\Inc\gpio.h
mklink include\i2c.h ..\..\..\EclipseCPP\MonitorCPP\Inc\i2c.h
mklink include\main.h ..\..\..\EclipseCPP\MonitorCPP\Inc\main.h
mklink include\spi.h ..\..\..\EclipseCPP\MonitorCPP\Inc\spi.h
mklink include\tim.h ..\..\..\EclipseCPP\MonitorCPP\Inc\tim.h
mklink include\cyclectr.h  ..\..\..\EclipseCPP\MonitorCPP\include\cyclectr.h
mklink include\stm32Info.h  ..\..\..\EclipseCPP\LatheCPP\include\stm32Info.h
