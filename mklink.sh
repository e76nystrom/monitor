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

ln -s src\stm32f1adc.c  ..\..\..\EclipseCPP\MonitorCPP\src\adc.c
ln -s src\stm32f1dma.c  ..\..\..\EclipseCPP\MonitorCPP\src\dma.c
ln -s src\stm32f1gpio.c ..\..\..\EclipseCPP\MonitorCPP\src\gpio.c
ln -s src\stm32f1i2c.c  ..\..\..\EclipseCPP\MonitorCPP\src\i2c.c
ln -s src\stm32f1spi.c  ..\..\..\EclipseCPP\MonitorCPP\src\spi.c
ln -s src\stm32f1tim.c  ..\..\..\EclipseCPP\MonitorCPP\src\tim.c
ln -s src\current.cpp  ..\..\..\EclipseCPP\MonitorCPP\src\current.cpp
ln -s src\i2cx.cpp ..\..\..\EclipseCPP\LatheCPP\lathe_src\i2cx.cpp
ln -s src\spix.cpp ..\..\..\EclipseCPP\LatheCPP\lathe_src\spix.cpp
ln -s src\lcd.cpp  ..\..\..\EclipseCPP\LatheCPP\lathe_src\lcd.cpp
ln -s src\stm32Info.cpp  ..\..\..\EclipseCPP\LatheCPP\lathe_src\stm32Info.cpp

ln -s include\adc.h ..\..\..\EclipseCPP\MonitorCPP\Inc\adc.h
ln -s include\current.h ..\..\..\EclipseCPP\MonitorCPP\include\current.h
ln -s include\dma.h ..\..\..\EclipseCPP\MonitorCPP\Inc\dma.h
ln -s include\gpio.h ..\..\..\EclipseCPP\MonitorCPP\Inc\gpio.h
ln -s include\i2c.h ..\..\..\EclipseCPP\MonitorCPP\Inc\i2c.h
ln -s include\main.h ..\..\..\EclipseCPP\MonitorCPP\Inc\main.h
ln -s include\spi.h ..\..\..\EclipseCPP\MonitorCPP\Inc\spi.h
ln -s include\tim.h ..\..\..\EclipseCPP\MonitorCPP\Inc\tim.h
ln -s include\cyclectr.h  ..\..\..\EclipseCPP\MonitorCPP\include\cyclectr.h
ln -s include\stm32Info.h  ..\..\..\EclipseCPP\LatheCPP\include\stm32Info.h
