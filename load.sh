#!/bin/bash

if [ $# -eq 0 ]
then

 if [ -z "$PLATFORMIO_UPLOAD_PORT" ]
 then
  port=COM5
 fi

else
 port=$1
fi

echo "port" $port
export PLATFORMIO_UPLOAD_PORT=$port
echo "upload port" $PLATFORMIO_UPLOAD_PORT

optiboot=1
usbasp=0

if [ $optiboot -eq 0 ]
then
    if [ $usbasp -eq 1 ]
    then
	/cygdrive/c/DevSoftware/avrdude-v6.3.1.1-windows/avrdude \
	    -u \
	    -v \
	    -C "C:\DevSoftware\avrdude-v6.3.1.1-windows\avrdude.conf" \
	    -p m328p \
	    -c usbasp-clone \
	    -U flash:w:".pio\build\pro16MHzatmega328\firmware.hex":a
    fi

    if [ $usbasp -eq 0 ]
    then

	/cygdrive/c/DevSoftware/avrdude-v6.3.1.1-windows/avrdude \
	    -v \
	    -C "C:\DevSoftware\avrdude-v6.3.1.1-windows\avrdude.conf" \
	    -p atmega2560 \
	    -c wiring \
	    -b 115200 \
	    -P $port \
	    -U flash:w:.pio/build/mega2560/firmware.hex:i

	"putty" -load ${port}-19200
    fi

else

    /cygdrive/c/DevSoftware/avrdude-v6.3.1.1-windows/avrdude \
	-v \
	-C "C:\DevSoftware\avrdude-v6.3.1.1-windows\avrdude.conf" \
	-p atmega2560 \
	-c arduino \
	-b 115200 \
	-P $port \
	-U flash:w:.pio/build/mega2560/firmware.hex:i

    "putty" -load ${port}-19200
fi
