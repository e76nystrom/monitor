#!/bin/bash

optiboot=0
usbasp=1
avrdude=/cygdrive/c/Users/Eric/.platformio/packages/tool-avrdude/avrdude.exe
conf="C:\\Users\\Eric\\.platformio\\packages\\tool-avrdude\\avrdude.conf"

if [ $optiboot -eq 0 ]
then
    if [ $usbasp -eq 1 ]
    then
        $avrdude \
	    -u \
	    -v \
	    -C $conf \
	    -p m328p \
	    -c usbasp-clone \
	    -U flash:w:".pio/build/mega328/firmware.hex":a
    fi

    if [ $usbasp -eq 0 ]
    then
export PLATFORMIO_UPLOAD_PORT=$port
echo "upload port" $PLATFORMIO_UPLOAD_PORT

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

	$avrdude
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

    $avrdude
	-v \
	-C "C:\DevSoftware\avrdude-v6.3.1.1-windows\avrdude.conf" \
	-p atmega2560 \
	-c arduino \
	-b 115200 \
	-P $port \
	-U flash:w:.pio/build/mega2560/firmware.hex:i

    "putty" -load ${port}-19200
fi

