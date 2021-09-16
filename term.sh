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

"putty" -load $port-19200
