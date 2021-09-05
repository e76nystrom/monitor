#!/bin/bash

if [ $# -eq 0 ]
then

 if [ -z "$PLATFORMIO_UPLOAD_PORT" ]
 then
  port=COM5
 fi

else
 port=$1
 export PLATFROMIO_UPLOAD_PORT=$port
fi

"putty" -load $port-19200
