#!/bin/bash

avr-g++ \
-o .pioenvs\megaatmega2560\src\monitor.o \
-c \
-fno-exceptions \
-fno-threadsafe-statics \
-std=gnu++11 \
-g \
-Os \
-Wall \
-ffunction-sections \
-fdata-sections \
-mmcu=atmega2560 \
-save-temps \
-DF_CPU=16000000L \
-DPLATFORMIO=030001 \
-DARDUINO_ARCH_AVR \
-DARDUINO_AVR_MEGA2560 \
-DARDUINO=10608 \
-IC:\cygwin\home\Eric\.platformio\packages\framework-arduinoavr\cores\arduino \
-IC:\cygwin\home\Eric\.platformio\packages\framework-arduinoavr\variants\mega \
-IC:\cygwin\home\Eric\.platformio\lib\Time_ID44 \
-IC:\cygwin\home\Eric\.platformio\packages\framework-arduinoavr\libraries\Wire\src \
-IC:\cygwin\home\Eric\.platformio\lib\DS3232RTC_ID78 \
-IC:\cygwin\home\Eric\.platformio\lib\OneWire_ID1 \
-IC:\cygwin\home\Eric\.platformio\packages\framework-arduinoavr\libraries\SoftwareSerial\src \
-IC:\cygwin\home\Eric\.platformio\lib\DallasTemperature_ID54 \
-IC:\cygwin\home\Eric\.platformio\lib\DHT_sensor_library_ID19 \
-IC:\cygwin\home\Eric\.platformio\packages\framework-arduinoavr\libraries\EEPROM\src \
-Isrc \
-save-temps \
-Wa,-adhln >monitor.lst \
src\monitor.cpp
