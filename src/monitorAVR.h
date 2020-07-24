#ifdef ARDUINO_ARCH_AVR
#include <Arduino.h>
#include <Wire.h>
#include "serial.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define DBG 1

#define INT_MILLIS 0

#include "millis.h"
#include "monitorMega.h"
#include "monitorPro.h"

#endif	/* ARDUINO_ARCH_AVR */
