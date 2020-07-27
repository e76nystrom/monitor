#if !defined(EXT)
#define EXT extern
#endif

#include "monitorAVR.h"
#include "monitorSTM32.h"
#include "monitorMega32.h"

#if !defined(TEMP_SENSOR)
#define TEMP_SENSOR 0
#endif

#if !defined(RTC_CLOCK)
#define RTC_CLOCK 0
#endif

#if !defined(DHT_SENSOR)
#define DNT_SENSOR 0
#endif

#if !defined(CURRENT_SENSOR)
#define CURRENT_SENSOR 0
#endif

#if !defined(WATER_MONITOR)
#define WATER_MONITOR 0
#endif

#if !defined(CHECK_IN)
#define CHECK_IN 0
#endif

#if !defined(DEHUMIDIFIER)
#define DEHUMIDIFIER 0
#endif

#if !defined(ESP8266_TIME)
#define ESP8266_TIME 0
#endif

#if !defined(ESP8266_DNS)
#define ESP8266_DNS 0
#endif

#if !defined(DBG0_PIN)
#define DBG0_PIN 0
#endif

#if !defined(DBG1_PIN)
#define DBG1_PIN 0
#endif

#if !defined(DBG2_PIN)
#define DBG2_PIN 0
#endif

/*
#if !defined()
#define 0
#endif
*/
