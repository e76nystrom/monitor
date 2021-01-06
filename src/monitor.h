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

char emonData(char *data);
char *cpyStr(char *dst, const char *str);

#if 0
#if !defined(DBG0_Pin)
#define DBG0_Pin 0
#define dbg0Set()
#define dbg0Clr()
#endif

#if !defined(DBG1_Pin)
#define DBG1_Pin 0
#define dbg1Set()
#define dbg1Clr()
#endif

#if !defined(DBG2_Pin)
#define DBG2_Pin 0
inline void dbg2Set() {}
inline void dbg2Clr() {}
#endif
#endif

/*
#if !defined()
#define 0
#endif
*/
