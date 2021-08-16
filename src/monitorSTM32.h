#if !defined(__MONITOR_STM32__)
#define __MONITOR_STM32__

#ifdef ARDUINO_ARCH_STM32

#include <Arduino.h>
#include <Wire.h>
#include "serial.h"

#define MONITOR_INDEX 2

#define DBG 1

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1

#define SSID "hug2g996565"
#define PASS "candle14salt"

/* -------------------- monitor index 1 -------------------- */

#if (MONITOR_INDEX == 1)
#define MONITOR_ID "Monitor5"
#define EMONCMS_NODE "2"

#define CHECK_IN 1

#define INT_MILLIS 0
#define ESP8266_TIME 0

#define CURRENT_STM32 1

#define MAX_CHAN_POWER 0
#define MAX_CHAN_RMS 1
#define MAX_CHAN (MAX_CHAN_POWER + MAX_CHAN_RMS)

#endif	/* MONITOR_INDEX == 1 */

/* -------------------- monitor index 2 -------------------- */

#if (MONITOR_INDEX == 2)
#define MONITOR_ID "stm32Pwr0"
#define EMONCMS_NODE "stm32-2"

#define WIFI_ENA 1
#define CHECK_IN 1

#define INT_MILLIS 0
#define ESP8266_TIME 0

#define CURRENT_STM32 1

#define MAX_CHAN_POWER 1
#define MAX_CHAN_RMS 2
#define MAX_CHAN (MAX_CHAN_POWER + MAX_CHAN_RMS)

#endif	/* MONITOR_INDEX == 2 */

/* -------------------- monitor end of definitions -------------------- */

#define WIFI_RESET 19

#if CURRENT_STM32

#define ADC1_0 LL_ADC_CHANNEL_2
#define ADC1_1 LL_ADC_CHANNEL_0
#define ADC2_0 LL_ADC_CHANNEL_1
#define ADC2_1 LL_ADC_CHANNEL_3

#if defined(__CURRENT_INC__) && (CURRENT_STM32 != 0) && !defined(__CURRENT__)
extern T_CHANCFG chanCfg[MAX_CHAN];
#endif	/* (CURRENT_STM32 != 0) && !defined(__CURRENT__) */

#endif

//#define LCD_ENA 0
#define LCD_ADDRESS 0x3f

#include "millis.h"
#include "main.h"

#ifdef Dbg0_Pin
inline void dbg0Ini() {}
inline void dbg0Set() {Dbg0_GPIO_Port->BSRR = Dbg0_Pin;}
inline void dbg0Clr() {Dbg0_GPIO_Port->BSRR = (Dbg0_Pin << 16);}
#else
inline void dbg0Ini() {}
inline void dbg0Set() {}
inline void dbg0Clr() {}
#endif

#ifdef Dbg1_Pin
inline void dbg1Ini() {}
inline void dbg1Set() {Dbg1_GPIO_Port->BSRR = Dbg1_Pin;}
inline void dbg1Clr() {Dbg1_GPIO_Port->BSRR = (Dbg1_Pin << 16);}
#else
inline void dbg1Ini() {}
inline void dbg1Set() {}
inline void dbg1Clr() {}
#endif

#ifdef Dbg2_Pin
inline void dbg2Ini() {}
inline void dbg2Set() {Dbg2_GPIO_Port->BSRR = Dbg2_Pin;}
inline void dbg2Clr() {Dbg2_GPIO_Port->BSRR = (Dbg2_Pin << 16);}
#else
inline void dbg2Ini() {}
inline void dbg2Set() {}
inline void dbg2Clr() {}
#endif

#ifdef Dbg3_Pin
inline void dbg3Ini() {}
inline void dbg3Set() {Dbg3_GPIO_Port->BSRR = Dbg3_Pin;}
inline void dbg3Clr() {Dbg3_GPIO_Port->BSRR = (Dbg3_Pin << 16);}
#else
inline void dbg3Ini() {}
inline void dbg3Set() {}
inline void dbg3Clr() {}
#endif

#ifdef Dbg4_Pin
inline void dbg4Ini() {}
inline void dbg4Set() {Dbg4_GPIO_Port->BSRR = Dbg4_Pin;}
inline void dbg4Clr() {Dbg4_GPIO_Port->BSRR = (Dbg4_Pin << 16);}
inline bool dbg4() {return((Dbg4_GPIO_Port->ODR & Dbg4_Pin) != 0);}
inline void dbg4Toggle()
{
 if (dbg4())
  dbg4Clr();
 else
  dbg4Set();
}
#else
inline void dbg4Ini() {}
inline void dbg4Set() {}
inline void dbg4Clr() {}
inline bool dbg4() {return(0);}
inline void dbg4Toggle() {}
#endif

#ifdef Dbg5_Pin
inline void dbg5Ini() {}
inline void dbg5Set() {Dbg5_GPIO_Port->BSRR = Dbg5_Pin;}
inline void dbg5Clr() {Dbg5_GPIO_Port->BSRR = (Dbg5_Pin << 16);}
inline bool dbg5() {return((Dbg5_GPIO_Port->ODR & Dbg5_Pin) != 0);}
inline void dbg5Toggle()
{
 if (dbg5())
  dbg5Clr();
 else
  dbg5Set();
}
#else
inline void dbg5Ini() {}
inline void dbg5Set() {}
inline void dbg5Clr() {}
inline bool dbg5() {return(0);}
inline void dbg5Toggle() {}
#endif

#endif /* ARDUINO_ARCH_STM32 */
#endif /* __MONITOR_STM32__ */
