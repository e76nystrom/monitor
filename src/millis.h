#ifdef ARDUINO_ARCH_AVR
typedef union
{
 struct
 {
  uint16_t low;
  uint16_t high;
 };
 struct
 {
  unsigned long val;
 };
} T_SHORT_LONG, *P_SHORT_LONG;

//extern volatile T_SHORT_LONG timer0_millis;
extern volatile unsigned long timer0_millis;

#if INT_MILLIS

unsigned int intMillis();

#define millis intMillis
#define millisDef unsigned int

#else
#define millisDef unsigned long
#endif

#endif

#ifdef ARDUINO_ARCH_STM32
#define millisDef unsigned int
#endif

#if 0 //STM32MON
#include "stm32f1xx_hal.h"

extern __IO uint32_t uwTick;

inline unsigned int millis(void)
{
 return((unsigned int) uwTick);
}

inline void delay(uint32_t ms)
{
  if (ms != 0)
  {
   uint32_t start = uwTick;
   do
   {
//    yield();
   } while (uwTick - start < ms);
  }
}

#define millisDef unsigned int

#endif

