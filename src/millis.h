#if ARDUINO_ARCH_AVR
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
#endif

#if INT_MILLIS

unsigned int intMillis();

#define millis intMillis
#define millisDef unsigned int

#else
#define millisDef unsigned long
#endif
