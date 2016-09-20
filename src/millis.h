#if ARDUINO_ARCH_AVR
extern volatile unsigned long timer0_millis;
#endif

#if INT_MILLIS

unsigned int intMillis();

#define millis intMillis
#define millisDef unsigned int

#else
#define millisDef unsigned long
#define intMillis millis
#endif
