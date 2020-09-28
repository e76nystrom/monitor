//#include <cstdio>
//#include <cstring>

#ifdef ARDUINO_ARCH_AVR

#define PRINTF 0		/* 0 clib printf, 1 printf.cpp */
#if PRINTF
#include "printf.h"
#endif	/* PRINTF */

const char *argConv(const __FlashStringHelper *s);
const char *argConv(const __FlashStringHelper *s, char *buf);

#ifdef ARDUINO_AVR_PRO
#include <SoftwareSerial.h>

extern SoftwareSerial dbgPort;

#define rxPin 5
#define txPin 6

#define WIFI Serial
#define DBGPORT dbgPort

#define F0(x) argConv(F(x))	/* uses stringBuffer */
#define F1(x) F(x)
#define F2(x) F(x)
#define F3(x) argConv(F(x))	/* printf strings */

#endif	/* ARDUINO_AVR_PRO */

#if ARDUINO_AVR_MEGA2560

#define DBGPORT Serial
#define WIFI Serial2

#define FLASH_STRINGS 1

#if FLASH_STRINGS
#define F0(x) argConv(F(x))	/* uses stringBuffer */
#define F1(x) F(x)
#define F2(x) F(x)
#define F3(x) argConv(F(x))	/* printf strings */
#else
#define argConv(s, buf) s
#define F0(x) x
#define F1(x) x
#define F2(x) x
#define F3(x) x
#endif	/* FLASH_STRINGS */

#endif	/* ARDUINO_AVR_NEGA2560 */

inline void flush() {}

#endif  /* ARDUINO_ARCH_AVR */

#ifdef ARDUINO_ARCH_STM32

#define DBGPORT Serial1
#define WIFI Serial3

inline const char *argConv(const char *s) {return(s);}
inline const char *argConv(const char *s, char *buf) {return(s);}
#define F0(x) x
#define F1(x) x
#define F2(x) x
#define F3(x) x

inline void flush() {fflush(stdout); DBGPORT.flush();}

#endif  /* ARDUINO_ARCH_STM32 */

#if	0//def STM32MON
inline void argConv(char *s) {return(s);}
inline void argConv(char *s, char *buf) {return(s);}
#define F0(x) x
#define F1(x) x
#define F2(x) x
#define F3(x) x
#endif	/* STM32MON */

#ifdef WIN32
#define argConv(s, buf) s
#define F0(x) x
#define F1(x) x
#define F2(x) x
#define F3(x) x
#endif	/* WIN32 */

void newLine();

#if !defined(DBG)
#define DBG 1
#endif
