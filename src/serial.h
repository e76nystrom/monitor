#if ARDUINO_AVR_PRO
#include <SoftwareSerial.h>

extern SoftwareSerial dbgPort;

#define rxPin 5
#define txPin 6

#define WIFI Serial
#define DBGPORT dbgPort

#endif	/* ARDUINO_AVR_PRO */

#if ARDUINO_AVR_MEGA2560
#define DBGPORT Serial
#define WIFI Serial1
#endif	/* ARDUINO_AVR_MEGA2560 */

#define PRINTF 0		/* 0 clib printf, 1 printf.cpp */
#if PRINTF
#include "printf.h"
#endif

const char *argConv(const __FlashStringHelper *s);
const char *argConv(const __FlashStringHelper *s, char *buf);

#if ARDUINO_AVR_PRO
#define F0(x) argConv(F(x))
#define F1(x) F(x)
#define F2(x) F(x)
#define F3(x) argConv(F(x))	/* printf strings */
#endif	/* ARDUINO_AVR_PRO */

#if ARDUINO_AVR_MEGA2560
#define FLASH_STRINGS 1
#if FLASH_STRINGS
#define F0(x) argConv(F(x))
#define F1(x) F(x)
#define F2(x) F(x)
#define F3(x) argConv(F(x))	/* printf strings */
#else
#define F0(x) x
#define F1(x) x
#define F2(x) x
#define F3(x) x
#endif
#endif	/* ARDUINO_AVR_NEGA2560 */

#ifdef WIN32
#define F0(x) x
#define F1(x) x
#define F2(x) x
#define F3(x) x
#endif	/* WIN32 */

#define newLine() printf(F0("\n"))

#if !defined(DBG)
#define DBG 1
#endif
