#ifdef ARDUINO_AVR_PRO
#include <SoftwareSerial.h>

extern SoftwareSerial dbgPort;

#define rxPin 5
#define txPin 6

#define WIFI Serial
#define DBGPORT dbgPort

#endif	/* ARDUINO_AVR_PRO */

#define PRINTF 0		/* 0 clib printf, 1 printf.cpp */
#if PRINTF
#include "printf.h"
#endif

#ifdef ARDUINO_AVR_MEGA2560
#define WIFI Serial1
#define DBGPORT Serial
#endif

const char *argConv(const __FlashStringHelper *s);

#if ARDUINO_AVR_PRO
//#define F0(x) ((const char *) F(x))
#define F0(x) x
#define F1(x) x
#define F2(x) F(x)
#define F3(x) argConv(F(x))	/* printf strings */
#endif

#ifdef ARDUINO_AVR_MEGA2560
//#define F0(x) x
#define F0(x) argConv(F(x))
#define F1(x) x
#define F2(x) F(x)
#define F3(x) argConv(F(x))	/* printf strings */
#endif

#ifdef WIN32
#define F0(x) x
#define F1(x) x
#define F2(x) x
#define F3(x) x
#endif

#define newLine() printf(F0("\n"))

#if !defined(DBG)
#define DBG 1
#endif
