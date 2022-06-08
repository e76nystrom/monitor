//******************************************************************************
/* installed libraries

OneWire
=======
#ID: 1
Control 1-Wire protocol (DS18S20, DS18B20, DS2408 and etc)

Version: 2.3.5
Homepage: https://www.pjrc.com/teensy/td_libs_OneWire.html
Keywords: onewire, temperature, bus, 1-wire, ibutton, sensor
Compatible frameworks: arduino

DHT sensor library
==================
#ID: 19
Arduino library for DHT11, DHT22, etc Temp & Humidity Sensors

Keywords: sensors
Compatible frameworks: Arduino
Compatible platforms: Infineon XMC, Kendryte K210, GigaDevice GD32V, ASR Microel
ectronics ASR605x, Atmel AVR, Atmel SAM, Espressif 8266, Intel ARC32, Microchip
PIC32, Nordic nRF51, ST STM32, Teensy, TI MSP430, TI TIVA, Espressif 32, Nordic
nRF52, ST STM8, Atmel megaAVR, SIWI GSM Platform

Time
====
#ID: 44
Time keeping library

Version: 1.6
Homepage: http://playground.arduino.cc/Code/Time
Keywords: week, rtc, hour, year, month, second, time, date, day, minute
Compatible frameworks: arduino

DallasTemperature
=================
#ID: 54
Arduino Library for Dallas Temperature ICs (DS18B20, DS18S20, DS1822, DS1820)

Version: 3.8.1
Keywords: bus, sensor, 1-wire, onewire, temperature
Compatible frameworks: arduino
Compatible platforms: atmelavr, atmelmegaavr, atmelsam, espressif32,
espressif8266, gd32v, infineonxmc, intel_arc32, kendryte210, microchippic32,
nordicnrf51, nordicnrf52, ststm32, ststm8, teensy, timsp430

DS3232RTC
=========
#ID: 78
Arduino Library for Maxim Integrated DS3232 and DS3231 Real-Time Clocks

Version: 261ca7d0e6
Keywords: rtc, time
Compatible frameworks: arduino

*/

#define __MONITOR__
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DallasTemperature.h>
#include <TimeLib.h>
#include <OneWire.h>
#include <Wire.h>

#define EMONCMS_ADDR0 "192.168.1.111"
#define EMONCMS_KEY0 "b53ec1abe610c66009b207d6207f2c9e"

#define EMONCMS_ADDR1 "192.168.42.10"
#define EMONCMS_KEY1 "cd5f31b05f8008756e76f87ecb762199"

#define TEST_NODE 0

#define RETRY_JOIN 3
#define RETRY_SEND_HTTP 2
#define RETRY_EMON_DATA 2

unsigned char getNum();
int val;

#define EXT
#include "monitor.h"

void setTime();
unsigned char uartSave;
void putInit(void);
void putRestore();
void putx0(char c);
void putstr0(const char *str);
void putstr0(const __FlashStringHelper *str);
void sndhex(unsigned char *p, int size);

#if TEMP_SENSOR

void findAddresses(void);

#if TEMP_SENSOR == 1
OneWire oneWire(ONE_WIRE_BUS);	/* one wire instance */
DallasTemperature sensors(&oneWire); /* dallas temp sensor instance */
float lastTemp[TEMPDEVS];
#elif TEMP_SENSOR == 2
OneWire oneWire0(ONE_WIRE_BUS0);	/* one wire instance */
OneWire oneWire1(ONE_WIRE_BUS1);	/* one wire instance */
DallasTemperature sensor0(&oneWire0); /* dallas temp sensor instance */
DallasTemperature sensor1(&oneWire1); /* dallas temp sensor instance */
float temp0[TEMPDEVS0];
float temp1[TEMPDEVS1];
float lastTemp0[TEMPDEVS0];
float lastTemp1[TEMPDEVS1];
typedef struct s_temp_sensor
{
 OneWire *oneWire;
 DallasTemperature *sensor;
 uint8_t deviceCount;
 DeviceAddress *tempDev;
 float *temp;
 float *lastTemp;
 char oneWireBus;
} T_TEMP_SENSOR, *P_TEMP_SENSOR;
T_TEMP_SENSOR tempSensor[] =
{
 {.oneWire = &oneWire0, .sensor = &sensor0, .deviceCount = TEMPDEVS0,
  .tempDev = tempDev0, .temp = temp0, .lastTemp = lastTemp0,
  .oneWireBus = ONE_WIRE_BUS0},
 {.oneWire = &oneWire1, .sensor = &sensor1, .deviceCount = TEMPDEVS1,
  .tempDev = tempDev1, .temp = temp1, .lastTemp = lastTemp1,
  .oneWireBus = ONE_WIRE_BUS1},
};
#endif	/* TEMP_SENSOR == 2 */
//float printTemperature(DeviceAddress deviceAddress);

#endif  /* TEMP_SENSOR */

#if DHT_SENSOR
#include <DHT.h>

#if DEHUMIDIFIER
void switchRelay(char pin);
#endif	/* DEHUMIDIFIER */

#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE, 15);

#if DEHUMIDIFIER
char dehumState;		/* dehumidifier state */
float dehumOn;			/* on humidity */
float dehumOff;			/* off humidity */
int dehumDelay;			/* on or off delay counter */
#define DEHUM_DELAY 1		/* on or off delay time */
#endif	/* DEHUMIDIFIER */

#endif  /* DHT_SENSOR */

#if RTC_CLOCK
#undef TEMP_MSB
#undef TEMP_LSB
#include <DS3232RTC.h>
#endif  /* RTC_CLOCK */

#if CURRENT_SENSOR

#include "struct.h"
#include "timer3.h"

#define READVCC_CALIBRATION_CONST 1126400L
#define ADC_BITS 10
#define ADC_COUNTS (1<<ADC_BITS)

#endif  /* CURRENT_SENSOR */

#include <TimeLib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "wdt.h"
#include "dns.h"
#if ESP8266_TIME == 0
#include "ntp.h"
#endif	/* ESP8266_TIME == 0 */
#include "wifi.h"

#if defined(ARDUINO_ARCH_AVR)
#include <EEPROM.h>

#define INITEE 1		/* init eeprom values on boot */
#define LOCAL 0			/* use local server */

#include "dbgInfo.h"

#if ESP8266_TIME
char esp8266TimeEnable();
char esp8266Time();
#endif	/* ESP8266_TIME */

#if defined(ARDUINO_AVR_PRO)
SoftwareSerial dbgPort = SoftwareSerial(rxPin, txPin);
#endif	/* ARDUINO_AVR_PRO */

#if PRINTF
void putx1(void *p, char c)
{
 if (c == '\n')
  DBGPORT.write('\r');
 DBGPORT.write(c);
}
#endif	/* PRINTF */

#if !PRINTF
static FILE uartout = {0};

static int putx(char c, FILE *stream)
{
 if (c == '\n')
  DBGPORT.write('\r');
 DBGPORT.write(c);
 return 0;
}
#endif	/* ! PRINTF */

const char *argConv(const __FlashStringHelper *s)
{
 PGM_P p = reinterpret_cast <PGM_P> (s);
 char *dst = stringBuffer;
 while (1)
 {
  unsigned char c = pgm_read_byte(p++);
  *dst++ = c;
  if (c == 0)
   break;
 }
 return((const char *) stringBuffer);
}

const char *argConv(const __FlashStringHelper *s, char *buf)
{
 PGM_P p = reinterpret_cast <PGM_P> (s);
 char *dst = buf;
 while (1)
 {
  unsigned char c = pgm_read_byte(p++);
  *dst++ = c;
  if (c == 0)
   break;
 }
 return((const char *) buf);
}

#endif  /* ARDUINO_ARCH_AVR */

#if defined(ARDUINO_ARCH_STM32)
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "stm32Info.h"
#undef EXT
#define EXT extern
#include "current.h"
#include "max31856.h"
#include "max31865.h"

#define DATA_SIZE 1

#if DATA_SIZE
extern char _sbss;
extern char _ebss;
extern char _sdata;
extern char _edata;
extern char _estack;
#endif	/* DATA_SIZE */

extern "C" unsigned int getSP(void);

#endif	/* ARDUINO_ARCH_STM32 */

char emonIP[IP_ADDRESS_LEN];	/* emoncms ip address */

#if CHECK_IN | WATER_MONITOR
#define SERVER_IP_TIMEOUT (1UL * 60UL * 60UL * 1000UL) /* 1 hour dns lookup */
char serverIP[IP_ADDRESS_LEN];	/* server ip address */
unsigned long serverIPTime;	/* time when server ip set */
char failCount;			/* send failure count */

#if LOCAL
#define SERVER "10.0.0.2"
#define TCPPORT 8080
#define HOST "10.0.0.2"
#define SITE
#else
#define TCPPORT 80
#define HOST "test.ericnystrom.com"
//#define HOST "www.ericnystrom.com"
#define SITE "/alert"
#endif  /* LOCAL */

#define HTTP " HTTP/1.1\r\nHost: " HOST "\r\nConnection: Close\r\n\r\n"

#endif	/* CHECK_IN | WATER_MONITOR */

#if WATER_MONITOR

typedef struct
{
 boolean state;
 boolean inp;
 char index;
 char counter;
} T_INPUT, *P_INPUT;

#define COUNT 3

T_INPUT water0;
T_INPUT water1;

#define STATE_ALARM 0
#define STATE_CLEAR 1

boolean beeperOn;
char beeperCount;

#define BEEPERCOUNT 5
#define BEEPLEN 100
#define BEEPSPACE 1000
#define BEEPS 5

#endif	/* WATER_MONITOR */ 

#if TEMP_SENSOR | DHT_SENSOR
void loopTemp();
char *writeTemp(char *buf, float temp);
void printTemp(float temp);
#endif	/* TEMP_SENSOR | DHT_SENSOR */

unsigned long time = 0;

#if CHECK_IN
char checkIn();
#endif	/* CHECK_IN */

#if WATER_MONITOR
void loopWater();
void alarmPoll();
void procAlarm(P_INPUT water, boolean inp);
char notify(int alarm, boolean val);
#endif  /* WATER_MONITOR */

#if CHECK_IN | WATER_MONITOR
char sendHTTP(char *data);
#endif  /* CHECK_IN | WATER_MONITOR */

#if WATER_MONITOR
void updateFail();
#endif  /* WATER_MONITOR */

char *cpyStr(char *dst, const char *str);
char *strEnd(char *p);

#if (TEMP_SENSOR | DHT_SENSOR | CURRENT_SENSOR | CURRENT_STM32)

#define TEST_GET "get /emoncms/input/post.json?node=4"\
 "&csv=0.0&apikey=" EMONCMS_KEY\
 " HTTP/1.0\r\nHost: 192.168.42.10\r\nConnection: close\r\n\r\n"

#define HTTP1 " HTTP/1.1\r\nHost: " EMONCMS_ADDR "\r\nConnection: Close\r\n\r\n"

char emonData(char *data);

#endif	/* TEMP_SENSOR | DHT_SENSOR | CURRENT_SENSOR | CURRENT_STM32 */

#if RTC_CLOCK
float rtcTemp();
#endif	/* RTC_CLOCK */

void cmdLoop();
unsigned int tLast;
int loopCount;

#if !defined(monDbg)
char monDbg;
#endif	/* monDbg */

#define TINTERVAL (10000U)	/* timer interval */
#define T1SEC (1000U)		/* one second interval */
#define TEMP_COUNT (0)		/* temp reading interval number */
#define WATER_COUNT (1)		/* water alarm interval number */
#define CHECKIN_COUNT (2)	/* checkin interval number */
#define NTP_COUNT (3)		/* time setting */
#define LOOP_MAX (6)		/* max number of intervals */

#if CURRENT_SENSOR

void initCurrent(char isr);	/* init current readings */
void printCurrent();
void currentCheck();		/* check for time to send data */
void timer3();			/* timer isr for reading current */
//int adcRead(char chan);

#if 1
#define CYCLE_COUNT 60		/* cycles per sample */
#else
#define SAMPLES (100)		/* samples per reading */
#endif	/* 1 */

#define ADCCHANS (2)		/* number of adc channels to read */

#define CSENDTIME (10 * 60)	/* if no change for this time send current */

typedef struct
{
 //unsigned long iTime;		/* time of data reading */
 char index;			/* channel number */
 unsigned long lastTime;	/* last time data sent */
 float lastIRms0;
 float lastIRms1;
 float iRms;
 float lastIRms;
 float offsetI;
 float iCal;
 float iRatio;			/* conversion ratio */
 float sumI;			/* current sum of squares */
 float lastPrint;
 int iVcc;
 unsigned int samples;
 const char *node;
 int count;
 int sent;
 int adc;
 char send;
} T_CURRENT, *P_CURRENT;

#define CROSS_TMO 240

unsigned char iChan;		/* data channel for current reading */
P_CURRENT curPSave;		/* saved current pointer */
int sampleCount;		/* sample counter */
char adcState;			/* adc state */
char cState;			/* current processing state */
#if 1
char waitCrossing;		/* wait for zero crossing */
int crossTmr;			/* crossing timer */
char lastBelow;			/* last value below zero */
int cycleCount;			/* cycle counter */
#endif	/* 1 */
T_CURRENT iData[ADCCHANS];	/* current channel data */
float tempSumI;			/* current sum accumulator */

typedef struct
{
 P_CURRENT chan;		/* channel channel */
 unsigned int iVcc;		/* voltage result */
 unsigned int sampleCount;	/* sample count */
 float sumI;			/* sum of squares */
} T_CUR_DATA, *P_CUR_DATA;

#define MAX_CUR_DATA 6
typedef struct
{
 char count;			/* number in queue */
 unsigned int fil;		/* fill pointer */
 unsigned int emp;		/* empty pointer */
 T_CUR_DATA curData[MAX_CUR_DATA]; /* data */
} T_CUR_QUE, *P_CUR_QUE;

char curBusy;			/* busy sending data */
T_CUR_QUE curQue;		/* current queue */

#endif  /* CURRENT_SENSOR */

#if defined(CURRENT_STM32)
extern "C" void ADC_MspInit(ADC_HandleTypeDef* adcHandle);
#endif	/* CURRENT_STM32 */

#if defined(LCD_ENA)
#include <LiquidCrystal_I2C.h>
#include "lcd.h"

#define COLUMS           20
#define ROWS             4

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01,
		      4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

#endif	/* LCD_ENA */

#if defined(OLED_ENA)

#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif	/* U8X8_HAVE_HW_I2C */

//U8X*_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

#endif	/* OLED_ENA */

void putx0(void *p, char c);
void putx(char c);

#if defined(ARDUINO_ARCH_AVR)

uint16_t intMillis()
{
 uint16_t m;
 uint8_t oldSREG = SREG;	/* save interrupt flag */
 cli();				/* disable interrupts */
 m = ((P_SHORT_LONG) &timer0_millis)->low; /* read low part of millis */
 SREG = oldSREG;		/* enable interrupts */
 return(m);			/* return value */
}

char updateEE(const char *prompt, char eeLoc, char eeLen);
char updateEE(const __FlashStringHelper *prompt, char eeLoc, char eeLen);

char updateEE(const __FlashStringHelper *prompt, char eeLoc, char eeLen)
{
 char str[12];
 return(updateEE(argConv(prompt, str), eeLoc, eeLen));
}

char updateEE(const char *prompt, char eeLoc, char eeLen)
{
 printf(F3("%s"), prompt);
 readEE(cmdBuffer, eeLoc, eeLen);
 printf(F3(" %s "), cmdBuffer);
 memset(cmdBuffer, 0, eeLen);
 char len = readStr(cmdBuffer, eeLen);
 printf(F3("len %d\n"), len);
 if (len > 0)
 {
  if (len < eeLen)
   len++;
  char *p = cmdBuffer;
  char addr = eeLoc;
  while (--len >= 0)
  {
   EEPROM.write(addr, *p);
   addr++;
  }
  return(1);
 }
 return(0);
}

void dumpBuf(unsigned char *p, unsigned int len)
{
#define MAX_COL 16
 char col = 0;
 for (unsigned int i = 0; i < len; i++)
 {
  if (col == 0)		/* if column 0 */
  {
   printf(F0("%04x  "), (int) p);
  }
  int val = *p++ & 0xff;
  printf(F0("%02x "), val);	/* output value */
  col += 1;			/* count a column */
  if (col == MAX_COL)	/* if at end of line */
  {
   col = 0;			/* reset column counter */
   newLine();
  }
 }
 if (col != 0)
  newLine();
}

void bufferCheck(const char *name, unsigned char *buf, int size)
{
 int val = size;
 unsigned char *p = buf + size;
 for (int i = 0; i < size; i++)
 {
  if (*--p != 0)
  {
   val = size - i;
   break;
  }
 }
 char tmpBuf[16];
 printf(argConv(F("%-6s %3d %3d\n"), tmpBuf), name, val, size);
 if (0)
  dumpBuf(buf, size);
}

void checkBuffers()
{
 printf(F3("RAMEND %x sp %x noinit_end %x free %d of %d\n"),
	RAMEND, SP, &__noinit_end, SP - (int) (&__noinit_end),
	RAMEND - (int) (&__noinit_end));

 bufferCheck(F0("string"), (unsigned char *) stringBuffer, sizeof(stringBuffer));
 bufferCheck(F0("data"),   (unsigned char *) dataBuffer, sizeof(dataBuffer));
 bufferCheck(F0("cmd"),    (unsigned char *) cmdBuffer, sizeof(cmdBuffer));
 bufferCheck(F0("rsp"),    (unsigned char *) packetRsp, sizeof(packetRsp));

#if !defined(PRINT_STACK) | (PRINT_STACK == 0)
 unsigned int len = SP - (int) &__noinit_end;
 unsigned char *p = ((unsigned char *) &__noinit_end);
 for (unsigned int i = 0; i < len; i++)
 {
  if (*p != 0)
  {
   printf(F3("unused stack %d\n"), (int) (p - &__noinit_end));
   break;
  }
  p += 1;
 }
#else
 dumpBuf(p, len);
#endif	/* PRINT_STACK */
}

#endif	/* ARDUINO_ARCH_AVR */

#if defined(ARDUINO_ARCH_STM32)

extern __IO uint32_t uwTick;

uint16_t intMillis()
{
 return((uint16_t) uwTick);	/* return value */
}

#endif	/* ARDUINO_ARCH_STM32 */

/* setup routine */

#if CURRENT_SENSOR

T_TIMER_CTL tmr3;

void showTimer(P_TIMER_CTL tmr)
{
 P_PORT p = tmr->aPort;
 if (p != 0)
  printf(F0("aPort %02x aDDR %02x aMask %02x "),
	 p->port, p->ddr, tmr->aMask);

 P_TMR t = tmr->timer;
 printf("tccra %02x tccrb %02x tccrc %02x\n",
	t->tccra, t->tccrb, t->tccrc);
 printf(F0("tcnt %04x ocra %04x ocrb %04x ocrc %04x icr %04X\n"),
	t->tcnt, t->ocra, t->ocrb, t->ocrc, t->icr);
 printf(F0("timsk %02x tifr %02x\n"),
	*tmr->timsk, *tmr->tifr);
}

#endif	/* CURRENT_SENSOR */

#define TRACE_OFFSET 3

void trace()
{
#if defined(ARDUINO_AVR_MEGA2560)
 int i = dbgData.i;
 unsigned char *sp = (unsigned char *) SP;
 unsigned char *p = (unsigned char *) &dbgData.trace[i];
 *(p + 0) = *(sp + TRACE_OFFSET);
 *(p + 1) = *(sp + TRACE_OFFSET - 1);
 i += 1;
 if (i >= TRACE_SIZE)
  i = 0;
 dbgData.i = i;
#endif	/* ARDUINO_AVR_MEGA2560 */
}

void setup()
{
 char ch;

#if defined(ARDUINO_ARCH_AVR)

//#if defined(ARDUINO_AVF_PRO)
 wdt_enable(WDT_TO);
//#endif	/* ARDUINO_AVR_PRO */

#if defined(ARDUINO_AVR_MEGA2560)
 noInterrupts();
 WDTCSR = _BV(WDCE) | _BV(WDE);
 WDTCSR = _BV(WDE) | _BV(WDIE) | _BV(WDP3) | _BV(WDP2) | _BV(WDP1) | _BV(WDP0);
 interrupts();

 DBGPORT.begin(19200);
#endif	/* ARDUINO_AVR_MEGA2560 */

#if defined(ARDUINO_AVR_PRO)
 DBGPORT.begin(9600);
#endif	/* ARDUINO_AVR_PRO */

#if PRINTF
 init_printf(NULL, putx1);
#endif	/* PRINTF */

#if !PRINTF
 fdev_setup_stream(&uartout, putx, NULL, _FDEV_SETUP_WRITE);
 stdout = &uartout;
#endif	/* !PRINTF */

#endif  /* ARDUINO_ARCH_AVR */

#if defined(ARDUINO_ARCH_STM32)
 __HAL_RCC_WWDG_CLK_DISABLE();
 DBGPORT.begin(19200);
#endif	/* ARDUINO_ARCH_STM32 */

#define addr(x) (2 * (unsigned int) &x)
#define daddr(x) (unsigned int) &x
 
 if (DBG)
 {
#if defined(ARDUINO_ARCH_AVR)
  printf(F3("\nmcusr %x wdtcsr %02x\n"), MCUSR, WDTCSR);
  checkBuffers();
#if defined(ARDUINO_AVR_MEGA2650)
  printf(F3("setup %04x loop %04x cmdLoop %04x\n"),
	 addr(setup), addr(loop), addr(cmdLoop));
  printf(F3("__noinit_start %04x __noinit_end %04x __heap_start %04x\n"),
	 daddr(__noinit_start), daddr(__noinit_end), daddr(__heap_start));
  
  if (dbgData.trace[0] != 0)
  {
   printf(F3("adcFlag %d\n"), dbgData.adcFlag);
   unsigned int index = dbgData.i;
   if (index < TRACE_SIZE)
   {
    char col = 0;
    int count = 0;
    for (int i = 0; i < TRACE_SIZE; i++)
    {
     unsigned int pc = dbgData.trace[index] * 2;
     index += 1;
     if (index >= TRACE_SIZE)
      index = 0;
     if (pc != 0)
     {
      if (col == 0)
       printf(F3("%02x  "), count);
      printf(F3("%04x "), pc);
      col += 1;
      count += 1;
      if (col == 16)
      {
       col = 0;
       newLine();
      }
     }
    }
    if (col != 0)
     newLine();
   }
   printf(F3("i %3d index %3d\n"), dbgData.i, index);
  }

  if (MCUSR & _BV(WDRF))
  {
   newLine();

#if defined(ARDUINO_AVR_MEGA2560)
   printf(F3("wdt pc %06lx\n"), wdtData.pc);
#endif	/* ARDUINO_AVR_MEGA */
#if defined(ARDUINO_AVR_PRO)
   printf(F3("wdt pc %04x\n"), wdtData.pc);
#endif	/* ARDUINO_AVR_PRO */

   dumpBuf((unsigned char *) &wdtData, sizeof(wdtData));
  }
  MCUSR = 0;
#endif	/* ARDUINO_AVR_MEGA2560 */

#else
  printf(F3("\nstarting 0\n"));
#endif	/* ARDUINO_ARCH_AVR */
 }

 memset(stringBuffer, 0, sizeof(stringBuffer));
 memset(dataBuffer, 0, sizeof(dataBuffer));
 memset(cmdBuffer, 0, sizeof(cmdBuffer));
 memset(packetRsp, 0, sizeof(packetRsp));

#if defined(ARDUINO_ARCH_AVR)
 int len = (int) SP - 16 - (int) &__bss_end;
 memset((void *) (&__bss_end), 0, len);
#endif	/* ARDUINO_ARCH_AVR */

#if defined(LCD_ENA)
 while (lcd.begin(COLUMS, ROWS, LCD_5x8DOTS) != 1)
 {
  printf("lcd not connected\n");
  delay(5000);
 }
 i2cInfo(I2C1, "I2C1");
 rccInfo();
 lcd.print("PCF8574 is OK...");
#endif	/* LCD_ENA */

#if defined(OLED_ENA)

 //u8g2.clearBuffer();		 /* clear the internal memory */
 //u8g2.setFont(u8g2_font_5x8_tr); /* choose a suitable font */
 u8x8.begin();
 u8x8.setPowerSave(0);
 u8x8.setFont(u8x8_font_chroma48medium8_r);
 
#endif	/* OLED_ENA */

#if DBG0_Pin
 pinMode(DBG0_Pin, OUTPUT);
 dbg0Clr();
#endif /* DBG0_Pin */

#if DBG1_Pin
 pinMode(DBG1_Pin, OUTPUT);
 dbg1Clr();
#endif /* DBG1_Pin */

#if DBG2_Pin
 pinMode(DBG2_Pin, OUTPUT);
 dbg2Clr();
#endif /* DBG2_Pin */

#if DBG3_Pin
 pinMode(DBG3_Pin, OUTPUT);
 dbg3Clr();
#endif /* DBG3_Pin */

#if DBG4_Pin
 pinMode(DBG4_Pin, OUTPUT);
 dbg4Clr();
#endif /* DBG4_Pin */

#if DBG5_Pin
 pinMode(DBG5_Pin, OUTPUT);
 dbg5Clr();
#endif /* DBG5_Pin */

#if DBG6_Pin
 pinMode(DBG6_Pin, OUTPUT);
 dbg6Clr();
#endif /* DBG6_Pin */

#if DBG7_Pin
 pinMode(DBG7_Pin, OUTPUT);
 dbg7Clr();
#endif /* DBG7_Pin */

#if WATER_MONITOR
 pinMode(LED, OUTPUT);
 pinMode(WATER0, INPUT);
 pinMode(WATER1, INPUT);
 digitalWrite(BEEPER, LOW);
 pinMode(BEEPER, OUTPUT);

 digitalWrite(LED, LOW);

 water0.inp = 0xff;
 water0.state = STATE_CLEAR;
 water0.index = 0;

 water1.inp = 0xff;
 water1.state = STATE_CLEAR;
 water1.index = 1;
#endif  /* WATER_MONITOR */

#if CHECK_IN
 memset(&serverIP, 0, sizeof(serverIP));
 serverIPTime = millis() - SERVER_IP_TIMEOUT;
 failCount = 0;
#endif /* CHECK_IN */
 
 memset(&ntpIP, 0, sizeof(ntpIP));

#if defined(ARDUINO_ARCH_AVR)
// uint32_t checksum = sumEE();
 sumEE();
 uint32_t csum;
 readEE((char *) &csum, CSUM_LOC, CSUM_LEN);
// if (checksum != csum)
 {
  if (DBG)
   printf(F3("init eeprom with default\n"));
#if INITEE
  writeEE(F0(SSID), SSID_LOC, SSID_LEN);
  writeEE(F0(PASS), PASS_LOC, PASS_LEN);
  writeEE(F0(MONITOR_ID), ID_LOC, ID_LEN);
#if defined(EMONCMS_ADDR)
  writeEE(F0(EMONCMS_ADDR), IP_LOC, IP_LEN);
#endif	/* EMONCMS_ADDR */
#endif  /* INITEE */
  writeSumEE();
 }

 readEE(id, ID_LOC, ID_LEN);
 printf(F0("MONITOR_ID %s id %s\n"), MONITOR_ID, id);
#if defined(EMONCMS_ADDR)
 readEE(emonIP, IP_LOC, IP_LEN);
#endif	/* EMONCMS_ADDR */

 printf(F3("monitor id %s\n"), id);
#else
 strcpy(id, MONITOR_ID);
 strcpy(emonIP, EMONCMS_ADDR);
#endif /* ARDUINO_ARCH_AVR */

#if MEGA32
 init_printf(NULL, putx0);
#endif	/* MEGA32 */

 if (DBG)
  printf(F3("\nstarting 1\n"));

#if TEMP_SENSOR
#if TEMP_SENSOR == 1
 printf(F3("start temp sensor bus %d\n"), ONE_WIRE_BUS);
 sensors.begin();
 for (unsigned char i = 0; i < TEMPDEVS; i++)
 {
  sensors.setResolution(tempDev[i], 12);
  lastTemp[i] = 0.0;
 }
#elif TEMP_SENSOR == 2
 P_TEMP_SENSOR ts = tempSensor;
 for (uint8_t j = 0; j < TEMP_SENSOR; j++)
 {
  char deviceCount = ts->deviceCount;
  DeviceAddress *addr = ts->tempDev;
  float *lastTemp = ts->lastTemp;
  DallasTemperature *sensor = ts->sensor;
  printf(F3("start temp sensor bus %d\n"), ts->oneWireBus);
  sensor->begin();
  for (unsigned char i = 0; i < deviceCount; i++)
  {
   sensor->setResolution((const uint8_t *) addr, 12);
   *lastTemp++ = 0.0f;
   addr += 1;
  }
  ts += 1;
 }
#endif	/* TEMP_SENSOR == 2 */
#endif	/* TEMP_SENSOR */
 
#if DHT_SENSOR
 dht.begin();
#if DEHUMIDIFIER
 dehumOn = 63.0;
 dehumOff = 60.5;
 pinMode(DEHUM_ON_PIN, OUTPUT);
 pinMode(DEHUM_OFF_PIN, OUTPUT);
 digitalWrite(DEHUM_ON_PIN, LOW);
 switchRelay(DEHUM_OFF_PIN);
#endif	/* DEHUMIDIFIER */
#endif	/* DHT_SENSOR */

#if defined(ARDUINO_AVR_MEGA2560)
 pinMode(51, OUTPUT);		/* pg0 */
 pinMode(52, OUTPUT);		/* pg1 */
 PORTG &= ~(_BV(PG0) | _BV(PG1));

 PORTG |= _BV(PG0);
 delay(2);
 PORTG &= ~_BV(PG0);
#endif	/* ARDUINO_AVR_MEGA2560 */

#if defined(WIFI_ENA)

 wifiInitSio();			/* enable wifi serial port */
#if defined(WIFI_RESET)
 pinMode(WIFI_RESET, OUTPUT);	/* set wifi reset pin to output */
 digitalWrite(WIFI_RESET, HIGH); /* set it high */
 delay(10);			/* short wait */

 wifiReset();			/* reset wifi */
#endif	/* WIFI_RESET */

#if defined(DBG0_Pin)
 pinMode(DBG0_Pin, OUTPUT);
 dbg0Clr();
#endif	/* DBG0_PIN */

#if defined(DBG1_Pin)
 pinMode(DBG1_Pin, OUTPUT);
 dbg1Clr();
#endif	/* DBG1_PIN */

#if defined(DBG2_Pin)
 pinMode(DBG2_Pin, OUTPUT);
 dbg2Clr();
#endif	/* DBG2_PIN */

 wifiWriteStr(F2("AT+CWQAP"), 1000);
 delay(100);
 
 char retry = RETRY_JOIN;
 while (1)
 {
  char result = wifiJoin();
  if (result)
  {
   wifiWriteStr(F2("AT+CIFSR"), 1000);
   break;
  }
  --retry;
  if (retry <= 0)
  {
   retry = RETRY_JOIN;
   wifiReset();
   delay(500);
  }
 }

#if ESP8266_TIME == 0
 ntpStart = millis();
 ntpTimeout = 0;
#else
 esp8266TimeEnable();
#endif /* ESP8266_TIME */

#endif	/* WIFI_ENA */

#if defined(ARDUINO_ARCH_STM32)

 printf("STM32 MONITOR_INDEX %d MONITOR_ID %s EMONCMS_NODE %s\n",
	MONITOR_INDEX, MONITOR_ID, EMONCMS_NODE);

#if defined(CURRENT_STM32)
 printf("MAX_CHAN %d MAX_CHAN_POWER %d MAX_CHAN_RMS %d\n",
	MAX_CHAN, MAX_CHAN_POWER, MAX_CHAN_RMS);
#endif	/* CURRENT_STM32 */

#if DATA_SIZE
 unsigned int bss = (unsigned int) (&_ebss - &_sbss);
 unsigned int data = (unsigned int) (&_edata - &_sdata);
 printf("data %u bss %u total %u\n", data, bss, data + bss);
 printf("stack %08x sp %08x\n",
	(unsigned int) &_estack, getSP());
#endif	/* DATA_SIZE */

 printf("AFIO MAPR %08x\n", (unsigned int) AFIO->MAPR);
 AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;
 printf("AFIO MAPR %08x\n", (unsigned int) AFIO->MAPR);

 unsigned int clockFreq = HAL_RCC_GetHCLKFreq();
 unsigned int FCY = HAL_RCC_GetPCLK2Freq();
 printf("clock frequency %u FCY %u\n", clockFreq, FCY);
 printf("sysTick load %d\n", (int) SysTick->LOAD);

 printf("initialize adc\n");
 flush();

#if defined(STM32F103xB)
 __HAL_RCC_USART1_CLK_DISABLE();

 /*
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
 */

 HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

 MX_GPIO_Init();
 MX_ADC1_Init();
 ADC_MspInit(&hadc1);
 MX_ADC2_Init();
 ADC_MspInit(&hadc2);
#else
 MX_GPIO_Init();
 MX_ADC1_Init();
 ADC_MspInit(&hadc1);
 MX_ADC2_Init();
 ADC_MspInit(&hadc2);
 MX_DMA_Init();
 MX_TIM1_Init();
 HAL_TIM_Base_MspInit(&htim1);
// HAL_TIM_MspPostInit(&htim1);
#endif	/* STM32F103xB */

 printf("adc initialization complete\n");
 flush();

#endif	/* ARDUINO_ARCH_STM32 */
  
#if defined(CURRENT_STM32)
 adcRun();
#endif	/* CURRENT_STM32 */

#if DBG
 printf(F3("debug mode\n"));
 unsigned int t = intMillis();
 while ((unsigned int) (intMillis() - t) < 1000U)
 {
  ch = DBGPORT.read();
 }

 t = intMillis();
 ch = 0;
#if !defined(WIFI_ENA)
 printf(F3("**wifi not enabled**\n"));
#endif	/* WIFI_ENA */
 printf(F3("any char for cmd mode..."));
 flush();
 newLine();
 while ((unsigned int) (intMillis() - t) < 5000U)
 {
  wdt_reset();
  if (DBGPORT.available())
  {
   ch = DBGPORT.read();
   newLine();
   break;
  }
 }

 if (ch != 0)
 {
  cmdLoop();
  printf(F3("end debug mode\n"));
 }
#endif /* DBG */

 setTime();

#if !defined(monDbg)
 monDbg = MON_DBG;
#endif	/* monDdb */
#if !defined(wifiDbg)
 wifiDbg = WIFI_DBG;
#endif	/* wifiDbg */
 printf(F3("monDbg %d wifiDbg %d\n"), monDbg, wifiDbg);

#if CURRENT_SENSOR
 trace();
 initCurrent(1);		/* initial current sensor */
#endif	/* CURRENT_SENSOR */

 tLast = intMillis();		/* initialize loop timer */
}

void setTime()
{
 trace();
#if RTC_CLOCK
 if (monDbg)
 {
  printf(F3("setTime rtc time "));
  printTime(RTC.get());
 }
#endif	/* RTC_CLOCK */

 if ((millis() - ntpStart) > ntpTimeout)
 {
  char status = 0;

#if defined(WIFI_ENA)
#if (ESP8266_TIME == 0)
  for (char retry = 0; retry < 3; retry++)
  {
   status = ntpSetTime();	/* look up ntp time */
   if (status != 0)
    break;
   unsigned int t = intMillis();
   while ((unsigned int) (intMillis() - t) < 2000U)
    ;
   printf(F3("retry ntpSetTime\n"));
  }
#else
  status = esp8266Time();
#endif /* ESP8266_TIME */
#endif /* WIFI_ENA */

  if (status)			/* if valid time found */
  {
#if RTC_CLOCK
   RTC.set(now());		/* set the clock */
   setSyncProvider(RTC.get);	/* set rtc to provide clock time */
#endif	/* RTC_CLOCK */
  }
#if RTC_CLOCK
  else
  {
   if (monDbg)
    printf(F3("rtc setting time\n"));
   setTime(RTC.get());
   setSyncProvider(RTC.get);	/* set rtc to provide clock time */
  }
#endif	/* RTC_CLOCK */
  if (monDbg)
  {
   printf(F3("time set "));
   printTime();
  }
 }
}

#if defined(ARDUINO_ARCH_STM32)
char prompt(const char *str)
{
 char ch;
 
 if (str != 0)
 {
  printf(str);
  flush();
 }
 while (DBGPORT.available() == 0)
  ;
 ch = DBGPORT.read();
 DBGPORT.print(ch);
 newLine();
 return(ch);
}
#endif	/* ARDUINO_ARCH_STM32 */

#ifdef ARDUINO_AVR_MEGA2560

asm volatile(
 ".global getPC\n\t"
 "getPC:\n\t"
 "pop 23\n\t"
 "pop 25\n\t"
 "pop 24\n\t"
 "push 24\n\t"
 "push 25\n\t"
 "push 23\n\t"
 "ret\n\t"
 );

#endif	/* ARDUINO_AVR_MEGA2560 */

#ifdef ARDUINO_AVR_PRO
	
asm volatile(
 "getPC:\n\t"
 "pop r25\n\t"
 "pop r24\n\t"
 "push r24\n\t"
 "push r25\n\t"
 "ret\n\t"
 );
	
#endif	/* ARDUINO_AVR_PRO */

void cmdLoop()
{
 wdt_disable();
 printf(F3("command loop\n"));
 flush();
 while (1)
 {
  if (DBGPORT.available())
  {
   flush();
   char ch = DBGPORT.read();
   DBGPORT.write(ch);
   newLine();
   if (ch == 'x')		/* exit command loop */
    break;
#if defined(EMONCMS_NODE)
   else if (ch == '?')		/* file name */
   {
    printf(F3("monitor.cpp emon " EMONCMS_NODE " id " MONITOR_ID "\n"));
   }
#endif	/* EMONCMS_NODE */
   
#if defined(LCD_ENA)
   else if (ch == 'L')
   {
    while (lcd.begin(COLUMS, ROWS, LCD_5x8DOTS) != 1)
    {
     printf("lcd not connected\n");
     delay(5000);
    }
    i2cInfo(I2C1, "I2C1");
    rccInfo();
    lcd.print("PCF8574 is OK...");
   }
   else if (ch == 'M')
   {
    lcdInit();
   }
#endif	/* LCD_ENA */
   
#if defined(ARDUINO_ARCH_STM32)
   else if (ch == 't')		/* thermocouple commands */
   {
    while (1)
    {
     printf("thermocouple: ");
     flush();
     char ch = DBGPORT.read();
     DBGPORT.write(ch);
     newLine();
     if (ch == 'i')
     {
      max56Init(MX56_TCTYPE_K, MX56_ONESHOT);
     }
     else if (ch == 't')
     {
      char buf[10];
      printf("temp %s\n", max56FmtTemp(buf, sizeof(buf)));
     }
     else if (ch == 'x')
     {
      break;
     }
    }
   }
#endif	/* ARDUINO_ARCH_STM32 */
   
#if defined(ARDUINO_ARCH_AVR)
   else if (ch == 'F')		/* arduino check buffers */
   {
    checkBuffers();
   }
   else if (ch == 'w')		/* arduino write ssid and password to eeprom */
   {
    char flag = updateEE(F1("ssid"), SSID_LOC, SSID_LEN);
    flag |= updateEE(F1("pass"), PASS_LOC, PASS_LEN);
    flag |= updateEE(F1("id"), ID_LOC, ID_LEN);
    flag |= updateEE(F1("emonIp"), IP_LOC, IP_LEN);
    if (flag)
     writeSumEE();
    readEE(id, ID_LOC, ID_LEN);
    readEE(emonIP, IP_LOC, IP_LEN);
   }
   else if (ch == 'W')		/* arduino test watchdog timer */
   {
    printf(F3("test watchdog timer\n"));

    wdt_enable(WDT_TO);
    noInterrupts();
    WDTCSR = _BV(WDCE) | _BV(WDE);
    WDTCSR = _BV(WDE) | _BV(WDIE) | _BV(WDP2) | _BV(WDP1) | _BV(WDP0);
    interrupts();

    printf(F3("mcusr %02x wdtcsr %02x "), MCUSR, WDTCSR);
#if defined(ARDUINO_AVR_MEGA2560)
    printf(F3("PC %04x\n"), 2 * getPC());
#endif	/* ARDUINO_AVR_MEGA2560 */
#if defined(ARDUINO_AVR_PRO)
    printf(F3("PC %04x\n"), 2 * getPC());
#endif	/* ARDUINO_AVR_PRO */
    DBGPORT.flush();
    dbg2Set();
    while (1)
     ;
   }
#endif  /* ARDUINO_ARCH_AVR */

#if defined(ARDUINO_AVR_MEGA2560)
   else if (ch == 'p')		/* mega2560 write to port g */
   {
    if (getNum())
    {
     unsigned char tmp = PORTG;
     PORTG = (char) val;
     printf(F3("\nportg %x %x\n"), tmp, PORTG);
    }
   }
   else if (ch == 'k')		/* mega2560 write to port g */
   {
    if (getNum())
    {
     unsigned char tmp = PORTK;
     PORTK = (unsigned char) val;
     printf(F3("\nddrk %02x portk %02x %02x\n"), DDRK, tmp, PORTK);
    }
   }
#endif	/* ARDUINO_AVR_MEGA2560 */

#if defined(CURRENT_STM32)
   else if (ch == 'C')		/* stm32 adc status */
   {
    currentCmds();
   }
#endif	/* CURRENT_STM32 */

#if defined(ARDUINO_ARCH_STM32)
   else if (ch == 'U')
    max65Cmds();
   else if (ch == 'V')
    max56Cmds();
#endif	/* ARDUINO_ARCH_STM32 */

#if CURRENT_SENSOR
   else if (ch == '*')
   {
    showTimer(&tmr3);
   }
   else if (ch == 'e')		/* arduino current read a to d converter */
   {
    printf(F3("chan: "));
    char len = readStr(cmdBuffer, sizeof(cmdBuffer) - 1);
    unsigned char chan = 0;
    if (len != 0)
    {
     chan = atoi(cmdBuffer);
     if (chan >= ADCCHANS)
      chan = ADCCHANS - 1;
    }
    P_CURRENT p = &iData[chan];
    
    printf(F3("value: "));
    len = readStr(cmdBuffer, sizeof(cmdBuffer) - 1);
    printf(F3("lastTime %ld len %d cState %d\n"), p->lastTime, len, cState);
    if (len != 0)
    {
     p->lastIRms1 = p->lastIRms0;
     p->lastIRms0 = atof(cmdBuffer);
     p->lastTime = now();
     printTime(p->lastTime);
    }
   }
   else if (ch == 'C')		/* arduino run current check code */
   {
    printCurrent();
    currentCheck();
   }
   else if (ch == 'T')		/* arduino setup current sensor isr */
   {
    printf(F3("isr: "));
    char len = readStr(stringBuffer, sizeof(stringBuffer) - 1);
    initCurrent(len);
   }
   else if (ch == 'I')		/* arduino print current results */
   {
    char tmp[12];
    for (unsigned char i = 0; i < ADCCHANS; i++)
    {
     P_CURRENT p = &iData[i];
     printf(F3("iRms %s\n"), dtostrf(p->iRms, 4, 2, tmp));
     printf(F3("iRatio %s\n"), dtostrf(p->iRatio, 8, 6, tmp));
     printf(F3("node %d count %d sent %d adc %02x\n"),
	    p->node, p->count, p->sent, p->adc);
     if (p->send != 0)
     {
      p->lastTime = now();
      printTime(p->lastTime);
      printf(F3("lastIRms0 %s\n"), dtostrf(p->lastIRms0, 4, 2, tmp));
      printf(F3("offset %s\n"), dtostrf(p->offsetI, 4, 2, tmp));
      p->send = 0;
     }
    }
   }
#endif  /* CURRENT_SENSOR */

   else if (ch == 'u')		/* test long print */
   {
    long tmp = 0x55aa55aa;
    DBGPORT.print(tmp, 16);
    DBGPORT.println();
    printf(F3("%lx\n"), tmp);
   }

#if CHECK_IN | WATER_MONITOR
   else if (ch == 'L')		/* loop water */
   {
#if CHECK_IN
    checkIn();
#endif  /* CHECK_IN */
#if WATER_MONITOR
    loopWater();
#endif  /* WATER_MONITOR */
   }
#endif  /* CHECK_IN | WATER_MONITOR */

   else if (ch == 'j')		/* join wifi */
   {
    wifiJoin();
   }
   else if (ch == 'o')		/* send at */
   {
    wifiWriteStr(F2("AT"), 1000);
   }
   else if (ch == 'l')		/* list access points */
   {
    wifiWriteStr(F2("AT+CWLAP"), 3000);
   }
   else if (ch == 'q')		/* disconnect */
   {
    wifiWriteStr(F2("AT+CWQAP"), 1000);
   }
   else if (ch == 's')		/* get local ip address */
   {
    wifiWriteStr(F2("AT+CIFSR"), 1000);
   }
   else if (ch == 'u')		/* start udp connection */
   {
    wifiWriteStr(F2("AT+CIPSTART=\"UDP\",\"129.6.15.28\",123"), 3000);
   }
   else if (ch == 'm')		/* wifi mux */
   {
    wifiMux();
   }
   else if (ch == 't')		/* start tcp connection */
   {
    wifiWriteStr(F2("AT+CIPSTART=4,\"TCP\",\"184.106.153.149\",80"), 4000);
   }
   else if (ch == 'z')		/* close wiif */
   {
    wifiClose(4, 15000);
   }
   else if (ch == 'c')		/* enter wifi command */
   {
    printf(F3("enter command\n"));
    char len = readStr(dataBuffer, sizeof(dataBuffer));
    if (len != 0)
    {
     printf(F3("send command\n"));
     wifiClrRx();
     wifiWrite(dataBuffer, len, 2000);
     wifiTerm();
     newLine();
    }
   }
   else if (ch == 'd')		/* set time from ntp */
   {
    ntpTimeout = 0;
    ntpStart = millis() - 1;
    setTime();
   }

#if RTC_CLOCK
   else if (ch == 'v')		/* read rtc temp */
   {
    rtcTemp();
   }
#endif  /* RTC_CLOCK */

#if DHT_SENSOR
   else if (ch == 'h')		/* read humidity sensor */
   {
    float h = dht.readHumidity();
    if (!isnan(h))
    {
     float t = dht.readTemperature(true);
     if (!isnan(t))
     {
      printf(F3("temp "));
      printTemp(t);
      printf(F3(" F humidity "));
      printTemp(h);
      newLine();
     }
     else
      printf(F3("temp read failure\n"));
    }
    else
     printf(F3("humidity read failure\n"));
   }
#endif  /* DHT_SENSOR */

#if DEHUMIDIFIER
   else if (ch == 'r')		/* set dehumidifier relay */
   {
    if (getNum())
    {
     if (val == 0)
     {
      printf(F3("turn relay off\n"));
      switchRelay(DEHUM_OFF_PIN);
     }
     else
     {
      printf(F3("turn relay on\n"));
      switchRelay(DEHUM_ON_PIN);
     }
    }
   }
#endif	/* DEHUMIDIFIER */

#if TEMP_SENSOR
   else if (ch == 'f')		/* find temp sensor addresses */
   {
    findAddresses();
   }
   else if (ch == 'y')		/* read temp sensors */
   {
#if TEMP_SENSOR == 1
    sensors.requestTemperatures();
    for (unsigned char i = 0; i < TEMPDEVS; i++)
    {
     float temp = sensors.getTempF(tempDev[i]);
     printf(F3("temp "));
     DBGPORT.print(temp);
     printf(F3("F\n"));
    }
#elif TEMP_SENSOR == 2
    P_TEMP_SENSOR ts = tempSensor;
    for (uint8_t j = 0; j < TEMP_SENSOR; j++)
    {
     char deviceCount = ts->deviceCount;
     DeviceAddress *addr = ts->tempDev;
     DallasTemperature *sensor = ts->sensor;
     sensor->requestTemperatures();
     for (unsigned char i = 0; i < deviceCount; i++)
     {
      float temp = sensor->getTempF((const uint8_t *) addr);
      printf(F3("temp "));
      DBGPORT.print(temp);
      printf(F3("F\n"));
      addr += 1;
     }
     ts += 1;
    }
#endif	/* TEMP_SENSOR == 2 */
   }
   else if (ch == 'g')		/* run loopTemp() */
   {
    loopTemp();
   }
#endif  /* TEMP_SENSOR */
  }
  
#if defined(CURRENT_STM32)
  if (pwrActive)
  {
   powerUpdate();
  }
#endif	/* CURRENT_STM32 */
 }
#if defined(ARDUINO_ARCH_AVR)
 wdt_enable(WDT_TO);

#if defined(ARDUINO_AVR_MEGA2560)
 noInterrupts();
 WDTCSR = _BV(WDCE) | _BV(WDE);
 WDTCSR = _BV(WDE) | _BV(WDIE) | _BV(WDP3) | _BV(WDP2) | _BV(WDP1) | _BV(WDP0);
 interrupts();
#endif	/* ARDUINO_AVR_MEGA2560 */

#endif /* ARDUINO_ARCH_AVR */
} /* end cmdloop */

void loop()
{
 trace();
 unsigned int tPrev = intMillis(); /* init time for short interval */
 while (1)			/* wait for end of interval */
 {
  wdt_reset();
  if (DBGPORT.available())
  {
   DBGPORT.print('>');
   char ch = DBGPORT.read();
   DBGPORT.print(ch);
   DBGPORT.flush();
   if (ch == 'C')
   {
    while (DBGPORT.available())
     ch = DBGPORT.read();
    newLine();
   
    cmdLoop();
   }
   newLine();
  }
 
  unsigned int t0 = intMillis(); /* read time */
  if ((unsigned int) (t0 - tLast) > TINTERVAL) /* if long interval up */
  {
   tLast = t0;			/* update previous time */
   break;			/* exit to interval processing */
  }

  delay(100);			/* wait a while */
  t0 = intMillis();		/* read time */
  if ((unsigned long) (t0 - tPrev) >= T1SEC) /* if short interval up */
  {
   tPrev = t0;			/* update previous time */
#if WATER_MONITOR
   alarmPoll();			/* poll water alarm */
#endif	/* WATER_MONITOR */

#if CURRENT_SENSOR
   currentCheck();		/* check and send current */
#endif	/* CURRENT_SENSOR */

#if defined(Led_Pin)
   ledToggle();
#endif	/* Led_Pin */
  }

#if defined(OLED_ENA)
  char buf[2];
  buf[0] = loopCount + '0';
  buf[1] = 0;
  //u8g2.drawStr(0, 8, buf);  // write something to the internal memory
  //u8g2.sendBuffer();	    // transfer internal memory to the display
  u8x8.drawString(0, 1, buf);
  u8x8.refreshDisplay();	// only required for SSD1606/7  
#endif	/* OLED_ENA */
 
#if defined(CURRENT_STM32)
  powerUpdate();
#endif	/* CURRENT_STM32 */

 } /* short interval */

 if (monDbg)
 {
  printf(F3("%d "), loopCount);
  printTime();
  flush();
 }

#if CURRENT_SENSOR
 if (1 || monDbg)
  printCurrent();
#endif	/* CURRENT_SENSOR */

#if TEMP_SENSOR | DHT_SENSOR
 if (loopCount == TEMP_COUNT)	/* if time for temperature reading */
 {
  loopTemp();
 }
#endif	/* TEMP_SENSOR | DHT_SENSOR */

#if WATER_MONITOR
 if (loopCount == WATER_COUNT)	/* if time to check water alarm */
 {
  digitalWrite(LED, LOW); 	/* turn off led */
  loopWater();			/* loop processing */
 }
#endif	/* WATER_MONITOR */

#if CHECK_IN
 if (loopCount == CHECKIN_COUNT) /* if time to check water alarm */
 {
#if defined(ARDUINO_ARCH_AVR)
  if (monDbg)
   printf(F3("mcusr %02x wdtcsr %02x\n"), MCUSR, WDTCSR);
#endif	/* ARDUINO_ARCH_AVR */
  checkIn();
 }
#endif /* CHECK_IN */

 if (loopCount == NTP_COUNT)	/* if time to set time */
 {
  setTime();
 }

 loopCount++;			/* update loop counter */
 if (loopCount >= LOOP_MAX)	/* if at maximum */
 {
#if defined(ARDUINO_ARCH_AVR)
  if (monDbg)
   checkBuffers();
#endif	/* ARDUINO_ARCH_AVR */
  loopCount = 0;		/* reset to beginning */
 }

} /* *end loop */

#if DEHUMIDIFIER
void switchRelay(char pin)
{
 digitalWrite(pin, HIGH);	/* turn relay on */
 delay(100);
 digitalWrite(pin, LOW);	/* turn relay off */
}
#endif	/* DEHUMIDIFIER */

#if TEMP_SENSOR | DHT_SENSOR

void loopTemp()
{
#if TEMP_SENSOR
 char count;
#if TEMP_SENSOR == 1
 float temp0[TEMPDEVS];
#elif TEMP_SENSOR == 2
// float temp1[TEMPDEVS];
#endif	/* TEMP_SENSOR == 2 */
 
#if TEMP_SENSOR == 1
 for (unsigned char i = 0; i < TEMPDEVS; i++)
 {
  float t;
  count = 5;
  while (1)
  {
   sensors.requestTemperatures();
   t = sensors.getTempF(tempDev[i]);
   if (t != DEVICE_DISCONNECTED_F)
   {
    lastTemp[i] = t;
    break;
   }

   --count;
   if (count == 0)
   {
    printf(F3("Error getting temperature sensor %d\n"), i);
    t = lastTemp[i];
    break;
   }
  }
  temp0[i] = t;
  if (monDbg)
  {
   printf(F3("%d temp "), i);
   DBGPORT.print(t);
   printf(F3("F\n"));
  }
 }
#elif TEMP_SENSOR == 2
 P_TEMP_SENSOR ts = tempSensor;
 for (uint8_t j = 0; j < TEMP_SENSOR; j++)
 {
  char deviceCount = ts->deviceCount;
  DallasTemperature *sensor = ts->sensor;
  DeviceAddress *addr = ts->tempDev;
  float *lastTemp = ts->lastTemp;
  float *temp = ts->temp;
  for (unsigned char i = 0; i < deviceCount; i++)
  {
   float t;
   count = 5;
   while (1)
   {
    sensor->requestTemperatures();
    t = sensor->getTempF((const uint8_t *) addr);
    if (t != DEVICE_DISCONNECTED_F)
    {
     *lastTemp++ = t;
     break;
    }
   
    --count;
    if (count == 0)
    {
     printf(F3("Error getting temperature sensor %d\n"), i);
     t = *lastTemp++;
     break;
    }
   }
   if (monDbg)
   {
    printf(F3("%d %d temp "), j, i);
    DBGPORT.print(t);
    printf(F3("F\n"));
   }
   *temp++ = t;
   addr += 1;
  }
  ts += 1;
 }
#endif	/* TEMP_SENSOR == 2 */
#endif  /* TEMP_SENSOR */

#if RTC_CLOCK
 float rtcTempVal = rtcTemp();
#endif /* RTC_CLOCK */

#if DHT_SENSOR
 float dhtHumidity = dht.readHumidity();
 float dhtTemp = dht.readTemperature(true);
 if (monDbg)
 {
  printf(F3("temp "));
  printTemp(dhtTemp);
  printf(F3(" F humidity "));
  printTemp(dhtHumidity);
  newLine();
 }
#if DEHUMIDIFIER
 if (monDbg)
  printf(F3("dehumState %d dehumDelay %d\n"), dehumState, dehumDelay);
 if (dehumState)		/* if dehumidifer on */
 {
  if (dhtHumidity <= dehumOff)	/* if humidity below turn off point */
  {
   if (dehumDelay != 0)		/* if timer active */
   {
    if (--dehumDelay == 0)	/* if counts down to zero */
    {
     dehumState = 0;		/* set state to off */
     switchRelay(DEHUM_OFF_PIN);
     if (monDbg)
      printf(F3("dehumidifier off\n"));
    }
   }
   else				/* if timer not active */
   {
    dehumDelay = DEHUM_DELAY;	/* start counter */
   }
  }
  else				/* if not below turn off point */
  {
   dehumDelay = 0;		/* reset counter */
  }
 }
 else				/* if dehumidifier off */
 {
  if (dhtHumidity >= dehumOn)	/* if humidity above turn on point */
  {
   if (dehumDelay != 0)		/* if timer active */
   {
    if (--dehumDelay == 0)	/* if counts down to zero */
    {
     dehumState = 1;		/* set state to off */
     switchRelay(DEHUM_ON_PIN); /* turn dehumidifier on */
     if (monDbg)
      printf(F3("dehumidifier on\n"));
    }
   }
   else				/* if timer not active */
   {
    dehumDelay = DEHUM_DELAY;	/* start counter */
   }
  }
  else				/* if not above turn on point */
  {
   dehumDelay = 0;		/* reset counter */
  }
 }
#endif	/* DEHUMIDIFIER */
#endif  /* DHT_SENSOR */

 char buf[128];
 char *p;

 p = cpyStr(buf, F0("node=" EMONCMS_NODE "&csv="));
#if TEMP_SENSOR
#if TEMP_SENSOR == 1
 for (unsigned char i = 0; i < TEMPDEVS; i++)
 {
  p = writeTemp(p, temp0[i]);	/* output data from each temp sensor */
  *p++ = ',';
 }
#elif TEMP_SENSOR == 2
 ts = tempSensor;
 for (uint8_t j = 0; j < TEMP_SENSOR; j++)
 {
  char deviceCount = ts->deviceCount;
  float *temp = ts->temp;
  for (unsigned char i = 0; i < deviceCount; i++)
  {
   p = writeTemp(p, *temp++);
   *p++ = ',';
  }
  ts += 1;
 }
#endif	/* TEMP_SENSOR == 2 */
#endif	/* TEMP_SENSOR */

#if RTC_CLOCK
 p = writeTemp(p, rtcTempVal);	/* output real time clock temp data */
 *p++ = ',';
#endif /* RTC_CLOCK */

#if DHT_SENSOR
 p = writeTemp(p, dhtTemp);	/* output dht sensor temp */
 *p++ = ',';
 p = writeTemp(p, dhtHumidity);	/* output dht sensor humidity */
 *p++ = ',';
#endif /* DHT_SENSOR */

 if (*(p - 1) == ',')		/* if ends with comma */
 {
  p -= 1;			/* back up pointer */
  *p = 0;			/* terminat line */
 }
 
 if (wifiDbg)
  printf(F3("emonData %s\n"), buf);
 emonData(buf);
} /* *end loopTemp */

#endif	/* TEMP_SENSOR | DHT_SENSOR */

#if CHECK_IN

char checkIn()
{
 trace();
#if WATER_MONITOR
 char state = 0;
 if (water0.state == STATE_ALARM)
  state |= 1;
 if (water1.state == STATE_ALARM)
  state |= 2;
 sprintf((char *) dataBuffer, F0("GET " SITE "/check?id=%s&st=%d"), id, state);
#else
 sprintf((char *) dataBuffer, F0("GET " SITE "/check?id=%s"), id);
#endif	 /* WATER_MONITOR */
 return(sendHTTP(dataBuffer));
}

char sendHTTP(char *data)
{
 trace();
#if LOCAL
 strncpy(serverIP, HOST, sizeof(serverIP));
#else
 char hostBuffer[sizeof(HOST)];
 unsigned long int t = millis();
 if (wifiDbg)
  printf(F0("sendHTTP ip %s time %lu\n"),
	 serverIP, (unsigned long) (t - serverIPTime));
 if ((serverIP[0] == 0) || ((t - serverIPTime) > SERVER_IP_TIMEOUT))
 {     
  if (dnsLookup(serverIP, argConv(F2(HOST), hostBuffer)))
  {
   serverIPTime = t;
  }
 }
#endif	/* LOCAL */

 if (serverIP[0] != 0)
 {
  strcat(data, F3(HTTP));
  for (char retry = 0; retry < RETRY_SEND_HTTP; retry++)
  {
   dbg0Set();
   char *p = sendData(serverIP, TCPPORT, data, 10000);
   dbg0Clr();
   if (p != 0)
   {
    for (int i = 0; i < rspCount; i++)
    {
     p = rspPtr[i];
     int dLen = rspL[i];
     *(p + dLen) = 0;
     if (find(lc(p), (char *) F0("*ok*")) >= 0)
     {
      failCount = 0;
      return(1);
     }
    }
    printf(F0("*ok* not found p %08x retry %d\n"), (unsigned int) p, retry);
    printBuf();
    printf(F3("**sendHTTP retry %d\n"), retry);
   }
   
#if DBG0_PIN
   dbg0Set();
   delay(2);
   dbg0Clr();
#endif /* DBG0_PIN */
   delay(500);
  }
 }

#if WATER_MONITOR
 updateFail();
#endif	/* WATER_MONITOR */
 return(0);
}

#endif	/* CHECK_IN */

#if WATER_MONITOR

/* called once per second to update led status from alarm inputs */

void alarmPoll()
{
 if ((digitalRead(WATER0) == STATE_ALARM) /* if either alarm */
 ||  (digitalRead(WATER1) == STATE_ALARM))
 {
  digitalWrite(LED, HIGH); 	/* turn on led */
 }
 else				/* if no alarm */
 {
  if (digitalRead(LED))		/* if led on */
   digitalWrite(LED, LOW); 	/* turn off led */
  else
   digitalWrite(LED, HIGH); 	/* turn on led */
 }
}

/* called once per minute to send alarm messages */

void loopWater()
{
 if (DBG)
  printTime();
 procAlarm(&water0, (boolean) digitalRead(WATER0));
 procAlarm(&water1, (boolean) digitalRead(WATER1));

 if (beeperOn)
 {
  if ((water0.state != STATE_ALARM)
  &&  (water1.state != STATE_ALARM))
  {
   beeperOn = 0;
   beeperCount = 0;
  }
 }
 else
 {
  if ((water0.state == STATE_ALARM)
  ||  (water1.state == STATE_ALARM))
  {
   beeperOn = 1;
   beeperCount = BEEPERCOUNT;
  }
 }

 if (beeperCount != 0)
 {
  --beeperCount;
  char tmp = BEEPS;
  while (--tmp >= 0)
  {
   digitalWrite(BEEPER, HIGH);
   delay(BEEPLEN);
   digitalWrite(BEEPER, LOW);
   delay(BEEPSPACE);
  }
 }
} /* *end loopWater */

void procAlarm(P_INPUT water, boolean inp)
{
 if (monDbg)
  printf(F3("inp %d alarm%d cur %d counter %d state %d\n"),
	 inp, water->index, water->inp, water->counter, water->state);
 if (water->inp != inp)		/* if input state changed */
 {
  water->inp = inp;		/* save state */
  water->counter =  (inp == STATE_ALARM) ? COUNT : 1; /* set counter */
 }
 else
 {
  if (water->counter != 0)	/* if state changing */
  {
   --water->counter;		/* count of change timer */
   if (water->counter == 0)	/* if state stable */
   {
    if (inp != STATE_ALARM)	/* if alarm cleared */
     beeperCount = 0;		/* stop beeper */
    
    if (water->state != inp)	/* if state changed */
    {
     water->state = inp;	/* save current state */
#if CHECK_IN
     if (!notify(water->index, inp == STATE_ALARM)) /* if notify failure */
     {
      water->counter = 1;	/* set counter to send again */
     }
#endif	/* CHECK_IN */
    }
   }
  }
 }
 if (monDbg)
  printf(F3("procAlarm done\n"));
}

#if CHECK_IN
char notify(int alarm, boolean val)
{
 sprintf((char *) dataBuffer,
	 F3("GET " SITE "/notify?id=%s&alarm=%d&val=%d"), id, alarm, val);
 return(sendHTTP(dataBuffer));
}
#endif	/* CHECK_IN */

void updateFail()
{
 if (failCount >= 3)
 {
  failCount = 0;
  printf(F3("**reset wifi\n"));
  wifiReset();
 }
 else
 {
  failCount += 1;
  printf(F3("**updateFail %d\n"), failCount);
 }
}

#endif  /* WATER_MONITOR */

char *strEnd(char *p)
{
 while (1)
 {
  char ch = *p;
  if (ch == 0)
   return p;
  p++;
 }
}

char *cpyStr(char *dst, const char *src)
{
 while (1)
 {
  char ch = *src++;
  *dst = ch;
  if (ch == 0)
   break;
  dst++;
 }
 return(dst);
}

char *writeTemp(char *buf, float temp)
{
 int tmp = (int) (temp * 10);
 int deg = tmp / 10;
 int frac = tmp % 10;
 sprintf(buf, F3("%d.%d"), deg, frac);
 char *p = buf;
 int ofs = 0;
 while (1)
 {
  if (*p == 0)
   break;
  ofs++;
  p++;
 }
 return(p);
}

#if DHT_SENSOR | TEMP_SENSOR
void printTemp(float temp)
{
 int tmp = (int) (temp * 10);
 int deg = tmp / 10;
 int frac = tmp % 10;
 printf(F3("%d.%d"), deg, frac);
}
#endif	/* DHT_SENSOR */

#if TEMP_SENSOR

#if 0
float printTemperature(DeviceAddress deviceAddress)
{
 char count = 5;
 float temp;
 while (1)
 {
  temp = sensors.getTempC(deviceAddress);
  if (temp != DEVICE_DISCONNECTED_C)
   break;
  --count;
  if (count == 0)
  {
   printf(F3("Error getting temperature\n"));
   return(DEVICE_DISCONNECTED_F);
  }
 }
 printf(F3("sensor "));
 printTemp(temp);
 printf(F3(" C "));
 temp = DallasTemperature::toFahrenheit(temp);
 printTemp(temp);
 printf(F3(" F\n"));
 return(temp);
}
#endif	/* 0 */

#endif	/* TEMP_SENSOR */

#if TEMP_SENSOR | DHT_SENSOR | CURRENT_SENSOR | CURRENT_STM32
char emonData(char *data)
{
 trace();
#if 1
 sprintf((char *) dataBuffer,
	 F3("get /emoncms/input/post.json?%s&apikey=" EMONCMS_KEY), data);
 strcat(dataBuffer, F3(HTTP1));
#else
 argConv(F(TEST_GET), dataBuffer);
#endif	/* 1 */

 if (DBG & 0)
  printf(F3("emonData len %d\n%s\n"), strlen(dataBuffer), dataBuffer);

 for (char retry = 0; retry < RETRY_EMON_DATA; retry++)
 {
  if (retry != 0)
   printf(F3("**emonData retry %d\n"), retry);
  char *p = sendData(emonIP, (const char *) dataBuffer);
  if (p != 0)
  {
   failCount = 0;
   return(1);
  }

#if DBG0_PIN
  dbg0Set();
  delay(2);
  dbg0Clr();
#endif /* DBG0_PIN */
  delay(500);
 }

#if WATER_MONITOR
 updateFail();
#endif	/* WATER_MONITOR */
 return(0);
}
#endif	/* TEMP_SENSOR | DHT_SENSOR | CURRENT_SENSOR | CURRENT_STM32 */

#if RTC_CLOCK

float rtcTemp()
{
 int val = RTC.temperature();
 float degC = val / 4.0;
 if (monDbg)
 {
  printf(F3("rtc temp "));
  printTemp(degC);
  printf(F3(" C "));
 }
 float degF = (degC * 9.0) / 5.0 + 32.0;
 if (monDbg)
 {
  printTemp(degF);
  printf(F3(" F\n"));
 }
 return(degF);
}

#endif  /* RTC_CLOCK */

void putx0(void *p, char c)
{
 DBGPORT.write(c);
 if (c == '\n')
  DBGPORT.write('\r');
}

void putx(char c)
{
 DBGPORT.write(c);
 if (c == '\n')
  DBGPORT.write('\r');
}

#if TEMP_SENSOR

void findAddresses(void)
{
 byte i;
 byte addr[8];
  
 printf(F3("Looking for 1-Wire devices\n"));
#if TEMP_SENSOR == 1
 while(oneWire.search(addr))
 {
  printf(F3("Found one wire device with address: \n"));
  for( i = 0; i < 8; i++)
  {
   printf(F3("0x%02x"), addr[i]);
   if (i < 7)
    printf(F3(", "));
   else
    newLine();
  }
  if (OneWire::crc8(addr, 7) != addr[7])
  {
   printf(F3("CRC is not valid!\n"));
   return;
  }
 }
 printf(F3("done\n"));
 oneWire.reset_search();
#elif TEMP_SENSOR == 2
 P_TEMP_SENSOR ts = tempSensor;
 for (uint8_t j = 0; j < TEMP_SENSOR; j++)
 {
  OneWire *oneWire = ts->oneWire;
  while (oneWire->search(addr))
  {
   printf(F3("Found one wire device with address: \n"));
   for( i = 0; i < 8; i++)
   {
    printf(F3("0x%02x"), addr[i]);
    if (i < 7)
     printf(F3(", "));
    else
     newLine();
   }
   if (OneWire::crc8(addr, 7) != addr[7])
   {
    printf(F3("CRC is not valid!\n"));
    return;
   }
  }
  printf(F3("done\n"));
  oneWire->reset_search();
  ts += 1;
 }
#endif	/* TEMP_SENSOR == 2 */
 return;
}

#endif  /* TEMP_SENSOR */

#if CURRENT_SENSOR

#include <avr/io.h>

void putInit()
{
 uartSave = UCSR0B;
 UCSR0B &= ~(_BV(RXCIE0) | _BV(RXCIE0) | _BV(UDRIE0) | _BV(RXEN0));
 UCSR0B |= _BV(TXEN0);
}

void putRestore()
{
 UCSR0B = uartSave;
}

void putx0(char c)
{
 while ((UCSR0A & _BV(UDRE0)) == 0)
  wdt_reset();
 UDR0 = c;
}

void putstr0(const char *str)
{
 char c;
 while (true)
 {
  c = *str++;
  if (c == 0)
   break;
  if (c == '\n')
   putx0('\r');
  putx0(c);
 }
}

void putstr0(const __FlashStringHelper *str)
{
 PGM_P p = reinterpret_cast <PGM_P> (str);
 char c;
 while (true)
 {
  c = pgm_read_byte(p++);
  if (c == 0)
   break;
  if (c == '\n')
   putx0('\r');
  putx0(c);
 }
}

void sndhex(unsigned char *p, int size)
{
 char tmp;
 char ch;

 p += size;
 while (size != 0)
 {
  --size;
  p--;
  tmp = *p;
  ch = tmp;
  ch >>= 4;
  ch &= 0xf;
  if (ch < 10)
   ch += '0';
  else
   ch += 'a' - 10;
  putx0(ch);

  tmp &= 0xf;
  if (tmp < 10)
   tmp += '0';
  else
   tmp += 'a' - 10;
  putx0(tmp);
 }
}

ISR(WDT_vect)
{
 dbg2Clr();

 wdtData.tag0 = 0x55AA;
 unsigned char *src = (unsigned char *) SP;
 unsigned char *dst = (unsigned char *) &wdtData.pc;
 *dst++ = *(src + PC_OFFSET);
 *dst++ = *(src + PC_OFFSET - 1);
 *dst++ = *(src + PC_OFFSET - 2);
 *dst++ = 0;
 dst = (unsigned char *) wdtData.data;
 for (int i = 0; i < 64; i++)
  *dst++ = *src++;
 wdtData.tag1 = 0xAA55;

 putInit();

 putx0('\n');
 putx0('\r');
 dst = (unsigned char *) &wdtData;
 char col = 0;
 for (unsigned int i = 0; i < sizeof(wdtData); i++)
 {
  if (col == 0)
  {
   sndhex((unsigned char *) &dst, 2);
  }
  putx0(' ');
  sndhex(dst, 1);
  dst += 1;
  col += 1;
  if (col == 16)
  {
   col = 0;
   putx0('\n');
   putx0('\r');
  }
 }
 if (col != 0)
 {
  putx0('\n');
  putx0('\r');
 }

 dbg3Set();
}

void timer3() {}

void initCurrent(char isr)
{
 curPSave = 0;			/* clear saved current pointer */
 iChan = 0;			/* initialize channel number */
 ADCSRB = 0;			/* set adc channel */
 for (unsigned char i = 0; i < ADCCHANS; i++)
 {
  P_CURRENT p = &iData[i];
  //p->iTime = 0;
  p->index = i;
  p->lastTime = now();
  p->lastIRms0 = 0.0;
  p->lastIRms = 0.0;
  p->offsetI = ADC_COUNTS >> 1;
  p->iCal = 1.0;
  p->iRatio = p->iCal * ((5000 / 1000.0) / (ADC_COUNTS));
  p->adc = -1;
  p->count = 0;
  p->lastPrint = 0.0;
 }
 iData[0].node = CURRENT0_NODE;
#if ADCCHANS > 1
 iData[1].node = CURRENT1_NODE;
#endif	/* ADCCHANS */
 tempSumI = 0.0;
 adcState = 0;
 cState = 0;
#if 1
 waitCrossing = true;
 crossTmr= CROSS_TMO;
 lastBelow = false;
#else
 sampleCount = SAMPLES;
#endif	/* 1 */
 trace();
 if (isr)
 {
  P_TIMER_CTL t = &tmr3;
  t->timer = (P_TMR) &TCCR3A;
  t->timsk = (uint8_t *) &TIMSK3;
  t->tifr = (uint8_t *) &TIFR3;
  t->aPort = 0;
  t->aMask = 0;
//  t->aPort = (P_PORT) &OC3A_Port;
//  t->aMask = OC3A_Mask;

  printf(F3("attach interrupt\n"));
  DBGPORT.flush();
  dbg0Set();
  unsigned long int uSec = 1024;
#if 0
  Timer3.attachInterrupt(timer3, uSec);
#else
  initTimer3(uSec);
  printf(F3("period %d preScaler %d\n"), timer3Period, timer3Prescale);
#endif	/* 0 */
  showTimer(&tmr3);
  dbg0Clr();
 }
 trace();
}

void printCurrent()
{
 char tmp[10];

 for (unsigned char i = 0; i < ADCCHANS; i++)
 {
  P_CURRENT p = &iData[i];
//  printf("iRatio %s\n", dtostrf(p->iRatio, 8, 6, tmp));
//  printf("iCal %s\n", dtostrf(p->iCal, 8, 6, tmp));
  float delta = abs(p->iRms - p->lastPrint);
  if (delta > .05)
  {
   p->lastPrint = p->iRms;
   printTime(p->lastTime, false);
   printf(F3(" %d iRms %s "), i, dtostrf(p->iRms, 4, 2, tmp));
   printf(F3("offset %s iVcc %5d samples %5d"),
	  dtostrf(p->offsetI, 5, 1, tmp), p->iVcc, p->samples);
   p->lastIRms = p->iRms;
   printf(F3(" delta %3d\n"), (int) (delta * 100));
  }
 }
}

void currentCheck()
{
 trace();
#if 1

 char send = false;
 unsigned long t = 0;
 float *rms = 0;

 P_CURRENT p = curPSave;
 if (p != 0)
 {
  curPSave = 0;		/* clear save pointer */
  send = true;
  t = p->lastTime;
  rms = &p->lastIRms0;
 }
 else if (curQue.count > 0)
 {
  unsigned int emp = curQue.emp;
  P_CUR_DATA curData = &curQue.curData[emp];
  emp += 1;
  if (emp >= MAX_CUR_DATA)
   emp = 0;
  curQue.emp = emp;

  p = curData->chan;

 #define V_REF 5.0
  
  int iVcc = (int) (READVCC_CALIBRATION_CONST / curData->iVcc);
  //p->iRatio = p->iCal * ((iVcc / 1000.0) / (ADC_COUNTS));
  p->iRatio = V_REF / 1024.0;
  // Vin = ADC * VRef / 1024;
  p->iRms = p->iRatio * sqrt(curData->sumI / curData->sampleCount);
  
  if (abs(p->iRms - p->lastIRms0) > 0.2) /* if change in current */
  {
   dbg7Set();
   curBusy = 2;
   curPSave = p;
   p->iVcc = iVcc;
   p->samples = curData->sampleCount;
   p->lastIRms1 = p->lastIRms0; /* set current value */
   p->lastIRms0 = p->iRms;
   p->lastTime = now();

   send = true;
   t = p->lastTime - 1;
   rms = &p->lastIRms1;
  }
  else if ((now() - p->lastTime) > CSENDTIME) /* if time to send */
  {
   dbg7Set();
   curBusy = 1;
   p->iVcc = iVcc;
   p->samples = curData->sampleCount;
   p->lastTime = now();

   send = true;
   t = p->lastTime;
   rms = &p->iRms;
  }

  cli();
  curQue.count -= 1;
  sei();
 }

 if (send)
 {
  char buf[64];
  char tmp[10];

  p->sent++;
  sprintf(buf, F3("time=%ld&node=%s&csv=%s"),
	  t, p->node, dtostrf(*rms, 4, 2, tmp));
  printf(F3("%d %d %d emonData %s\n"), iChan, p->index, curBusy, buf);
  emonData(buf);		/* send current data */
  if (curBusy > 0)
   curBusy -= 1;
  if (curBusy == 0)
   dbg7Clr();
 }

#else
 P_CURRENT p = curPSave;
 if (p == 0)			/* if no current saved */
 {
  for (unsigned char i = 0; i < ADCCHANS; i++) /* for all channels */
  {
   p = &iData[i];
   if (p->send != 0)		/* if current reading */
   {
    p->lastTime = now();
    curPSave = p;		/* save current pointer */
    break;
   }
   else				/* if no current reading */
   {
    if ((now() - p->lastTime) > CSENDTIME) /* if time to send */
    {
     char buf[64];
     char tmp[10];
     printTime();
     p->lastTime = now();
     /* format time, node, and current value */
     sprintf(buf, F3("time=%ld&node=%s&csv=%s"),
	     p->lastTime, p->node, dtostrf(p->iRms, 4, 2, tmp));
     printf(F3("emonData %s\n"), buf);
     emonData(buf);		/* send current data */
    }
    p = 0;
   }
  }
 }

 if (p)				/* if current saved */
 {
  char buf[64];
  char tmp[10];
  if (cState == 0)		/* if time to send prev value */
  {
   sprintf(buf, F3("time=%ld&node=%s&csv=%s"), /* format last reading sent */
           p->lastTime - 1, p->node, dtostrf(p->lastIRms1, 4, 2, tmp));
   cState = 1;			/* set to send current value */
  }
  else
  {
   sprintf(buf, F3("time=%ld&node=%s&csv=%s"), /* format current reading */
           p->lastTime, p->node, dtostrf(p->lastIRms0, 4, 2, tmp));
   p->send = 0;			/* reset send flag */
   cState = 0;			/* reset state */
   curPSave = 0;		/* clear save pointer */
  }
  printTime();
  printf(F3("%d %d emonData %s\n"), p->index, cState, buf);
  emonData(buf);		/* send data */
 }
#endif	/* 1 */
}

//void timer3()
ISR(TIMER3_OVF_vect)
{
 PORTB ^= _BV(PB4);
 PORTB |= _BV(PB5);
 
 if (adcState == 0)		/* if sampling data */
 {
  //if (sampleCount)		/* if sample count non zero */
  {
   ADCSRB = 0;			/* set channel number */
   ADMUX = _BV(REFS0) | iChan;	/* VCC Ref and channel */
   ADCSRA |= _BV(ADSC) | _BV(ADIE); /* start conversion */
  }
 }
 else				/* if calibrating voltage */
 {
  ADCSRA |= _BV(ADSC) | _BV(ADIE); /* start conversion */
 }

 PORTB &= ~_BV(PB5);
}

ISR(ADC_vect)
{
 dbg5Set();

 dbgData.adcFlag = 1;
 
 P_CURRENT p = &iData[iChan];
 if (adcState == 0)		/* if sampling data */
 {
#if 1
  
  int sampleI = ADCL;
  sampleI |= (ADCH << 8);
  p->offsetI = (p->offsetI + (sampleI - p->offsetI) / 1024);
  float filteredI = sampleI - p->offsetI;

  if (waitCrossing)		/* if waiting for zero crossing */
  {
   crossTmr -= 1;
   if (crossTmr <= 0)
   {
    iChan++;			/* advance to next channel */
    if (iChan >= ADCCHANS)	/* if at end */
     iChan = 0;			/* back to channel zero */

    crossTmr = CROSS_TMO;
    lastBelow = false;
   }
   else
   {
    if (filteredI >= 0)		/* if value gt eqal zero */
    {
     if (lastBelow)
     {
      dbg4Set();
      waitCrossing = false;
      crossTmr = CROSS_TMO;
      lastBelow = false;
      cycleCount = CYCLE_COUNT;
      sampleCount = 0;
      tempSumI = 0.0;
     }
    }
    else				/* data negative */
     lastBelow = true;
   }
  }
  else				/* if sampling data */
  {
   crossTmr -= 1;
   if (crossTmr <= 0)
   {
    iChan++;			/* advance to next channel */
    if (iChan >= ADCCHANS)	/* if at end */
     iChan = 0;			/* back to channel zero */

    waitCrossing = true;
    crossTmr = CROSS_TMO;
    lastBelow = false;
   }
   else
   {
    if (cycleCount > 0)		/* if not done sampling */
    {
     float sqI = filteredI * filteredI;
     tempSumI += sqI;
     sampleCount += 1;
     if (filteredI >= 0)	/* if data positive */
     {
      if (lastBelow)		/* if zero crossing */
      {
       crossTmr = CROSS_TMO;
       cycleCount -= 1;
       if (cycleCount == 0)
       {
	dbg4Clr();
	p->adc = ADMUX;
	p->sumI = tempSumI;	/* update sum */
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	ADCSRB = 0;
	adcState = 1;		/* set state to calibrate voltage */
	dbg6Set();
       }
       lastBelow = false;
      }
     }
     else			/* data negative */
      lastBelow = true;
    }
   }
  }
  
#else
  if (sampleCount)		/* if not done */
  {
   int sampleI = ADCL;
   sampleI |= (ADCH << 8);

   p->offsetI = (p->offsetI + (sampleI - p->offsetI) / 1024);
   float filteredI = sampleI - p->offsetI;
   float sqI = filteredI * filteredI;
   tempSumI += sqI;
   --sampleCount;		/* count off a sample */
   if (sampleCount <= 0)	/* if done */
   {
    p->count++;
    p->adc = ADMUX;
    p->sumI = tempSumI;		/* update sum */
    tempSumI = 0.0;		/* reset temporary sum */
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    ADCSRB = 0;
    adcState = 1;		/* set state to calibrate voltage */
    dbg6Set();
   }
  }
#endif	/* 1 */
 }
 else				/* calibrate voltage and process data */
 {
#if 1
  
  if ((curBusy == 0)
  &&  (curQue.count < (MAX_CUR_DATA - 1)))
  {
   unsigned int fil = curQue.fil;
   P_CUR_DATA curData = &curQue.curData[fil];
   fil += 1;
   if (fil >= MAX_CUR_DATA)
    fil = 0;
   curQue.fil = fil;

   p->count++;
   curData->chan = p;
   curData->iVcc = ADCL;
   curData->iVcc |= ADCH << 8;
   curData->sampleCount = sampleCount;
   curData->sumI = p->sumI;
   curQue.count += 1;
  }
  
#else
  long result = ADCL;
  result |= ADCH << 8;
  result = READVCC_CALIBRATION_CONST / result;
  vcc = (int) result;		/* save result */

  p->iRatio = p->iCal * ((result / 1000.0) / (ADC_COUNTS));
  p->iRms = p->iRatio * sqrt(p->sumI / SAMPLES);
  if (p->send == 0)		/* if previous reading sent */
  {
   if (abs(p->iRms - p->lastIRms0) > .1) /* if change in current */
   {
    p->sent++;
    p->lastIRms1 = p->lastIRms0; /* set current value */
    p->lastIRms0 = p->iRms;
    p->send = 1;
   }
  }
#endif	/* 1 */
  
  adcState = 0;			/* set to start sampling */
  iChan++;			/* advance to next channel */
  if (iChan >= ADCCHANS)	/* if at end */
   iChan = 0;			/* back to channel zero */
  
#if 1
  waitCrossing = true;		/* set to wait for zero crossing */
  crossTmr = CROSS_TMO;
  lastBelow = false;
#else
  sampleCount = SAMPLES;	/* set number of samples */
#endif	/* 1 */
  dbg6Clr();
 }

 dbgData.adcFlag = 0;
 dbg5Clr();
}

#if 0
int adcRead(char pin)
{
 ADCSRB = 0; 			/* chan 0 = 7 */
 ADMUX = _BV(REFS0) | pin;	/* VCC Ref and channel */

 // without a delay, we seem to read from the wrong channel
 //delay(1);

 // start the conversion
 ADCSRA |= _BV(ADSC); /* Convert */

 // ADSC is cleared when the conversion finishes
 while (bit_is_set(ADCSRA, ADSC));

 // we have to read ADCL first; doing so locks both ADCL
 // and ADCH until ADCH is read.  reading ADCL second would
 // cause the results of each conversion to be discarded,
 // as ADCL and ADCH would be locked when it completed.
 uint8_t low  = ADCL;
 uint8_t high = ADCH;

 // combine the two bytes
 return (high << 8) | low;
}
#endif	/* 0 */

#endif  /* CURRENT_MONITOR */

#define MAXDIG 10		/* maximum input digits */

unsigned char getNum()
{
 char ch;			/* input character */
 char chbuf[MAXDIG];		/* input digit buffer */
 unsigned char chidx;		/* input character index */
 unsigned char count;		/* input character count */
 char neg;			/* negative flag */
 char hex;			/* hex flag */

 neg = 0;
 hex = 0;
 val = 0;
 chidx = 0;
 count = 0;
 while (1)
 {
  getChar(ch);
  if ((ch >= '0')
  &&  (ch <= '9'))
  {
   if (chidx < MAXDIG)
   {
    putx(ch);
    chbuf[chidx] = ch - '0';
    chidx++;
   }
  }
  else if ((ch >= 'a')
  &&       (ch <= 'f'))
  {
   if (chidx < MAXDIG)
   {
    hex = 1;
    putx(ch);
    chbuf[chidx] = ch - 'a' + 10;
    chidx++;
   }
  }
  else if ((ch == 8)
       ||  (ch == 127))
  {
   if (chidx > 0)
   {
    --chidx;
    putx(8);
    putx(' ');
    putx(8);
   }
  }
  else if ((ch == 13)
       ||  (ch == ' '))
  {
   if (hex)
   {
    while (count < chidx)
    {
     val = (val << 4) + chbuf[count];
     count++;
    }
   }
   else
   {
    while (count < chidx)
    {
     val = val * 10 + chbuf[count];
     count++;
    }
   }
   if (neg)
    val = -val;
   return(count);
  }
  else if (chidx == 0)
  {
   if (ch == '-')
   {
    putx(ch);
    neg = 1;
   }
   else if (ch == 'x')
   {
    putx(ch);
    hex = 1;
   }
  }
  else
   printf(F3("%d "),ch);
 }
}

#if ESP8266_TIME
char esp8266TimeEnable()
{
 return(wifiWriteStr(F2("AT+CIPSNTPCFG=1,0,\"us.pool.ntp.org\""), 3000));
}

const char jan[] PROGMEM = "Jan";
const char feb[] PROGMEM = "Feb";
const char mar[] PROGMEM = "Mar";
const char apr[] PROGMEM = "Apr";
const char may[] PROGMEM = "may";
const char jun[] PROGMEM = "Jun";
const char jul[] PROGMEM = "Jul";
const char aug[] PROGMEM = "Aug";
const char sep[] PROGMEM = "Sep";
const char oct[] PROGMEM = "Oct";
const char nov[] PROGMEM = "Nov";
const char dec[] PROGMEM = "Dec";

const char *const months[] PROGMEM =
{
 jan, feb, mar, apr, may, jun, jul, aug, sep, oct, nov, dec
};

char *findField(char *p)
{
 char ch;
// printf(F3("findField %s"), p);
 while (1)
 {
  ch = *p;
  if (isalnum(ch))
  {
//   printf(F3("*%c "), ch);
   p += 1;
  }
  else
  {
   *p = 0;
   p += 1;
   return(p);
  }
 }

 if (ch == 0)
  return(0);
}

char esp8266Time()
{
 int hour;
 int min;
 int sec;
 int day;
 int month;
 int year;
  
 char found = 0;
 for (int i = 0; i < 3; i++)
 {
  wifiWriteStr(F2("AT+CIPSNTPTIME?"), 3000);
  if (strstr(packetRsp, "1970") == 0)
   found = 1;
   break;
  delay(1000);
 }
 if (found == 0)
  return(0);

 /* +CIPSNTPTIME:Wed Jul 22 08:31:45 2020 */
 /*           1         2         3          */ 
 /* 0123456789012345678901234567890123456789 */
 // void setTime(int hr,int min,int sec,int dy, int mnth, int yr)

 char *p0;
 char *p1;

 p0 = strstr(packetRsp, ":");
 p0 = strstr(p0, " ");
 p0 += 1;
 // printf(F3("packetRsp %s\n"), p0);
 p1 = findField(p0);	/* month */
 if (p1 == 0)
  return(0);
 // printf(F3("month %s\n"), p0);
 char buf[4];
 for (int i = 0; i < 12; i++)
 {   
  const __FlashStringHelper *val = pgm_read_word(&months[i]);
  //  printf(F3("%2d months[i] %x\n"), i, val);
  argConv(val, buf);
  //  printf(F3("%2d month (%s)\n"), i, buf);
  if (strcmp(buf, p0) == 0)
  {
   month = i + 1;
   break;
  }
 }

 p0 = p1;
 p1 = findField(p0);	/* day */
 if (p1 == 0)
  return(0);
 day = atoi((const char *) p0);
 // printf(F3("day %s %d\n"), p0, day);

 p0 = p1;
 p1 = findField(p0);	/* hour */
 if (p1 == 0)
  return(0);
 hour = atoi((const char *) p0);
 // printf(F3("hour %s %d\n"), p0, hour);

 p0 = p1;
 p1 = findField(p0);	/* minute */
 if (p1 == 0)
  return(0);
 min = atoi((const char *) p0);
 // printf(F3("min %s %d\n"), p0, min);

 p0 = p1;
 p1 = findField(p0);	/* second */
 if (p1 == 0)
  return(0);
 sec = atoi((const char *) p0);
 // printf(F3("sec %s %d\n"), p0, sec);

 p0 = p1;
 p1 = findField(p0);	/* year */
 if (p1 == 0)
  return(0);
 year = atoi((const char *) p0);
 // printf(F3("year %s %d\n"), p0, year);

 // printf(F3("h %2d m %2d s %2d d %2d m %2d y %4d\n"), hour, min, sec, day, month, year);
 setTime(hour, min, sec, day, month, year);
 return(1);
}

#endif	/* ESP8266_TIME */

#if defined(ARDUINO_ARCH_STM32)

extern "C" int _write(int fd, char *ptr, int len)
{
 while (--len >= 0)
 {
  char ch = *ptr++;
  DBGPORT.write(ch);
  if (ch == '\n')
   DBGPORT.write('\r');
 }
 return(0);
}

extern "C" int _write_r(void *p, int fd, char *ptr, int len)
{
 while (--len >= 0)
 {
  char ch = *ptr++;
  DBGPORT.write(ch);
  if (ch == '\n')
   DBGPORT.write('\r');
 }
 return(0);
}

#endif	/* ARDUINO_ARCH_STM32 */
