#ifdef ARDUINO_ARCH_AVR
#include <Arduino.h>
#include <Wire.h>
#include "serial.h"

#define INT_MILLIS 0


#include "millis.h"

#define EMONCMS_KEY "b53ec1abe610c66009b207d6207f2c9e"
#define TEST_NODE 0
#define THING_SPEAK 0

#if ARDUINO_AVR_MEGA2560

#define MONITOR_INDEX 3

/* outside temperature */

#if (MONITOR_INDEX == 1)
#define TEMP_SENSOR 1
#define RTC_CLOCK 1
#define DHT_SENSOR 0
#define CURRENT_SENSOR 0
#define WATER_MONITOR 0
#define MONITOR_ID "Monitor1"
#endif	/* MONITOR_INDEX == 0 */

/* basement dehumidifer and furnace monitor */

#if (MONITOR_INDEX == 2)
#define TEMP_SENSOR 1
#define RTC_CLOCK 1
#define DHT_SENSOR 1
#define CURRENT_SENSOR 1
#define WATER_MONITOR 1
#define DEHUMIDIFIER 0

#define EMONCMS_NODE "2"
#define CURRENT0_NODE 3
#define CURRENT1_NODE 4

#define SSID "nystrom"
#define PASS "minidonk"
#define MONITOR_ID "Monitor2"
#endif	/* MONITOR_INDEX == 2 */

/* basement water alarm and pump shutoff */

#if (MONITOR_INDEX == 3)
#define TEMP_SENSOR 1
#define RTC_CLOCK 1
#define DHT_SENSOR 1
#define CURRENT_SENSOR 0
#define WATER_MONITOR 1
#define DEHUMIDIFIER 1
#define EMONCMS_NODE "5"

#define SSID "nystrom"
#define PASS "minidonk"
#define MONITOR_ID "Monitor3"
#endif	/* MONITOR_INDEX == 3 */

#if (MONITOR_INDEX == 4)
#define TEMP_SENSOR 0
#define RTC_CLOCK 0
#define DHT_SENSOR 0
#define CURRENT_SENSOR 0
#define WATER_MONITOR 1
#define DEHUMIDIFIER 0

#define SSID "nystrom"
#define PASS "minidonk"
#define MONITOR_ID "Monitor4"
#endif	/* MONITOR_INDEX == 4 */

#if TEST_NODE

#undef EMONCMS_NODE
#define EMONCMS_NODE "12"

#if CURRENT_SENSOR
#undef CURRENT0_NODE
#define CURRENT0_NODE 13
#undef CURRENT1_NODE
#define CURRENT1_NODE 14
#endif	/* CURRENT_SENSOR */

#endif	/* ARDUINO_AVR_MEGA2560 */

#endif	/* TEST_MODE */

#if ARDUINO_AVR_PRO

SoftwareSerial dbgPort = SoftwareSerial(rxPin, txPin);

#define MONITOR_INDEX 2

#if (MONITOR_INDEX == 0)
#define TEMP_SENSOR 0
#define RTC_CLOCK 0
#define CURRENT_SENSOR 0
#define DHT_SENSOR 0
#define WATER_MONITOR 1
#define DEHUMIDIFIER 0

#define SSID "TKGCL"
#define PASS "K4PLVFMCXGMLXM9P"
#define MONITOR_ID "Monitor0"
#endif	/* MONITOR_INDEX == 0 */

#if (MONITOR_INDEX == 2)
#define TEMP_SENSOR 0
#define RTC_CLOCK 0
#define CURRENT_SENSOR 0
#define DHT_SENSOR 0
#define WATER_MONITOR 1
#define DEHUMIDIFIER 0

#define SSID "nystrom"
#define PASS "minidonk"
#define MONITOR_ID "Monitor2"
#endif	/* MONITOR_INDEX == 2 */

#endif	/* ARDUINO_AVR_PRO */

#endif	/* ARDUINO_ARCH_AVR */

#ifdef MEGA32
#include "WProgram.h"
#include "printf.h"

#define TEMP_SENSOR 1
#define DHT_SENSOR 1
#define RTC_CLOCK 0
#define CURRENT_SENSOR 0
#define WATER_MONITOR 0
#define DEHUMIDIFIER 0

#define TS_KEY "67TDONLKRDNVF7L4"
#define EMONCMS_NODE "0"

#endif	/* MEGA32 */

#if TEMP_SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>

#if (MONITOR_INDEX == 1)
#define TEMPDEVS 1
DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0xB8, 0x50, 0x9B, 0x06, 0x00, 0x00, 0x89
};
#endif	/* MONITOR_INDEX == 1 */

#if (MONITOR_INDEX == 2)
#define TEMPDEVS 2
DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0xff, 0xd3, 0x09, 0x63, 0x14, 0x02, 0xe1},
 {0x28, 0xc8, 0xae, 0x9b, 0x06, 0x00, 0x00, 0x15}
};
#endif	/* MONITOR_INDEX == 2 */

#if (MONITOR_INDEX == 3)
#define TEMPDEVS 1
DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0x7d, 0xe3, 0x96, 0x06, 0x00, 0x00, 0x61}
};
#endif	/* MONITOR_INDEX == 3 */

#ifdef MEGA32
#define TEMPDEVS 1
DeviceAddress tempDev[TEMPDEVS] =
{
 {0x10, 0xDC, 0x5D, 0xD4, 0x01, 0x08, 0x00, 0xE9}
};
#endif	/* MEGA32 */

float lastTemp[TEMPDEVS];

#ifdef MEGA32
#define ONE_WIRE_BUS 4		/* one wire bus pin */
#endif	/* MEGA32 */

#ifdef ARDUINO_ARCH_AVR
#define ONE_WIRE_BUS 4		/* one wire bus pin */
#endif	/* ARDUINO_ARCH_AVR */

void findAddresses(void);
OneWire oneWire(ONE_WIRE_BUS);	/* one wire instance */
DallasTemperature sensors(&oneWire); /* dallas temp sensor instance */
float printTemperature(DeviceAddress deviceAddress);

#endif  /* TEMP_SENSOR */

#if DHT_SENSOR
#include <DHT.h>

#if ARDUINO_AVR_MEGA2560
#define DHTPIN 3
#if DEHUMIDIFIER
void switchRelay(char pin);
#define DEHUM_ON_PIN 9
#define DEHUM_OFF_PIN 8
#endif	/* DEHUMIDIFIER */
#endif	/* ARDUINO_AVR_MEGA2560 */

#if MEGA32
#define DHTPIN 4
#endif	/* MEGA32 */

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
#include "TimerThree.h"

#define READVCC_CALIBRATION_CONST 1126400L
#define ADC_BITS 10
#define ADC_COUNTS (1<<ADC_BITS)
#endif  /* CURRENT_SENSOR */

#include "wdt.h"
#include <Time.h>
#include "string.h"
#include "dns.h"
#include "ntp.h"
#include "wifi.h"

#ifdef ARDUINO_ARCH_AVR
#include <EEPROM.h>
#include <stdio.h>

#define INITEE 1		/* init eeprom values on boot */
#define LOCAL 0			/* use local server */

#if LOCAL
#define SERVER "10.0.0.2"
#define TCPPORT 8080
#define HOST "10.0.0.2"
#define SITE
#else
#define TCPPORT 80
#define HOST "test.ericnystrom.com"
#define SITE "/alert"
#endif  /* LOCAL */

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

unsigned char getNum();
int val;

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

#if WATER_MONITOR

#if ARDUINO_AVR_PRO

#define WATER0 3
#define WATER1 4

#define BEEPER 10
#define LED 13

#endif	/* ARDUINO_AVR_PRO */

#if ARDUINO_AVR_MEGA2560

#define WATER0 6
#define WATER1 7

#define BEEPER 12
#define LED 13

#endif	/* ARDUINO_AVR_MEGA2560 */

char serverIP[IP_ADDRESS_LEN];	/* server ip address */
char failCount;			/* send failure count */

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

void printTemp(float temp);
char *writeTemp(char *buf, float temp);

unsigned long time = 0;

void loopTemp();

#if WATER_MONITOR
void loopWater();
void alarmPoll();
void procAlarm(P_INPUT water, boolean inp);
char notify(int alarm, boolean val);
char checkIn();
char sendHTTP(char *data);
#endif  /* WATER_MONITOR */

char *cpyStr(char *dst, const char *str);
char *strEnd(char *p);

#if THING_SPEAK
void tsData(char *data);
#define TS_KEY "86Z0KTDYLEC28FU3"
#endif

char emonData(char *data);

#if RTC_CLOCK
float rtcTemp();
#endif

void cmdLoop();
unsigned int tLast;
int loopCount;

#define TINTERVAL (10000U)	// timer interval
#define T1SEC (1000U)		// one second interval
#define TEMP_COUNT (0)		// temp reading interval number
#define WATER_COUNT (1)		// water alarm interval number
#define NTP_COUNT (3)		// time setting
#define LOOP_MAX (6)		// max number of intervals

#if CURRENT_SENSOR

void initCurrent(char isr);	// init current readings
void printCurrent();
void currentCheck();		// check for time to send data
void timer3();			// timer isr for reading current
//int adcRead(char chan);

#define SAMPLES (100)		// samples per reading
#define ADCCHANS (2)		// number of adc channels to read

#define CSENDTIME (10 * 60)	// if no change for this time send current

typedef struct
{
 unsigned long iTime;		// time of data reading
 unsigned long lastTime;	// last time data sent
 float lastIRms0;
 float lastIRms1;
 float iRms;
 float lastIRms;
 float offsetI;
 float iCal;
 float iRatio;			// conversion ratio
 float sumI;			// current sum of squares
 char node;
 int count;
 int sent;
 int adc;
} T_CURRENT, *P_CURRENT;

unsigned char iChan;		// data channel for current reading
P_CURRENT curPSave;		// saved current pointer
int sampleCount;		// sample counter
char adcState;			// adc state
char cState;			// current processing state
T_CURRENT iData[ADCCHANS];	// current channel data
int vcc;			// current vcc
float tempSumI;			// current sum accumulator

#endif  /* CURRENT_SENSOR */

void putx0(void *p, char c);
void putx(char c);

#ifdef ARDUINO_ARCH_AVR

uint16_t intMillis()
{
 uint16_t m;
 uint8_t oldSREG = SREG;	// save interrupt flag
 cli();				// disable interrupts
 m = (uint16_t) timer0_millis;	// read low part of millis
 SREG = oldSREG;		// enable interrupts
 return(m);			// return value
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
  char *dst = id;
  char addr = eeLoc;
  while (--len >= 0)
  {
   EEPROM.write(addr, *p);
   addr++;
   *dst++ = *p++;
  }
  return(1);
 }
 return(0);
}

#endif	/* ARDUINO_ARCH_AVR */

/* setup routine */

void setup()
{
 char ch;
#ifdef ARDUINO_ARCH_AVR
 wdt_enable(WDT_TO);
#if ARDUINO_AVR_MEGA2560
 DBGPORT.begin(19200);
#endif
#if ARDUINO_AVR_PRO
 DBGPORT.begin(9600);
#endif

#if PRINTF
 init_printf(NULL, putx1);
#endif

#if !PRINTF
 fdev_setup_stream(&uartout, putx, NULL, _FDEV_SETUP_WRITE);
 stdout = &uartout;
#endif
#endif  /* ARDUINO_ARCH_AVR */

 if (DBG)
  printf(F3("\nstarting 0\n"));

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

 memset(&serverIP, 0, sizeof(serverIP));
 memset(&ntpIP, 0, sizeof(ntpIP));
 failCount = 0;

 ntpStart = millis();
 ntpTimeout = 0;

 uint32_t checksum = sumEE();
 uint32_t csum;
 readEE((char *) &csum, CSUM_LOC, CSUM_LEN);
 if (checksum != csum)
 {
  if (DBG)
   printf(F3("init eeprom with default\n"));
#if INITEE
  writeEE(SSID, SSID_LOC, SSID_LEN);
  writeEE(PASS, PASS_LOC, PASS_LEN);
  writeEE(MONITOR_ID, ID_LOC, ID_LEN);
#endif  /* INITEE */
  writeSumEE();
 }

#if MEGA32
 init_printf(NULL, putx0);
#endif

 if (DBG)
  printf(F3("\nstarting 1\n"));

#if TEMP_SENSOR
 sensors.begin();
 for (unsigned char i = 0; i < TEMPDEVS; i++)
 {
  sensors.setResolution(tempDev[i], 12);
  lastTemp[i] = 0.0;
 }
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

#if ARDUINO_ARCH_AVR
 readEE(id, ID_LOC, ID_LEN);
#endif

 printf(F3("id %s\n"), id);

#if ARDUINO_AVR_MEGA2560
 pinMode(51, OUTPUT);		// pg0
 pinMode(52, OUTPUT);		// pg1
 PORTG &= ~(_BV(PG0) | _BV(PG1));

 PORTG |= _BV(PG0);
 delay(2);
 PORTG &= ~_BV(PG0);
#endif	/* ARDUINO_AVR_MEGA2560 */

 wifiInitSio();			// enable wifi serial port
 pinMode(WIFI_RESET, OUTPUT);	// set wifi reset pin to output
 digitalWrite(WIFI_RESET, HIGH); // set it high
 delay(10);			// short wait

 wifiReset();			// reset wifi

 char retry = 3;
 while (1)
 {
  char result = wifiJoin();
  if (result)
   break;
  --retry;
  if (retry <= 0)
  {
   retry = 3;
   wifiReset();
   delay(500);
  }
 }

#if DBG
 unsigned int t = intMillis();
 while ((unsigned int) (intMillis() - t) < 1000U)
 {
  ch = DBGPORT.read();
 }

 t = intMillis();
 ch = 0;
 printf(F3("\nany char for cmd mode..."));
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
 }
#endif /* DBG */

#if RTC_CLOCK
 char status = ntpSetTime();	// look up ntp time
 if (status)			// if valid time found
 {
  RTC.set(now());		// set the clock
 }
 setSyncProvider(RTC.get);	// set rtc to provide clock time
#else
 ntpSetTime();			// look up ntp time
#endif	/* RTC_CLOCK */

#if CURRENT_SENSOR
 initCurrent(1);		// initial current sensor
#endif	/* CURRENT_SENSOR */

 tLast = intMillis();		// initialize loop timer
}

void cmdLoop()
{
 wdt_disable();
 printf(F3("command loop\n"));
 while (1)
 {
  if (DBGPORT.available())
  {
   char ch = DBGPORT.read();
   DBGPORT.write(ch);
   newLine();
   if (ch == 'x')		// exit command loop
    break;
   else if (ch == '?')
   {
    printf(F3("monitor.cpp\n"));
   }
#if ARDUINO_ARCH_AVR
   else if (ch == 'w')		// write ssid and password to eeprom
   {
    char flag = updateEE("ssid", SSID_LOC, SSID_LEN);
    flag |= updateEE("pass", PASS_LOC, PASS_LEN);
    if (flag)
     writeSumEE();
   }
   else if (ch == 'i')		// write unit id to eeprom
   {
    char flag = updateEE("id", ID_LOC,ID_LEN);
    if (flag)
     writeSumEE();
   }
   else if (ch == 'W')		// test watchdog timer
   {
    printf(F3("test watchdog timer\n"));
    wdt_enable(WDT_TO);
    while (1)
     ;
   }
#endif  /* ARDUINO_ARCH_AVR */
#if ARDUINO_AVR_MEGA2560
   else if (ch == 'p')
   {
    if (getNum())
    {
     unsigned char tmp = PORTG;
     PORTG = (char) val;
     printf(F3("\nportg %x %x\n"), tmp, PORTG);
    }
   }
#endif	/* ARDUINO_AVR_MEGA2560 */
#if CURRENT_SENSOR
   else if (ch == 'e')		// read a to d converter
   {
    printf(F3("chan: "));
    len = readStr(cmdBuffer, sizeof(cmdBuffer) - 1);
    unsigned char chan = 0;
    if (len != 0)
    {
     chan = atoi(cmdBuffer);
     if (chan >= ADCCHANS)
      chan = ADCCHANS - 1;
    }
    P_CURRENT p = &iData[chan];
    printf(F3("value: "));
    char len = readStr(cmdBuffer, sizeof(cmdBuffer) - 1);
    printf(F3("iTime %ld len %d cState %d\n"), p->iTime, len, cState);
    if (len != 0)
    {
     p->lastIRms1 = p->lastIRms0;
     p->lastIRms0 = atof(cmdBuffer);
     p->iTime = now();
     printTime(p->iTime);
    }
   }
   else if (ch == 'C')		// run current check code
   {
    printCurrent();
    currentCheck();
   }
   else if (ch == 'T')		// setup current sensor isr
   {
    printf(F3("isr: "));
    char len = readStr(stringBuffer, sizeof(stringBuffer) - 1);
    initCurrent(len);
   }
   else if (ch == 'I')		// print current results
   {
    char tmp[12];
    printf(F3("vcc %d\n"), vcc);
    for (unsigned char i = 0; i < ADCCHANS; i++)
    {
     P_CURRENT p = &iData[i];
     printf(F3("iRms %s\n"), dtostrf(p->iRms, 4, 2, tmp));
     printf(F3("iRatio %s\n"), dtostrf(p->iRatio, 8, 6, tmp));
     printf(F3("node %d count %d sent %d adc %02x\n"),
	    p->node, p->count, p->sent, p->adc);
     if (p->iTime != 0)
     {
      printTime(p->iTime);
      printf(F3("lastIRms0 %s\n"), dtostrf(p->lastIRms0, 4, 2, tmp));
      printf(F3("offset %s\n"), dtostrf(p->offsetI, 4, 2, tmp));
      p->iTime = 0;
     }
    }
   }
#endif  /* CURRENT_SENSOR */
   else if (ch == 'u')
   {
    long tmp = 0x55aa55aa;
    DBGPORT.print(tmp, 16);
    DBGPORT.println();
    printf(F3("%lx\n"), tmp);
   }
#if WATER_MONITOR
   else if (ch == 'L')		// loop water
   {
    loopWater();
   }
#endif  /* WATER_MONITOR */
   else if (ch == 'j')		// join wifi
   {
    wifiJoin();
   }
   else if (ch == 'o')		// send at
   {
    wifiWriteStr(F2("AT"), 1000);
   }
   else if (ch == 'l')		// list access points
   {
    wifiWriteStr(F2("AT+CWLAP"), 3000);
   }
   else if (ch == 'q')
   {
    wifiWriteStr(F2("AT+CWQAP"), 1000);
   }
   else if (ch == 's')
   {
    wifiWriteStr(F2("AT+CIFSR"), 1000);
   }
   else if (ch == 'u')		// start udp connection
   {
    wifiWriteStr(F2("AT+CIPSTART=\"UDP\",\"129.6.15.28\",123"), 3000);
   }
   else if (ch == 'm')		// wifi mux
   {
    wifiMux();
   }
   else if (ch == 't')		// start tcp connection
   {
    wifiWriteStr(F2("AT+CIPSTART=4,\"TCP\",\"184.106.153.149\",80"), 4000);
   }
   else if (ch == 'z')
   {
    wifiClose(4, 15000);
   }
   else if (ch == 'c')		// enter wifi command
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
   else if (ch == 'd')		// set time from ntp
   {
    ntpStart = millis();
    ntpTimeout = 0;
#if RTC_CLOCK
    char status = ntpSetTime();
    if (status)
    {
     RTC.set(now());
    }
    printTime(RTC.get());
    setSyncProvider(RTC.get);
#else
    ntpSetTime();
#endif  /* RTC_CLOCK */
    printTime();
   }
#if RTC_CLOCK
   else if (ch == 'v')		// read rtc temp
   {
    rtcTemp();
   }
#endif  /* RTC_CLOCK */
#if DHT_SENSOR
   else if (ch == 'h')		// read humidity sensor
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
#endif  /* DNT_SENSOR */
#if DEHUMIDIFIER
   else if (ch == 'r')		// set dehumidifier relay
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
   else if (ch == 'f')		// find temp sensor addresses
   {
    findAddresses();
   }
   else if (ch == 'y')		// read temp sensors
   {
    sensors.requestTemperatures();
    for (unsigned char i = 0; i < TEMPDEVS; i++)
    {
     float temp = sensors.getTempF(tempDev[i]);
     printf(F3("temp "));
     DBGPORT.print(temp);
     printf(F3("F\n"));
    }
   }
   else if (ch == 'g')
   {
    loopTemp();
   }
#endif  /* TEMP_SENSOR */
  }
 }
 wdt_enable(WDT_TO);
}

void loop()
{
 while (DBGPORT.available())
 {
  char ch = DBGPORT.read();
  if (ch == 'C')
  {
   while (DBGPORT.available())
    ch = DBGPORT.read();
   cmdLoop();
  }
 }
 
 unsigned int tPrev = intMillis(); // init time for short interval
 while (1)
 {
  wdt_reset();
  PORTD |= _BV(PD4);
  unsigned int t0 = intMillis(); // read time
  if ((unsigned int) (t0 - tLast) > TINTERVAL) // if long interval up
  {
//   printf("t0 %ld tLast %ld delta %ld\n", t0, tLast, t0 - tLast);
   tLast = t0;			// update previous time
   PORTD &= ~_BV(PD4);
   break;			// exit loop
  }

  delay(100);			// wait a while
  t0 = intMillis();		// read time
  if ((unsigned long) (t0 - tPrev) >= T1SEC) // if short interval up
  {
//   printf("t0 %ld tPrev %ld delta %ld\n", t0, tPrev, t0 - tPrev);
   tPrev = t0;			// update previous time
#if WATER_MONITOR
   alarmPoll();			// poll water alarm
#endif	/* WATER_MONITOR */

#if CURRENT_SENSOR
   currentCheck();		// check and send current
#endif	/* CURRENT_SENSOR */
  }
 }

#if CURRENT_SENSOR
 printCurrent();
#endif	/* CURRENT_SENSOR */

 printf(F3("%d "), loopCount);
 printTime();

#if TEMP_SENSOR | DHT_SENSOR
 if (loopCount == TEMP_COUNT)	// if time for temperature reading
 {
  loopTemp();
 }
#endif	/* TEMP_SENSOR | DHT_SENSOR */

#if WATER_MONITOR
 if (loopCount == WATER_COUNT)	// if time to check water alarm
 {
  digitalWrite(LED, LOW); 	/* turn off led */
  loopWater();			/* loop processing */
 }
#endif	/* WATER_MONITOR */

 if (loopCount == NTP_COUNT)	// if time to set time
 {
#if RTC_CLOCK
  char status = ntpSetTime();
  if (status)			// if good reading
  {
   RTC.set(now());		// set the clock
  }
#else  /* RTC_CLOCK */
  ntpSetTime();
#endif	/* RTC_CLOCK */
 }

 loopCount++;			// update loop counter
 if (loopCount >= LOOP_MAX)	// if at maximum
  loopCount = 0;		// reset to beginning
}

#if DEHUMIDIFIER
void switchRelay(char pin)
{
 digitalWrite(pin, HIGH);	// turn relay on
 delay(100);
 digitalWrite(pin, LOW);	// turn relay off
}
#endif	/* DEHUMIDIFIER */

#if TEMP_SENSOR | DHT_SENSOR

void loopTemp()
{
#if TEMP_SENSOR
 char count = 5;
 float temp1[TEMPDEVS];
 for (unsigned char i = 0; i < TEMPDEVS; i++)
 {
  float t;
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
  temp1[i] = t;
 }
#endif  /* TEMP_SENSOR */

#if RTC_CLOCK
 float rtcTempVal = rtcTemp();
#else
 float rtcTempVal = 0;
#endif /* RTC_CLOCK */

#if DHT_SENSOR
 float dhtHumidity = dht.readHumidity();
 float dhtTemp = dht.readTemperature(true);
 printf(F3("temp "));
 printTemp(dhtTemp);
 printf(F3(" F humidity "));
 printTemp(dhtHumidity);
 newLine();
#if DEHUMIDIFIER
 printf(F3("dehumState %d dehumDelay %d\n"), dehumState, dehumDelay);
 if (dehumState)		// if dehumidifer on
 {
  if (dhtHumidity <= dehumOff)	// if humidity below turn off point
  {
   if (dehumDelay != 0)		// if timer active
   {
    if (--dehumDelay == 0)	// if counts down to zero
    {
     dehumState = 0;		// set state to off
     switchRelay(DEHUM_OFF_PIN);
     printf(F3("dehumidifier off\n"));
    }
   }
   else				// if timer not active
   {
    dehumDelay = DEHUM_DELAY;	// start counter
   }
  }
  else				// if not below turn off point
  {
   dehumDelay = 0;		// reset counter
  }
 }
 else				// if dehumidifier off
 {
  if (dhtHumidity >= dehumOn)	// if humidity above turn on point
  {
   if (dehumDelay != 0)		// if timer active
   {
    if (--dehumDelay == 0)	// if counts down to zero
    {
     dehumState = 1;		// set state to off
     switchRelay(DEHUM_ON_PIN); // turn dehumidifier on
     printf(F3("dehumidifier on\n"));
    }
   }
   else				// if timer not active
   {
    dehumDelay = DEHUM_DELAY;	// start counter
   }
  }
  else				// if not above turn on point
  {
   dehumDelay = 0;		// reset counter
  }
 }
#endif	/* DEHUMIDIFIER */
#else  /* DHT_SENSOR */
 float dhtHumidity = 0;
 float dhtTemp = 0;
#endif  /* DHT_SENSOR */

 char buf[128];
 char *p;
#if THING_SPEAK
 p = cpyStr(buf, "field1=");
 p = writeTemp(p, temp1);
 p = cpyStr(p, "&field2=");
 p = writeTemp(p, rtcTempVal);
 p = cpyStr(p, "&field3=");
 p = writeTemp(p, dhtHumidity);
 p = cpyStr(p, "&field4=");
 writeTemp(p, dhtTemp);
 tsData(buf);
#endif  /* THING_SPEAK */

 p = cpyStr(buf, "node=" EMONCMS_NODE "&csv=");
#if TEMP_SENSOR
 for (unsigned char i = 0; i < TEMPDEVS; i++)
 {
  p = writeTemp(p, temp1[i]);	/* output data from each temp sensor */
  *p++ = ',';
 }
#endif	/* TEMP_SENSOR */
 p = writeTemp(p, rtcTempVal);	/* output real time clock temp data */
 *p++ = ',';
 p = writeTemp(p, dhtHumidity);	/* output dht sensor humidity */
 *p++ = ',';
 writeTemp(p, dhtTemp);		/* output dht sensor temp */
 emonData(buf);
}

#endif	/* TEMP_SENSOR | DHT_SENSOR */

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
 checkIn();
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
}

void procAlarm(P_INPUT water, boolean inp)
{
 printf(F3("inp %d alarm%d cur %d counter %d state %d\n"),
	inp, water->index, water->inp, water->counter, water->state);
 if (water->inp != inp)		// if input state changed
 {
  water->inp = inp;		// save state
  water->counter =  (inp == STATE_ALARM) ? COUNT : 1; // set counter
 }
 else
 {
  if (water->counter != 0)	// if state changing
  {
   --water->counter;		// count of change timer
   if (water->counter == 0)	// if state stable
   {
    if (inp != STATE_ALARM)	// if alarm cleared
     beeperCount = 0;		// stop beeper
    
    if (water->state != inp)	// if state changed
    {
     water->state = inp;	// save current state
     if (!notify(water->index, inp == STATE_ALARM)) // if notify failure
     {
      water->counter = 1;	// set counter to send again
     }
    }
   }
  }
 }
 printf(F3("procAlarm done\n"));
}

char notify(int alarm, boolean val)
{
 sprintf((char *) dataBuffer,
	 F3("GET " SITE "/notify?id=%s&alarm=%d&val=%d"), id, alarm, val);
 return(sendHTTP(dataBuffer));
}

char checkIn()
{
 char state = 0;
 if (water0.state == STATE_ALARM)
  state |= 1;
 if (water1.state == STATE_ALARM)
  state |= 2;
 sprintf((char *) dataBuffer,
	 F0("GET " SITE "/check?id=%s&st=%d"), id, state);
 return(sendHTTP(dataBuffer));
}

#define HTTP " HTTP/1.1\r\n\
Host: " HOST "\r\n\
Connection: Close\r\n\r\n"

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

char sendHTTP(char *data)
{
#if LOCAL
 strncpy(serverIP, HOST, sizeof(serverIP));
#else
 dnsLookup(serverIP, F0(HOST));
#endif	/* LOCAL */

 if (serverIP[0] != 0)
 {
  strcat(data, F3(HTTP));
  for (char retry = 0; retry < 2; retry++)
  {
   char *p = sendData(serverIP, TCPPORT, data, 10000);
   if (p != 0)
   {
    if (find(lc(p), (char *) F0("*ok*")) >= 0)
    {
     failCount = 0;
     return(1);
    }
   }
   printf(F3("**sendHTTP retry %d\n"), retry);
#if ARDUINO_AVR_MEGA2560
   PORTG |= _BV(PG0);
   delay(2);
   PORTG &= ~_BV(PG0);
   delay(500);
#endif	/* ARDUINO_AVR_MEGA2560 */
  }
 }
 updateFail();
 return(0);
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

void printTemp(float temp)
{
 int tmp = (int) (temp * 10);
 int deg = tmp / 10;
 int frac = tmp % 10;
 printf(F3("%d.%d"), deg, frac);
}

char *writeTemp(char *buf, float temp)
{
 int tmp = (int) (temp * 10);
 int deg = tmp / 10;
 int frac = tmp % 10;
 sprintf(buf, "%d.%d", deg, frac);
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

#if TEMP_SENSOR

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

#endif	/* TEMP_SENSOR */

#if THING_SPEAK
void tsData(char *data)
{
 sprintf((char *) dataBuffer,
	 "GET /update?key=" TS_KEY "&%s\n", data);
 sendData("184.106.153.149", (const char *) dataBuffer);
}
#endif	/* THING_SPEAK */

char emonData(char *data)
{
 sprintf((char *) dataBuffer,
	 "get /emoncms/input/post.json?%s"
	 "&apikey=" EMONCMS_KEY "\n",
	 data);
 for (char retry = 0; retry < 2; retry++)
 {
  char *p = sendData("192.168.1.111", (const char *) dataBuffer);
  if (p != 0)
  {
   failCount = 0;
   return(1);
  }
  printf(F3("**emonData retry %d\n"), retry);
#if ARDUINO_AVR_MEGA2560
  PORTG |= _BV(PG0);
  delay(2);
  PORTG &= ~_BV(PG0);
  delay(500);
#endif	/* ARDUINO_AVR_MEGA2560 */
 }
 updateFail();
 return(0);
}

#if RTC_CLOCK

float rtcTemp()
{
 int val = RTC.temperature();
 float degC = val / 4.0;
 printf(F3("rtc temp "));
 printTemp(degC);
 printf(F3(" C "));
 float degF = (degC * 9.0) / 5.0 + 32.0;
 printTemp(degF);
 printf(F3(" F\n"));
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
 return;
}

#endif  /* TEMP_SENSOR */

#if CURRENT_SENSOR

void initCurrent(char isr)
{
 curPSave = 0;			// clear saved current pointer
 iChan = 0;			// initialize channel number
 ADCSRB = 0;			// set adc channel
 for (unsigned char i = 0; i < ADCCHANS; i++)
 {
  P_CURRENT p = &iData[i];
  p->iTime = 0;
  p->lastTime = 0;
  p->lastIRms0 = 0.0;
  p->lastIRms = 0.0;
  p->offsetI = ADC_COUNTS >> 1;
  p->iCal = 1.0;
  p->iRatio = p->iCal * ((5000 / 1000.0) / (ADC_COUNTS));
  p->adc = -1;
  p->count = 0;
 }
 iData[0].node = CURRENT0_NODE;
#if ADCCHANS > 1
 iData[1].node = CURRENT1_NODE;
#endif
 tempSumI = 0.0;
 adcState = 0;
 cState = 0;
 sampleCount = 100;
 if (isr)
 {
  pinMode(10, OUTPUT);		// pb4
  pinMode(11, OUTPUT);		// pb5
  pinMode(8, OUTPUT);		// ph5
  pinMode(9, OUTPUT);		// ph6
  pinMode(17, OUTPUT);		// pd4
  printf(F3("attach interrupt\n"));
  digitalWrite(7, HIGH);
  Timer3.attachInterrupt(timer3, 1000000L / 600);
  digitalWrite(7, LOW);
 }
}

void printCurrent()
{
 char tmp[10];

 printf(F3("iVcc %d\n"), vcc);
 for (unsigned char i = 0; i < ADCCHANS; i++)
 {
  P_CURRENT p = &iData[i];
//  printf("iRatio %s\n", dtostrf(p->iRatio, 8, 6, tmp));
//  printf("iCal %s\n", dtostrf(p->iCal, 8, 6, tmp));
  printf(F3("iRms %s\n"), dtostrf(p->iRms, 4, 2, tmp));
  if (p->iTime != 0)
   printTime(p->iTime);
   

  if (abs(p->iRms - p->lastIRms) > .05)
  {
   p->lastIRms = p->iRms;
   printf(F3("iRms %s\n"), dtostrf(p->iRms, 4, 2, tmp));
  }
 }
}

void currentCheck()
{
 P_CURRENT p = curPSave;
 if (p == 0)			/* if no current saved */
 {
  for (unsigned char i = 0; i < ADCCHANS; i++) /* for all channels */
  {
   p = &iData[i];
   if (p->iTime != 0)		/* if current reading */
   {
    p->lastTime = p->iTime;
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
     sprintf(buf, "time=%ld&node=%d&csv=%s",
	     p->lastTime, p->node, dtostrf(p->iRms, 4, 2, tmp));
     printf(F3("%s\n"), buf);
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
  if (cState == 0)		/* if time to send last value */
  {
   sprintf(buf, "time=%ld&node=%d&csv=%s", /* format last reading sent */
           p->iTime - 1, p->node, dtostrf(p->lastIRms1, 4, 2, tmp));
   cState = 1;			/* set to send current value */
  }
  else
  {
   sprintf(buf, "time=%ld&node=%d&csv=%s", /* format current reading */
           p->iTime, p->node, dtostrf(p->lastIRms0, 4, 2, tmp));
   p->iTime = 0;		/* reset time */
   cState = 0;			/* reset state */
   curPSave = 0;		/* clear save pointer */
  }
  printTime();
  printf(F3("%s\n"), buf);
  emonData(buf);		/* send data */
 }
}

void timer3()
{
 PORTB ^= _BV(PB4);
 PORTB |= _BV(PB5);
 
 if (adcState == 0)		// if sampling data
 {
  if (sampleCount)		// if sample count non zero
  {
   ADCSRB = 0;			// set channel number
   ADMUX = _BV(REFS0) | iChan;	// VCC Ref and channel

   ADCSRA |= _BV(ADSC) | _BV(ADIE); // start conversion
  }
 }
 else				// if calibrating voltage
 {
  ADCSRA |= _BV(ADSC) | _BV(ADIE); // start conversion
 }

 PORTB &= ~_BV(PB5);
}

ISR(ADC_vect)
{
 PORTH |= _BV(PH5);

 P_CURRENT p = &iData[iChan];
 if (adcState == 0)		// if sampling data
 {
  if (sampleCount)		// if not done
  {
   int sampleI = ADCL;
   sampleI |= (ADCH << 8);

   p->offsetI = (p->offsetI + (sampleI - p->offsetI) / 1024);
   float filteredI = sampleI - p->offsetI;
   float sqI = filteredI * filteredI;
   tempSumI += sqI;
   --sampleCount;		// count off a sample
   if (sampleCount == 0)	// if done
   {
    p->count++;
    p->adc = ADMUX;
    p->sumI = tempSumI;		// update sum
    tempSumI = 0.0;		// reset temporary sum
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    ADCSRB = 0;
    adcState = 1;		// set state to calibrate voltage
    PORTH |= _BV(PH6);
   }
  }
 }
 else				// if calibrating voltage 
 {
  long result = ADCL;
  result |= ADCH << 8;
  result = READVCC_CALIBRATION_CONST / result;
  vcc = (int) result;		// save result
  p->iRatio = p->iCal * ((result / 1000.0) / (ADC_COUNTS));
  p->iRms = p->iRatio * sqrt(p->sumI / SAMPLES);
  if (p->iTime == 0)		// if previous reading sent
  {
   if (abs(p->iRms - p->lastIRms0) > .1) // if change in current
   {
    p->sent++;
    p->lastIRms1 = p->lastIRms0; // set current value
    p->lastIRms0 = p->iRms;
    p->iTime = now();		// set time of change
   }
  }
  adcState = 0;			// set to start sampling
  iChan++;			// advance to next channel
  if (iChan >= ADCCHANS)	// if at end
   iChan = 0;			// back to channel zero
  sampleCount = SAMPLES;	// set number of samples
  PORTH &= ~_BV(PH6);
 }
 
 PORTH &= ~_BV(PH5);
}

#if 0
int adcRead(char pin)
{
 ADCSRB = 0; 			// chan 0 = 7
 ADMUX = _BV(REFS0) | pin;	// VCC Ref and channel

 // without a delay, we seem to read from the wrong channel
 //delay(1);

 // start the conversion
 ADCSRA |= _BV(ADSC); // Convert

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
#endif

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

