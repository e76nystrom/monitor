#ifdef ARDUINO_ARCH_AVR
#include <Arduino.h>
#include <Wire.h>
#define RTC_CLOCK
#endif

#ifdef MEGA32
#include "WProgram.h"
#include "printf.h"
#define DHT_SENSOR
#endif

#include <OneWire.h>
#include <DallasTemperature.h>

#ifdef DHT_SENSOR
#include <DHT.h>
#endif

#ifdef RTC_CLOCK
#include <DS3232RTC.h>
#endif

#include "Time.h"
#include "string.h"
#include "dns.h"
#include "wifi.h"

#ifdef ARDUINO_ARCH_AVR
#include <stdio.h>

static FILE uartout = {0};

static int putx(char c, FILE *stream)
{
 if (c == '\n')
  Serial.write('\r');
 Serial.write(c);
 return 0;
}
#endif

#ifdef MEGA32
#define TS_KEY "67TDONLKRDNVF7L4"
#define EMONCMS_NODE "0"
#endif

#ifdef ARDUINO_ARCH_AVR
#define TS_KEY "86Z0KTDYLEC28FU3"
#define EMONCMS_NODE "1"
#endif

// Data wire is plugged into pin 3 on the Arduino
#define ONE_WIRE_BUS 3

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

#ifdef MEGA32
DeviceAddress insideThermometer =
{0x10, 0xDC, 0x5D, 0xD4, 0x01, 0x08, 0x00, 0xE9};
#endif

#ifdef ARDUINO_ARCH_AVR
DeviceAddress insideThermometer =
{0x28, 0xB8, 0x50, 0x9B, 0x06, 0x00, 0x00, 0x89};
#endif

#ifdef DHT_SENSOR
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE, 15);
#endif

// Target Access Point
#define ssid         "nystrom"
#define pass         "minidonk"

unsigned long time = 0;

const int NTP_PACKET_SIZE = 48; 
byte packetBuffer[NTP_PACKET_SIZE];
char tempData[64];

typedef struct
{
 char version;
 char stratum;
 char poll;
 char precision;
 int32_t rootDelay;
 int32_t rootDispersion;
 int32_t refID;
 int32_t RefSec;
 int32_t refFrac;
 int32_t originSec;
 int32_t originFrac;
 int32_t rcvSec;
 int32_t rcvFrac;
 int32_t txSec;
 int32_t txFrac;
} T_NTP, *P_NTP;

#define wifiSerial Serial1

void loopData();
void printTime();
void printTime(time_t t);
char *cpyStr(char *dst, const char *str);
void printTemp(float temp);
char *writeTemp(char *buf, float temp);
float printTemperature(DeviceAddress deviceAddress);
void tsData(char *data);
void emonData(char *data);
void setTime();
#ifdef RTC_CLOCK
float rtcTemp();
#endif
void putx0(void *p, char c);
void putx(char c);

void setup()
{
 char ch;
 Serial.begin(19200);

#ifdef ARDUINO_ARCH_AVR
 fdev_setup_stream(&uartout, putx, NULL, _FDEV_SETUP_WRITE);
 stdout = &uartout;

 pinMode(WIFI_RESET, OUTPUT);
 digitalWrite(WIFI_RESET, HIGH);
#endif

#if MEGA32
 init_printf(NULL,putx0);
#endif

 printf("starting\n");

 sensors.begin();
 sensors.setResolution(insideThermometer, 12);
 
#ifdef DHT_SENSOR
 dht.begin();
#endif

 initSio();
 putx('0');
 printf("%x\n",wifiAvail());

#if 1
 while (wifiAvail())
 {
  ch = wifiGetc();
  putx(ch);
 }
 putx('1');
#endif

 unsigned long t = millis() + 1000;
 printf("flushing input\n");
 while (t > millis())
 {
  ch = Serial.read();
 }

 t = millis() + 5000;
 ch = 0;
 printf("any char for cmd mode...");
 while (t > millis())
 {
  if (Serial.available())
  {
   ch = Serial.read();
   break;
  }
 }
 printf("\n");
 
 if (ch != 0)
 {
  printf("command mode\n");
  while (1)
  {
   if (Serial.peek() >= 0)
   {
    char ch = Serial.read();
    Serial.write(ch);
    if (ch == 'x')
     break;
    else if (ch == 'j')
    {
     wifiWriteStr("AT+CWJAP=\"nystrom\",\"minidonk\"",6000);
    }
    else if (ch == 'o')
    {
     wifiWriteStr("AT",1000);
    }
    else if (ch == 'l')
    {
     wifiWriteStr("AT+CWLAP",3000);
    }
    else if (ch == 'q')
    {
     wifiWriteStr("AT+CWQAP",1000);
    }
    else if (ch == 's')
    {
     wifiWriteStr("AT+CIFSR",1000);
    }
    else if (ch == 'u')
    {
     wifiWriteStr("AT+CIPSTART=\"UDP\",\"129.6.15.28\",123",3000);
    }
    else if (ch == 'm')
    {
     wifiMux();
    }
    else if (ch == 't')
    {
     wifiWriteStr("AT+CIPSTART=4,\"TCP\",\"184.106.153.149\",80",4000);
    }
    else if (ch == 'z')
    {
     printf("\n");
     wifiCloseTCP(15000);
    }
    else if (ch == 'g')
    {
     printf("\n");
     loopData();
    }
    else if (ch == 'd')
    {
     printf("\n");
     setTime();
    }
#ifdef RTC_CLOCK
    else if (ch == 'v')
    {
     printf("\n");
     rtcTemp();
    }
#endif
#ifdef DHT_SENSOR
    else if (ch == 'h')
    {
     printf("\n");
     float h = dht.readHumidity();
     if (!isnan(h))
     {
      float t = dht.readTemperature(true);
      if (!isnan(t))
      {
       printf("temp ");
       printTemp(t);
       printf(" F humidity ");
       printTemp(h);
       printf("\n");
      }
      else
       printf("temp read failure\n");
     }
     else
      printf("humidity read failure\n");
    }
#endif
    else if (ch == 'w')
    {
     printf("\n");
     float temp = sensors.getTempF(insideThermometer);
     printf("temp ");
     Serial.print(temp);
     printf("F\n");
    }
    else if (ch == '?')
    {
     printf("\nntp.cpp\n");
    }
    else if (ch == 'c')
    {
     char buf[80];
     char *p;
     p = buf;
     unsigned int len = 0;
     printf("\nenter command\n");
     while (1)
     {
      ch = Serial.read();
      if (ch < 0)
       continue;
      if (ch == '\r')
      {
       printf("\nsend command\n");
       wifiClrRx();
       wifiWrite(buf,len,2000);
       wifiTerm();
       printf("\n");
       break;
      }
      else if (ch == 127)
      {
       if (len > 0)
       {
	--len;
	--p;
	Serial.write(0x8);
	Serial.write(' ');
	Serial.write(0x8);
       }
      }
      else
      {
       if (len < (sizeof(buf) - 1))
       {
	Serial.write(ch);
	len++;
	*p++ = ch;
       }
      }
     }
    }
   }
  }
 }
 wifiWriteStr("AT",1000);
 setTime();
#ifdef RTC_CLOCK
 setSyncProvider(RTC.get);
#endif
}

void loop()
{
 unsigned long t = millis();
 printTime();

 loopData();

 delay(t + 60000 - millis());
}

void loopData()
{
 sensors.requestTemperatures();

 float temp1 = printTemperature(insideThermometer);

#ifdef RTC_CLOCK
 float temp2 = rtcTemp();
#else
 float temp2 = 0;
#endif

#ifdef DHT_SENSOR
 float h = dht.readHumidity();
 float t = dht.readTemperature(true);
 printf("temp ");
 printTemp(t);
 printf(" F humidity ");
 printTemp(h);
 printf("\n");
#else
 float h = 0;
 float t = 0;
#endif

 char buf[128];
 char *p = cpyStr(buf,"field1=");
 p = writeTemp(p,temp1);
 p = cpyStr(p,"&field2=");
 p = writeTemp(p,temp2);
 p = cpyStr(p,"&field3=");
 p = writeTemp(p,h);
 p = cpyStr(p,"&field4=");
 writeTemp(p,t);
 tsData(buf);

 p = cpyStr(buf,"node="EMONCMS_NODE"&csv=");
 p = writeTemp(p,temp1);
 *p++ = ',';
 p = writeTemp(p,temp2);
 *p++ = ',';
 p = writeTemp(p,h);
 *p++ = ',';
 writeTemp(p,t);
 emonData(buf);
}

void printTime()
{
 printTime(now());
}

void printTime(time_t t)
{
 tmElements_t tm;
 breakTime(t,tm);

 printf("%02d/%02d/%d %2d:%02d:%02d\n",
	tm.Month,tm.Day,tmYearToCalendar(tm.Year),
	tm.Hour,tm.Minute,tm.Second);
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
 printf("%d.%d",deg,frac);
}

char *writeTemp(char *buf, float temp)
{
 int tmp = (int) (temp * 10);
 int deg = tmp / 10;
 int frac = tmp % 10;
 sprintf(buf,"%d.%d",deg,frac);
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

float printTemperature(DeviceAddress deviceAddress)
{
 float temp = sensors.getTempC(deviceAddress);
 if (temp == -127.00)
 {
  printf("Error getting temperature\n");
 }
 else
 {
  printf("sensor ");
  printTemp(temp);
  printf(" C ");
  temp = DallasTemperature::toFahrenheit(temp);
  printTemp(temp);
  printf(" F\n");
 }
 return(temp);
}

void tsData(char *data)
{
 sprintf((char *) dataBuffer,
	 "GET /update?key="TS_KEY"&%s\n",data);
 sendData("184.106.153.149",(const char *) dataBuffer);
}

void emonData(char *data)
{
 sprintf((char *) dataBuffer,
	 "get /emoncms/input/post.json?%s"
	 "&apikey=b53ec1abe610c66009b207d6207f2c9e\n",
	 data);
 sendData("192.168.1.111",(const char *) dataBuffer);
}

void setTime()
{
 wifiMux();
 wifiWriteStr("AT+CIPSTART=3,\"UDP\",\"129.6.15.28\",123",3000);

 memset(packetBuffer, 0, sizeof(packetBuffer));
 // Initialize values needed to form NTP request
 // (see URL above for details on the packets)
 packetBuffer[0] = 0b11100011; // LI, Version, Mode
 packetBuffer[1] = 0; // Stratum, or type of clock
 packetBuffer[2] = 6; // Polling Interval
 packetBuffer[3] = 0xEC; // Peer Clock Precision
 // 8 bytes of zero for Root Delay & Root Dispersion
 packetBuffer[12] = 49;
 packetBuffer[13] = 0x4E;
 packetBuffer[14] = 49;
 packetBuffer[15] = 52;

 wifiWriteStr("AT+CIPSEND=3,48",1000);
 printf("\n");

 wifiWriteData((char *) packetBuffer, sizeof(packetBuffer),3000);
 printBuf();

 int dataLen;
 int pos = findData((int) sizeof(packetBuffer),&dataLen);
 if (pos >= 0)
 {
  char *p = (char *) &packetRsp[pos + 40];
  time_t val = 0;
  for (int i = 0; i < 4; i++)
  {
   val <<= 8;
   val += *p++ & 0xff;
  }
//  printf("time %ld %lx\n",val,val);
 
  const time_t seventyYears = 2208988800UL;
  time_t epoch = val - seventyYears;

  setTime(epoch);

#ifdef RTC_CLOCK

 val = RTC.get();
 printTime(epoch);
 printTime(val);

 printf("time error %d\n",(int) (epoch - val));

 tmElements_t tm;
 breakTime(now(),tm);
 RTC.write(tm);

#endif

  printTime();
 }
 wifiClose(3,1000);
}

#ifdef RTC_CLOCK
float rtcTemp()
{
 int val = RTC.temperature();
 float degC = val / 4.0;
 printf("rtc temp ");
 printTemp(degC);
 printf(" C ");
 float degF = (degC * 9.0) / 5.0 + 32.0;
 printTemp(degF);
 printf(" F\n");
 return(degF);
}
#endif

void putx0(void *p, char c)
{
 Serial.write(c);
 if (c == '\n')
  Serial.write('\r');
}

void putx(char c)
{
 Serial.write(c);
 if (c == '\n')
  Serial.write('\r');
}
