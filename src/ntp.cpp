#define __NTP__

#if defined(WIFI_ESP32)

#include <WiFiUdp.h>

extern WiFiUDP udp;

void printBuf();
void printBuf(char *p, unsigned int len);

#endif	/* WIFI_ESP32 */

#ifdef ARDUINO_ARCH_AVR
#include <Arduino.h>
#endif

#ifdef ARDUINO_ARCH_STM32
#include <Arduino.h>
#endif /* ARDUINO_ARCH_STM32 */

#if	0 //def STM32MON
#include "stdio.h"
#include "string.h"
#include "millis.h"
#endif

#include <TimeLib.h>

#define EXT extern
#include "monitor.h"

#if defined(WIFI_SERIAL)
#include "wifiSerial.h"
#endif	/* WIFI_SERIAL */

#include "dns.h"
#include "ntp.h"

#if defined(__NTP_INC__)	// <-

const int NTP_PACKET_SIZE = 48; 

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

void printTime();
void printTime(time_t t);
void printTime(time_t t, bool flag);
char *fmtTime(char *buf, size_t len);
char *fmtTime(char *buf, size_t len, time_t t);

char ntpSetTime();

EXT unsigned long ntpStart;	/* reference for time compare */
EXT unsigned long ntpTimeout;	/* ntp timeout */
EXT char ntpIP[IP_ADDRESS_LEN];	/* ntp ip address */

#if defined(WIFI_SERIAL)
#define NTP_PORT "123"
#endif	/* WIFI_SERIAL */

#if defined(WIFI_ESP32)
#define NTP_PORT 123
#endif	/* WIFI_ESP32 */

#endif	/* __NTP_INC__ */ // ->

#if defined(__NTP__)

void printTime()
{
 if (DBG)
  printTime(now(), true);
}

void printTime(time_t t)
{
 if (DBG)
  printTime(t, true);
}

void printTime(time_t t, bool flag)
{
 if (DBG)
 {
  tmElements_t tm;
  breakTime(t, tm);

  printf(F0("%02d/%02d/%d %2d:%02d:%02d"),
	 tm.Month, tm.Day, tmYearToCalendar(tm.Year),
	 tm.Hour, tm.Minute, tm.Second);
  if (flag)
   printf(F0("\n"));
 }
}

char *fmtTime(char *buf, size_t len)
{
 return fmtTime(buf, len, now());
}

char *fmtTime(char *buf, size_t len, time_t t)
{
 char *p = buf;
 tmElements_t tm;
 breakTime(t, tm);

 int size = snprintf(buf, len, "%02d/%02d/%d %2d:%02d:%02d",
		     tm.Month, tm.Day, tmYearToCalendar(tm.Year),
		     tm.Hour, tm.Minute, tm.Second);
 return p + size;
}

char ntpSetTime()
{

 ntpTimeout = 5UL * 60UL * 1000UL; /* try again in 5 minutes if failure */

 char status = 0;
 for (char retry = 0; retry < 3; retry++)
 {
#if defined(WIFI_SERIAL)
  char ntpBuf[32];
  char result = dnsLookup((char *) ntpIP, argConv(F2("pool.ntp.org"), ntpBuf));
#endif  /* WIFI_SERIAL */

#if defined(WIFI_ESP32)
  printf("ntp dnsLookup try %d\n", retry);
  char result = dnsLookup((char *) ntpIP, "pool.ntp.org");
#endif  /* WIFI_ESP32 */

  if (result
  ||  (ntpIP[0] != 0))
   break;
 }

 if (ntpIP[0] != 0)
 {
  for (char retry = 0; retry < 1; retry++)
  {
   printf("ntp try %d\n", retry);

   memset(dataBuffer, 0, NTP_PACKET_SIZE);
   // Initialize values needed to form NTP request
   // (see URL above for details on the packets)
   dataBuffer[0] = (char) 0b11100011; /* LI, Version, Mode */
   dataBuffer[1] = 0; /* Stratum, or type of clock */
   dataBuffer[2] = 6; /* Polling Interval */
   dataBuffer[3] = (char) 0xEC; /* Peer Clock Precision */
   // 8 bytes of zero for Root Delay & Root Dispersion
   dataBuffer[12] = 49;
   dataBuffer[13] = 0x4E;
   dataBuffer[14] = 49;
   dataBuffer[15] = 52;

#if defined(WIFI_SERIAL)

   newLine();

   sprintf((char *) cmdBuffer, F0("AT+CIPSTART=3,\"UDP\",\"%s\"," NTP_PORT), ntpIP);
   wifiWriteStr(cmdBuffer, 3000);

   unsigned int timeLen = NTP_PACKET_SIZE;
   sprintf((char *) cmdBuffer, F0("AT+CIPSEND=3,%d"), timeLen);
   wifiStartData((char *) cmdBuffer, strlen(cmdBuffer), 1000);

   unsigned int dataLen;
   wifiWriteTCPx((char *) dataBuffer, timeLen, &dataLen, 5000);
   newLine();

   if (0)
    printBuf();

   int pos = findData(0, &dataLen);
   printf(F0("findData pos %d\n"), pos);

   if (pos >= 0)
   {
    char *p = (char *) &packetRsp[pos + 40];
    time_t val = 0;
    for (int i = 0; i < 4; i++)
    {
     val <<= 8;
     val += *p++ & 0xff;
    }
    printf(F0("time %lu %lx\n"), (unsigned long) val, (unsigned long) val);

    const time_t seventyYears = 2208988800UL; /* 1970 - 1900 */
    time_t epoch = val - seventyYears;

    printf("ntp set time\n");
    setTime(epoch);
    ntpTimeout = 24UL * 60UL * 60UL * 1000UL;
    status = 1;
   }

   wifiClose(3, 1000);
 
#endif  /* WIFI_SERIAL */

#if defined(WIFI_ESP32)

   // printf("ntp request ");
   // printBuf(dataBuffer, (unsigned int) NTP_PACKET_SIZE);

   IPAddress ip = IPAddress();
   ip.fromString((const char *) ntpIP);
   // printf("ntpIP %s port %d\n", ip.toString().c_str(), NTP_PORT);
   
   udp.beginPacket(ip, NTP_PORT);
   udp.write((const uint8_t *) dataBuffer, NTP_PACKET_SIZE);
   udp.endPacket();

   uint32_t t0 = millis();
   int i = 0;
   while ((millis()) - t0 < 3000)
   {
    delay(100);
    rspLen = udp.parsePacket();
    // printf("ntp parsePacket %2d len %d\n", i, rspLen);
    if (rspLen != 0)
    {
     udp.read((char *) packetRsp, rspLen);

     // printf("ntp response ");
     // printBuf();

     char *p = (char *) &packetRsp[40];
     time_t val = 0;
     for (int i = 0; i < 4; i++)
     {
      val <<= 8;
      val += *p++ & 0xff;
     }
     printf(F0("time %lu %lx\n"), (unsigned long) val, (unsigned long) val);

     const auto seventyYears = (time_t) 2208988800UL; /* 1970 - 1900 */
     time_t epoch = val - seventyYears;
     printTime(epoch);
     
     printf("ntp set time\n");
     setTime(epoch);
     ntpTimeout = 24UL * 60UL * 60UL * 1000UL;

     status = 1;
     break;
    }
    i += 1;
   }

#endif	/* WIFI_ESP32 */

   if (status)			/* if time set */
    break;
   delay(1000);			/* wait before retry */
  }
 }

 if (DBG && (status == 0))
 {
  printf(F0("**err time not set\n"));
 }

 ntpStart = millis();		    /* save current time */
 return(status);
}

#endif	/* __NTP__ */
