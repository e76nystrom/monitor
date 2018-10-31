#if !INCLUDE
#ifdef ARDUINO_ARCH_AVR
#include <Arduino.h>
#endif

#include <Time.h>
#include "serial.h"
#include "wifi.h"
#include "dns.h"

#define EXT

#endif	/* !INCLUDE */

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
char ntpSetTime();

EXT unsigned long ntpStart;	/* reference for time compare */
EXT unsigned long ntpTimeout;	/* ntp timeout */
EXT char ntpIP[IP_ADDRESS_LEN];	/* ntp ip address */

#if !INCLUDE

void printTime()
{
 if (DBG)
  printTime(now());
}

void printTime(time_t t)
{
 if (DBG)
 {
  tmElements_t tm;
  breakTime(t, tm);

  printf(F0("%02d/%02d/%d %2d:%02d:%02d\n"),
	 tm.Month, tm.Day, tmYearToCalendar(tm.Year),
	 tm.Hour, tm.Minute, tm.Second);
 }
}

char ntpSetTime()
{
 if ((millis() - ntpStart) > ntpTimeout)
 {
  char status = 0;
  for (char retry = 0; retry < 3; retry++)
  {
   char result = dnsLookup(ntpIP, F0("pool.ntp.org"));
   if (result
   || (ntpIP[0] != 0))
    break;
  }

  if (ntpIP[0] != 0)
  {
   for (char retry = 0; retry < 3; retry++)
   {
    sprintf((char *) cmdBuffer, F0("AT+CIPSTART=3,\"UDP\",\"%s\",123"), ntpIP);
    wifiWriteStr(cmdBuffer, 3000);

    memset(dataBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    dataBuffer[0] = 0b11100011; // LI, Version, Mode
    dataBuffer[1] = 0; // Stratum, or type of clock
    dataBuffer[2] = 6; // Polling Interval
    dataBuffer[3] = 0xEC; // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    dataBuffer[12] = 49;
    dataBuffer[13] = 0x4E;
    dataBuffer[14] = 49;
    dataBuffer[15] = 52;

    newLine();
    int timeLen = NTP_PACKET_SIZE;

    sprintf((char *) cmdBuffer, F0("AT+CIPSEND=3,%d"), timeLen);
    wifiStartData((char *) cmdBuffer, strlen(cmdBuffer), 1000);

    int dataLen;
    wifiWriteTCPx((char *) dataBuffer, timeLen, &dataLen, 5000);
    newLine();

    if (0)
     printBuf();

    int pos = findData(timeLen, &dataLen);
    if (pos >= 0)
    {
     char *p = (char *) &packetRsp[pos + 40];
     time_t val = 0;
     for (int i = 0; i < 4; i++)
     {
      val <<= 8;
      val += *p++ & 0xff;
     }
//  printf(F0("time %ld %lx\n"), val, val);
 
     const time_t seventyYears = 2208988800UL;
     time_t epoch = val - seventyYears;

     setTime(epoch);
     ntpTimeout = 24UL * 60UL * 60UL * 1000UL;
     ntpStart = millis();
     status = 1;
    }
    wifiClose(3, 1000);
    if (status)
     break;
    else
    {
     ntpTimeout = 10UL * 60UL * 1000UL;
     ntpStart = millis();
     if (DBG)
      printf(F0("**err time not set\n"));
    }
    delay(100);
   }
  }
  return(status);
 }
 return(0);
}

#endif
