#if !INCLUDE
#ifdef ARDUINO_ARCH_AVR
#include <Arduino.h>
#endif

#include <Time.h>
#include "serial.h"
#include "wifi.h"
#include "dns.h"

#endif

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
  breakTime(t,tm);

  printf(F0("%02d/%02d/%d %2d:%02d:%02d\n"),
	 tm.Month,tm.Day,tmYearToCalendar(tm.Year),
	 tm.Hour,tm.Minute,tm.Second);
 }
}

char ntpSetTime()
{
 char status = 0;
 wifiMux();
 char server[16];
 char *ip = dnsLookup(server,(char *) "pool.ntp.org");
 if (ip)
 {
  sprintf((char *) cmdBuffer,F0("AT+CIPSTART=3,\"UDP\",\"%s\",123"),ip);
  wifiWriteStr(cmdBuffer,3000);

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

  sprintf((char *) cmdBuffer,"AT+CIPSEND=3,%d",timeLen);
  wifiStartData((char *) cmdBuffer,strlen(cmdBuffer),1000);

  int dataLen;
  wifiWriteTCPx((char *) dataBuffer,timeLen,&dataLen,5000);
  newLine();

  if (DBG)
   printBuf();

  int pos = findData(timeLen,&dataLen);
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
  }
  else
  {
   if (DBG)
    printf(F0("error time not set\n"));
   status = 1;
  }
  wifiClose(3,1000);
 }
 else
  status = 1;
 return(status);
}

#endif
