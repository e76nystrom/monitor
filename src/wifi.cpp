#if !defined(INCLUDE)
#ifdef ARDUINO_ARCH_AVR
#include <Arduino.h>
#include <EEPROM.h>
#include <CRC32.h>
#endif

#ifdef MEGA32
#include "WProgram.h"
#include "printf.h"
#endif

#ifdef WIN32
#include "stdio.h"
#include "string.h"
#include "conio.h"
#endif

#include "wdt.h"
#include "serial.h"

#define EXT

#ifdef DBG
#undef DBG
#endif

#define DBG 1

#endif	/* defined(INCLUDE) */

#define SSID_LOC 0
#define SSID_LEN 12
#define PASS_LOC (SSID_LOC + SSID_LEN)
#define PASS_LEN 20
#define ID_LOC (PASS_LOC + PASS_LEN)
#define ID_LEN 16
#define CSUM_LOC (ID_LOC + ID_LEN)
#define CSUM_LEN 4

#ifdef ARDUINO_ARCH_AVR

#define wifiAvail() (WIFI.available())
#define wifiGetc() ((char) WIFI.read())
#define wifiTxBusy() (0)
#define wifiPutc(c) WIFI.write(c)
#if DBG
#define putChar(c) DBGPORT.write(c)
#else
#define putChar(c)
#endif
#define putChar1(c) DBGPORT.write(c)
#define getChar(ch) \
 while (!DBGPORT.available()) \
  wdt_reset(); \
 ch = DBGPORT.read()
uint32_t sumEE();
void writeSumEE();
void readEE(char *buf, char addr, char len);
void writeEE(const char *buf, char addr, char eeLen);

#endif

#ifdef MEGA32

#define wifiAvail() U4STAbits.URXDA
#define wifiGetc() U4RXREG
#define wifiTxBusy() U4STAbits.UTXBF
#define wifiPutc(c) U4TXREG = c
#define putChar(c) Serial.write(c)
#define putChar1(c) Serial.write(c)

#endif

#ifdef WIN32

int sioAvailable();
int sioGetc();
void sioPutc(char c);

#define wifiAvail() sioAvailable()
#define wifiGetc() sioGetc()
#define wifiTxBusy() (0)
#define wifiPutc(c) sioPutc(c)
#if DBG
#define putChar(c) putchar(c)
#else
#define putChar(c)
#endif
#define putChar1(c) putchar(c)
#define getChar(ch) ch = _getch()

#include "stdint.h"

#if !INCLUDE
#include "Windows.h"
#include "Winbase.h"

uint32_t millis()
{
 ULONGLONG t = GetTickCount64();
 return ((uint32_t) t);
}
#else
uint32_t millis();
#endif

EXT char ssid[SSID_LEN];
EXT char pass[PASS_LEN];

#endif	/* WIN32 */

void dbgChar(char ch);
char readStr(char *buf, int bufLen);
char *sendData(const char *ip, const char *data);
char *sendData(const char *ip, const char *data, unsigned long timeout);
char *sendData(const char *ip, int port, const char *data,
	       unsigned long timeout);

void printBuf();
char *lc(char *p);
#ifdef ARDUINO_ARCH_AVR 
int strlen(const __FlashStringHelper *s);
#endif
int find(char *str1, char *str2);
int find(char *str1, char *str2, int offset, int len1);
int cmp(char *str1, char *str2, int size);
int cmp(char *str1, char *str2);
int findData(int cmdLen, int *datalen);
int getVal(char *p, int pos, int *rtnVal, int size);
void getData(char *dst, unsigned int dstSize, char *buf, unsigned int bufSize);
void wifiInitSio();

void wifiReset();
#if 0
char wifiRead();
#endif
void wifiClrRx();
#ifdef ARDUINO_ARCH_AVR 
void wifiPut(const __FlashStringHelper *s, int size);
void wifiPut(const __FlashStringHelper *s);
#endif
void wifiPut(char *s, int size);
void wifiPut(char *s);
void wifiTerm();
char wifiWriteStr(const char *s, unsigned long timeout);
char wifiWrite(char *s, int size, unsigned long timeout);
#ifdef ARDUINO_ARCH_AVR 
char wifiWriteStr(const __FlashStringHelper *s, unsigned long timeout);
char wifiWrite(const __FlashStringHelper *s, int size, unsigned long timeout);
#endif
void wifiMux();
char *wifiGetIP(char *buf);
void wifiCWMode();
void wifiQuit();
char wifiJoin();
void wifiStart(int chan, char *protocol, char *ip, int port,
	       unsigned long timeout);
void wifiStartData(char *s, int size, unsigned long timeout);
void wifiWriteData(char *s, int size, unsigned long timeout);
char *wifiWriteTCPx(char *s, int size, int *dataLen, unsigned long timeout);
void wifiClose(int port, unsigned long timeout);

#define WIFI_RESET 2

EXT char stringBuffer[80];
EXT char dataBuffer[128];	// buffer for input data
EXT char packetRsp[384];
EXT char cmdBuffer[64];
EXT char *rsp;
EXT unsigned int len;
EXT char id[ID_LEN];

#define RSPLEN (sizeof(packetRsp) - 1)

#if !defined(INCLUDE)

void dbgChar(char ch)
{
 if (ch == '\n')
 {
  putChar('\r');
  putChar(ch);
 }
 else if (ch >= ' ')
  putChar(ch);
}

#ifdef ARDUINO_ARCH_AVR

uint32_t sumEE()
{
 uint32_t checksum = ~0L;
 for (int i = 0; i < CSUM_LOC; i++)
 {
  char data = EEPROM.read(i);
  checksum = CRC32::update(checksum, data);
 }
 return(checksum);
}

void writeSumEE()
{
 uint32_t checksum = sumEE();
 writeEE((char *) &checksum, CSUM_LOC, sizeof(checksum));
}

void readEE(char *buf, char addr, char len)
{
 char *p = buf;
 while (--len >= 0)
 {
  char ch = EEPROM.read(addr);
  if (ch == 0)
   break;
  *p++ = ch;
  addr++;
 }
 *p++ = 0;
}

void writeEE(const char *buf, char addr, char eeLen)
{
 char len = strlen(buf);
 char i;
 for (i = 0; i < len; i++)
 {
  EEPROM.write(addr,*buf);
  addr++;
  buf++;
 }
 for (; i < eeLen; i++)
 {
  EEPROM.write(addr, 0);
  addr++;
 }
}

#endif

#ifdef WIN32

void readEE(char *buf, char addr, char len)
{
 char *p = buf;
 char *src;
 if (addr == SSID_LOC)
 {
  src = ssid;
 }
 else if (addr == PASS_LOC)
 {
  src = pass;
 }
 while (--len >= 0)
 {
  char ch = *src++;
  if (ch == 0)
   break;
  *p++ = ch;
 }
 *p++ = 0;
}

#endif

#define WIFIBAUDRATE 9600

void putx4(char c)
{
#ifdef MEGA32
 while (U4STAbits.UTXBF)
  ;
 U4TXREG = c;
 if (c == '\n')
 {
  while (U4STAbits.UTXBF)
   ;
  U4TXREG = '\r';
 }
#endif

#ifdef ARDUINO_ARCH_AVR
 WIFI.write(c);
 if (c == '\n')
 {
  WIFI.write('\r');
 }
#endif
}

char readStr(char *buf, int bufLen)
{
 int len = 0;
 char *p;
 char ch;
 p = buf;
 while (1)
 {
  getChar(ch);
  if ((ch == '\n') || (ch == '\r'))
  {
   if (len < bufLen)
   {
    *p++ = 0;
   }
   newLine();
   break;
  }
  else if ((ch == 8) || (ch == 127))
  {
   if (len > 0)
   {
    --len;
    --p;
    putChar1(8);
    putChar1(' ');
    putChar1(8);
   }
  }
  else
  {
   if (len < bufLen)
   {
    putChar1(ch);
    *p++ = ch;
    len++;
   }
  }
 }
 return(len);
}

char *sendData(const char *ip, const char *data)
{
 return(sendData(ip,80,data,5000));
}

char *sendData(const char *ip, const char *data, unsigned long timeout)
{
 return(sendData(ip,80,data,timeout));
}

char *sendData(const char *ip, int port, const char *data,
	       unsigned long timeout)
{
 int cmdLen = strlen((const char *) data);
 if (DBG)
  printf(F0("sendData %d %s\n"),cmdLen,data);

 sprintf((char *) cmdBuffer,F0("AT+CIPSTART=4,\"TCP\",\"%s\",%d"),ip,port);
 wifiWriteStr(cmdBuffer,4000);

 sprintf((char *) cmdBuffer,F0("AT+CIPSEND=4,%d"),cmdLen);
 wifiStartData((char *) cmdBuffer,strlen(cmdBuffer),1000);

 int dataLen = 0;
 char *p = wifiWriteTCPx((char *) data,cmdLen,&dataLen,timeout);

 if (p != 0)
 {
  if (find(p,(char *) F0(",CLOSED")) < 0)
  {
   wifiClose(4,1000);
  }

  *(p + dataLen) = 0;
  if (DBG)
   printf(F0("\nlength %d dataLen %d %s\n"),len,dataLen,p);
 }

 return(p);
}

#if DBG

void printBuf()
{
 printf(F0("\nlen %d\n"),len);
 char *p = (char *) packetRsp;
 int col = 8;			/* number of columns */
 for (unsigned int i = 0; i < len; i++)
 {
  if (col == 8)			/* if column 0 */
  {
   printf(F0("%08x  "),(int) p);
  }
  int val = *p++ & 0xff;
  char ch = ' ';
  if ((val >= ' ') && (val < 127))
   ch = val;
  printf(F0("%02x %c "),val,ch); /* output value */
  --col;			/* count off a column */
  if (col == 0)			/* if at end of line */
  {
   col = 8;			/* reset column counter */
   if (len != 0)		/* if not done */
    printf(F0("\n"));
  }
 }
 printf(F0("\n"));
}

#else
void printBuf()
{
}
#endif

#ifdef ARDUINO_ARCH_AVR

int strlen(const __FlashStringHelper *s)
{
 PGM_P p = reinterpret_cast <PGM_P> (s);
 int len = 0;
 char ch;
 while ((ch = pgm_read_byte(p++)) != 0)
  len++;
 return(len);
}

#endif

char *lc(char *p)
{
 char *p0 = p;
 char ch;
 while ((ch = *p0) != 0)
 {
  if ((ch >= 'A')
  &&  (ch <= 'Z'))
   ch += 'a' - 'A';
  *p0++ = ch;
 }
 return(p);
}

int find(char *str1, char *str2)
{
 unsigned int len1 = strlen((const char *) str1);
 unsigned int len2 = strlen((const char *) str2);
 int offset = 0;
// printf("find len1 %d len2 %d %s\n",len1,len2,str2);
 len1 -= len2;
 if (len1 > 0)
 {
  while (--len1 > 0)
  {
   if (cmp(str1,str2,len2))
   {
//    printf("offset %d\n",offset);
    return(offset);
   }
   str1++;
   offset++;
  }
 }
// printf("not found\n");
 return(-1);
}

int find(char *str1, char *str2, int offset, int len1)
{
 int len2 = strlen((const char *) str2);
// printf("find offset %d len1 %d len2 %d %s\n",offset,len1,len2,str2);
 str1 += offset;
 len1 -= offset;
 len1 -= len2;
 if (len1 > 0)
 {
  while (len1 >= 0)
  {
//   printf("%2d %c\n",len1,*str1);
   if (cmp(str1,str2,len2))
   {
    offset += len2;
//    printf("offset %d\n",offset);
    return(offset);
   }
   str1++;
   offset++;
   --len1;
  }
 }
// printf("not found\n");
 return(-1);
}

int cmp(char *str1, char *str2, int size)
{
 while (--size >= 0)
 {
  if (*str1++ != *str2++)
  {
   return(0);
  }
 }
 return(1);
}

int cmp(char *str1, char *str2)
{
 while (1)
 {
  if (*str1 == 0)
   return(0);
  if (*str2 == 0)
   return(0);
  if (*str1++ != *str2++)
  {
   return(0);
  }
 }
 return(1);
}

int getVal(char *p, int offset, int *rtnVal, int size)
{
 *rtnVal = 0;
 char ch;
 int val = 0;
 p += offset;
 while (--size >= 0)
 {
  ch = *p;
  if ((ch >= '0') && (ch <= '9'))
  {
   val *= 10;
   val += (ch - '0');
   p++;
   offset += 1;
  }
  else
  {
   *rtnVal = val;
  }
 }
 return(offset);
}

int findData(int cmdLen, int *dataLen)
{
 *dataLen = 0;
 int pos = find((char *) packetRsp,(char *) F0("+IPD,"),cmdLen,(int) len);
 if (pos >= 0)
 {
  pos = find((char *) packetRsp,(char *) ",",pos,(int) len);
  if (pos > 0)
  {
   pos = getVal((char *) packetRsp,pos,dataLen,(int) (len - pos));
   pos += 1;
   return(pos);
  }
 }
 return(-1);
}

void getData(char *dst, unsigned int dstSize, char *buf, unsigned int bufSize)
{
 --dstSize;			/* allow for null at end */
 while (--dstSize >= 0)
 {
  if (bufSize > 0)
  {
   --bufSize;
   *dst++ = *buf++;
  }
  else
  {
   break;
  }
 }
 *dst++ = 0;
}

void wifiReset()
{
 printf(F3("wifiReset\n"));
 digitalWrite(WIFI_RESET, LOW);
 delay(200);
 digitalWrite(WIFI_RESET, HIGH);
 if (DBG)
  printf(F3("flushing wifi input\n"));
 while (wifiAvail())
 {
  char ch = wifiGetc();
  if (ch != 0)
   putChar(ch);
 }
 char retry = 5;
 while (--retry >= 0)
 {
  delay(1000);
  if (wifiWriteStr(F2("AT"), 1000))
   break;
 }
}

void wifiInitSio()
{
#ifdef MEGA32
 IEC0bits.INT4IE = 0;
 IFS0bits.INT4IF = 0;
 U4MODE = 0;
 U4MODEbits.ON = 1;
 U4BRG = (F_CPU / (16 * WIFIBAUDRATE)) - 1;
 U4STA = 0;
 U4STAbits.URXEN = 1;
 U4STAbits.UTXEN = 1;
#endif

#ifdef ARDUINO_ARCH_AVR
 WIFI.begin(WIFIBAUDRATE);
#endif

 }

#if 0
char wifiRead()
{
#ifdef MEGA32
 while (!U4STAbits.URXDA)
  ;
 return(U4RXREG);
#endif

#ifdef ARDUINO_ARCH_AVR
 int ch;
 while ((ch = WIFI.read()) < 0)
  ;
 return((char) ch);
#endif

#ifdef WIN32
 return(sioGetc());
#endif
}
#endif

void  wifiClrRx()
{
 rsp = (char *) packetRsp;
 len = 0;

#ifdef MEGA32
 if (U4STAbits.OERR)
  U4STACLR = _U4STA_OERR_MASK;
#endif

 while (wifiAvail())
 {
  wdt_reset();
  if (wifiGetc())
  {
  }
 }
}

void wifiPut(char *s)
{
 wifiPut(s,strlen(s));
}

void wifiPut(char *s, int size)
{
 char ch;
 while (--size >= 0)
 {
  while (wifiTxBusy())
  {
   wdt_reset();
   if (wifiAvail())
   {
    ch = wifiGetc();
    *rsp++ = ch;
    len++;
    dbgChar(ch);
   }
  }
  wifiPutc(*s++);
 }
}

#ifdef ARDUINO_ARCH_AVR

void wifiPut(const __FlashStringHelper *s)
{
 wifiPut(s,strlen(s));
}

void wifiPut(const __FlashStringHelper *s, int size)
{
 PGM_P p = reinterpret_cast <PGM_P> (s);
 char ch;
 while (--size >= 0)
 {
  while (wifiTxBusy())
  {
   wdt_reset();
   if (wifiAvail())
   {
    ch = wifiGetc();
    *rsp++ = ch;
    len++;
    dbgChar(ch);
   }
  }
  wifiPutc(pgm_read_byte(p++));
 }
}

#endif

void wifiTerm()
{
 char c;
 const char *s;
 s = "\r\n";
 while ((c = *s++) != 0)
 {
  while (wifiTxBusy())
  {
   wdt_reset();
   if (wifiAvail())
   {
    char ch = wifiGetc();
    *rsp++ = ch;
    len++;
    dbgChar(ch);
   }
  }
  wifiPutc(c);
 }
}

char wifiWriteStr(const char *s, unsigned long timeout)
{
 return(wifiWrite((char *) s, strlen(s), timeout));
}

char wifiWrite(char *s, int size, unsigned long timeout)
{
 wifiClrRx();
 if (DBG)
  printf(F0("\nSending %d "), size);
 timeout += millis();

 wifiPut(s,size);
 wifiTerm();

 char result = 0;
 char last = 0;
 while (timeout >= millis())
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   if (len < RSPLEN)
   {
    *rsp++ = ch;
    len++;
   }
   dbgChar(ch);
   if ((last == 'O') && (ch == 'K'))
   {
    timeout = millis() + 10;
    result = 1;
   }
   last = ch;
  }
 }
 *rsp++ = 0;
 return(result);
}

#ifdef ARDUINO_AVR_PRO

char wifiWriteStr(const __FlashStringHelper *s, unsigned long timeout)
{
 return(wifiWrite(s, strlen(s), timeout));
}

char wifiWrite(const __FlashStringHelper *s, int size, unsigned long timeout)
{
 wifiClrRx();
 if (DBG)
  printf(F0("\nSending %d "), size);
 timeout += millis();

 wifiPut(s,size);
 wifiTerm();

 char result = 0;
 char last = 0;
 while (timeout >= millis())
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   if (len < RSPLEN)
   {
    *rsp++ = ch;
    len++;
   }
   dbgChar(ch);
   if ((last == 'O') && (ch == 'K'))
   {
    timeout = millis() + 10;
    result = 1;
   }
   last = ch;
  }
 }
 *rsp++ = 0;
 return(result);
}

#endif

void wifiMux()
{
 wifiWriteStr(F2("AT+CIPMUX=1"),1000);
}

char *wifiGetIP(char *buf)
{
 wifiWriteStr(F2("AT+CIFSR"),1000);

 char *dst = buf;
 int pos = find((char *) packetRsp,(char *) F0("STAIP,\""),0,(int) len);
 if (pos > 0)
 {
  char *p = (char *) &packetRsp[pos];
  char ch;
  while ((ch = *p++) != 0)
  {
   if (ch == '"')
    break;
   *dst++ = ch;
  }
 }
 *dst = 0;
 return(buf);
}

void wifiCWMode()
{
 wifiWriteStr("AT+CWMODE=1",1000);
}

void wifiQuit()
{
 wifiWriteStr("AT+CWQAP",1000);
}

char wifiJoin()
{
 wifiCWMode();
#if ARDUINO_ARCH_AVR
 strcpy(cmdBuffer,"AT+CWJAP=\"");
 readEE(stringBuffer,SSID_LOC,SSID_LEN);
 strcat(cmdBuffer,stringBuffer);
 strcat(cmdBuffer,"\",\"");
 readEE(stringBuffer,PASS_LOC,PASS_LEN);
 strcat(cmdBuffer,stringBuffer);
 strcat(cmdBuffer,"\"");
#endif
#if MEGA32
#endif
 return(wifiWrite(cmdBuffer,strlen(cmdBuffer),8000));
}

void wifiStart(int chan, char *protocol, char *ip, int port,
	       unsigned long timeout)
{
 sprintf(cmdBuffer,F0("AT+CIPSTART=%d,\"%s\",%s,%d\n"),
	 chan,protocol,ip,port);
 wifiWrite(cmdBuffer,strlen(cmdBuffer),timeout);
}

void wifiStartData(char *s, int size, unsigned long timeout)
{
 wifiClrRx();
 if (DBG)
  printf(F0("StartData Sending %d "),size);
 timeout += millis();

 wifiPut(s,size);
 wifiTerm();

 char last = 0;
 while (timeout >= millis())
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   dbgChar(ch);
   if ((last == '>') && (ch == ' '))
    timeout = millis() + 10;
   last = ch;
  }
 }
}

void wifiWriteData(char *s, int size, unsigned long timeout)
{
 wifiClrRx();
 if (DBG)
  printf(F0("Data Sending %d "),size);
 timeout += millis();

 wifiPut(s, size);
 wifiTerm();

 const char *chkstr; 
 chkstr = F1("SEND OK");
 unsigned int chklen = strlen(chkstr);
 while (timeout >= millis())
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   if (len < RSPLEN)
   {
    *rsp++ = ch;
    len++;
    if (len > chklen)
    {
     if (cmp(rsp - chklen,(char *) chkstr,chklen))
     {
      timeout = millis() + 10;
     }
    }
   }
  }
 }
}

#define IPD F1("+IPD,")
#define IPDLEN (sizeof(IPD) - 1)

#define OK F1("OK")
#define OKLEN (sizeof(OK) - 1)

#define RSP_ERR F1("ERROR")
#define RSP_ERRLEN (sizeof(RSP_ERR) - 1)

#define CLOSE F1("CLOSED")
#define CLOSE_LEN (sizeof(CLOSED) - 1)

#define CHKLEN RSP_ERRLEN

char *wifiWriteTCPx(char *s, int size, 
		    int *dataLen, unsigned long timeout)
{
 wifiClrRx();
 if (DBG)
  printf(F0("Sending %d\n"), size);
 timeout += millis();

 wifiPut(s,size);
 wifiTerm();

 int rspNum = 0;
 int dLen = 0;
 char *p;
 p = 0;
 char ok = 0;
 size += IPDLEN;
 while (timeout >= millis())
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   dbgChar(ch);
   if (len < RSPLEN)		// if room in buffer
   {
    *rsp++ = ch;
    *rsp = 0;
    len++;
    if (len > (unsigned int) size) // if past message echo
    {
     printf("% d %02x %c\n",rspNum,ch,ch);
     switch (rspNum)
     {
     case 0:
      if (cmp(rsp - IPDLEN,(char *) IPD,IPDLEN))
      { 
       rspNum = 1;
      }
      else if (cmp(rsp - OKLEN,(char *) OK,OKLEN))
      {
       ok = 1;
       printf("ok set\n");
      }
      break;

     case 1:			// ipd found look for ,
      if (ch == ',')
      {
       rspNum = 2;
       dLen = 0;
      }
      break;

     case 2:			// comma found read length and look for :
      if (ch == ':')
      {
       rspNum = 3;
       *dataLen = dLen;
       p = rsp;
      }
      else
      {
       dLen *= 10;
       dLen += ch - '0';
      }
      break;

     case 3:			// read response
      --dLen;
      if (dLen == 0)
      {
       rspNum = 4;
       if (ok)
       {
	timeout = millis() + 100;
	rspNum = 6;
       }
      }
      break;

     case 4:
     case 5:
      timeout = millis() + 100;
      if (cmp(rsp - OKLEN,(char *) OK,OKLEN))
      {
       rspNum++;
      }
      break;

     case 6:
      if (cmp(rsp - IPDLEN,(char *) IPD,IPDLEN))
      { 
       rspNum = 1;
      }
      timeout = millis() + 100;
      break;
     }
     if (rspNum != 3)
     {
      if (cmp(rsp - RSP_ERRLEN,(char *) RSP_ERR,RSP_ERRLEN))
      {
       rspNum = 6;
       timeout = millis() + 10;
      }
     }
    }
   }
  }
 }
 return(p);
}

void wifiClose(int chan, unsigned long timeout)
{
 wifiClrRx();
 timeout += millis();

 sprintf(cmdBuffer,F0("AT+CIPCLOSE=%d"),chan);
 wifiPut((char *) cmdBuffer);
 wifiTerm();

 const char *chkstr;
 chkstr = F1("CLOSED");
 unsigned int chklen = strlen(chkstr);
 while (timeout >= millis())
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   dbgChar(ch);
   if (len < RSPLEN)
   {
    *rsp++ = ch;
    len++;
    if (len > chklen)
    {
     if (cmp(rsp - chklen,(char *) chkstr,chklen))
     {
      timeout = millis() + 10;
     }
    }
   }
  }
 }
}

#endif
