#if !defined(INCLUDE)
#ifdef ARDUINO_ARCH_AVR
#include <Arduino.h>
#endif

#ifdef MEGA32
#include "WProgram.h"
#include "printf.h"
#endif

#ifdef WIN32
#include "stdio.h"
#include "string.h"
#endif

#define EXT
#endif

void sendData(const char *ip, const char *data);

void printBuf();
int find(char *str1, char *str2);
int find(char *str1, char *str2, int offset, int len1);
int cmp(char *str1, char *str2, int size);
int cmp(char *str1, char *str2);
int findData(int cmdLen, int *datalen);
int getVal(char *p, int pos, int *rtnVal, int size);
void getData(char *dst, unsigned int dstSize, char *buf, unsigned int bufSize);
void initSio();

#ifdef MEGA32

#define wifiAvail() U4STAbits.URXDA
#define wifiGetc() U4RXREG
#define wifiTxBusy() U4STAbits.UTXBF
#define wifiPutc(c) U4TXREG = c
#define putChar(c) Serial.write(c)

#endif

#ifdef ARDUINO_ARCH_AVR

#define wifiAvail() (Serial1.available())
#define wifiGetc() ((char) Serial1.read())
#define wifiTxBusy() (0)
#define wifiPutc(c) Serial1.write(c)
#define putChar(c) Serial.write(c)

#endif

#ifdef WIN32

int sioAvailable();
int sioGetc();
void sioPutc(char c);

#define wifiAvail() sioAvailable()
#define wifiGetc() sioGetc()
#define wifiTxBusy() (0)
#define wifiPutc(c) sioPutc(c)
#define putChar(c) putchar(c)

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

#endif

#if 0
char wifiRead();
#endif
void wifiClrRx();
void wifiPut(char *s, int size);
void wifiPut(char *s);
void wifiTerm();
void wifiWriteStr(const char *s, unsigned long timeout);
void wifiWrite(char *s, int size, unsigned long timeout);
void wifiMux();
void wifiStart(int chan, char *protocol, char *ip, int port, int timeout);
void wifiStartData(char *s, int size, unsigned long timeout);
void wifiWriteData(char *s, int size, unsigned long timeout);
char *wifiWriteTCPx(char *s, int size, int *dataLen, unsigned long timeout);
void wifiWriteTCP(char *s, int size, unsigned long timeout);
void wifiCloseTCP(unsigned long timeout);
void wifiClose(int port,unsigned long timeout);

#define WIFI_RESET 2

EXT char dataBuffer[128];
EXT char cmdBuffer[64];
EXT char packetRsp[512];
EXT char *rsp;
EXT unsigned int len;

#define RSPLEN (sizeof(packetRsp) - 1)

#if !defined(INCLUDE)

#define BAUDRATE 9600

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
 Serial1.write(c);
 if (c == '\n')
 {
  Serial1.write('\r');
 }
#endif
}

void sendData(const char *ip, const char *data)
{
 int cmdLen = strlen((const char *) data);
 printf("sendData %d %s\n",cmdLen,data);

 wifiMux();

 sprintf((char *) cmdBuffer,"AT+CIPSTART=4,\"TCP\",\"%s\",80",ip);
 wifiWriteStr(cmdBuffer,4000);

 sprintf((char *) cmdBuffer,"AT+CIPSEND=4,%d",cmdLen);
 wifiStartData((char *) cmdBuffer,strlen(cmdBuffer),1000);

 wifiWriteTCP((char *) data,cmdLen,5000);
 *rsp = 0;

 int dataLen = 0;
 int pos = findData(cmdLen,&dataLen);
 if (pos >= 0)
 {
  getData((char *) cmdBuffer,sizeof(cmdBuffer),(char *) &packetRsp[pos],dataLen);
  printf("data %s\n",cmdBuffer);
 }
 if (find((char *) packetRsp,(char *) "CLOSED") < 0)
 {
  wifiCloseTCP(1000);
 }
}

void printBuf()
{
 printf("len %d\n",len);
 char *p = (char *) packetRsp;
 int col = 8;			/* number of columns */
 for (unsigned int i = 0; i < len; i++)
 {
  if (col == 8)			/* if column 0 */
  {
   printf("%08x  ",(int) p);
  }
  int val = *p++ & 0xff;
  char ch = ' ';
  if ((val >= ' ') && (val < 127))
   ch = val;
  printf("%02x %c ",val,ch);	/* output value */
  --col;			/* count off a column */
  if (col == 0)			/* if at end of line */
  {
   col = 8;			/* reset column counter */
   if (len != 0)		/* if not done */
    printf("\n");
  }
 }
 printf("\n");
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
/*
 char ch = *str1;
 if (ch < 32)
  ch = ' ';
 printf("str1 %02x %c str2 %s\n",*str1,ch,str2);
*/
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
 int pos = find((char *) packetRsp,(char *) "+IPD,",cmdLen,(int) len);
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

void initSio()
{
#ifdef MEGA32
 IEC0bits.INT4IE = 0;
 IFS0bits.INT4IF = 0;
 U4MODE = 0;
 U4MODEbits.ON = 1;
 U4BRG = (F_CPU / (16 * BAUDRATE)) - 1;
 U4STA = 0;
 U4STAbits.URXEN = 1;
 U4STAbits.UTXEN = 1;
#endif

#ifdef ARDUINO_ARCH_AVR
 Serial1.begin(BAUDRATE);
#endif
}

char wifiRead()
{
#ifdef MEGA32
 while (!U4STAbits.URXDA)
  ;
 return(U4RXREG);
#endif

#ifdef ARDUINO_ARCH_AVR
 int ch;
 while ((ch = Serial1.read()) < 0)
  ;
 return((char) ch);
#endif

#ifdef WIN32
 return(sioGetc());
#endif
}


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
  if (wifiGetc())
  {
  }
 }
}

void wifiPut(char *s, int size)
{
 char ch;
 while (--size >= 0)
 {
  while (wifiTxBusy())
  {
   if (wifiAvail())
   {
    ch = wifiGetc();
    *rsp++ = ch;
    len++;
    putChar(ch);
   }
  }
  wifiPutc(*s++);
 }
}

void wifiPut(char *s)
{
 wifiPut(s,strlen(s));
}

void wifiTerm()
{
 char c;
 const char *s;
 s = "\r\n";
 while ((c = *s++) != 0)
 {
  while (wifiTxBusy())
  {
   if (wifiAvail())
   {
    char ch = wifiGetc();
    *rsp++ = ch;
    len++;
    putChar(ch);
   }
  }
  wifiPutc(c);
 }
}

void wifiWriteStr(const char *s, unsigned long timeout)
{
 wifiWrite((char *) s, strlen(s), timeout);
}

void wifiWrite(char *s, int size, unsigned long timeout)
{
 wifiClrRx();
 printf("\nSending %d ",size);
 timeout += millis();

 wifiPut(s,size);
 wifiTerm();

 char last = 0;
 while (timeout >= millis())
 {
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   if (len < RSPLEN)
   {
    *rsp++ = ch;
    len++;
   }
   putChar(ch);
   if ((last == 'O') && (ch == 'K'))
    timeout = millis() + 10;
   last = ch;
  }
 }
 *rsp++ = 0;
}

void wifiMux()
{
 wifiWriteStr("AT+CIPMUX=1",1000);
}

void wifiStart(int chan, char *protocol, char *ip, int port, int timeout)
{
 sprintf(cmdBuffer,"AT+CIPSTART=%d,\"%s\",%s,%d\n",
	 chan,protocol,ip,port);
 wifiWrite(cmdBuffer,strlen(cmdBuffer),timeout);
}

void wifiStartData(char *s, int size, unsigned long timeout)
{
 wifiClrRx();
 printf("Sending %d ",size);
 timeout += millis();

 wifiPut(s,size);
 wifiTerm();

 char last = 0;
 while (timeout >= millis())
 {
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   putChar(ch);
   if ((last == '>') && (ch == ' '))
    timeout = millis() + 10;
   last = ch;
  }
 }
}

void wifiWriteData(char *s, int size, unsigned long timeout)
{
 wifiClrRx();
 printf("Sending %d ",size);
 timeout += millis();

 wifiPut(s, size);
 wifiTerm();

 char chkstr[] = "SEND OK";
 unsigned int chklen = strlen(chkstr);
 while (timeout >= millis())
 {
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   if (len < RSPLEN)
   {
    *rsp++ = ch;
    len++;
    if (len > chklen)
    {
     if (cmp(rsp - chklen,chkstr,chklen))
     {
      timeout = millis() + 10;
     }
    }
   }
  }
 }
}

#define IPD "+IPD,"
#define IPDLEN (sizeof(IPD) - 1)

#define OK "OK"
#define OKLEN (sizeof(OK) - 1)

#define RSP_ERR "ERROR"
#define RSP_ERRLEN (sizeof(RSP_ERR) - 1)

#define CHKLEN RSP_ERRLEN

char *wifiWriteTCPx(char *s, int size, 
		    int *dataLen, unsigned long timeout)
{
 wifiClrRx();
 printf("Sending %d\n",size);
 timeout += millis();

 wifiPut(s,size);
 wifiTerm();

 int rspNum = 0;
 int dLen = 0;
 char *p = 0;
 size += IPDLEN;
 while (timeout >= millis())
 {
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   if (len < RSPLEN)		// if room in buffer
   {
    *rsp++ = ch;
    *rsp = 0;
    len++;
    if (len > (unsigned int) size) // if past message echo
    {
     switch (rspNum)
     {
     case 0:
      if (cmp(rsp - IPDLEN,(char *) IPD,IPDLEN))
      { 
       rspNum = 1;
      }
      break;

     case 1:
      if (ch == ',')
      {
       rspNum = 2;
       dLen = 0;
      }
      break;

     case 2:
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

     case 3:
      --dLen;
      if (dLen == 0)
      {
       rspNum = 4;
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
      timeout = millis() + 10;
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
 printf("\n");
}

void wifiWriteTCP(char *s, int size, unsigned long timeout)
{
 wifiClrRx();
 printf("Sending %d ",size);
 timeout += millis();

 wifiPut(s,size);
 wifiTerm();

 int rspNum = 0;
 const char *chkstr;
 chkstr = "+IPD";
 unsigned int chklen = strlen(chkstr);
 const char* errstr;
 errstr = "ERR";
 unsigned int errLen = strlen(errstr);
 while (timeout >= millis())
 {
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
      if (rspNum == 0)
      {
       chkstr = "CLOSED";
       chklen = strlen(chkstr);
       rspNum = 1;
      }
      else if (rspNum == 1)
       timeout = millis() + 10;
     }
     else if (cmp(rsp - errLen,(char *) errstr,errLen))
     {
      timeout = millis() + 10;
     }
    }
   }
  }
 }
 printf("\n");
}

void wifiCloseTCP(unsigned long timeout)
{
 wifiClrRx();
 timeout += millis();

 wifiPut((char *) "AT+CIPCLOSE=4");
 wifiTerm();

 char chkstr[] = "CLOSED";
 unsigned int chklen = strlen((const char *) chkstr);
 while (timeout >= millis())
 {
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   putChar(ch);
   if (len < RSPLEN)
   {
    *rsp++ = ch;
    len++;
    if (len > chklen)
    {
     if (cmp(rsp - chklen,chkstr,chklen))
     {
      timeout = millis() + 10;
     }
    }
   }
  }
 }
}

void wifiClose(int chan, unsigned long timeout)
{
 wifiClrRx();
 timeout += millis();

 sprintf(cmdBuffer,"AT+CIPCLOSE=%d",chan);
 wifiPut((char *) cmdBuffer);
 wifiTerm();

 char chkstr[] = "CLOSED";
 unsigned int chklen = strlen((const char *) chkstr);
 while (timeout >= millis())
 {
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   putChar(ch);
   if (len < RSPLEN)
   {
    *rsp++ = ch;
    len++;
    if (len > chklen)
    {
     if (cmp(rsp - chklen,chkstr,chklen))
     {
      timeout = millis() + 10;
     }
    }
   }
  }
 }
}

#endif
