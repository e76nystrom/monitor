#define __WIFI__
#if defined(ARDUINO_ARCH_AVR)
#include <Arduino.h>
#include <EEPROM.h>
#include "CRC32.h"
#include <ctype.h>
#endif /* ARDUINO_ARCH_AVR */

#define EXT extern

#if defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>
#include "CRC32.h"
#include "millis.h"

#endif /* ARDUINO_ARCH_STM32 */

#if 0 //defined(STM32MON)
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "serialio.h"
#endif /* STM32MON */

#if defined(MEGA32)
#include "WProgram.h"
#include "printf.h"
#endif	/* MEGA32 */

#if defined(WIN32)
#include "stdio.h"
#include "string.h"
#include "conio.h"
#endif	/* WIN32 */

#include "monitor.h"

#include "wdt.h"

#if defined(DBG)
#undef DBG
#endif	/* DBG */

#define DBG 1

#include "wifi.h"

#if defined(__WIFI_INC__)	// <-

#define SSID_LEN 12
#define PASS_LEN 20
#define ID_LEN 16
#define IP_LEN 16
#define CSUM_LEN 4

#if defined(ARDUINO_ARCH_AVR)

typedef struct s_eeprom
{
 char ssid[SSID_LEN];
 char pass[PASS_LEN];
 char id[ID_LEN];
 char ip[IP_LEN];
 char csum[CSUM_LEN];
} T_EEPROM, *P_EEPROM;

#define SSID_LOC 0
#define PASS_LOC (SSID_LOC + SSID_LEN)
#define ID_LOC (PASS_LOC + PASS_LEN)
#define IP_LOC (ID_LOC + ID_LEN)
#define CSUM_LOC  (IP_LOC + IP_LEN)
#define CSUM_LEN 4

#define wifiAvail() (WIFI.available())
#define wifiGetc() ((char) WIFI.read())
#define wifiTxBusy() (0)
#define wifiPutc(c) WIFI.write(c)
#if DBG
#define putChar(c) DBGPORT.write(c)
#else
#define putChar(c)
#endif	/* DBG */
#define putChar1(c) DBGPORT.write(c)
#define getChar(ch) \
 while (!DBGPORT.available()) \
  wdt_reset(); \
 ch = DBGPORT.read()

uint32_t sumEE();
void writeSumEE();
void readEE(char *buf, char addr, char len);
void writeEE(const char *buf, char addr, char eeLen);

#endif	/* ARDUINO_ARCH_AVR */

#if defined(ARDUINO_ARCH_STM32)

#define wifiAvail() (WIFI.available())
#define wifiGetc() ((char) WIFI.read())
#define wifiTxBusy() (0)
#define wifiPutc(c) WIFI.write(c)

#if DBG
#define putChar(c) DBGPORT.write(c)
#else
#define putChar(c)
#endif	/* DBG */

#define putChar1(c) DBGPORT.write(c)
#define getChar(ch) \
 while (!DBGPORT.available()) \
  wdt_reset(); \
 ch = DBGPORT.read()

#endif /* ARDUINO_ARCH_STM32 */

#if 0 //defined(STM32MON)

#define wifiAvail() (remRxReady())
#define wifiGetc() ((char) remRxRead())
#define wifiTxBusy() (remTxEmpty() != 0)
#define wifiPutc(c) (remTxSend(c))
#if DBG
#define putChar(c) putx1(c)
#else
#define putChar(c)
#endif	/* DBG */
#define putChar1(c) putx1(c)
#define getChar(ch) \
 while (dbgRxReady() == 0)				\
  wdt_reset(); \
 ch = dbgRxRead()

#endif /* STM32MON */

#if defined(MEGA32)

#define wifiAvail() U4STAbits.URXDA
#define wifiGetc() U4RXREG
#define wifiTxBusy() U4STAbits.UTXBF
#define wifiPutc(c) U4TXREG = c
#define putChar(c) Serial.write(c)
#define putChar1(c) Serial.write(c)

#endif	/* MEGA32 */

#if defined(WIN32)

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
#endif	/* DBG */
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
char *sendData(const char *ip, const char *data, unsigned int timeout);
char *sendData(const char *ip, int port, const char *data,
	       unsigned int timeout);

void printBuf();
char *lc(char *p);
int find(char *str1, const char *str2);
int find(char *str1, const char *str2, int offset, int len1);
int cmp(char *str1, const char *str2, int size);
int cmp(char *str1, const char *str2);
int findData(int cmdLen, int *datalen);
int getVal(char *p, int pos, int *rtnVal, int size);
void getData(char *dst, unsigned int dstSize, char *buf, unsigned int bufSize);

#if defined(ARDUINO_ARCH_AVR)
int strlen(const __FlashStringHelper *s);
int find(char *str1, const __FlashStringHelper *str2);
int find(char *str1, const __FlashStringHelper *str2, int offset, int len1);
int cmp(char *str1, const __FlashStringHelper *str2, int size);
int cmp(char *str1, const __FlashStringHelper *str2);
#endif	/* ARDUINO_ARCH_AVR */

void wifiInitSio();
void wifiReset();
#if 0
char wifiRead();
#endif
void wifiClrRx();
void wifiPut(char *s, int size);
void wifiPut(char *s);
void wifiTerm();
char wifiWriteStr(const char *s, unsigned int timeout);
char wifiWrite(char *s, int size, unsigned int timeout);
void wifiMux();
char *wifiGetIP(char *buf);
void wifiCWMode();
void wifiQuit();
char wifiJoin();

void wifiStart(int chan, char *protocol, char *ip, int port,
	       unsigned int timeout);
void wifiStartData(char *s, int size, unsigned int timeout);
void wifiWriteData(char *s, int size, unsigned int timeout);
char *wifiWriteTCPx(char *s, int size, int *dataLen, unsigned int timeout);
void wifiClose(int port, unsigned int timeout);

#if defined(ARDUINO_ARCH_AVR)
void wifiPut(const __FlashStringHelper *s, int size);
void wifiPut(const __FlashStringHelper *s);
char wifiWriteStr(const __FlashStringHelper *s, unsigned int timeout);
char wifiWrite(const __FlashStringHelper *s, int size, unsigned int timeout);
#endif	/* ARDUINO_ARCH_AVR */

EXT char stringBuffer[80];	/* buffer for strings made from program data */
EXT char dataBuffer[192];	/* buffer for data sent */
EXT char cmdBuffer[64];		/* buffer for command sent */
EXT char packetRsp[384];	/* buffer for response */
EXT char *rsp;
EXT unsigned int rspLen;
EXT char id[ID_LEN];

#define IPD_STR "+IPD,"
#define IPD F1(IPD_STR)
#define IPDLEN (sizeof(IPD_STR) - 1)

#define OK_STR "OK"
#define OK F1(OK_STR)
#define OKLEN (sizeof(OK_STR) - 1)

#define ERR_STR "ERROR"
#define RSP_ERR F1(ERR_STR)
#define RSP_ERRLEN (sizeof(ERR_STR) - 1)

#define FAIL_STR "SEND FAIL"
#define RSP_FAIL F1(FAIL_STR)
#define RSP_FAILLEN (sizeof(FAIL_STR) - 1)

#define CLOSED_STR "CLOSED" 
#define CLOSED F1(CLOSED_STR)
#define CLOSED_LEN (sizeof(CLOSED_STR) - 1)

#define STAIP_STR "STAIP,\""
#define STAIP F1(STAIP_STR)
#define STAIP_LEN (sizeof(STAIP_STR) - 1)

#define SEND_OK_STR "SEND OK"
#define SEND_OK F1(SEND_OK_STR)
#define SEND_OK_LEN (sizeof(SEND_OK_STR) - 1)

#define CHKLEN RSP_ERRLEN

#define RSPLEN (sizeof(packetRsp) - 1)

#endif	/* __WIFI_INC__*/ // ->
#if defined(__WIFI__)

void newLine()
{
 printf("\n");
}

void dbgChar(char ch)
{
 if (ch == '\n')
 {
  putChar('\r');
  putChar(ch);
 }
 else if ((ch >= ' ') && (ch < 0x80))
  putChar(ch);
}

#if defined(ARDUINO_ARCH_AVR)

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
  EEPROM.write(addr, *buf);
  addr++;
  buf++;
 }
 for (; i < eeLen; i++)
 {
  EEPROM.write(addr, 0);
  addr++;
 }
}

#endif	/* ARDUINO_ARCH_AVR */

#if defined(WIN32)

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

#endif	/* WIN32 */

#define WIFIBAUDRATE 9600

void putx4(char c)
{
#if defined(MEGA32)
 while (U4STAbits.UTXBF)
  ;
 U4TXREG = c;
 if (c == '\n')
 {
  while (U4STAbits.UTXBF)
   ;
  U4TXREG = '\r';
 }
#endif	/* MEGA32 */

#if defined(ARDUINO_ARCH_AVR)
 WIFI.write(c);
 if (c == '\n')
 {
  WIFI.write('\r');
 }
#endif	/* ARDUINO_ARCH_AVR */

#if 0 //defined(STM32MON)
 putx1(c);
 if (c == '\n')
 {
  putx1('\r');
 }
#endif	/* STM32MON */
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
 return(sendData(ip, 80, data, 5000));
}

char *sendData(const char *ip, const char *data, unsigned int timeout)
{
 return(sendData(ip, 80, data, timeout));
}

char *sendData(const char *ip, int port, const char *data,
	       unsigned int timeout)
{
 wifiMux();			/* in case device restarted */
 int cmdLen = strlen((const char *) data);
 if (DBG)
  printf(F0("sendData %d %s\n"), cmdLen, data);

 char *p = 0;
 sprintf((char *) cmdBuffer, F0("AT+CIPSTART=4,\"TCP\",\"%s\",%d"), ip, port);
 if (wifiWriteStr(cmdBuffer, 4000))
 {
  sprintf((char *) cmdBuffer, F0("AT+CIPSEND=4,%d"), cmdLen);
  wifiStartData((char *) cmdBuffer, strlen(cmdBuffer), 1000);

  int dataLen = 0;
  p = wifiWriteTCPx((char *) data, cmdLen, &dataLen, timeout);
  if (p != 0)			/* if success */
  {
   if (find(p, CLOSED) < 0)
   {
    wifiClose(4, 1000);
   }

   *(p + dataLen) = 0;
   if (0)
    printf(F0("\nlength %d dataLen %d %s\n"), rspLen, dataLen, p);
  }
  else				/* if failed */
  {
   printf(F0("send failure close\n"));
   wifiClose(4, 1000);
#if DBG1_Pin
   dbg1Set();
   delay(2);
   dbg1Clr();
#endif /* DBG1_Pin */
   printBuf();
  }
 }
 return(p);
}

#if DBG

void printBuf()
{
 printf(F0("\nrspLen %d\n"), rspLen);
 char *p = (char *) packetRsp;
 char col = 0;			/* number of columns */
 for (unsigned int i = 0; i < rspLen; i++)
 {
  if (col == 0)			/* if column 0 */
  {
   printf(F0("%08x %04x  "), (int) p, i);
  }
  int val = *p++ & 0xff;
  char ch = ' ';
  if ((val >= ' ') && (val < 127))
   ch = val;
  printf(F0("%02x %c "), val, ch); /* output value */
  col++;			/* count a column */
  if (col == 8)			/* if at end of line */
  {
   col = 0;			/* reset column counter */
   newLine();
  }
 }
 if (col != 0)
  newLine();
}

#else  /*  DBG */
void printBuf()
{
}
#endif	/* DBG */

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

#define FIND_FMT "find str1 %d 0x%x len2 %d %s\n"
#define FIND_DBG 0

int find(char *str1, const char *str2)
{
 int len1 = (int) strlen((const char *) str1);
 unsigned int len2 = strlen(str2);
 int offset = 0;
#if FIND_DBG
  char buf[sizeof(FIND_FMT)];
  printf(argConv(F2(FIND_FMT), buf), len1, (int) str1, len2, str2);
#endif /* FIND_DBG */
 len1 -= len2;
 if (len1 > 0)
 {
  while (--len1 > 0)
  {
   if (cmp(str1, str2, len2))
   {
    if (FIND_DBG)
     printf(F0("offset %d\n"), offset);
    return(offset);
   }
   str1++;
   offset++;
  }
 }
/* printf("not found\n"); */
 return(-1);
}

int find(char *str1, const char *str2, int offset, int len1)
{
 int len2 = strlen(str2);
 // printf(F0("find offset %d len1 %d len2 %d %s\n"), offset, len1, len2, str2);
 str1 += offset;
 len1 -= offset;
 len1 -= len2;
 if (len1 > 0)
 {
  while (len1 >= 0)
  {
   /* printf("%2d %c\n", len1, *str1); */
   if (cmp(str1, str2, len2))
   {
    offset += len2;
    /* printf("offset %d\n", offset); */
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

int cmp(char *str1, const char *str2, int size)
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

int cmp(char *str1, const char *str2)
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

#if defined(ARDUINO_ARCH_AVR)

int strlen(const __FlashStringHelper *s)
{
 PGM_P p = reinterpret_cast <PGM_P> (s);
 int len = 0;
 char ch;
 while ((ch = pgm_read_byte(p++)) != 0)
  len++;
 return(len);
}

#define fdbg 0

int find(char *str1, const __FlashStringHelper *str2)
{
 unsigned int len1 = strlen((const char *) str1);
 unsigned int len2 = strlen(str2);
 int offset = 0;
 if (fdbg)
 {
  char *tmp[12];
  printf("find len1 %d %s len2 %d %s\n",
	 len1, str1, len2, argConv(str2, (char *) tmp));
 }
 len1 -= len2;
 if (len1 > 0)
 {
  while (--len1 > 0)
  {
   if (cmp(str1, str2, len2))
   {
    if (fdbg)
    printf("offset %d\n", offset);
    return(offset);
   }
   str1++;
   offset++;
  }
 }
 if (fdbg)
  printf("not found\n");
 return(-1);
}

int find(char *str1, const __FlashStringHelper *str2, int offset, int len1)
{
 int len2 = strlen(str2);
 if (fdbg)
 {
  char *tmp[12];
  printf("find offset %d len1 %d len2 %d %s\n",
	 offset, len1, len2, argConv(str2, (char *) tmp));
 }
 str1 += offset;
 len1 -= offset;
 len1 -= len2;
 if (len1 > 0)
 {
  while (len1 >= 0)
  {
   if (fdbg)
    printf("%2d %c\n", len1, *str1);
   if (cmp(str1, str2, len2))
   {
    offset += len2;
    if (fdbg)
     printf("offset %d\n", offset);
    return(offset);
   }
   str1++;
   offset++;
   --len1;
  }
 }
 if (fdbg)
  printf("not found\n");
 return(-1);
}

int cmp(char *str1, const __FlashStringHelper *str2, int size)
{
 PGM_P p = reinterpret_cast <PGM_P> (str2);
 while (--size >= 0)
 {
  char ch = pgm_read_byte(p++);
  if (*str1++ != ch)
  {
   return(0);
  }
 }
 return(1);
}

int cmp(char *str1, const __FlashStringHelper *str2)
{
 PGM_P p = reinterpret_cast <PGM_P> (str2);
 while (1)
 {
  char ch = pgm_read_byte(p++);
  if (*str1 == 0)
   return(0);
  if (ch == 0)
   return(0);
  if (*str1++ != ch)
  {
   return(0);
  }
 }
 return(1);
}

#endif	/* ARDUINO_ARCH_AVR */

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
 int pos = find((char *) packetRsp, IPD, cmdLen, (int) rspLen);
 if (pos >= 0)
 {
  pos = find((char *) packetRsp, ",", pos, (int) rspLen);
  if (pos > 0)
  {
   pos = getVal((char *) packetRsp, pos, dataLen, (int) (rspLen - pos));
   pos += 1;
   return(pos);
  }
 }
 return(-1);
}

void getData(char *dst, unsigned int dstSize, char *buf, unsigned int bufSize)
{
 --dstSize;			/* allow for null at end */
 while (--dstSize != 0)
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
 printf(F0("wifiReset\n"));
#if defined(WIFI_RESET)
 digitalWrite(WIFI_RESET, LOW);
 delay(200);
 digitalWrite(WIFI_RESET, HIGH);
 if (DBG)
  printf(F0("flushing wifi input\n"));
 while (wifiAvail())
 {
  wdt_reset();
  char ch = wifiGetc();
  if (isprint(ch))
   putChar(ch);
 }
 printf(F0("flush done\n"));
#endif	/* WIFI_RESET */
 signed char retry = 5;
 while (--retry >= 0)
 {
  delay(200);
  printf(F0("write AT %d\n"), retry);
  if (wifiWriteStr(F2("AT"), 1000))
   break;
 }
 printf(F0("AT done\n"));
 wifiCWMode();			/* set correct cw mode */
 wifiMux();			/* set to mux mode */
}

void wifiInitSio()
{
#if defined(ARDUINO_ARCH_AVR) | defined(ARDUINO_ARCH_STM32)
 WIFI.begin(WIFIBAUDRATE);
#endif	/* ARDUINO_ARCH_AVR | ARDUINO_ARCH_STM32 */

#if defined(MEGA32)
 IEC0bits.INT4IE = 0;
 IFS0bits.INT4IF = 0;
 U4MODE = 0;
 U4MODEbits.ON = 1;
 U4BRG = (F_CPU / (16 * WIFIBAUDRATE)) - 1;
 U4STA = 0;
 U4STAbits.URXEN = 1;
 U4STAbits.UTXEN = 1;
#endif	/* MEGA32 */
 }

#if 0
char wifiRead()
{
#if defined(MEGA32)
 while (!U4STAbits.URXDA)
  ;
 return(U4RXREG);
#endif	/* MEGA32 */

#if defined(ARDUINO_ARCH_AVR)
 int ch;
 while ((ch = WIFI.read()) < 0)
  ;
 return((char) ch);
#endif	/* ARDUINO_ARCH_AVR */

#if defined(WIN32)
 return(sioGetc());
#endif	/* WIN32 */
}
#endif

void  wifiClrRx()
{
 rsp = (char *) packetRsp;
 rspLen = 0;

#if defined(MEGA32)
 if (U4STAbits.OERR)
  U4STACLR = _U4STA_OERR_MASK;
#endif	/* MEGA32 */

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
 wifiPut(s, strlen(s));
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
    rspLen++;
    dbgChar(ch);
   }
  }
  ch = *s++;
  wifiPutc(ch);
 }
}

#if defined(ARDUINO_ARCH_AVR)

void wifiPut(const __FlashStringHelper *s)
{
 wifiPut(s, strlen(s));
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
    rspLen++;
    dbgChar(ch);
   }
  }
  wifiPutc(pgm_read_byte(p++));
 }
}

#endif	/* ARDUINO_ARCH_AVR */

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
    rspLen++;
    dbgChar(ch);
   }
  }
  wifiPutc(c);
 }
}

char wifiWriteStr(const char *s, unsigned int timeout)
{
 return(wifiWrite((char *) s, strlen(s), timeout));
}

char wifiWrite(char *s, int size, unsigned int timeout)
{
 wifiClrRx();
 if (DBG)
  printf(F0("\nSending %d "), size);

 wifiPut(s, size);
 wifiTerm();

 char result = 0;
 char last = 0;
 millisDef start = millis();
 while ((millis() - start) < timeout)
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   if (rspLen < RSPLEN)
   {
    *rsp++ = ch;
    rspLen++;
   }
   dbgChar(ch);
   if ((last == 'O') && (ch == 'K'))
   {
    start = millis();
    timeout = 10;
    result = 1;
   }
   last = ch;
  }
 }
 *rsp++ = 0;
 return(result);
}

#if defined(ARDUINO_ARCH_AVR)

char wifiWriteStr(const __FlashStringHelper *s, unsigned int timeout)
{
 return(wifiWrite(s, strlen(s), timeout));
}

char wifiWrite(const __FlashStringHelper *s, int size, unsigned int timeout)
{
 wifiClrRx();
 if (DBG)
  printf(F0("\nSending %d "), size);

 wifiPut(s, size);
 wifiTerm();

 char result = 0;
 char last = 0;
 millisDef start = millis();
 while ((millis() - start) < timeout)
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   if (rspLen < RSPLEN)
   {
    *rsp++ = ch;
    rspLen++;
   }
   dbgChar(ch);
   if (result)
   {
    start = millis();
   }
   else
   {
    if ((last == 'O') && (ch == 'K'))
    {
     start = millis();
     timeout = 10;
     result = 1;
    }
    last = ch;
   }
  }
 }
 *rsp++ = 0;
 return(result);
}

#endif	/* ARDUINO_AVR_AVR */

void wifiMux()
{
 wifiWriteStr(F2("AT+CIPMUX=1"), 1000);
}

char *wifiGetIP(char *buf)
{
 wifiWriteStr(F2("AT+CIFSR"), 1000);

 char *dst = buf;
 int pos = find((char *) packetRsp, STAIP, 0, (int) rspLen);
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
 wifiWriteStr(F2("AT+CWMODE=1"), 1000);
}

void wifiQuit()
{
 wifiWriteStr(F2("AT+CWQAP"), 1000);
}

char wifiJoin()
{
#if ARDUINO_ARCH_AVR
 strcpy(cmdBuffer, F3("AT+CWJAP=\""));
 readEE(stringBuffer, SSID_LOC, SSID_LEN);
 strcat(cmdBuffer, stringBuffer);
 strcat(cmdBuffer, F3("\",\""));
 readEE(stringBuffer, PASS_LOC, PASS_LEN);
 strcat(cmdBuffer, stringBuffer);
 strcat(cmdBuffer, F3("\""));
#else
 strcpy(cmdBuffer, "AT+CWJAP=\"");
 strcat(cmdBuffer, SSID);
 strcat(cmdBuffer, "\",\"");
 strcat(cmdBuffer, PASS);
 strcat(cmdBuffer, "\"");
#endif	/* ARDUINO_ARCH_AVR */

 return(wifiWrite(cmdBuffer, strlen(cmdBuffer), 8000));
}

void wifiStart(int chan, char *protocol, char *ip, int port,
	       unsigned int timeout)
{
 sprintf(cmdBuffer, F0("AT+CIPSTART=%d,\"%s\",%s,%d\n"),
	 chan, protocol, ip, port);
 wifiWrite(cmdBuffer, strlen(cmdBuffer), timeout);
}

void wifiStartData(char *s, int size, unsigned int timeout)
{
 wifiClrRx();
 if (DBG)
  printf(F0("StartData Sending %d "), size);

 wifiPut(s, size);
 wifiTerm();

 char last = 0;
 millisDef start = millis();
 while ((millis() - start) < timeout)
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   dbgChar(ch);
   if ((last == '>') && (ch == ' '))
   {
    start = millis();
    timeout = 10;
   }
   last = ch;
  }
 }
}

void wifiWriteData(char *s, int size, unsigned int timeout)
{
 wifiClrRx();
 if (DBG)
  printf(F0("Data Sending %d "), size);

 wifiPut(s, size);
 wifiTerm();

 char ok = 0;
 millisDef start = millis();
 while ((millis() - start) < timeout)
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   if (ok)			/* if okay received */
   {
    start = millis();		/* reset start on each character */
   }
   if (rspLen < RSPLEN)
   {
    *rsp++ = ch;
    rspLen++;
    if (ok == 0)
    {
     if (rspLen > SEND_OK_LEN)
     {
      if (cmp(rsp - SEND_OK_LEN, SEND_OK, SEND_OK_LEN))
      {
       ok = 1;
       start = millis();
       timeout = 10;
      }
     }
    }
   }
  }
 }
}

#define dbg0 0			/* print received characters */
#define dbg1 0			/* too long misses characters */
#define dbg2 0			/* key strings found */
#define dbg3 0			/* print state changes */
#define echo 0			/* commands echoed */

char *wifiWriteTCPx(char *s, int size, 
		    int *dataLen, unsigned int timeout)
{
 wifiClrRx();
 if (DBG)
  printf(F0("Sending %d\n"), size);

 wifiPut(s, size);
 wifiTerm();

 char lastRsp;
 if (dbg3)
 {
  lastRsp= 0;
  printf(F0("receiving\n"));
 }
 char rspNum = 0;
 int dLen = 0;
 char *p;
 p = 0;
 char ok = 0;
 if (echo)
  size += IPDLEN;
 else
  size = IPDLEN;
 dbg2Set();
 millisDef start = millis();
 while ((millis() - start) < timeout)
 {
  wdt_reset();
  if  (wifiAvail())
  {
   unsigned char ch = wifiGetc();
   if (dbg1)
   {
    printf("%d %02x ", rspNum, ch);
    if ((ch >= ' ') & (ch < 0x80))
     dbgChar(ch);
    else
     dbgChar(' ');
    printf(" %3d", rspLen);
    if (rspNum == 3)
     printf(" %3d", dLen);
    newLine();
   }   
   else if (dbg0)
    dbgChar(ch);

   if (rspLen < RSPLEN)		/* if room in buffer */
   {
    *rsp++ = ch;
    *rsp = 0;
    rspLen++;
    if (rspLen > (unsigned int) size) /* if past message echo */
    {
     switch (rspNum)
     {
     case 0:			/* wait for data */
      if (cmp(rsp - IPDLEN, IPD, IPDLEN))
      { 
       rspNum = 1;
      }
      else if (cmp(rsp - OKLEN, OK, OKLEN))
      {
       if (dbg2)
	printf("\nok set\n");
       ok = 1;
      }
      break;

     case 1:			/* ipd found look for , */
      if (ch == ',')
      {
       rspNum = 2;
       dLen = 0;
      }
      break;

     case 2:			/* comma found read length and look for : */
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

     case 3:			/* read response */
      --dLen;
      if (dLen == 0)
      {
       rspNum = 4;
       if (ok)
       {
	start = millis();
	timeout = 250;
	rspNum = 6;
       }
       if (dbg2)
	printf("\ndata done %d\n", rspNum);
      }
      break;

     case 4:			/* read ok */
     case 5:
      start = millis();
      timeout = 100;
      if (cmp(rsp - OKLEN, OK, OKLEN))
      {
       rspNum++;
       if (dbg2)
	printf("\nok %d\n", rspNum);
      }
      break;

     case 6:			/* wait for data */
      if (cmp(rsp - IPDLEN, IPD, IPDLEN))
      { 
       rspNum = 1;
       start = millis();
       timeout = 100;
      }
      break;
     } /* end switch */

     if (dbg3)
     {
      if (rspNum != lastRsp)
       printf(F0("\nrsp %d last %d %d %d\n"), rspNum, lastRsp, rspLen, dLen);
      lastRsp = rspNum;
     }

     if (rspNum != 3)		/* if not reading data */
     {
      if (cmp(rsp - RSP_ERRLEN, RSP_ERR, RSP_ERRLEN))
      {
       rspNum = 6;
       start = millis();
       timeout = 2000;
      }

      if (cmp(rsp - RSP_FAILLEN, RSP_FAIL, RSP_FAILLEN))
      {
       rspNum = 6;
       start = millis();
       timeout = 10;
      }
     }
    } /* if (rspLen >= size) message echo */
   } /* if (len <= rspLen) buffer not full*/
  } /* if (wifiAvail()) */
 } /*  while ((millis() - start) < timeout) not timed out */
 dbg2Clr();
 return(p);
}

void wifiClose(int chan, unsigned int timeout)
{
 wifiClrRx();

 sprintf(cmdBuffer, F0("AT+CIPCLOSE=%d"), chan);
 wifiPut((char *) cmdBuffer);
 wifiTerm();

 millisDef start = millis();
 while ((millis() - start) < timeout)
 {
  wdt_reset();
  if  (wifiAvail())
  {
   char ch = wifiGetc();
   dbgChar(ch);
   if (rspLen < RSPLEN)
   {
    *rsp++ = ch;
    rspLen++;
    if (rspLen > CLOSED_LEN)
    {
     if (cmp(rsp - CLOSED_LEN, CLOSED, CLOSED_LEN))
     {
      start = millis();
      timeout = 10;
     }
    }
   }
  }
 }
}

#endif	/* __WIFI__ */
