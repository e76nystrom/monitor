#if 1	// <-

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
#include "dbgInfo.h"

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
void printBuf(char *p, unsigned int len);
char *lc(char *p);
int find(char *str1, const char *str2);
int find(char *str1, const char *str2, int offset, int len1);
int cmp(char *str1, const char *str2, int size);
int cmp(char *str1, const char *str2);
int findData(int cmdLen, unsigned int *datalen);
int getVal(char *p, int pos, unsigned int *rtnVal, int size);
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
char *wifiWriteTCPx(char *s, int size, unsigned int *dataLen,
		    unsigned int timeout);
void wifiClose(int port, unsigned int timeout);

#if defined(ARDUINO_ARCH_AVR)
void wifiPut(const __FlashStringHelper *s, int size);
void wifiPut(const __FlashStringHelper *s);
char wifiWriteStr(const __FlashStringHelper *s, unsigned int timeout);
char wifiWrite(const __FlashStringHelper *s, int size, unsigned int timeout);
#endif	/* ARDUINO_ARCH_AVR */

/* buffer for strings made from program data */
EXT char stringBuffer[80] __attribute__((section(".noinit")));

/* buffer for data sent */
#define DATA_BUF_SIZE ((size_t) 192)
EXT char dataBuffer[DATA_BUF_SIZE] __attribute__((section(".noinit")));

/* buffer for command sent */
EXT char cmdBuffer[64] __attribute__((section(".noinit")));

/* buffer for response */
EXT char packetRsp[460] __attribute__((section(".noinit")));

EXT char *rsp;
EXT unsigned int rspLen;
#define MAX_RSP 2
EXT unsigned char rspCount;
EXT char *rspPtr[MAX_RSP];
EXT int rspL[MAX_RSP];
EXT char monitorId[ID_LEN];

#if !defined(wifiDbg)
EXT bool wifiDbg;
#endif	/* wifiDbg */

int wifiRSSI(void);

EXT unsigned int lastRSSI;
EXT int rssi;

#define RSSI_INTERVAL 5000

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
