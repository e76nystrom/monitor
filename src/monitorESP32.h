#if !defined(__MONITOR_ESP32_H__)
#define __MONITOR_ESP32_H__

#include <Arduino.h>

#include <stdio.h>

#include "serial.h"

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
EXT unsigned char rspCount;

#define MAX_RSP 2
EXT char *rspPtr[MAX_RSP];
EXT int rspL[MAX_RSP];

#define ID_LEN ((unsigned int) 16)
EXT char monitorId[ID_LEN];

#if !defined(wifiDbg)
EXT bool wifiDbg;
#endif	/* wifiDbg */

#define intMillis millis

#define DBGPORT Serial

#define getChar(ch) \
 while (!DBGPORT.available()) \
  wdt_reset(); \
 ch = DBGPORT.read()

#define DBG 1
#define MON_DBG 1
#define WIFI_DBG 1

#define argConv(str, buf) str
inline void flush(void) {fflush(stdout);}
inline void newLine()
{
 printf("\n");
}
inline void dbg0Set(void) {}
inline void dbg0Clr(void) {}

//extern "C" unsigned int __attribute__ ((noinline)) xGetSP();

#if !defined(MONITOR_INDEX)

#define MONITOR_INDEX 1

#endif	/* MONITOR_INDEX */

/* -------------------- monitor index 1 -------------------- */

/* outside temperature */

// -----------------------------------------------------------------------------

#if (MONITOR_INDEX == 1)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1
#define EMONCMS_NODE "esp32mon"

#define WIFI_ENA

#define OLED_ENA

#define TEMP_SENSOR 1
#define ONE_WIRE_BUS 26

// #define RTC_CLOCK 0
#define DHT_SENSOR 1
#define DHT_PIN 27
// #define CURRENT_SENSOR 0
// #define WATER_MONITOR 0
#define CHECK_IN 1
// #define DEHUMIDIFIER 0

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "esp32mon"

#define TEMPDEVS 1

#if defined(__MONITOR__)
EXT DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0xff, 0x64, 0x1f, 0x69, 0xe0, 0x09, 0x26}
};
#else
extern DeviceAddress tempDev[TEMPDEVS];
#endif /* defined(__MONITOR__) */

#endif	/* MONITOR_INDEX == 1 */

// -----------------------------------------------------------------------------

#if (MONITOR_INDEX == 2)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1
#define EMONCMS_NODE "esp32c3"

#define WIFI_ENA

#define OLED_ENA

#define SHT_SENSOR 1

#define TEMP_SENSOR 1
#define ONE_WIRE_BUS 5

// #define RTC_CLOCK 0
// #define DHT_SENSOR 1
// #define DHT_PIN 27
// #define CURRENT_SENSOR 0
// #define WATER_MONITOR 0
#define CHECK_IN 1
// #define DEHUMIDIFIER 0

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "esp32c3"

#define TEMPDEVS 1

#if defined(__MONITOR__)
EXT DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0x1e, 0xc4, 0x45, 0xd4, 0x59, 0x63, 0xcf}
};
#else
extern DeviceAddress tempDev[TEMPDEVS];
#endif /* defined(__MONITOR__) */

#endif	/* MONITOR_INDEX == 2 */

// -----------------------------------------------------------------------------

#if (MONITOR_INDEX == 3)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1
#define EMONCMS_NODE "esp32c3dev"

#define WIFI_ENA

#define OLED_ENA
// #define SHT_SENSOR 1

#define I2C_SCL 6
#define I2C_SDA 7

#define TEMP_SENSOR 1
#define ONE_WIRE_BUS 0

#define DHT_SENSOR 1
#define DHT_PIN 1

#define CHECK_IN 1

// #define RTC_CLOCK 0
// #define CURRENT_SENSOR 0
// #define WATER_MONITOR 0
// #define DEHUMIDIFIER 0

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "esp32c3dev"

#define TEMPDEVS 1

#if defined(__MONITOR__)
EXT DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0x1e, 0xc4, 0x45, 0xd4, 0x59, 0x63, 0xcf}
};
#else
extern DeviceAddress tempDev[TEMPDEVS];
#endif /* defined(__MONITOR__) */

#endif	/* MONITOR_INDEX == 3 */

// -----------------------------------------------------------------------------

#if (MONITOR_INDEX == 4)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1
#define EMONCMS_NODE "esp32s3"

#define WIFI_ENA

#define OLED_ENA
#define SHT_SENSOR 1

#define I2C_SCL 5
#define I2C_SDA 4

#define TEMP_SENSOR 1
#define ONE_WIRE_BUS 10

#define DHT_SENSOR 1
#define DHT_PIN 9

#define CHECK_IN 1

// #define RTC_CLOCK 0
// #define CURRENT_SENSOR 0
// #define WATER_MONITOR 0
// #define DEHUMIDIFIER 0

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "esp32s3"

#define TEMPDEVS 1

#if defined(__MONITOR__)
EXT DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0x08, 0xde, 0x45, 0xd4, 0xdd, 0x4a, 0x4c}
};
#else
extern DeviceAddress tempDev[TEMPDEVS];
#endif /* defined(__MONITOR__) */

#endif	/* MONITOR_INDEX == 4 */

#endif	/* __MONITOR_ESP32_H__ */
