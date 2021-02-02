#ifdef ARDUINO_AVR_PRO

/*

ATMega328 Pin 6 - Arduino Pin 3 - Ext0 - RJ45 Pin 4 - S1 - 4 pin DHT & Ds18S20
ATMega328 Pin 7 - Arduino Pin 4 - Ext1 - RJ45 Pin 6 - S2 - 3 Pin DS18S20

*/

#define MONITOR_INDEX 7

/* -------------------- monitor index 1 -------------------- */

#if (MONITOR_INDEX == 1)

#define EMONCMS_ADDR EMONCMS_ADDR0
#define EMONCMS_KEY EMONCMS_KEY0

#define WATER_MONITOR 1

#define SSID "TKGCL"
#define PASS "K4PLVFMCXGMLXM9P"
#define MONITOR_ID "Monitor0"
#endif	/* MONITOR_INDEX == 0 */

/* -------------------- monitor index 2 -------------------- */

#if (MONITOR_INDEX == 2)

#define WATER_MONITOR 1

#define SSID "dd-wrt_vap"
#define PASS "minidonk"
#define MONITOR_ID "Monitor2"
#endif	/* MONITOR_INDEX == 2 */

/* -------------------- monitor index 3 -------------------- */

#if (MONITOR_INDEX == 3)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1

#define EMONCMS_NODE "4"

#define ESP8266_TIME 0
#define TEMP_SENSOR 1
#define CHECK_IN 1

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "Monitor3"

#define ONE_WIRE_BUS 3		/* one wire bus pin */

#define TEMPDEVS 1
#if defined(__MONITOR__)
DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0xff, 0x0f, 0x0b, 0x63, 0x14, 0x03, 0xc7}
};
#else
extern DeviceAddress tempDev[TEMPDEVS];
#endif /* __MONITOR__ */
 
#endif	/* MONITOR_INDEX == 3 */

/* -------------------- monitor index 4 -------------------- */

#if (MONITOR_INDEX == 4)

#define WIFI_ENA 1
#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1

#define EMONCMS_NODE "FamRm"

#define ESP8266_TIME 0
#define TEMP_SENSOR 0
#define CHECK_IN 1
#define DHT_SENSOR 1

#define DHTPIN 3

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "Monitor4"

#endif	/* MONITOR_INDEX == 4 */

/* -------------------- monitor index 5 -------------------- */

#if (MONITOR_INDEX == 5)

#define WIFI_ENA 1
#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1

#define EMONCMS_NODE "GeoH20"

#define ESP8266_TIME 0
#define TEMP_SENSOR 2
#define CHECK_IN 1

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "Monitor5"

#define ONE_WIRE_BUS0 3		/* one wire bus pin */
#define ONE_WIRE_BUS1 4		/* one wire bus pin */

#define TEMPDEVS 1
#define TEMPDEVS0 TEMPDEVS
#define TEMPDEVS1 1
#if defined(__MONITOR__)
DeviceAddress tempDev0[TEMPDEVS0] =
{
 0x28, 0x35, 0x49, 0x07, 0x33, 0x20, 0x01, 0x21
};
DeviceAddress tempDev1[TEMPDEVS1] =
{
 0x28, 0xd5, 0x2f, 0x2c, 0x33, 0x20, 0x01, 0x61
};
#else
extern DeviceAddress tempDev[TEMPDEVS];
#endif /* __MONITOR__ */

#endif	/* MONITOR_INDEX == 5 */

/* -------------------- monitor index 5 -------------------- */

#if (MONITOR_INDEX == 6)

#define WIFI_ENA 1
#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1

#define EMONCMS_NODE "Garage"

#define ESP8266_TIME 0
#define TEMP_SENSOR 0
#define CHECK_IN 1
#define DHT_SENSOR 1

#define DHTPIN 3

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "Monitor6"

#endif	/* MONITOR_INDEX == 6 */

/* -------------------- monitor index 3 -------------------- */

#if (MONITOR_INDEX == 7)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1

#define WIFI_ENA 1
#define EMONCMS_NODE "Test"

#define ESP8266_TIME 0
#define TEMP_SENSOR 2
#define CHECK_IN 1

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "Test"

#define ONE_WIRE_BUS0 4		/* one wire bus pin */
#define ONE_WIRE_BUS1 4		/* one wire bus pin */
#define ONE_WIRE_BUS ONE_WIRE_BUS1 /* one wire bus pin */

#define TEMPDEVS0 1
#define TEMPDEVS TEMPDEVS0
#define TEMPDEVS1 2
#if defined(__MONITOR__)

#if TEMP_SENSOR == 1

DeviceAddress tempDev[TEMPDEVS] =
{
 {0x10, 0xdc, 0x5d, 0xd4, 0x01, 0x08, 0x00, 0xe9},
};

#elif TEMP_SENSOR == 2

DeviceAddress tempDev0[TEMPDEVS0] =
{
 {0x10, 0xdc, 0x5d, 0xd4, 0x01, 0x08, 0x00, 0xe9},
};
DeviceAddress tempDev1[TEMPDEVS1] =
{
 {0x10, 0xdc, 0x5d, 0xd4, 0x01, 0x08, 0x00, 0xe9},
 {0x10, 0xdc, 0x5d, 0xd4, 0x01, 0x08, 0x00, 0xe9},
};

#endif	/* TEMP_SENSOR == 2 */
#else
extern DeviceAddress tempDev[TEMPDEVS];
#endif /* __MONITOR__ */
 
#endif	/* MONITOR_INDEX == 7 */

/* -------------------- monitor end of definitions -------------------- */

#if WATER_MONITOR

#define WATER0 3
#define WATER1 4

#define BEEPER 10
#define LED 13

#endif

#define WIFI_RESET 2		/* wifi reset */

/*
 [0, "PD0"],
 [1, "PD1"],
 [2, "PD2"],
 [3, "PD3"],
 [4, "PD4"],
 [5, "PD5"],
 [6, "PD6"],
 [7, "PD7"],
 [8, "PB0"],
 [9, "PB1"],
 [10, "PB2"],
 [11, "PB3"],
 [12, "PB4"],
 [13, "PB5"],
 [14, "PC0"],
 [15, "PC1"],
 [16, "PC2"],
 [17, "PC3"],
 [18, "PC4"],
 [19, "PC5"],
*/

#define DBG0_Pin 7
#define DBG0_Port PORTD
#define DBG0_DDR DDRD
#define DBG0_In PIND
#define DBG0_Bit PD7
#define DBG0_Mask _BV(DBG0_Bit)
#define dbg0Read() ((DBG0_Port & DBG0_Mask) != 0)
#define dbg0Set() DBG0_Port |= DBG0_Mask
#define dbg0Clr() DBG0_Port &= ~DBG0_Mask

#define DBG1_Pin 8
#define DBG1_Port PORTB
#define DBG1_DDR DDRB
#define DBG1_In PINB
#define DBG1_Bit PB0
#define DBG1_Mask _BV(DBG1_Bit)
#define dbg1Read() ((DBG1_Port & DBG1_Mask) != 0)
#define dbg1Set() DBG1_Port |= DBG1_Mask
#define dbg1Clr() DBG1_Port &= ~DBG1_Mask

#define DBG2_Pin 9
#define DBG2_Port PORTB
#define DBG2_DDR DDRB
#define DBG2_In PINB
#define DBG2_Bit PB1
#define DBG2_Mask _BV(DBG2_Bit)
#define dbg2Read() ((DBG2_Port & DBG2_Mask) != 0)
#define dbg2Set() DBG2_Port |= DBG2_Mask
#define dbg2Clr() DBG2_Port &= ~DBG2_Mask

#endif	/* ARDUINO_AVR_PRO */
