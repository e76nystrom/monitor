#if defined(ARDUINO_AVR_MEGA2560)

#define ESP8266_TIME 0

#define MONITOR_INDEX 4

#define MON_DBG 1
#define WIFI_DBG 1

/* -------------------- monitor index 1 -------------------- */

/* outside temperature */

#if (MONITOR_INDEX == 1)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1
#define EMONCMS_NODE "FrontPorch"

#define WIFI_ENA

#define TEMP_SENSOR 1
#define RTC_CLOCK 1
#define DHT_SENSOR 0
#define CURRENT_SENSOR 0
#define WATER_MONITOR 0
#define CHECK_IN 1
#define DEHUMIDIFIER 0

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "FrontPorch"

#define TEMPDEVS 1
#if defined(__MONITOR__)
EXT DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0x7d, 0xe3, 0x96, 0x06, 0x00, 0x00, 0x61}
// {0x28, 0xB8, 0x50, 0x9B, 0x06, 0x00, 0x00, 0x89}
};
#else
extern DeviceAddress tempDev[TEMPDEVS];
#endif /* defined(__MONITOR__) */

#endif	/* MONITOR_INDEX == 1 */

/* -------------------- monitor index 2 -------------------- */

/* basement dehumidifer and furnace monitor */

#if (MONITOR_INDEX == 2)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1

#define EMONCMS_NODE "BasementTemp"
#define CURRENT0_NODE "cur0"
#define CURRENT1_NODE "cur1"

#define WIFI_ENA

#define TEMP_SENSOR 1
#define RTC_CLOCK 1
#define DHT_SENSOR 1
#define CURRENT_SENSOR 1
#define WATER_MONITOR 0
#define CHECK_IN 1
#define DEHUMIDIFIER 0

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "Basement"

#define TEMPDEVS 2
#if defined(__MONITOR__)
EXT DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0xff, 0xd3, 0x09, 0x63, 0x14, 0x02, 0xe1},
 {0x28, 0xc8, 0xae, 0x9b, 0x06, 0x00, 0x00, 0x15}
};
#else
extern DeviceAddress tempDev[TEMPDEVS];
#endif /* defined(__MONITOR__) */

#endif	/* MONITOR_INDEX == 2 */

/* -------------------- monitor index 3 -------------------- */

/* basement water alarm and pump shutoff */

#if (MONITOR_INDEX == 3)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1

#define EMONCMS_NODE "MegaTest"
#define CURRENT0_NODE "cTst0"
#define CURRENT1_NODE "cTst1"

#define WIFI_ENA

#define TEMP_SENSOR 1
#define RTC_CLOCK 0
#define DHT_SENSOR 0
#define CURRENT_SENSOR 1
#define WATER_MONITOR 0
#define CHECK_IN 1
#define DEHUMIDIFIER 0

#define OLED_ENA 1

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "MegaTest"

#define TEMPDEVS 1
#if defined(__MONITOR__)
EXT DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0x7d, 0xe3, 0x96, 0x06, 0x00, 0x00, 0x61}
};
#else
extern DeviceAddress tempDev[TEMPDEVS];
#endif /* defined(__MONITOR__) */

#endif	/* MONITOR_INDEX == 3 */

/* -------------------- monitor index 4 -------------------- */

#if (MONITOR_INDEX == 4)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1

//#define WIFI_ENA

#define TEMP_SENSOR 0
#define RTC_CLOCK 0
#define DHT_SENSOR 0
#define CURRENT_SENSOR 0
#define WATER_MONITOR 1
#define CHECK_IN 0
#define DEHUMIDIFIER 0

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "MegaTest1"

#endif	/* MONITOR_INDEX == 4 */

#define WIFI_RESET 2		/* wifi reset */

#define ONE_WIRE_BUS 4		/* one wire bus pin */

#if TEST_NODE

#undef EMONCMS_NODE
#define EMONCMS_NODE "12"

#if CURRENT_SENSOR
#undef CURRENT0_NODE
#define CURRENT0_NODE 13
#undef CURRENT1_NODE
#define CURRENT1_NODE 14
#endif	/* CURRENT_SENSOR */

#endif	/* TEST_MODE */ 

#if WATER_MONITOR

#define WATER0 6
#define WATER1 7

#define BEEPER 12
#define LED 13

#endif /* WATER_MONITOR */

#if DHT_SENSOR

#define DHTPIN 3

#if DEHUMIDIFIER

#define DEHUM_ON_PIN 9
#define DEHUM_OFF_PIN 8

#endif /* DEHUMIDIFIER */
#endif /* DHT_SENSOR */

#if 1

#define DBG0_Pin 62
#define DBG0_Port PORTK
#define DBG0_DDR DDRK
#define DBG0_In PINK
#define DBG0_Bit PK0
#define DBG0_Mask _BV(DBG0_Bit)
#define dbg0Read() ((DBG0_Port & DBG0_Mask) != 0)
#define dbg0Set() DBG0_Port |= DBG0_Mask
#define dbg0Clr() DBG0_Port &= ~DBG0_Mask

#define DBG1_Pin 63
#define DBG1_Port PORTK
#define DBG1_DDR DDRK
#define DBG1_In PINK
#define DBG1_Bit PK1
#define DBG1_Mask _BV(DBG1_Bit)
#define dbg1Read() ((DBG1_Port & DBG1_Mask) != 0)
#define dbg1Set() DBG1_Port |= DBG1_Mask
#define dbg1Clr() DBG1_Port &= ~DBG1_Mask

#define DBG2_Pin 64
#define DBG2_Port PORTK
#define DBG2_DDR DDRK
#define DBG2_In PINK
#define DBG2_Bit PK2
#define DBG2_Mask _BV(DBG2_Bit)
#define dbg2Read() ((DBG2_Port & DBG2_Mask) != 0)
#define dbg2Set() DBG2_Port |= DBG2_Mask
#define dbg2Clr() DBG2_Port &= ~DBG2_Mask

#define DBG3_Pin 65
#define DBG3_Port PORTK
#define DBG3_DDR DDRK
#define DBG3_In PINK
#define DBG3_Bit PK3
#define DBG3_Mask _BV(DBG3_Bit)
#define dbg3Read() ((DBG3_Port & DBG3_Mask) != 0)
#define dbg3Set() DBG3_Port |= DBG3_Mask
#define dbg3Clr() DBG3_Port &= ~DBG3_Mask

#define DBG4_Pin 66
#define DBG4_Port PORTK
#define DBG4_DDR DDRK
#define DBG4_In PINK
#define DBG4_Bit PK4
#define DBG4_Mask _BV(DBG4_Bit)
#define dbg4Read() ((DBG4_Port & DBG4_Mask) != 0)
#define dbg4Set() DBG4_Port |= DBG4_Mask
#define dbg4Clr() DBG4_Port &= ~DBG4_Mask

#define DBG5_Pin 67
#define DBG5_Port PORTK
#define DBG5_DDR DDRK
#define DBG5_In PINK
#define DBG5_Bit PK5
#define DBG5_Mask _BV(DBG5_Bit)
#define dbg5Read() ((DBG5_Port & DBG5_Mask) != 0)
#define dbg5Set() DBG5_Port |= DBG5_Mask
#define dbg5Clr() DBG5_Port &= ~DBG5_Mask

#define DBG6_Pin 68
#define DBG6_Port PORTK
#define DBG6_DDR DDRK
#define DBG6_In PINK
#define DBG6_Bit PK6
#define DBG6_Mask _BV(DBG6_Bit)
#define dbg6Read() ((DBG6_Port & DBG6_Mask) != 0)
#define dbg6Set() DBG6_Port |= DBG6_Mask
#define dbg6Clr() DBG6_Port &= ~DBG6_Mask

#define DBG7_Pin 69
#define DBG7_Port PORTK
#define DBG7_DDR DDRK
#define DBG7_In PINK
#define DBG7_Bit PK7
#define DBG7_Mask _BV(DBG7_Bit)
#define dbg7Read() ((DBG7_Port & DBG7_Mask) != 0)
#define dbg7Set() DBG7_Port |= DBG7_Mask
#define dbg7Clr() DBG7_Port &= ~DBG7_Mask

#else
#define dbg2Set()
#define dbg2Clr()
#endif

#endif	/* ARDUINO_AVR_MEGA2560 */
