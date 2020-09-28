#if ARDUINO_AVR_MEGA2560

#define ESP8266_TIME 0

#define MONITOR_INDEX 2

/* outside temperature */

#if (MONITOR_INDEX == 1)
#define EMONCMS_ADDR EMONCMS_ADDR0
#define EMONCMS_KEY EMONCMS_KEY0
#define EMONCMS_NODE "1"

#define TEMP_SENSOR 1
#define RTC_CLOCK 1
#define DHT_SENSOR 0
#define CURRENT_SENSOR 0
#define WATER_MONITOR 0
#define CHECK_IN 0
#define DEHUMIDIFIER 0

#define SSID "dd-wrt_vap"
#define PASS "minidonk"

#define MONITOR_ID "Monitor1"

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

/* basement dehumidifer and furnace monitor */

#if (MONITOR_INDEX == 2)
#define EMONCMS_ADDR EMONCMS_ADDR0
#define EMONCMS_KEY EMONCMS_KEY0
#define EMONCMS_NODE "2"
#define CURRENT0_NODE 3
#define CURRENT1_NODE 4

#define TEMP_SENSOR 1
#define RTC_CLOCK 1
#define DHT_SENSOR 1
#define CURRENT_SENSOR 1
#define WATER_MONITOR 1
#define CHECK_IN 1
#define DEHUMIDIFIER 0

#define SSID "dd-wrt_vap"
#define PASS "minidonk"
#define MONITOR_ID "Monitor2"

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

/* basement water alarm and pump shutoff */

#if (MONITOR_INDEX == 3)
#define EMONCMS_ADDR EMONCMS_ADDR0
#define EMONCMS_KEY EMONCMS_KEY0
#define EMONCMS_NODE "5"

#define TEMP_SENSOR 1
#define RTC_CLOCK 1
#define DHT_SENSOR 1
#define CURRENT_SENSOR 0
#define WATER_MONITOR 1
#define CHECK_IN 0
#define DEHUMIDIFIER 1

#define SSID "dd-wrt_vap"
#define PASS "minidonk"
#define MONITOR_ID "Monitor3"

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

#if (MONITOR_INDEX == 4)
#define EMONCMS_ADDR EMONCMS_ADDR0
#define EMONCMS_KEY EMONCMS_KEY0

#define TEMP_SENSOR 0
#define RTC_CLOCK 0
#define DHT_SENSOR 0
#define CURRENT_SENSOR 0
#define WATER_MONITOR 1
#define CHECK_IN 0
#define DEHUMIDIFIER 0

#define SSID "dd-wrt_vap"
#define PASS "minidonk"
#define MONITOR_ID "Monitor4"
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

#if 0
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
#else
#define dbg2Set()
#define dbg2Clr()
#endif

#endif	/* ARDUINO_AVR_MEGA2560 */
