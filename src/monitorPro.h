#ifdef ARDUINO_AVR_PRO

SoftwareSerial dbgPort = SoftwareSerial(rxPin, txPin);

#define MONITOR_INDEX 3

#if (MONITOR_INDEX == 0)

#define EMONCMS_ADDR EMONCMS_ADDR0
#define EMONCMS_KEY EMONCMS_KEY0

#define ESP8266_TIME 0
#define TEMP_SENSOR 0
#define RTC_CLOCK 0
#define CURRENT_SENSOR 0
#define DHT_SENSOR 0
#define WATER_MONITOR 1
#define CHECK_IN 0
#define DEHUMIDIFIER 0

#define SSID "TKGCL"
#define PASS "K4PLVFMCXGMLXM9P"
#define MONITOR_ID "Monitor0"
#endif	/* MONITOR_INDEX == 0 */

#if (MONITOR_INDEX == 2)

#define ESP8266_TIME 0
#define TEMP_SENSOR 0
#define RTC_CLOCK 0
#define CURRENT_SENSOR 0
#define DHT_SENSOR 0
#define WATER_MONITOR 1
#define CHECK_IN 0
#define DEHUMIDIFIER 0

#define SSID "dd-wrt_vap"
#define PASS "minidonk"
#define MONITOR_ID "Monitor2"
#endif	/* MONITOR_INDEX == 2 */

#if (MONITOR_INDEX == 3)

#define EMONCMS_ADDR EMONCMS_ADDR1
#define EMONCMS_KEY EMONCMS_KEY1

#define EMONCMS_NODE "4"

#define ESP8266_TIME 0
#define TEMP_SENSOR 1
#define RTC_CLOCK 0
#define CURRENT_SENSOR 0
#define DHT_SENSOR 0
#define WATER_MONITOR 0
#define CHECK_IN 1
#define DEHUMIDIFIER 0

#define SSID "hug2g996565"
#define PASS "candle14salt"
#define MONITOR_ID "Monitor3"

#define ONE_WIRE_BUS 4		/* one wire bus pin */

#define TEMPDEVS 1
DeviceAddress tempDev[TEMPDEVS] =
{
 {0x28, 0x7d, 0xe3, 0x96, 0x06, 0x00, 0x00, 0x61}
};
#endif	/* MONITOR_INDEX == 3 */

#if WATER_MONITOR

#define WATER0 3
#define WATER1 4

#define BEEPER 10
#define LED 13

#endif

#endif	/* ARDUINO_AVR_PRO */
