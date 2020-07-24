#ifdef MEGA32
#include "WProgram.h"
#include "printf.h"

#define TEMP_SENSOR 1
#define DHT_SENSOR 1
#define RTC_CLOCK 0
#define CURRENT_SENSOR 0
#define WATER_MONITOR 0
#define DEHUMIDIFIER 0

#define TS_KEY "67TDONLKRDNVF7L4"
#define EMONCMS_NODE "0"

#define TEMPDEVS 1
DeviceAddress tempDev[TEMPDEVS] =
{
 {0x10, 0xDC, 0x5D, 0xD4, 0x01, 0x08, 0x00, 0xE9}
};

#if DHT_SENSOR
#define DHTPIN 4
#endif /* DHT_SENSOR */

#endif	/* MEGA32 */
