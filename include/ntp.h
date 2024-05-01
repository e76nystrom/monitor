#if 1	// <-

const int NTP_PACKET_SIZE = 48;

typedef struct
{
 char version;
 char stratum;
 char poll;
 char precision;
 int32_t rootDelay;
 int32_t rootDispersion;
 int32_t refID;
 int32_t RefSec;
 int32_t refFrac;
 int32_t originSec;
 int32_t originFrac;
 int32_t rcvSec;
 int32_t rcvFrac;
 int32_t txSec;
 int32_t txFrac;
} T_NTP, *P_NTP;

void printTime();
void printTime(time_t t);
void printTime(time_t t, bool flag);
char *fmtTime(char *buf, size_t len);
char *fmtTime(char *buf, size_t len, time_t t);

char ntpSetTime();

EXT unsigned long ntpStart;	/* reference for time compare */
EXT unsigned long ntpTimeout;	/* ntp timeout */
EXT char ntpIP[IP_ADDRESS_LEN];	/* ntp ip address */

#if defined(WIFI_SERIAL)
#define NTP_PORT "123"
#endif	/* WIFI_SERIAL */

#if defined(WIFI_ESP32)
#define NTP_PORT 123
#endif	/* WIFI_ESP32 */

#endif	/* __NTP_INC__ */ // ->
