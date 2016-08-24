#if !INCLUDE
#ifdef ARDUINO_ARCH_AVR
#include "Arduino.h"
#endif

#ifdef MEGA32
#inclue "WProgram.h"
#endif

#ifdef WIN32
#include "stdio.h"
#include "string.h"
#endif

#include "serial.h"
#include "wifi.h"

#define EXT
#endif

#include "stdint.h"

#define htons(x) ((int16_t) (((x) << 8) | (((x) >> 8) & 0xFF)))
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

#define DNS_IP "8.8.8.8"
#define DNS_PORT "53"

#define QUERY_FLAG               (0)
#define OPCODE_STANDARD_QUERY    (0)
#define RECURSION_DESIRED_FLAG   (1<<8)
#define LABEL_COMPRESSION_MASK   (0xC0)

#define TYPE_A                   (0x0001)
#define CLASS_IN                 (0x0001)

    //                                    1  1  1  1  1  1
    //      0  1  2  3  4  5  6  7  8  9  0  1  2  3  4  5
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                      ID                       |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |QR|   Opcode  |AA|TC|RD|RA|   Z    |   RCODE   |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    QDCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    ANCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    NSCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    ARCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+

typedef struct
{
 int16_t id;
 int16_t flags;
 int16_t qdcount;
 int16_t ancount;
 int16_t nscount;
 int16_t arcount;
} T_DNS, *P_DNS;

typedef struct
{
 int16_t type;
 int16_t dnsClass;
 int32_t ttl;
 int16_t len;
} T_DNSHDR, *P_DNSHDR;

#define IP_ADDRESS_LEN 16	/* ip address buffer length */

char *htonsCpy(char *p, int16_t val);
int dnsMsg(char *buffer, int buflen, char *name);
char *dnsDecode(char *buffer, int len, char *ip);
char dnsLookup(char *buf, char *hostName);
EXT char dnsBuffer[64];

#if !INCLUDE

char dnsLookup(char *buf, char *hostName)
{
 wifiMux();
 wifiWriteStr(F2("AT+CIPSTART=3,\"UDP\",\"" DNS_IP "\"," DNS_PORT), 3000);
 int dnsLen = dnsMsg(dnsBuffer, sizeof(dnsBuffer), hostName);
 if (DBG)
  printf(F0("\nhost %s dnsLen %d\n"), hostName, dnsLen);

 sprintf((char *) cmdBuffer, F0("AT+CIPSEND=3,%d"), dnsLen);
 wifiStartData((char *) cmdBuffer, strlen(cmdBuffer), 1000);

 int dataLen;
 char *p = wifiWriteTCPx((char *) dnsBuffer, dnsLen, &dataLen, 3000);
 newLine();
// printBuf();

 char *ip = 0;			/* pointer ip address */
 if (p != 0)			/* if wifi returned result */
 {
  ip = dnsDecode(p, dataLen, (char *) stringBuffer); /* decode dns */
 }
 else
 {
  printf(F3("**dns no data returned\n"));
 }

 wifiWriteStr(F2("AT+CIPCLOSE=3"), 1000);

 if (ip != 0)			/* if successful decode */
 {
  strncpy(buf, ip, IP_ADDRESS_LEN); /* save to return buffer */
  if (DBG)
  {
   printf(F0("ip %s\n"), ip);
  }
  return(1);
 }
 else				/* if failure */
  printf(F3("**dns fail using %s\n"), buf);

 return(0);
}

char *htonsCpy(char *p, int16_t val)
{
 *p++ = (char) (val >> 8);
 *p++ = (char) (val & 0xff);
 return(p);
}

int dnsMsg(char *buffer, int buflen, char *name)
{
 P_DNS dns = (P_DNS) buffer;
 dns->id = (int16_t) millis();
 dns->flags = htons(QUERY_FLAG | OPCODE_STANDARD_QUERY |
		    RECURSION_DESIRED_FLAG);
 dns->qdcount = htons(1);
 dns->nscount = 0;
 dns->ancount = 0;
 dns->arcount = 0;
 char *p = (char *) &buffer[sizeof(T_DNS)];
 *p = 0;
 char *size = p;
 p++;
 char ch;
 while ((ch = *name++) != 0)
 {
  if (ch == '.')
  {
   *p = 0;
   size = p;
   p++;
  }
  else
  {
   *size += 1;
   *p++ = ch;
  }
 }
 *p++ = 0;
 p = htonsCpy(p, TYPE_A);
 p = htonsCpy(p, CLASS_IN); 
 return((int) (p - buffer));
}

typedef union
{
 struct
 {
  char lo;
  char hi;
 };
 struct
 {
  int16_t w;
 };
} BYTE_INT16;


char *ntohsCpy(char *p, int16_t *val)
{
 BYTE_INT16 tmp;
 tmp.hi = *p++;
 tmp.lo = *p++;
 *val = tmp.w;
 return(p);
}

char *dnsDecode(char *buffer, int len, char *ip)
{
 int16_t answerCount;
 char *p = (char *) &((P_DNS) buffer)->ancount;
 ntohsCpy(p, &answerCount);	// read answer count
 p = (char *) &buffer[sizeof(T_DNS)]; // skip over header
 char ch;
 while ((ch = *p++) != 0)	// skip over name
  p += ch;
 p += 2 * sizeof(int16_t); 	// skip over type and class
 printf(F3("dns answerCount %d\n"), answerCount);
 while (--answerCount >= 0)	// while answers to process
 {
  ch = *p++;			// read length
  if ((ch & LABEL_COMPRESSION_MASK) == 0) // if no compression
  {
   if (ch > 0)			// if len positive
    p += ch;
  }
  else				// if points somewhere else
  {
   p++;				// skip over value
  }
  int16_t dnsType;
  int16_t dnsClass;
  p = ntohsCpy(p, &dnsType);	// read type
  p = ntohsCpy(p, &dnsClass);	// read class
  p += sizeof(int32_t);		// skip time to live
  int16_t ansLen;
  p = ntohsCpy(p, &ansLen);	// read length
  printf(F3("dnsType %d dnsClass %d ansLen %d\n"), dnsType, dnsClass, ansLen);
  if ((dnsType == TYPE_A)	// if correct type
  &&  (dnsClass == CLASS_IN))
  {
   if (ansLen == 4)		// if answer correct length
   {
    char *p1 = ip;
    while (--ansLen >= 0)
    {
     sprintf(p1, F0("%u"), (int) (*p++ & 0xff));
     if (ansLen > 0)
     {
      while (1)
      {
       ch = *p1;
       if (ch == 0)
	break;
       p1++;
      }
      *p1++ = '.';
     }
    }
   }
   else				// if not correct answer
   {
    printf(F3("dns incorrect type %x class %x\n"), dnsType, dnsClass);
    ip = 0;
   }
   break;
  }
  else				// if not correct record type
  {
   p += ansLen;			// skip to next record
  }
 }
 return(ip);
}

#endif
