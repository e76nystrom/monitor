#if !defined(INCLUDE)
#include "Arduino.h"
#include <EEPROM.h>
#include "crc32.h"
#include "config.h"
#include "serial.h"
#define EXT
#endif

typedef struct s_eeprom
{
 unsigned char ssid[12];	/* wifi ssid */
 unsigned char pass[20];	/* wifi password */
 unsigned char id[16];		/* unit id */
 unsigned char emon[16];	/* emon ip address */
} T_EEDATA, *P_EEDATA;

#define EE_LOC ((unsigned char) 0)
#define EE_LEN ((char) sizeof(T_EEDATA))
#define CRC32_LOC ((unsigned char) EE_LEN)

EXT T_EEDATA eeprom;

void initEE();
char readAllEE();
#if CONSOLE
void updateEE();
#endif
uint32_t crc32EE();
void writeCRC32EE();

void readEE(char *buf, char addr, char len);
void writeEE(const char *buf, char addr, char eeLen);

void readEE(void *buf, unsigned char addr, char len);
void writeEE(void *buf, unsigned int addr, char eeLen);

#if !defined(INCLUDE)

void initEE()
{
 if (readAllEE())		/* if eeprom read error */
 {

  writeEE((void *) &eeprom, EE_LOC, EE_LEN);
  writeCRC32EE();		/* write new crc */
 }
}

char readAllEE()
{
 readEE((void *) &eeprom, EE_LOC, EE_LEN);
 uint32_t curCRC32;
 readEE((void *) &curCRC32, CRC32_LOC, (char) sizeof(uint32_t));
 uint32_t newCRC32 = crc32EE();
 if (curCRC32 != newCRC32)
 {
#if CONSOLE
  puts(F1("eeprom crc error\n"));
#endif
  return(1);
 }
 return(0);
}

#if CONSOLE

#if 0
typedef struct
{
 const char *str;		/* prompt */
 void *ramData;			/* ram location */
 unsigned int eeData;		/* eeprom location */
 char len;			/* data size */
} T_EE_DESC, *P_EE_DESC;

const PROGMEM T_EE_DESC eeDesc[] =
{
 {"ssid [%s]: ", (void *) &eeprom.ssid,
  (unsigned int) (&((P_EEDATA) EE_LOC)->ssid), sizeof(eeprom.ssid)},
 {"pass [%s]: ", (void *) &eeprom.pass,
  (unsigned int) (&((P_EEDATA) EE_LOC)->pass), sizeof(eeprom.pass)},
 {"id [%s]: ", (void *) &eeprom.id,
  (unsigned int) (&((P_EEDATA) EE_LOC)->id), sizeof(eeprom.id)},
 {"emon [%s]: ", (void *) &eeprom.emon,
  (unsigned int) (&((P_EEDATA) EE_LOC)->emon), sizeof(eeprom.emon)},
};

void updateEE1()
{
 void change = 0;
 for (unsigned char i = 0;
      i < (unsigned char) (sizeof(eeDesc) / sizeof(T_EE_DESC)); i++)
 {
  char *str = (char *) pgm_read_ptr_near(&eeDesc[i].str);
  void *ramData = (void *) pgm_read_ptr_near(&eeDesc[i].ramData);
  unsigned int eeData = (unsigned int) pgm_read_word_near(&eeDesc[i].eeData);
  char len = (char) pgm_read_byte_near(&eeDesc[i].len);
  unsigned int data;
  if (len == 1)
   data = *(char *) ramData;
  else if (len == 2)
   data = *(unsigned int *) ramData;
  if (query(&getnum, buf), data))
  {
   eeprom.threshold = val;
   writeEE((void *) &eeprom.threshold,
	   (unsigned int) (&((P_EEDATA) EE_LOC)->threshold),
	   sizeof(eeprom.threshold));
   change = 1;
  }
 }
}
#endif

void updateEE()
{
 char change = 0;
 if (query(&getnum, F1("ssid [%s]: "), eeprom.ssid))
 {
  eeprom.threshold = val;
  writeEE((void *) &eeprom.ssid,
	  (unsigned int) (&((P_EEDATA) EE_LOC)->ssid),
	  sizeof(eeprom.ssid));
  change = 1;
 }
 if (query(&getnum, F1("pass [%s]: "), eeprom.pass))
 {
  eeprom.lowSpeed = val;
  writeEE((void *) &eeprom.pass,
	  (unsigned int) (&((P_EEDATA) EE_LOC)->pass),
	  sizeof(eeprom.pass));
  change = 1;
 }
 if (query(&getnum, F1("id [%s]: "), eeprom.id))
 {
  eeprom.highSpeed = val;
  writeEE((void *) &eeprom.id,
	  (unsigned int) (&((P_EEDATA) EE_LOC)->id),
	  sizeof(eeprom.id));
  change = 1;
 }
 if (query(&getnum, F1("emon [%s]: "), eeprom.emon))
 {
  eeprom.divisor = val;
  writeEE((void *) &eeprom.emon,
	  (unsigned int) (&((P_EEDATA) EE_LOC)->emon),
	  sizeof(eeprom.emon));
  change = 1;
 }
 if (change)
 {
  printf(F1("crc updated\n"));
  writeCRC32EE();
 }
}

#endif

uint32_t crc32EE()
{
 uint32_t crc32 = ~0L;
 for (unsigned char i = EE_LOC; i < CRC32_LOC; i++)
 {
  char data = EEPROM.read(i);
  crc32 = CRC32::update(crc32, data);
 }
 return(crc32);
}

void writeCRC32EE()
{
 uint32_t crc32 = crc32EE();
 writeEE((void *) &crc32, CRC32_LOC, (char) sizeof(crc32));
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

void readEE(void *buf, unsigned char addr, char len)
{
 char *p = (char *) buf;
 while (--len >= 0)
 {
  *p++ = EEPROM.read(addr);
  addr++;
 }
}

void writeEE(void *buf, unsigned int addr, char len)
{
 char *p = (char *) buf;
 for (int i = 0; i < len; i++)
 {
  EEPROM.write(addr, *p);
  addr++;
  p++;
 }
}

#endif
