#define __MAX31856__
#ifdef STM32F1
#include "stm32f1xx_hal.h"
#endif

#if defined(ARDUINO_ARCH_STM32)
#define SPIn SPI2
#endif	/* ARDUINO_ARCH_STM32 */

#if defined(STM32MON)
#define SPIn SPI2
#endif	/* STM32MON */

#include <stdio.h>

#include "max31856.h"
#include "main.h"
#include "spix.h"
#include "i2cx.h"
#include "lcd.h"
#include "cyclectr.h"

#if defined(ARDUINO_ARCH_STM32)

#include "Arduino.h"
#define flushBuf flush
#define newLine newline
#include "serial.h"
#define delayMSec delay

#else

#include "serialio.h"
unsigned int millis(void);
void delayMSec(volatile uint32_t mSec);

#endif	/* ARDUINO_ARCH_STM32 */

#if 0 // <-
#if !defined(__MAX31856_INC__)
#define __MAX31856_INC__

#define MX56_CR0    0x00	/* Config 0 register */
#define MX56_CR1    0x01	/* Config 1 register */
#define MX56_MASK   0x02	/* Fault Mask register */
#define MX56_CJHF   0x03	/* cold junction High fault */
#define MX56_CJLF   0x04	/* cold junction Low fault */
#define MX56_LTHFTH 0x05	/* msb high temp fault */
#define MX56_LTHFTL 0x06	/* lsb high temp fault */
#define MX56_LTLFTH 0x07	/* msb low temp fault */
#define MX56_LTLFTL 0x08	/* lsb low temp fault */
#define MX56_CJTO   0x09	/* cold junction offset */
#define MX56_CJH    0x0A	/* msb cold junction temp */
#define MX56_CJL    0x0B	/* lsb cold junction temp */
#define MX56_LTB2   0x0C	/* byte 2 temp */
#define MX56_LTB1   0x0D	/* byte 1 temp */
#define MX56_LTB0   0x0E	/* byte 0 temp */
#define MX56_SR     0x0F	/* fault status */

#define MX56_W 0x80		/* register write flag */

#define MX56_CR0_AUTOCONVERT 0x80 /* auto convert flag */
#define MX56_CR0_1SHOT       0x40 /* one shot convert flag */
#define MX56_CR0_OCFAULT1    0x20 /* open circuit fault 1 flag */
#define MX56_CR0_OCFAULT0    0x10 /* open circuit fault 0 flag */
#define MX56_CR0_CJ          0x08 /* cold junction disable flag */
#define MX56_CR0_FAULT       0x04 /* fault mode flag */
#define MX56_CR0_FAULTCLR    0x02 /* fault clear flag */
#define MX56_CR0_N50	     0x01 /* 50hz noise filter */

#define MX56_FAULT_CJRANGE 0x80 /* cold junction out of range */
#define MX56_FAULT_TCRANGE 0x40 /* temp out of range */
#define MX56_FAULT_CJHIGH  0x20 /* cold junction high */
#define MX56_FAULT_CJLOW   0x10 /* cold junction low */
#define MX56_FAULT_TCHIGH  0x08 /* temp high */
#define MX56_FAULT_TCLOW   0x04 /* temp low */
#define MX56_FAULT_OVUV    0x02 /* over or under voltage */
#define MX56_FAULT_OPEN    0x01 /* open circuit */

typedef enum			/* noise filtering */
{
  MX56_FILTER_50HZ,
  MX56_FILTER_60HZ
} mx56_filter_t;

typedef enum			/* thermocouple types */
{
  MX56_TCTYPE_B = 0b0000,
  MX56_TCTYPE_E = 0b0001,
  MX56_TCTYPE_J = 0b0010,
  MX56_TCTYPE_K = 0b0011,
  MX56_TCTYPE_N = 0b0100,
  MX56_TCTYPE_R = 0b0101,
  MX56_TCTYPE_S = 0b0110,
  MX56_TCTYPE_T = 0b0111,
  MX56_VMODE_G8 = 0b1000,
  MX56_VMODE_G32 = 0b1100,
} mx56_type_t;

#define MX56_CR1_THERMO_MASK 0xf /* thermocouple type mask */

typedef enum			/* conversion modes */
{
  MX56_ONESHOT,
  MX56_ONESHOT_NOWAIT,
  MX56_CONTINUOUS
} mx56_conversion_t;

void max56Init(int dev, mx56_type_t type, mx56_conversion_t mode);
void max56SetConversionType(int dev, mx56_conversion_t type);
void max56SetThermocoupleType(int dev, mx56_type_t type);

int32_t max56ReadTemp();
float max56ConvTemp(int t);
char *max56FmtTemp(char *buf, size_t bufLen);
char *max56FmtTemp(int32_t temp, char *buf, size_t bufLen);

int32_t max56ReadCJ();
float max56ConvCJ(int t);
char *max56FmtCJ(char *buf, size_t bufLen);
char *max56FmtCJ(int32_t temp, char *buf, size_t bufLen);

void max56Cmds(void);

#endif	/* __MAX31856_INC__ */
#endif	// ->

#if defined(__MAX31856__)

void max56Init(int dev, mx56_type_t type, mx56_conversion_t mode)
{
 loadb(dev, MX56_MASK | MX56_W, 0x0);	/* enable all faults */
 loadb(dev, MX56_CR0 | MX56_W, MX56_CR0_OCFAULT0); /* enable open detection */
 loadb(dev, MX56_CJTO | MX56_W, 0x0);	/* set cold junct compensation */
 max56SetConversionType(dev, mode);		/* conversion mode */
 max56SetThermocoupleType(dev, type);	/* set thermocouple type */
}

void max56SetConversionType(int dev, mx56_conversion_t type)
{
 uint8_t tmp = readb(dev, MX56_CR0); /* read register */
 tmp &= ~(MX56_CR0_AUTOCONVERT | MX56_CR0_1SHOT);
 if (type == MX56_CONTINUOUS)
  tmp |= MX56_CR0_AUTOCONVERT;
 else
  tmp |= MX56_CR0_1SHOT;
 loadb(dev, MX56_CR0 | MX56_W, tmp);
}

void max56SetThermocoupleType(int dev, mx56_type_t type)
{
 uint8_t tmp = readb(dev, MX56_CR1); /* read register */
 tmp &= ~MX56_CR1_THERMO_MASK;
 tmp |= (uint8_t) type;
 loadb(dev, MX56_CR1 | MX56_W, tmp); /* set thermocouple type */
 tmp = readb(dev, MX56_CR1);
// printf("cr1 %02x\n", (unsigned int) tmp);
}

int32_t max56ReadTemp(int dev)
{
 int32_t rtnVal = read24(dev, MX56_LTB2); /* read temp */
// printf("max56ReadTemp %06x\n", (unsigned int) rtnVal);
 if (rtnVal & 0x100000)		/* if negative */
  rtnVal |= 0xff000000;		/* sign extend */
 return(rtnVal);
}

float max56ConvTemp(int t)
{
 return((float) t / 4096.0);
}

char *max56FmtTemp(int dev, char *buf, size_t bufLen)
{
 return(max56FmtTemp(max56ReadTemp(dev), buf, bufLen));
}

char *max56FmtTemp(int32_t temp, char *buf, size_t bufLen)
{
 if (bufLen > 0)
 {
  char *p = buf;
  if (temp < 0)
  {
   temp = -temp;
   *buf++ = '-';
   bufLen -= 1;
  } 
  int d = temp >> 12;
  int f = (temp >> 5) & 0x7f;
  snprintf(p, bufLen, "%d.%d", d, f);
 }
 return(buf);
}

int32_t max56ReadCJ(int dev)
{
 int32_t rtnVal = read16(dev, MX56_CJH); /* read cold junction temp */
// printf("max56ReadCJ %04x\n", (unsigned int) rtnVal);
 if (rtnVal & 0x8000)		/* if negative */
  rtnVal |= 0xffff0000;		/* sign extend */
 return(rtnVal);
}

float max56ConvCJ(int t)
{
 return((float) t / 256.0);
}

char *max56FmtCJ(int dev, char *buf, size_t bufLen)
{
 return(max56FmtCJ(max56ReadCJ(dev), buf, bufLen));
}

char *max56FmtCJ(int32_t temp, char *buf, size_t bufLen)
{
 if (bufLen > 0)
 {
  char *p = buf;
  if (temp < 0)
  {
   temp = -temp;
   *p++ = '-';
   bufLen -= 1;
  } 
  int d = temp >> 8;
  int f = (temp >> 2) & 0x3f;
  snprintf(p, bufLen, "%d.%d", d, f);
 }
 return(buf);
}

void max56Cmds(void)
{
 int spiDev = 0;
 
 while (1)
 {
  char ch = prompt("\nthermocouple: ");
  if (ch == 'i')
  {
   max56Init(spiDev, MX56_TCTYPE_K, MX56_ONESHOT);
  }
  else if (ch == 't')
  {
   max56SetConversionType(spiDev, MX56_ONESHOT);
   delayMSec(200);
   int32_t temp = max56ReadTemp(spiDev);
   char buf[10];
   printf("temp %sc %5.2ff\n", max56FmtTemp(temp, buf, sizeof(buf)),
	  (max56ConvTemp(temp) * 9) / 5 + 32);
  }
  else if (ch == 'c')
  {
   char buf[10];
   int32_t temp = max56ReadCJ(spiDev);
   printf("cold junction %s %5.2ff\n", max56FmtCJ(temp, buf, sizeof(buf)),
	  (max56ConvCJ(temp) * 9) / 5 + 32);
  }
  else if (ch == 'r')
  {
   printf(" 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15\n");
   for (int i = 0; i < 16; i++)
    printf("%02x ", (readb(spiDev, i) & 0xff));
   printf("\n");
  }

#if !defined(ARDUINO_ARCH_STM32)
  else if (ch == 'd')
  {
   lcdInit();
   unsigned int t = millis();
   while (1)
   {
    if (dbgRxReady() != 0)
    {
     ch = dbgRxRead();
     if (ch == 3)
      break;
    }
    unsigned int t1 = millis();
    if ((t1 - t) > 1000)
    {
     t = t1;
     max56SetConversionType(spiDev, MX56_ONESHOT);
     delayMSec(200);
     int32_t temp = max56ReadTemp(spiDev);
     setCursorBuf(0, 0);
     float c = max56ConvTemp(temp);
     char buf[22];
     snprintf(buf, sizeof(buf), "%5.2fc %6.2ff", c, (c * 9.0) / 5.0 + 32.0);
     //max56FmtTemp(temp, buf, sizeof(buf));
     lcdString(buf);
     i2cSend();
    }
    while (i2cCtl.state != I_IDLE)
     i2cControl();
    while (pollBufChar() != 0)
     ;
   }
  }
#endif	/* ARDUINO_ARCH_STM32 */

  else if (ch == 'x')
  {
   break;
  }
 }
}

#endif	/* __MAX31856 */
