#define __MAX31865__
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

#include "max31865.h"
#include "main.h"
#include "spix.h"
#include "i2cx.h"
#include "lcd.h"
#include "cyclectr.h"

#if defined(ARDUINO_ARCH_STM32)

#include "Arduino.h"
#define flushBuf flush
#define newLine newline
#define delayMSec delay
#include "serial.h"

#else

#include "serialio.h"
unsigned int millis(void);
void delayMSec(volatile uint32_t mSec);
#endif	/* ARDUINO_ARCH_STM32 */

#if 0 // <-
#if !defined(__MAX31865_INC__)
#define __MAX31865_INC__

#define MX65_CFG       0x00
#define MX65_RTD_MSB   0x01
#define MX65_RTD_LSB   0x02
#define MX65_H_FLT_MSB 0x03
#define MX65_H_FLT_LSB 0x04
#define MX65_L_FLT_MSB 0x05
#define MX65_L_FLT_LSB 0x06
#define MX65_FLT_STAT  0x07

#define MX65_WRITE     0x80

#define MX65_CFG_BIAS     0x80
#define MX65_CFG_MODEAUTO 0x40
#define MX65_CFG_MODEOFF  0x00
#define MX65_CFG_1SHOT    0x20
#define MX65_CFG_3WIRE    0x10
#define MX65_CFG_24WIRE   0x00
#define MX65_CFG_FLT_DET  0x0C
#define MX65_CFG_CLR_FLT  0x02
#define MX65_CFG_FILT50HZ 0x01
#define MX65_CFG_FILT60HZ 0x00

#define MX65_CFG_FLT_AUTO 0x04

#define MX65_FLT_HIGHTHRESH 0x80
#define MX65_FLT_LOWTHRESH  0x40
#define MX65_FLT_REFINLOW   0x20
#define MX65_FLT_REFINHIGH  0x10
#define MX65_FLT_RTDINLOW   0x08
#define MX65_FLT_OVUV       0x04

typedef enum mx65Wires_t
{
  MX65_2WIRE = 0,
  MX65_3WIRE = 1,
  MX65_4WIRE = 0
} MX65_WIRES;


#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

void max65Init(int dev, mx65Wires_t wires);
void max65SetWires(int dev, mx65Wires_t wires);
void max65EnableBias(int dev, bool bias);
void max65AutoConvert(int dev, bool convert);
uint8_t max65ReadFault(int dev);
void max65ClearFault(int dev);
uint8_t max65CheckFault(int dev);
unsigned int max65Temp(int dev);
uint16_t max65ReadRTD(int dev);

void max65Cmds();

#endif	/* __MAX31865_INC__ */
#endif	// ->
#if defined(__MAX31865__)

void max65Init(int dev, mx65Wires_t wires)
{
 max65SetWires(dev, wires);
 max65EnableBias(dev, false);
 max65AutoConvert(dev, false);
 max65ClearFault(dev);
}

void max65SetWires(int dev, mx65Wires_t wires)
{
 uint8_t t = readb(dev, MX65_CFG);
 if (wires == MX65_3WIRE)
  t |=  MX65_CFG_3WIRE;
 else
  t |=  ~MX65_CFG_3WIRE;
}

void max65EnableBias(int dev, bool bias)
{
 uint8_t t = readb(dev, MX65_CFG);
 if (bias)
  t |= MX65_CFG_BIAS;
 else
  t &= ~MX65_CFG_BIAS;
 loadb(dev, MX65_WRITE | MX65_CFG, t);
}

void max65AutoConvert(int dev, bool convert)
{
 uint8_t t = readb(dev, MX65_CFG);
 if (convert)
  t |= MX65_CFG_MODEAUTO;
 else
  t &= ~MX65_CFG_MODEAUTO;
 loadb(dev, MX65_WRITE | MX65_CFG, t);
}

uint8_t max65ReadFault(int dev)
{
 return(readb(dev, MX65_FLT_STAT));
}

void max65ClearFault(int dev)
{
 uint8_t t = readb(dev, MX65_CFG);
 t &= ~(MX65_CFG_1SHOT | MX65_CFG_FLT_DET);
 t |= MX65_CFG_CLR_FLT;
 loadb(dev, MX65_WRITE | MX65_CFG, t);
}

uint8_t max65CheckFault(int dev)
{
 max65ClearFault(dev);
 max65EnableBias(dev, true);
 delayMSec(10);
 uint8_t t = readb(dev, MX65_CFG);
 t |= MX65_CFG_FLT_AUTO;
 loadb(dev, MX65_WRITE | MX65_CFG, t);
 while (true)
 {
  t = readb(dev, MX65_CFG);
  if ((t & MX65_CFG_FLT_DET) == 0)
   break;
 }
 t = readb(dev, MX65_FLT_STAT);
 return(t);
}

/*

The calculation formulas of this Pt-RTD are defined in 
DIN EN 60751 as following:

For T <= 0C: R(T) = R(0) * ((1+a)*T + b*T^2)

For T < 0C: R(T) = R(0) * [(1+a)*T + b*T^2 + c*(T-100)*T^3]

Coefficients:

a = 3.9083E-03
b = -5.775E-07
c = -4.183E-12

*/

#define ADC_MIN 7227
#define ADC_MAX 11346
#define TEMP_MIN -30
#define TEMP_MAX 100
#define TEMP_INC 5
#define MAX_INDEX 26
#define TEMP_SCALE 100

const uint16_t pt100Table[] =
{
  7227,  7388,  7549,  7710,  7871,  8031,  8192,  8352,  8511,  8671,
  8830,  8989,  9148,  9306,  9465,  9623,  9781,  9938, 10096, 10253,
 10410, 10566, 10723, 10879, 11035, 11191, 11346
};

unsigned int max65Temp(int dev)
{
 uint16_t val = max65ReadRTD(dev);
 uint16_t index;
 if ((val > ADC_MIN)
 &&  (val <= ADC_MAX))
 {
  uint16_t loIndex = 0;
  uint16_t hiIndex = MAX_INDEX;
  index = (loIndex + hiIndex) >> 1;

  while (loIndex < hiIndex)
  {
   uint16_t tblVal = pt100Table[index];
   // printf("index %2d r %5d val %5d\n", index, val, tblVal);
   if (val == tblVal)
    break;
   else if (val > tblVal)
    loIndex = index + 1;
   else
    hiIndex = index;
   
   index = (loIndex + hiIndex) >> 1;
  }
  index -= 1;

  unsigned int t0 = (index * TEMP_INC + TEMP_MIN);
  unsigned int lo = pt100Table[index];
  unsigned int hi = pt100Table[index + 1];
  // printf("index %3d %5d %5d %5d\n", index, lo, val, hi);
  // offset = ((val - lo) / (hi - lo)) * TEMP_INC
  unsigned int offsetInt = ((val - lo) * TEMP_INC * TEMP_SCALE) / (hi - lo);
  // printf("t0 %3d offsetInt %4d\n", t0, offsetInt);
  unsigned int tempC = t0 * TEMP_SCALE + offsetInt;
  // unsigned int tempF = (9 * tempC) / 5 + 32 * TEMP_SCALE;
  // printf("temp %5uC %5uF\n", tempC, tempF);
  return(tempC);
 }
 return(-273);
}

uint16_t max65ReadRTD(int dev)
{
 max65ClearFault(dev);
 max65EnableBias(dev, true);
 delayMSec(10);
 uint8_t t = readb(dev, MX65_CFG);
 t |= MX65_CFG_1SHOT;
 loadb(dev, MX65_WRITE | MX65_CFG, t);
 delayMSec(65);
 uint16_t val = read16(dev, MX65_RTD_MSB);
 max65EnableBias(dev, false);
 val >>= 1;
 return(val);
}

char *fmtTemp(unsigned int val, char *buf)
{
 sprintf(buf, "%5d", val);
 buf[6] = 0;
 buf[5] = buf[4];
 buf[4] = buf[3];
 buf[3] = '.';
 return(buf);
}

#if defined(ARDUINO_ARCH_STM32)

char query(unsigned char (*get)(), const char *format, ...);
extern unsigned char getNum();
extern int val;

#else

#define getNum getnum

#endif	/* ARDUINO_ARCH_STM32 */

void max65Cmds(void)
{
 int spiDev = 0;
 
 while (1)
 {
  char ch = prompt("\nrtd: ");
  if (ch == 'i')
  {
   max65Init(spiDev, MX65_2WIRE);
  }
  else if (ch == 'b')
  {
   uint8_t t = readb(spiDev, MX65_CFG);
   if (query(&getNum, "bias [%d]: ", (t & MX65_CFG_BIAS) != 0))
    max65EnableBias(spiDev, val);
   t = readb(spiDev, MX65_CFG);
   printf("cfg %02x\n", (unsigned int) t);
  }
  else if (ch == 'f')
  {
   uint8_t t = max65CheckFault(spiDev);
   printf("fault status %2x\n", (unsigned int) t);
  }
  else if (ch == 'c')
  {
   uint16_t val = max65ReadRTD(spiDev);
   printf("val %4x %5d\n", (unsigned int) val, val);
  }
  else if (ch == 't')
  {
   char buf[8];
   unsigned int tempC = max65Temp(spiDev);
   unsigned int tempF = (9 * tempC) / 5 + 32 * TEMP_SCALE;
   printf("temp %sC ", fmtTemp(tempC, buf));
   printf("temp %sF\n", fmtTemp(tempF, buf));
  }
  else if (ch == 'r')
  {
   printf(" 0  1  2  3  4  5  6  7\n");
   for (int i = 0; i < 8; i++)
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
     unsigned int tempC = max65Temp(spiDev);
     unsigned int tempF = (9 * tempC) / 5 + 32 * TEMP_SCALE;
     char buf[22];
     char cBuf[8];
     char fBuf[8];
     snprintf(buf, sizeof(buf), "%sc %sf",
	      fmtTemp(tempC, cBuf), fmtTemp(tempF, fBuf));
     setCursorBuf(0, 0);
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

#endif	/* __MAX31865__ */
