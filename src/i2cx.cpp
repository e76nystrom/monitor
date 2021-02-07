#if !defined(INCLUDE)
#define __I2C__
#include <stdio.h>
#include <limits.h>

#define STD_LIB 0

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#endif
#ifdef STM32F1
#include "stm32f1xx_hal.h"
#endif

#include "main.h"
//#include "lathe.h"

#if defined(ARDUINO_ARCH_STM32)
#include "Arduino.h"
#include "serial.h"
#define flushBuf flush
#else
#include "serialio.h"
unsigned int millis(void);
#endif

#include "current.h"
#include "cyclectr.h"

#define MAX_TIMEOUT 100000

#ifdef EXT
#undef EXT
#endif

#define EXT
#include "i2cx.h"
#endif	/* !defined(INCLUDE) */

#if  defined(__I2C_INC__)	// <-

#if !defined(I2C_DEV)
#define I2C_DEV I2C1
#endif

#if !defined(EXT)
#define EXT extern
#endif

int i2c_start(I2C_TypeDef* I2Cx, uint8_t address);

void initI2c(void);
void i2cWrite(uint8_t data);

void i2cPut(uint8_t ch);
void i2cPutString(uint8_t *p, int size);
void i2cSend(void);
void i2cControl(void);

enum I2C_STATES
{
 I_IDLE,                        /* 0 idle state */
 I_WAIT_START,			/* 1 wait for not busy to start */
 I_START,			/* 2 wait for start and send address */
 I_ADDRESS,			/* 3 wait for address and start data */
 I_SEND_DATA,			/* 4 send data */
 I_WAIT_DATA,			/* 5 wait for data to be sent */
};

enum I2C_STATUS
{
 IS_DONE,			/* 0 done */
 IS_BUSY,			/* 1 busy */
 IS_TIMEOUT,			/* 2 timeout */
};

#define I2C_BUF_SIZE 256
#define I2C_TIMEOUT 500U

typedef struct
{
 int state;
 int lastState;
 int status;
 unsigned int startTime;
 unsigned int timeout;
 unsigned int maxTime;
 int fil;
 int emp;
 int count;
 uint8_t buffer[I2C_BUF_SIZE];
} T_I2C_CTL, *P_I2C_CTL;

EXT T_I2C_CTL i2cCtl;

#define SLAVE_ADDRESS 0x27 // the slave address (example)

EXT char slaveAddress;

inline void i2c_SendData(I2C_TypeDef* I2Cx, uint8_t data)
{
 I2Cx->DR = data;
}

inline void i2c_stop(I2C_TypeDef* I2Cx)
{
 I2Cx->CR1 |= I2C_CR1_STOP;
}

#endif	// ->
#ifdef __I2C__

int i2c_start(I2C_TypeDef* I2Cx, uint8_t address)
{
 unsigned int timeout = 100 * (HAL_RCC_GetHCLKFreq() / 1000000);
 unsigned int start = getCycles();
 while ((I2Cx->SR2 & I2C_SR2_BUSY) != 0)
 {
  if ((getCycles() - start) > timeout)
  {
   printf("i2c_start busy timeout\n");
   return(1);
  }
 }
  
 I2Cx->CR1 |= I2C_CR1_START;
	  
 start = getCycles();
 while ((I2Cx->SR1 & I2C_SR2_MSL) == 0) /* wait for master mode */
 {
  if ((getCycles() - start) > timeout)
  {
   printf("i2c_start master mode failed\n");
   return(1);
  }
 }
		
 address &= ~I2C_OAR1_ADD0;
 I2Cx->DR = address;
 start = getCycles();
 while (1)
 {
  if ((I2C_DEV->SR1 == (I2C_SR1_ADDR | I2C_SR1_TXE))
  &&  ((I2C_DEV->SR2 & 0xff) == (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA)))
  {
   break;
  }
  if ((getCycles() - start) > timeout)
  {
   printf("i2c_start %d address failed\n", address >> 1);
   return(1);
  }
 }
 return(0);
}

void i2cWrite(uint8_t data)
{
 i2c_start(I2C_DEV, slaveAddress);
 
 i2c_SendData(I2C_DEV, data);

 // wait for I2C1 EV8_2 --> byte has been transmitted
 // #define  I2C_EVENT_MASTER_BYTE_TRANSMITTED
 // ((uint32_t)0x00070084)  /* TRA, BUSY, MSL, TXE and BTF flags */
 while (1)
 {
  if ((I2C_DEV->SR1 == (I2C_SR1_BTF | I2C_SR1_TXE))
  &&  ((I2C_DEV->SR2 & 0xff) == (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA)))
  {
   break;
  }
 }

 i2c_stop(I2C_DEV);
}

void i2cPut(uint8_t ch)
{
 P_I2C_CTL i2c = &i2cCtl;
 if (i2c->count < I2C_BUF_SIZE)
 {
  int fill = i2c->fil;		/* temp copy of fill pointer */
  i2c->buffer[fill++] = ch;	/* put character in buffer */
  if (fill >= I2C_BUF_SIZE)	/* if past end of buffer */
   fill = 0;			/* reset to zero */
  i2c->fil = fill;		/* save fill pointer */
  i2c->count++;			/* update count */
 }
}

void i2cPutString(uint8_t *p, int size)
{
 P_I2C_CTL i2c = &i2cCtl;
 int fill = i2c->fil;		/* temp copy of fill pointer */
 uint8_t *dst = &i2c->buffer[fill]; /* get pointer to data buffer */
 while (i2c->count < I2C_BUF_SIZE) /* if room */
 {
  --size;			/* count of size */
  if (size < 0)			/* if done */
   break;			/* exit */
  i2c->count++;			/* update count */
  *dst++ = *p++;		/* put character in buffer */
  fill++;			/* update index */
  if (fill >= I2C_BUF_SIZE)	/* if past end of buffer */
  {
   dst = i2c->buffer;		/* reset pointer */
   fill = 0;			/* reset to zero */
  }
 }
 i2c->fil = fill;		/* save index */
}

void i2cSend(void)
{
 P_I2C_CTL i2c = &i2cCtl;
// printf("i2cSend %d\n", i2c->count);
 i2c->timeout = I2C_TIMEOUT;
 i2c->startTime = millis();
 i2c->state = I_WAIT_START;
 i2c->status = IS_BUSY;
 i2c->lastState = I_IDLE;
}

void i2cControl(void)
{
 P_I2C_CTL i2c = &i2cCtl;

 if (i2c->state != I_IDLE)
 {
  unsigned int delta = millis() - i2c->startTime;
  if (delta > i2c->timeout)
  {
   printf("state %d count %d sr2 %08x sr1 %08x*\n",
	  i2c->state, i2c->count,
	  (unsigned int) I2C_DEV->SR2, (unsigned int) I2C_DEV->SR1);
   flushBuf();
   printf("timeout\n");
   flushBuf();
   I2C_DEV->CR1 |= I2C_CR1_STOP;
   i2c->emp = 0;
   i2c->fil = 0;
   i2c->count = 0;
   i2c->state = I_IDLE;
   i2c->lastState = I_IDLE;
   i2c->status = IS_TIMEOUT;
   return;
  }
  if (delta > i2c->maxTime)
   i2c->maxTime = delta;
 }

#if 0
 if (i2c->state != i2c->lastState)
 {
  i2c->lastState = i2c->state;
  printf("state %d count %d sr2 %08x sr1 %08x*\n",
	 i2c->state, i2c->count,
	 (unsigned int) I2C_DEV->SR2, (unsigned int) I2C_DEV->SR1);
  flushBuf();
 }
#endif
 switch(i2c->state)		/* dispatch on state */
 {
 case I_IDLE:			/* 0x00 idle state */
  break;

 case I_WAIT_START:		/* 0x01 wait to start */
  if ((I2C_DEV->SR2 & I2C_SR2_BUSY) == 0)
  {
   I2C_DEV->CR1 |= I2C_CR1_START;
   i2c->state = I_START;
   i2c->startTime = millis();
  }
  break;

 case I_START:			/* 0x02 wait for start and send address */
  if ((I2C_DEV->SR1 == I2C_SR1_SB)
  &&  ((I2C_DEV->SR2 & 0xff) == (I2C_SR2_MSL | I2C_SR2_BUSY)))
  {
   I2C_DEV->DR = slaveAddress;
   i2c->state = I_ADDRESS;
   i2c->startTime = millis();
  }
  break;

 case I_ADDRESS:		/* 0x03 wait for address and send data */
  if ((I2C_DEV->SR1 == (I2C_SR1_ADDR | I2C_SR1_TXE))
  &&  ((I2C_DEV->SR2 & 0xff) == (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA)))
  {
   i2c->state = I_SEND_DATA;
  }
  break;
  /* fall through to send data */

 case I_SEND_DATA:		/* 0x04 send data */
  if (i2c->count != 0)
  {
   --i2c->count;
   int emp = i2c->emp;
   I2C_DEV->DR = i2c->buffer[emp++];
   if (emp >= I2C_BUF_SIZE)
    emp = 0;
   i2c->emp = emp;
   i2c->startTime = millis();
   i2c->state = I_WAIT_DATA;
  }
  else
  {
   I2C_DEV->CR1 |= I2C_CR1_STOP;
   i2c->timeout = MAX_TIMEOUT;
   i2c->state = I_IDLE;
   i2c->status = IS_DONE;
  }
  break;

 case I_WAIT_DATA:		/* 0x05 wait for data to be send */
  if ((I2C_DEV->SR1 == (I2C_SR1_BTF | I2C_SR1_TXE))
  &&  ((I2C_DEV->SR2 & 0xff) == (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA)))
  {
   i2c->state = I_SEND_DATA;
  }
  break;
 }
}

#endif
