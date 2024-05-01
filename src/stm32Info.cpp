#define __STM32INFO__
#include <cstdio>

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#endif
#if defined(STM32F3)
#include "stm32f3xx_hal.h"
#endif
#if defined(STM32F4)
#include "stm32f4xx_hal.h"
#endif
#if defined(STM32F7)
#include "stm32f7xx_hal.h"
#endif
#if defined(STM32H7)
#include "stm32h7xx_hal.h"
#include "core_cm7.h"
#endif

#include "stm32Info.h"

#if defined(ARDUINO_ARCH_STM32)
#include "monitorSTM32.h"
#define flushBuf flush
#define newline newLine
#else
#include "config.h"
#include "serialio.h"

#endif	/* ARDUINO_ARCH_AVR */

#if  defined(__STM32INFO_INC__)	// <-

typedef struct
{
 union
 {
  struct
  {
   char port;
   char num;
  };
  struct
  {
   uint16_t pinName;
  };
 };
} T_PIN_NAME;

T_PIN_NAME pinName(char *buf, GPIO_TypeDef *port, int pin);
char portName(GPIO_TypeDef *port);
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
char *gpioStr(char *buf, int size, T_PIN_NAME *pinInfo);
#endif
void gpioInfo(GPIO_TypeDef *gpio);
void tmrInfo(TIM_TypeDef *tmr);
void tmrInfo(TIM_TypeDef *tmr, int flag);
void extiInfo();
void usartInfo(USART_TypeDef *usart, const char *str);
void i2cInfo(I2C_TypeDef *i2c, const char *str);
void spiInfo(SPI_TypeDef *spi, const char *str);
void rccInfo();
void pwrInfo();
void adcInfo(ADC_TypeDef *adc, char n);
void bkpInfo();
void afioInfo();
void rtcInfo();
#if defined(STM32F1)
void dmaInfo(DMA_TypeDef *dma);
void dmaChannelInfo(DMA_Channel_TypeDef *dmaC, char n);
#endif
#if defined(STM32F4)
void dmaInfo(DMA_TypeDef *dma);
void dmaStreamInfo(DMA_Channel_TypeDef *dmaS);
#endif

enum tmrFlags
{T_CR1, T_CR2, T_SMCR, T_DIER, T_SR, T_EGR,
 T_CCMR1, T_CCMR2, T_CCER, T_CNT, T_PSC, T_ARR, T_RCR,
 T_CCR1, T_CCR2, T_CCR3, T_CCR4, T_BDTR, T_DCR, T_DMAR,
 T_RESERVED1, T_CCMR3, T_CCR5, T_CCR6, T_AF1, T_AF2, T_TISEL
};

#define T_MASK(x) (1 << T_##x)
#define T_CHECK(flag, x) (flag & (1 << T_##x))
#define T_NL() {n += 1; if ((n & 1) == 0) printf("\n");}
#define T_PRT(t, str) printf("%-6s %8lx ", #str, t->str)

#define TIM15_MASK \
 (T_MASK(CR1) \
  | T_MASK(CR2) \
  | T_MASK(DIER) \
  | T_MASK(SR) \
  | T_MASK(CCMR1) \
  | T_MASK(CCER) \
  | T_MASK(CNT) \
  | T_MASK(PSC) \
  | T_MASK(ARR) \
  | T_MASK(CCR1) \
  )

#define TIM17_MASK \
 (T_MASK(CR1) \
  | T_MASK(CR2) \
  | T_MASK(DIER) \
  | T_MASK(SR) \
  | T_MASK(CCMR1) \
  | T_MASK(CCER) \
  | T_MASK(CNT) \
  | T_MASK(PSC) \
  | T_MASK(ARR) \
  | T_MASK(CCR1) \
  )

void info();
//void bitState(const char *s, volatile unsigned long *p, unsigned long mask);

#if defined(ARDUINO_ARCH_STM32)
char query(unsigned char (*get)(), const char *format, ...);
#endif	/* ARDUINO_ARCH_STM32 */

#endif	// ->
#ifdef __STM32INFO__

uint32_t lastFlags;

typedef struct
{
 GPIO_TypeDef *port;
 char name;
} T_GPIO, *P_GPIO;

T_GPIO gpio[] =
{
 {GPIOA, 'A'},
 {GPIOB, 'B'},
 {GPIOC, 'C'},
#ifdef GPIOD
 {GPIOD, 'D'},
#endif
#ifdef GPIOE
 {GPIOE, 'E'},
#endif
#ifdef GPIOF
 {GPIOF, 'F'},
#endif
#ifdef GPIOG
 {GPIOG, 'G'},
#endif
};

char portName(GPIO_TypeDef *port)
{
 for (auto & j : gpio)
 {
  if (port == j.port)
  {
   return(j.name);
  }
 }
 return('*');
}

T_PIN_NAME pinName(char *buf, GPIO_TypeDef *port, int pin)
{
 char pName = portName(port);
 T_PIN_NAME val;
 val.port = pName;
 int pinNum = 0;
 while (pin != 0)
 {
  if (pin & 1)
   break;
  pin >>= 1;
  pinNum++;
 }
 sprintf(buf, "P%c%d", pName, pinNum);
 val.num = pinNum;
 return(val);
}

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)

char modeInfo[] = {'I', 'O', 'F', 'A'};
const char *typeInfo[] = {"PP", "OD", "  "};
const char *speedInfo[] = {"LS", "MS", "HS", "VH", "  "};
const char *pupdInfo[] = {"  ", "PU", "PD", "**"};

char *gpioStr(char *buf, int size, T_PIN_NAME *pinInfo)
{
 buf[0] = 0;
 GPIO_TypeDef *port;
 for (auto & j : gpio)
 {
  if (j.name == pinInfo->port)
  {
   port = j.port;
   int pin = pinInfo->num;
//   printf("port  %08x %2d %c %2d\n", (unsigned int) port, pin,
//	  pinInfo->port, pinInfo->num);

   unsigned int mode = (port->MODER >> (pin << 1)) & 3;
//   printf("mode  %08x %d\n", (unsigned int) port->MODER, mode);

   unsigned int outType = (port->OTYPER >> pin) & 1;
//   printf("type  %08x %d\n", (unsigned int) port->OTYPER, outType);

   unsigned int outSpeed = (port->OSPEEDR >> (pin << 1)) & 3;
//   printf("speed %08x %d\n", (unsigned int) port->OSPEEDR, outSpeed);

   unsigned int pupd = (port->PUPDR >> (pin << 1)) & 3;
//   printf("pupd  %08x %d\n", (unsigned int) port->PUPDR, pupd);

   unsigned int afr = (port->AFR[pin >> 3] >> ((pin << 2) & 0x1f)) & 0xf;

   char interrupt = ' ';
   char nvic = ' ';
   if (mode == GPIO_MODE_INPUT)
   {
    outType = (sizeof(typeInfo) / sizeof(char *)) - 1;
    outSpeed = (sizeof(speedInfo) / sizeof(char *)) - 1;

    if (EXTI->IMR & (1 << pin))
    {
     unsigned int exti =
         (SYSCFG->EXTICR[pin >> 2] >> ((pin << 2) & 0xf)) & 0xf;
     if ((unsigned int) (pinInfo->port - 'A') == exti)
     {
      interrupt = 'I';

      if (pin <= 4) {
       if (NVIC_GetEnableIRQ((IRQn_Type) (EXTI0_IRQn + pin)))
        nvic = '*';
      } else if (pin <= 9) {
       if (NVIC_GetEnableIRQ((IRQn_Type) (EXTI9_5_IRQn)))
        nvic = '*';
      } else if (pin <= 15) {
       if (NVIC_GetEnableIRQ((IRQn_Type) (EXTI15_10_IRQn)))
        nvic = '*';
      }

//     printf("exti %2d pinInfo->port - 'A' %d pin >> 2 %d pin << 2 %d\n",
//	    exti, pinInfo->port - 'A', pin >> 2, pin << 2);
     }
    }
   }

//   printf("afr   %08x %d (pin >> 3) %d ((pin << 2) & 0x1f) %2d\n",
//	  (unsigned int) port->AFR[pin >> 3], afr,
//	  (pin >> 3), ((pin << 2) & 0x1f));
//   flushBuf();

   snprintf(buf, size, "%c%c %c %2s %2s %2s %2d",
	    interrupt, nvic, modeInfo[mode], typeInfo[outType],
	    speedInfo[outSpeed], pupdInfo[pupd], afr);
   break;
  }
 }
 return(buf);
}

#endif

void gpioInfo(GPIO_TypeDef *gpioPtr)
{
 printf("gpio %x %c\n", (unsigned int) gpioPtr, portName(gpioPtr));
//#if defined(STM32F3) || defined(STM32F4) || defined(STM32H7)
// printf("MODER   %8lx ", gpio->MODER);
// printf("OTYPER  %8lx\n", gpio->OTYPER);
// printf("OSPEEDR %8lx ", gpio->OSPEEDR);
// printf("PUPDR   %8lx\n", gpio->PUPDR);
//#endif	/* STM32F3 */
#if defined(STM32F1)
 printf("CRL     %8lx ", gpioPtr->CRL);
 printf("CRH     %8lx\n", gpioPtr->CRH);
#endif	/* STM32F1 */
// printf("IDR     %8lx ", gpio->IDR);
// printf("ODR     %8lx\n", gpio->ODR);
// printf("BSRR    %8lx ", gpio->BSRR);
// printf("LCKR    %8lx\n", gpio->LCKR);
//#if defined(STM32F3) || defined(STM32F4) || defined(STM32H7)
// printf("AFR[0]  %8lx ", gpio->AFR[0]);
// printf("AFR[1]  %8lx\n", gpio->AFR[1]);
//#endif	/* STM32F3 */
 int i;
 printf("         ");
 for (i = 0; i < 16; i++)
  printf(" %2d", i);

 uint32_t val;

#if defined(STM32F3) || defined(STM32F4) || defined(STM32H7)
 printf("\nmoder    ");
 val = gpioPtr->MODER;
 for (i = 0; i < 16; i++)
  printf(" %2lu", (val >> (2 * i)) & 0x3);

 printf("\notyper   ");
 val = gpioPtr->OTYPER;
 for (i = 0; i < 16; i++)
  printf(" %2lu", (val >> i) & 0x1);

 printf("\nopspeedr ");
 val = gpioPtr->OSPEEDR;
 for (i = 0; i < 16; i++)
  printf(" %2lu", (val >> (2 * i)) & 0x3);

 printf("\npupdr    ");
 val = gpioPtr->PUPDR;
 for (i = 0; i < 16; i++)
  printf(" %2lu", (val >> (2 * i)) & 0x3);
#endif	/* STM32F3 */

#if defined(STM32F1)
 printf("\n");
 printf("mode     ");
 val = gpioPtr->CRL;
 for (i = 0; i < 8; i++)
  printf(" %2d", (int) (val >> (4 * i)) & 0x3);

 val = gpioPtr->CRH;
 for (i = 0; i < 8; i++)
  printf(" %2d", (int) (val >> (4 * i)) & 0x3);

 printf("\n");
 printf("cnf      ");
 val = gpioPtr->CRL;
 for (i = 0; i < 8; i++)
  printf(" %2d", (int) (val >> ((4 * i) + 2)) & 0x3);

 val = gpioPtr->CRH;
 for (i = 0; i < 8; i++)
  printf(" %2d", (int) (val >> ((4 * i) + 2)) & 0x3);
#endif	/* STM32F1 */

 printf("\nidr      ");
 val = gpioPtr->IDR;
 for (i = 0; i < 16; i++)
  printf(" %2lu", (val >> i) & 0x1);

 printf("\nodr      ");
 val = gpioPtr->ODR;
 for (i = 0; i < 16; i++)
  printf(" %2lu", (val >> i) & 0x1);

 printf("\nbsrr     ");
 val = gpioPtr->BSRR;
 for (i = 0; i < 16; i++)
  printf(" %2lu", (val >> i) & 0x1);

 printf("\nlckr     ");
 val = gpioPtr->LCKR;
 for (i = 0; i < 16; i++)
  printf(" %2lu", (val >> i) & 0x1);

#if defined(STM32F3) || defined(STM32F4) || defined(STM32H7)
 printf("\nafr      ");
 val = gpioPtr->AFR[0];
 for (i = 0; i < 8; i++)
  printf(" %2lu", (val >> (4 * i)) & 0xf);
 val = gpioPtr->AFR[1];
 for (i = 0; i < 8; i++)
  printf(" %2lu", (val >> (4 * i)) & 0xf);
#endif	/* STM32F3 */
 printf("\n");
 flushBuf();
}

#if 0
void gpioInfo(GPIO_TypeDef *gpio)
{
 printf("gpio %x %c\n",(unsigned int) gpio, portName(gpio));
#if defined(STM32F4)
 printf("MODER   %8lx ",gpio->MODER);
 printf("OTYPER  %8lx\n",gpio->OTYPER);
 printf("OSPEEDR %8lx ",gpio->OSPEEDR);
 printf("PUPDR   %8lx\n",gpio->PUPDR);
 printf("IDR     %8lx ",gpio->IDR);
 printf("ODR     %8lx\n",gpio->ODR);
 printf("BSRR    %8lx ",gpio->BSRR);
 printf("LCKR    %8lx\n",gpio->LCKR);
 printf("AFR[0]  %8lx ",gpio->AFR[0]);
 printf("AFR[1]  %8lx\n",gpio->AFR[1]);
 int i;
 printf("         ");
 for (i = 0; i < 16; i++)
  printf(" %2d", i);

 printf("\nmoder    ");
 int val = gpio->MODER;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> (2 * i)) & 0x3);

 printf("\notyper   ");
 val = gpio->OTYPER;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\nopspeedr ");
 val = gpio->OSPEEDR;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> (2 * i)) & 0x3);

 printf("\npupdr    ");
 val = gpio->PUPDR;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> (2 * i)) & 0x3);

 printf("\nidr      ");
 val = gpio->IDR;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\nodr      ");
 val = gpio->ODR;
 for (i = 0; i < 16; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\nafr      ");
 val = gpio->AFR[0];
 for (i = 0; i < 8; i++)
  printf(" %2d", (val >> (4 * i)) & 0xf);
 val = gpio->AFR[1];
 for (i = 0; i < 8; i++)
  printf(" %2d", (val >> (4 * i)) & 0xf);
#endif
 printf("\n");
 flushBuf();
}
#endif	/* 0 */

typedef struct
{
 TIM_TypeDef *tmr;
 char num;
} T_TIM, *P_TIM;

T_TIM tim[] =
{
 {TIM1,  1},
 {TIM2,  2},
 {TIM3,  3},
 {TIM4,  4},
#ifdef TIM5
 {TIM5,  5},
#endif
#ifdef TIM6
 {TIM6,  6},
#endif
#ifdef TIM7
 {TIM7,  7},
#endif
#ifdef TIM8
 {TIM8,  8},
#endif
#ifdef TIM9
 {TIM9,  9},
#endif
#ifdef TIM10
 {TIM10, 10},
#endif
#ifdef TIM11
 {TIM11, 11},
#endif
#ifdef TIM12
 {TIM12, 12},
#endif
#ifdef TIM13
 {TIM13, 13},
#endif
#ifdef TIM14
 {TIM14, 14},
#endif
#ifdef TIM15
 {TIM15, 15},
#endif
#ifdef TIM16
 {TIM16, 16},
#endif
#ifdef TIM17
 {TIM17, 17},
#endif
};

char timNum(TIM_TypeDef *tmr);

char timNum(TIM_TypeDef *tmr)
{
 for (auto & j : tim)
 {
  if (tmr == j.tmr)
  {
   return(j.num);
  }
 }
 return(0);
}

void tmrInfo(TIM_TypeDef *tmr)
{
 printf("tmr %x TIM%d\n", (unsigned int) tmr, timNum(tmr));
 printf("CR1   %8lx ", tmr->CR1);
 printf("CR2   %8lx\n",tmr->CR2);
 printf("SMCR  %8lx ",tmr->SMCR);
 printf("DIER  %8lx\n",tmr->DIER);
 printf("SR    %8lx ",tmr->SR);
 printf("EGR   %8lx\n",tmr->EGR);
 printf("CCMR1 %8lx ",tmr->CCMR1);
 printf("CCMR2 %8lx\n",tmr->CCMR2);
 printf("CCER  %8lx ",tmr->CCER);
 printf("CNT   %8lx\n",tmr->CNT);
 printf("PSC   %8lx ",tmr->PSC);
 printf("ARR   %8lx\n",tmr->ARR);
 printf("RCR   %8lx ",tmr->RCR);
 printf("CCR1  %8lx\n",tmr->CCR1);
 printf("CCR2  %8lx ",tmr->CCR2);
 printf("CCR3  %8lx\n",tmr->CCR3);
 printf("CCR4  %8lx ",tmr->CCR4);
 printf("BDTR  %8lx\n",tmr->BDTR);
 printf("DCR   %8lx ",tmr->DCR);
#if defined(__STM32F4xx_HAL_H) || defined(__STM32F7xx_HAL_H)
 printf("OR    %8lx\n",tmr->OR);
#endif
#if defined(STM32F1) ||  defined(STM32H7)
 newline();
#endif
 flushBuf();
}

void tmrInfo(TIM_TypeDef *tmr, int tmrFlag)
{
 int n = 0;
 printf("tmr %x TIM%d\n", (unsigned int) tmr, timNum(tmr));
 if (T_CHECK(tmrFlag, CR1)) {
  T_PRT(tmr, CR1);
  T_NL()}
 if (T_CHECK(tmrFlag, CR2)) {
  T_PRT(tmr, CR2);
  T_NL()}
 if (T_CHECK(tmrFlag, SMCR)) {
  T_PRT(tmr, SMCR);
  T_NL()}
 if (T_CHECK(tmrFlag, DIER)) {
  T_PRT(tmr, DIER);
  T_NL()}
 if (T_CHECK(tmrFlag, SR)) {
  T_PRT(tmr, SR);
  T_NL()}
 if (T_CHECK(tmrFlag, EGR)) {
  T_PRT(tmr, EGR);
  T_NL()}
 if (T_CHECK(tmrFlag, CCMR1)) {
  T_PRT(tmr, CCMR1);
  T_NL()}
 if (T_CHECK(tmrFlag, CCMR2)) {
  T_PRT(tmr, CCMR2);
  T_NL()}
 if (T_CHECK(tmrFlag, CCER)) {
  T_PRT(tmr, CCER);
  T_NL()}
 if (T_CHECK(tmrFlag, CNT)) {
  T_PRT(tmr, CNT);
  T_NL()}
 if (T_CHECK(tmrFlag, PSC)) {
  T_PRT(tmr, PSC);
  T_NL()}
 if (T_CHECK(tmrFlag, ARR)) {
  T_PRT(tmr, ARR);
  T_NL()}
 if (T_CHECK(tmrFlag, RCR)) {
  T_PRT(tmr, RCR);
  T_NL()}
 if (T_CHECK(tmrFlag, CCR1)) {
  T_PRT(tmr, CCR1);
  T_NL()}
 if (T_CHECK(tmrFlag, CCR2)) {
  T_PRT(tmr, CCR2);
  T_NL()}
 if (T_CHECK(tmrFlag, CCR3)) {
  T_PRT(tmr, CCR3);
  T_NL()}
 if (T_CHECK(tmrFlag, CCR4)) {
  T_PRT(tmr, CCR4);
  T_NL()}
 if (T_CHECK(tmrFlag, BDTR)) {
  T_PRT(tmr, BDTR);
  T_NL()}
 if (T_CHECK(tmrFlag, DCR)) {
  T_PRT(tmr, DCR);
  T_NL()}
 printf("\n");
 flushBuf();
}

 void extiBit(const char *label, uint32_t val)
 {
 printf("\n%6s", label);
 for (int i = 0; i <= 22; i++)
  printf(" %2d", (int) ((val >> i) & 0x1));
}

void extiInfo()
{
 printf("EXTI %x\n", (unsigned int) EXTI);
 int i;
 printf("      ");
 for (i = 0; i <= 22; i++)
  printf(" %2d", i);

 printf("\n");
 printf("IMR   ");
 int val = EXTI->IMR;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

#if defined(STM32F4)
 printf("\n");
 printf("EMR   ");
 val = EXTI->EMR;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\n");
 printf("RTSR  ");
 val = EXTI->RTSR;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\n");
 printf("FTSR  ");
 val = EXTI->FTSR;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);
 #endif	 /* STM32F4 */

 printf("\n");
 printf("SWIER ");
 val = EXTI->SWIER;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

 printf("\n");
 printf("PR    ");
 val = EXTI->PR;
 for (i = 0; i <= 22; i++)
  printf(" %2d", (val >> i) & 0x1);

#if defined(__STM32F4xx_HAL_H) || defined(__STM32F7xx_HAL_H)
 printf("\nSYSCFG %x\n", (unsigned int) SYSCFG);
 printf("      ");
 for (i = 0; i < 16; i++)
  printf(" %2d", i);

 printf("\nEXTICR");
 int mask = EXTI->IMR;
 for (i = 0; i < 4; i++)
 {
  val = SYSCFG->EXTICR[i];
  int j;
  for (j = 0; j < 4; j++)
  {
   printf("  %c", (mask & 1) ? 'a' + ((val >> (4 * j)) & 0xf) : ' ');
   mask >>= 1;
  }
 }
#endif

#if defined(STM32H743xx_H)
 extiBit("RTSR1", EXTI->RTSR1);
 extiBit("FTSR1", EXTI->FTSR1);
 extiBit("SWIER1", EXTI->SWIER1);
 extiBit("D3PMR1", EXTI->D3PMR1);
 extiBit("IMR1", EXTI->IMR1);
 extiBit("EMR1", EXTI->EMR1);
 extiBit("PR1", EXTI->PR1);
#endif

 printf("\n");
 flushBuf();
}

void usartInfo(USART_TypeDef *usart, const char *str)
{
 printf("usart %x %s\n",(unsigned int) usart, str);
#ifdef STM32F4
 printf("SR   %8lx ",usart->SR);
 printf("DR   %8lx\n",usart->DR);
#endif
#ifdef STM32F7
 printf("ISR  %8lx ",usart->ISR);
 printf("RDR  %8lx\n",usart->RDR);
#endif
 printf("BRR  %8lx ",usart->BRR);
 printf("CR1  %8lx\n",usart->CR1);
 printf("CR2  %8lx ",usart->CR2);
 printf("CR3  %8lx\n",usart->CR3);
 printf("GTPR %8lx\n",usart->GTPR);
 flushBuf();
}

#if 0
void i2cInfo(I2C_TypeDef *i2c, const char *str)
{
 printf("I2C %08x %s\n", (unsigned int) i2c, str);
 printf("CR1   %8lx ", i2c->CR1);
 printf("CR2   %8lx\n", i2c->CR2);
 printf("OAR1  %8lx ", i2c->OAR1);
 printf("OAR2  %8lx\n", i2c->OAR2);
 printf("SR1   %8lx ", i2c->SR1);
 printf("SR2   %8lx\n", i2c->SR2);
 printf("DR    %8lx ", i2c->DR);
 printf("CCR   %8lx\n", i2c->CCR);
 printf("TRISE %8lx\n", i2c->TRISE);
 flushBuf();
}
#endif

void i2cInfo(I2C_TypeDef *i2c, const char *str)
{
 printf("i2c %x %s\n",(unsigned int) i2c, str);
 printf("CR1      %8lx ",  i2c->CR1);
 printf("CR2      %8lx\n", i2c->CR2);
 printf("OAR1     %8lx ",  i2c->OAR1);
 printf("OAR2     %8lx\n", i2c->OAR2);
#if defined(STM32F1) || defined(STM32F3) || defined(STM32F4)
 printf("SR1      %8lx ",  i2c->SR1);
 printf("SR2      %8lx\n", i2c->SR2);
 printf("DR       %8lx ",  i2c->DR);
 printf("CCR      %8lx\n", i2c->CCR);
 printf("TRISE    %8lx\n", i2c->TRISE);
#endif
#ifdef STM32H7
 printf("TIMINGR  %8lx ",  i2c->TIMINGR);
 printf("TIMEOUTR %8lx\n", i2c->TIMEOUTR);
 printf("ISR      %8lx ",  i2c->ISR);
 printf("ICR      %8lx\n", i2c->ICR);
 printf("PECR     %8lx\n", i2c->PECR);
 printf("RXDR     %8lx ",  i2c->RXDR);
 printf("TXDR     %8lx\n", i2c->TXDR);
#endif
 flushBuf();
}

void spiInfo(SPI_TypeDef *spi, const char *str)
{
 printf("spi %x %s\n", (unsigned int) spi, str);
 printf("CR1      %8lx ",  spi->CR1);
 printf("CR2      %8lx\n", spi->CR2);
 printf("SR       %8lx\n", spi->SR);
}

void rccInfo()
{
#if defined(STM32F1)
 printf("RCC %08x\n", (unsigned int) RCC);
 printf("CR       %8lx ",  RCC->CR);
 printf("CFGR     %8lx\n", RCC->CFGR);
 printf("APB2RSTR %8lx ",  RCC->APB2RSTR);
 printf("APB1RSTR %8lx\n", RCC->APB1RSTR);
 printf("APB2ENR  %8lx ",  RCC->APB2ENR);
 printf("APB1ENR  %8lx\n", RCC->APB1ENR);
 printf("CIR      %8lx ",  RCC->CIR);
 printf("AHBENR   %8lx\n", RCC->AHBENR);
 printf("BDCR     %8lx ",  RCC->BDCR);
 printf("CSR      %8lx\n", RCC->CSR);
#endif
#ifdef STM32F4
 printf("CR         %8lx ",  RCC->CR);
 printf("PLLCFGR    %8lx\n", RCC->PLLCFGR);

 printf("CFGR       %8lx ",  RCC->CFGR);
 printf("CIR        %8lx\n", RCC->CIR);

 printf("AHB1RSTR   %8lx ",  RCC->AHB1RSTR);
 printf("AHB2RSTR   %8lx ",  RCC->AHB2RSTR);
 printf("AHB3RSTR   %8lx\n", RCC->AHB3RSTR);

 printf("APB1RSTR   %8lx ",  RCC->APB1RSTR);
 printf("APB2RSTR   %8lx\n", RCC->APB2RSTR);

 printf("AHB1ENR    %8lx ",  RCC->AHB1RSTR);
 printf("AHB2ENR    %8lx ",  RCC->AHB1RSTR);
 printf("AHB3ENR    %8lx\n", RCC->AHB1RSTR);

 printf("APB1ENR    %8lx ",  RCC->APB1ENR);
 printf("APB2ENR    %8lx\n", RCC->APB2ENR);

 printf("AHB1LPENR  %8lx ",  RCC->AHB1LPENR);
 printf("AHB2LPENR  %8lx ",  RCC->AHB2LPENR);
 printf("AHB3LPENR  %8lx\n", RCC->AHB3LPENR);

 printf("APB1LPENR  %8lx ",  RCC->APB1LPENR);
 printf("APB2LPENR  %8lx\n", RCC->APB2LPENR);

 printf("BDCR       %8lx ",  RCC->BDCR);
 printf("CSR        %8lx\n", RCC->CSR);

 printf("SSCGR      %8lx ",  RCC->BDCR);
 printf("PLLI2SCFGR %8lx\n", RCC->CSR);
#endif
#ifdef STM32H7
 printf("CR         %8lx ",  RCC->CR);
 printf("HSICFGR    %8lx\n", RCC->HSICFGR);

 printf("CRRCR      %8lx ",  RCC->CRRCR);
 printf("CSICFGR    %8lx ",  RCC->CSICFGR);
 printf("CFGR       %8lx\n", RCC->CFGR);

 printf("D1CFGR     %8lx ",  RCC->D1CFGR);
 printf("D2CFGR     %8lx ",  RCC->D2CFGR);
 printf("D3CFGR     %8lx\n", RCC->D3CFGR);

 printf("PLLCKSELR  %8lx ",  RCC->PLLCKSELR);
 printf("PLLCFGR    %8lx\n", RCC->PLLCFGR);

 printf("PLL1DIVR   %8lx ",  RCC->PLL1DIVR);
 printf("PLL1FRACR  %8lx\n", RCC->PLL1FRACR);

 printf("PLL2DIVR   %8lx ",  RCC->PLL2DIVR);
 printf("PLL2FRACR  %8lx\n", RCC->PLL2FRACR);

 printf("PLL3DIVR   %8lx ",  RCC->PLL3DIVR);
 printf("PLL3FRACR  %8lx\n", RCC->PLL3FRACR);

 printf("D1CCIPR    %8lx ",  RCC->D1CCIPR);
 printf("D2CCIP1R   %8lx ",  RCC->D2CCIP1R);
 printf("D2CCIP2R   %8lx ",  RCC->D2CCIP2R);
 printf("D3CCIPR    %8lx\n", RCC->D3CCIPR);

 printf("CIER       %8lx ",  RCC->CIER);
 printf("CIFR       %8lx\n", RCC->CIFR);

 printf("CICR       %8lx ",  RCC->CICR);
 printf("BDCR       %8lx ",  RCC->BDCR);
 printf("CSR        %8lx\n", RCC->CSR);

 printf("AHB3RSTR   %8lx ",  RCC->AHB3RSTR);
 printf("AHB1RSTR   %8lx ",  RCC->AHB1RSTR);
 printf("AHB2RSTR   %8lx\n", RCC->AHB2RSTR);
 printf("AHB4RSTR   %8lx ",  RCC->AHB4RSTR);
 printf("APB3RSTR   %8lx\n", RCC->APB3RSTR);

 printf("APB1LRSTR  %8lx ",  RCC->APB1LRSTR);
 printf("APB1HRSTR  %8lx ",  RCC->APB1HRSTR);
 printf("APB2RSTR   %8lx ",  RCC->APB2RSTR);
 printf("APB4RSTR   %8lx\n", RCC->APB4RSTR);

 printf("GCR        %8lx ",  RCC->GCR);
 printf("D3AMR      %8lx ",  RCC->D3AMR);
 printf("RSR        %8lx\n", RCC->RSR);

 printf("AHB3ENR    %8lx ",  RCC->AHB3ENR);
 printf("AHB1ENR    %8lx ",  RCC->AHB1ENR);
 printf("AHB2ENR    %8lx ",  RCC->AHB2ENR);
 printf("AHB4ENR    %8lx\n", RCC->AHB4ENR);

 printf("APB3ENR    %8lx ",  RCC->APB3ENR);
 printf("APB1LENR   %8lx ",  RCC->APB1LENR);
 printf("APB1HENR   %8lx\n", RCC->APB1HENR);
 printf("APB2ENR    %8lx ",  RCC->APB2ENR);
 printf("APB4ENR    %8lx\n", RCC->APB4ENR);

 printf("AHB3LPENR  %8lx ",  RCC->AHB3LPENR);
 printf("AHB1LPENR  %8lx ",  RCC->AHB1LPENR);
 printf("AHB2LPENR  %8lx ",  RCC->AHB2LPENR);
 printf("AHB4LPENR  %8lx ",  RCC->AHB4LPENR);

 printf("APB3LPENR  %8lx ",  RCC->APB3LPENR);
 printf("APB1LLPENR %8lx ",  RCC->APB1LLPENR);
 printf("APB1HLPENR %8lx\n", RCC->APB1HLPENR);
 printf("APB2LPENR  %8lx ",  RCC->APB2LPENR);
 printf("APB4LPENR  %8lx\n", RCC->APB4LPENR);
 #endif
}

#if 0
void rccInfo()
{
 printf("RCC %08x\n", (unsigned int) RCC);
 printf("CR       %8lx ",  RCC->CR);
 printf("CFGR     %8lx\n", RCC->CFGR);
 printf("APB2RSTR %8lx ",  RCC->APB2RSTR);
 printf("APB1RSTR %8lx\n", RCC->APB1RSTR);
 printf("APB2ENR  %8lx ",  RCC->APB2ENR);
 printf("APB1ENR  %8lx\n", RCC->APB1ENR);
 printf("CIR      %8lx ",  RCC->CIR);
 printf("AHBENR   %8lx\n", RCC->AHBENR);
 printf("BDCR     %8lx ",  RCC->BDCR);
 printf("CSR      %8lx\n", RCC->CSR);
}
#endif

void adcInfo(ADC_TypeDef *adc, char n)
{
 printf("ADC%d %08x  DR %08x\n",
	n, (unsigned int) adc, (unsigned int) &adc->DR);
#if defined(STM32F1)
 printf("SR    %8lx\n", adc->SR);
 printf("CR1   %8lx ", adc->CR1);
 printf("CR2   %8lx\n", adc->CR2);
 printf("HTR   %8lx ", adc->HTR);
 printf("LTR   %8lx\n", adc->LTR);
 printf("L     %8lx ", ((adc->SQR1 >> 20) & 0xf));
 printf("DR    %8lx\n", adc->DR);
 int i;
 printf("     ");
 for (i = 0; i < 16; i++)
  printf(" %2d", i);
 printf("\n");

 printf("SMPR ");
 int32_t tmp = adc->SMPR2;
 for (i = 0; i < 10; i++)
 {
  printf(" %2u", (unsigned int) (tmp & 7));
  tmp >>= 3;
 }
 tmp = adc->SMPR1;
 for (i = 0; i < 6; i++)
 {
  printf(" %2u", (unsigned int) (tmp & 7));
  tmp >>= 3;
 }
 printf("\n");

 printf("SQR  ");
 tmp = adc->SQR3;
 for (i = 0; i < 6; i++)
 {
  printf(" %2u", (unsigned int) (tmp & 7));
  tmp >>= 5;
 }
 tmp = adc->SQR2;
 for (i = 0; i < 6; i++)
 {
  printf(" %2u", (unsigned int) (tmp & 7));
  tmp >>= 5;
 }
 tmp = adc->SQR1;
 for (i = 0; i < 4; i++)
 {
  printf(" %2u", (unsigned int) (tmp & 7));
  tmp >>= 5;
 }
 printf("\n");
#endif	/* STM32F1 */
#if defined(STM32F3)
 printf("ISR   %8lx ", adc->ISR);
 printf("IER   %8lx\n", adc->IER);
 printf("CR    %8lx ", adc->CR);
 printf("CFGR  %8lx\n", adc->CFGR);
 printf("CAL   %8lx ", adc->CALFACT);
 printf("DR    %8lx\n", adc->DR);
 int i;
 printf("     ");
 for (i = 0; i < 16; i++)
  printf(" %2d", i);
 printf("\n");

 printf("SMPR ");
 int32_t tmp = adc->SMPR1;
 for (i = 0; i < 10; i++)
 {
  printf(" %2u", (unsigned char *) (tmp & 7));
  tmp >>= 3;
 }
 tmp = adc->SMPR2;
 for (i = 0; i < 9; i++)
 {
  printf(" %2u", (unsigned char *) (tmp & 7));
  tmp >>= 3;
 }
 printf("\n");

 printf("SQR  ");
 tmp = adc->SQR1;
 for (i = 0; i < 6; i++)
 {
  printf(" %2u", (unsigned char *) (tmp & 0xf));
  tmp >>= 6;
 }
 tmp = adc->SQR2;
 for (i = 0; i < 6; i++)
 {
  printf(" %2u", (unsigned char *) (tmp & 0x4));
  tmp >>= 6;
 }
 tmp = adc->SQR3;
 for (i = 0; i < 6; i++)
 {
  printf(" %2u", (unsigned char *) (tmp & 0xf));
  tmp >>= 6;
 }
 tmp = adc->SQR4;
 for (i = 0; i < 2; i++)
 {
  printf(" %2u", (unsigned char *) (tmp & 0xf));
  tmp >>= 6;
 }
 printf("\n");
#endif	/* STM32F3 */
 flushBuf();
}

#if defined(SMT32F1)
void dmaInfo(DMA_TypeDef *dma)
{
 printf("DMA1 %08x\n", (unsigned int) dma);
 printf("ISR   %8lx ", dma->ISR);
 printf("IFCR  %8lx\n", dma->IFCR);
 flushBuf();
}

void dmaChannelInfo(DMA_Channel_TypeDef *dmaC, char n)
{
 printf("DMA_Channel%d %08x\n", n, (unsigned int) dmaC);
 printf("CCR   %8lx ", dmaC->CCR);
 printf("CNDTR %8lx\n", dmaC->CNDTR);
 printf("CPAR  %8lx ", dmaC->CPAR);
 printf("CMAR  %8lx\n", dmaC->CMAR);
 flushBuf();
}
#endif

#if defined(STM32F4)
void dmaInfo(DMA_TypeDef *dma)
{
 printf("DMA   %08x\n", (unsigned int) dma);
 printf("LISR  %8lx ",  dma->LISR);
 printf("HISR  %8lx\n", dma->HISR);
 printf("LIFCR %8lx",   dma->LIFCR);
 printf("HIFCR %8lx\n", dma->HIFCR);
 flushBuf();
}

void dmaStreamInfo(DMA_StreamTypeDef *dmaS)
{
 printf("DMA_Stream %08x\n", (unsigned int) dmaS);
 printf("CR   %8lx ",   dmaS->CR);
 printf("NDTR %8lx\n",  dmaS->)NDTR;
 printf("PAR  %8lx ",   dmaS->PAR);
 printf("M0AR %8lx\n ", dmaS->M0AR);
 printf("M1AR %8lx ",   dmaS->M1AR);
 printf("FCR  %8lx\n ", dmaS->FCR);
}
#endif

void afioInfo()
{
#if defined(STM32F1)
 printf("AFIO %x\n", (unsigned int) AFIO);
 printf("EVCR      %8lx ",  AFIO->EVCR);
 printf("MAPR      %8lx\n", AFIO->MAPR);
 printf("EXTICR[0] %8lx ",  AFIO->EXTICR[0]);
 printf("EXTICR[1] %8lx\n", AFIO->EXTICR[1]);
 printf("EXTICR[2] %8lx ",  AFIO->EXTICR[2]);
 printf("EXTICR[3] %8lx\n", AFIO->EXTICR[3]);
 printf("MAPR2     %8lx\n", AFIO->MAPR2);
#endif
}

void bkpInfo()
{
#if defined(STM32F1)
 printf("BKP %x\n", (unsigned int) BKP);
 printf("RTCCR     %8lx ",  BKP->RTCCR);
 printf("CR        %8lx\n", BKP->CR);
 printf("CSR       %8lx\n", BKP->CSR);
#endif
}

void rtcInfo()
{
#if defined(STM32F1)
 printf("RTC %x\n", (unsigned int) RTC);
 printf("CRH       %8lx ",  RTC->CRH);
 printf("CRL       %8lx\n", RTC->CRL);
 printf("PRLH      %8lx ",  RTC->PRLH);
 printf("PRLL      %8lx\n", RTC->PRLL);
 printf("DIVH      %8lx ",  RTC->DIVH);
 printf("DIVL      %8lx\n", RTC->DIVL);
 printf("CNTH      %8lx ",  RTC->CNTH);
 printf("CNTL      %8lx\n", RTC->CNTL);
 printf("ALRH      %8lx ",  RTC->ALRH);
 printf("ALRL      %8lx\n", RTC->ALRL);
#endif
}

void pwrInfo()
{
 printf("PWR %x\n", (unsigned int) PWR);
#if defined(STM32F1) || defined(STM32F4)
 printf("CR        %8lx ",  PWR->CR);
 printf("CSR       %8lx\n", PWR->CSR);
#endif /* STM32F1 || STM32F4 */
#if defined(STM32H7)
 printf("CR1        %8lx ",  PWR->CR1);
 printf("CSR1       %8lx\n", PWR->CSR1);
 printf("CR2        %8lx ",  PWR->CR2);
 printf("CR3        %8lx\n", PWR->CR3);
 printf("CPUCR2     %8lx ",  PWR->CPUCR);
 printf("D3CR       %8lx\n", PWR->D3CR);
 printf("WKUPCR     %8lx ",  PWR->WKUPCR);
 printf("WKUPFR     %8lx\n", PWR->WKUPFR);
 printf("WKUPEPR    %8lx\n", PWR->WKUPEPR);
#endif /* STM32F7 */
}

#if defined(ARDUINO_ARCH_STM32)

extern unsigned char getNum();
extern int numVal;

char query(unsigned char (*get)(), const char *format, ...)
{
 va_list args;
 va_start(args, format);
 vprintf(format, args);
 va_end(args);
 flushBuf();
 char ch = get();
 newLine();
 return(ch);
}

#endif	/* ARDUINO_ARCH_STM32 */

void info()
{
#if defined(ARDUINO_ARCH_STM32)
 if (query(&getNum, " flag [0x%x]: ", lastFlags) == 0)
 {
  numVal = (int) lastFlags;
 }
 else
 {
  lastFlags = numVal;
 }
#else
 int numVal;
 if (query(&getNum, &numVal, " flag [0x%x]: ", lastFlags) == 0)
 {
  numVal = (int) lastFlags;
 }
 else
 {
  lastFlags = numVal;
 }
#endif
 newline();
 flushBuf();
 if (numVal & 0x01)
  tmrInfo(TIM1);
#ifdef TIM2
 if (numVal & 0x02)
  tmrInfo(TIM2);
#endif
#ifdef TIM3
 if (numVal & 0x04)
  tmrInfo(TIM3);
#endif
#ifdef TIM4
 if (numVal & 0x08)
  tmrInfo(TIM4);
#endif
#ifdef TIM5
 if (numVal & 0x10)
  tmrInfo(TIM5);
#endif

 if (numVal & 0x20)
 {
#ifdef TIM6
  tmrInfo(TIM6);
#endif
#ifdef TIM7
  tmrInfo(TIM7);
#endif
 }

#ifdef TIM8
 if (numVal & 0x40)
  tmrInfo(TIM8);
#endif

 if (numVal & 0x80)
 {
#ifdef TIM9
  tmrInfo(TIM9);
#endif
#ifdef TIM15
  tmrInfo(TIM15);
#endif
 }

 if (numVal & 0x100)
 {
#ifdef TIM10
  tmrInfo(TIM10);
#endif
#ifdef TIM16
  tmrInfo(TIM16);
#endif
 }

 if (numVal & 0x200)
 {
#ifdef TIM11
  tmrInfo(TIM11);
#endif
#ifdef TIM17
  tmrInfo(TIM17);
#endif
 }

#ifdef TIM12
 if (numVal & 0x400)
  tmrInfo(TIM12);
#endif

 if (numVal & 0x800)		/* exti */
  extiInfo();

 if (numVal & 0x01000)
  gpioInfo(GPIOA);
 if (numVal & 0x02000)
  gpioInfo(GPIOB);
 if (numVal & 0x04000)
  gpioInfo(GPIOC);
#ifdef GPIOD
 if (numVal & 0x08000)
  gpioInfo(GPIOD);
#endif
#ifdef GPIOE
 if (numVal & 0x10000)
  gpioInfo(GPIOE);
#endif
#ifdef GPIOF
 if (numVal & 0x20000)
  gpioInfo(GPIOF);
#endif
#ifdef GPIOG
 if (numVal & 0x40000)
  gpioInfo(GPIOH);
#endif

#if defined(STM32MON)
 if (numVal & 0x100000)
  usartInfo(USART1, "DBG");
 if (numVal & 0x200000)
  usartInfo(USART3, "WIFI");
#else
 if (numVal & 0x100000)
 {
  usartInfo(DBGPORT, "DBG");
  usartInfo(REMPORT, "REM");
#if defined(MEGAPORT)
  usartInfo(DBGPORT, "MEGA");
#endif	/* MEGAPORT */
 }
#endif	/* STM32MON */

 if (numVal & 0x400000)
 {
#if defined(STM32MON)
  i2cInfo(I2C1, "I2C1");
#else
#ifdef I2C1
  i2cInfo(I2C_DEV, I2C_NAME);
#endif
#if defined(SPI3)
  spiInfo(SPIn, SPI_NAME);
#endif  /* SPI3 */
#endif  // STM32MON
 }

 if (numVal & 0x800000)
 {
  adcInfo(ADC1, '1');
  newline();
  adcInfo(ADC2, '2');
 }
 if (numVal & 0x1000000)
 {
  rtcInfo();
 }
 if (numVal & 0x2000000)
 {
  pwrInfo();
  newline();
  bkpInfo();
  newline();
  afioInfo();
 }
}

//void bitState(const char *s, volatile const uint32_t *p, uint32_t mask)
//{
// printf("%s %c\n", s, ((*p & mask) == 0) ? '0' : '1');
//}

#endif	/* __STM32INFO */
