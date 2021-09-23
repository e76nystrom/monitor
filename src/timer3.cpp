#define __TIMER3__

#include <Arduino.h>
#include <stdio.h>
#include "struct.h"
#include "serial.h"
#include "millis.h"

#define EXTC
#define EXT extern

#include "timer3.h"

#if defined(__TIMER3_INC__)	// <-
#ifdef TCCR3A

#if !defined(EXTC)
#define EXTC
#endif	/* EXTC */

EXTC void initTimer3(unsigned long int uSec);
EXTC void pwm1Timer3(unsigned int duty);
EXTC void stopTimer3();

EXT unsigned char timer3Prescale;
EXT unsigned int timer3Period;

#endif	/* TCCR3A */
#endif // ->
#if defined(__TIMER3__)

#define CONSOLE

typedef struct
{
 unsigned char shift;
 unsigned char sel;
} T_TIMER, *P_TIMER;

T_TIMER tmr3Cfg[] =
{
 {0, _BV(CS30)},
 {3, _BV(CS31)},
 {3, _BV(CS31) | _BV(CS30)},
 {2, _BV(CS32)},
 {2, _BV(CS32) | _BV(CS30)}
};

#define T3_CFGLEN (sizeof(tmr3Cfg) / sizeof(T_TIMER))

void initTimer3(unsigned long int uSec)
{
 T_SHORT_LONG cycles;
 cycles.val = (F_CPU * uSec) / 2000000;
 P_TIMER cfg = tmr3Cfg;
 unsigned char i;
 for (i = 0; i < T3_CFGLEN; i++)
 {
  cycles.val >>= cfg->shift;
#if defined(CONSOLE)
  printf(F0("cycles %08lx high %04x low %04x\n"),
	 cycles.val, cycles.high, cycles.low);
#endif	/* CONSOLE */
  if (cycles.high == 0)
  {
   break;
  }
  cfg++;
 }

 if (i == T3_CFGLEN)
 {
  timer3Period = 0xffff;
  timer3Prescale = _BV(CS32) | _BV(CS30);
 }
 else
 {
  timer3Period = cycles.low;
  timer3Prescale = cfg->sel;
 }

 // stop timer, clear wavefrom gen bits, clear input capture bits
 TCCR3B = 0;
 // disable all pwm output and clear waveform generator bits
 TCCR3A = 0;
 // clear counter
 TCNT3 = 0;
//#if defined(T3_PWM_A_Pin)
#if 1
 // set mode 8, phase and frequency correct pwm, top ICR3
 TCCR3B = _BV(WGM33);
#else
 // set mode 12, counter timer on compare match, top ICR3
 TCCR3B = _BV(WGM33) | _BV(WGM32);
#endif	/* T3_PWM_A_PIN */
 // set cycle length
 ICR3 = cycles.low;
 // clear interrupt flags
 TIFR5 = _BV(ICF3) | _BV(OCF3C) | _BV(OCF3B) | _BV(OCF3A) | _BV(TOV3);
 // enable overflow interrrupt
 TIMSK3 = _BV(TOIE3);
 // set clock divider and enable clock
 TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
 TCCR3B |= timer3Prescale;
}

#ifdef T3_PWM_A_Pin

void pwm1Timer3(unsigned int duty)
{
 if (duty == 0)
 {
  TCCR3A = 0;
  TCCR3B = 0;
  T3_PWM_A_Port &= ~T3_PWM_A_Mask;
 }
 else if (duty >= (timer3Period - 1))
 {
  TCCR3A = 0;
  TCCR3B = 0;
  T3_PWM_A_Port |= T3_PWM_A_Mask;
 }
 else
 {
  unsigned long dutyCycle = timer3Period;
  dutyCycle *= duty;
  dutyCycle >>= 10;
  // enable output on pmw a pin
  T3_PWM_A_DDR |= T3_PWM_A_Mask;
  // set pwm value
  OCR3A = dutyCycle;
  // turn on pwm, clear on up count, set on down count
  TCCR3A |= _BV(COM3A1);
  TIMSK3 |= _BV(OCIE3A);
#if defined(CONSOLE)
  printf(F0("DDRE %02x TCCR3A %02x OCR3A %02x\n"), DDRE, TCCR3A, OCR3A);
#endif	/* CONSOLE */
 }
}

#else

void pwm1Timer3(unsigned int duty)
{}

#endif	/* T3_PWM_A_PIN */

void stopTimer3()
{
 // disable clock
 TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
}

#endif	/* __TIMER3__ */
