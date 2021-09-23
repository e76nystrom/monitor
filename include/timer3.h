#if 1	// <-
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
