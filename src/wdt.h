#ifdef ARDUINO_ARCH_AVR
#define WDT 1
#if WDT
#include <avr/wdt.h>
#define WDT_TO (WDTO_8S)
#endif	/* WDT */
#endif	/* ARDUINO_ARCH_AVR */

#ifdef MEGA32
#define WDT 0
#endif	/* MEGA32 */

#ifdef WIN32
#define WDT 0
#endif	/* WIN32 */

#if !WDT
#define WDT_TO
#define wdt_enable()
#define wdt_disable(x)
#define wdt_reset()
#endif  /* ! WDT */
