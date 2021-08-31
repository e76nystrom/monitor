#if defined(ARDUINO_ARCH_AVR)
#define WDT 1

#if defined(MEGA32)
#define WDT 1
#endif	/* MEGA32 */

#endif	/* ARDUINO_ARCH_AVR */

#if defined(WIN32)
#define WDT 0
#endif	/* WIN32 */

#if defined(ARDUINO_ARCH_STM32)
#define WDT 0
#endif	/* ARDUINO_ARCH_STM32) */

#if WDT
#include <avr/wdt.h>
#define WDT_TO (WDTO_8S)
#endif	/* WDT */

#if !WDT
#define WDT_TO
#define wdt_enable(x)
#define wdt_disable(x)
#define wdt_reset()
#endif  /* ! WDT */
