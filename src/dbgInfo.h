extern unsigned char __bss_end;
extern unsigned char __noinit_start;
extern unsigned char __noinit_end;
extern unsigned char __heap_start;

#define TRACE_SIZE 256

typedef struct wdtInfo
{
 unsigned int tag0;
 unsigned long int pc;
 unsigned char data[64];
 unsigned int tag1;
} T_WDT_INFO, *P_WDT_INFO;

typedef struct dbgInfo
{
 char adcFlag;
 char rsv0;
 unsigned int i;
 unsigned int trace[TRACE_SIZE];
} T_DBG_INFO, *P_DBG_INFO;

#if defined(ARDUINO_AVR_MEGA2560)

#define PC_OFFSET 26

#if !defined(ASM_GET_PC)
//extern "C" unsigned long int getPC(void);
extern "C" unsigned int getPC(void);
#else
inline unsigned long int getPC(void)
{
 asm (
  "clr 25\n\t"
  "pop r24\n\t"
  "pop r23\n\t"
  "pop r22\n\t"
  "push r22\n\t"
  "push r23\n\t"
  "push r25\n"
  "ret"
  );
 return(0);
}
#endif
#endif	/* ARDUINO_AVR_MEGA2560 */

#if defined(ARDUINO_AVR_PRO)

#define PC_OFFSET 12

extern "C" int getPC(void);
#if 0
{
 asm (
  "pop r25\n\t"
  "pop r24\n\t"
  "push r24\n\t"
  "push r25\n"
  "ret"
  );
 return(0);
}
#endif

#endif	/* ARDUINO_ARCH_PRO */

//#define DBG_INFO ((P_DBG_INFO) (&__noinit_start))
#define DBG_INFO ((P_DBG_INFO) (&dbgData))

#if defined(ARDUINO_AVR_MEGA2560)
#if defined(__MONITOR__)

T_WDT_INFO wdtData __attribute__((section(".noinit")));
T_DBG_INFO dbgData __attribute__((section(".noinit")));

#else

void trace();
extern T_WDT_INFO wdtData;
extern T_DBG_INFO dbgData;

#endif	/* __MONITOR__ */
#else
void trace();
#endif	/* ARDUINO_AVR_MEGA2560 */
