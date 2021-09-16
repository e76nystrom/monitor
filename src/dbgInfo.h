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
 int adcIsrCount;
 unsigned int i;
 unsigned int trace[TRACE_SIZE];
} T_DBG_INFO, *P_DBG_INFO;

#if defined(ARDUINO_AVR_MEGA2560)

#define PC_OFFSET 26

#if !defined(VMICRO)
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
#endif	/* ARDUINO_ARCH_MEGA */

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

#if defined(__MONITOR__)

#if 0
volatile char wdtData[64 + 8]
__attribute__((section(".noinit")));

volatile char dbgBuffer[sizeof(T_DBG_INFO)]
__attribute__((section(".noinit")));
#else
volatile T_WDT_INFO wdtData __attribute__((section(".noinit")));
volatile T_DBG_INFO dbgData __attribute__((section(".noinit")));
#endif

#if 0
void trace()
{
#if 1

#if 0
 int i = DBG_INFO->i;
 DBG_INFO->trace[i] = getPC();
 i += 1;
 if (i >= TRACE_SIZE)
  i = 0;
 DBG_INFO->i = i;
#else
 int i = dbgData.i;
 dbgData.trace[i] = getPC();
 i += 1;
 if (i >= TRACE_SIZE)
  i = 0;
 dbgData.i = i;
#endif

#endif
}
#endif
#else
void trace();
extern char wdtData[];
extern char dbgBuffer[];
#endif
