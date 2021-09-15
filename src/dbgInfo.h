extern unsigned char __bss_end;

#define TRACE_SIZE 256

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

#define DBG_INFO ((P_DBG_INFO) (&__bss_end))

inline void trace()
{
 int i = DBG_INFO->i;
 DBG_INFO->trace[i] = getPC();
 i += 1;
 if (i >= TRACE_SIZE)
  i = 0;
 DBG_INFO->i = i;
}
