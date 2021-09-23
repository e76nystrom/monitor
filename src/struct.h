#if 0
typedef union
{
 struct
 {
  uint16_t low;
  uint16_t high;
 };
 struct
 {
  unsigned long l;
 };
} T_SHORT_LONG, *P_SHORT_LONG;
#endif

typedef union
{
 struct
 {
  unsigned char lo;
  unsigned char hi;
 };
 struct
 {
  unsigned int w;
 };
} CHAR_INT;

typedef struct
{
 volatile uint8_t tccra;
 volatile uint8_t tccrb;
 volatile uint8_t tccrc;
 uint8_t r0;
 volatile uint16_t tcnt;
 volatile uint16_t icr;
 volatile uint16_t ocra;
 volatile uint16_t ocrb;
 volatile uint16_t ocrc;
} T_TMR, *P_TMR;

typedef struct
{
 volatile uint8_t pin;
 volatile uint8_t ddr;
 volatile uint8_t port;
} T_PORT, *P_PORT;

typedef struct
{
 P_TMR timer;
 volatile uint8_t *tifr;
 volatile uint8_t *timsk;
 P_PORT aPort;
 uint8_t aMask;
} T_TIMER_CTL, *P_TIMER_CTL;
