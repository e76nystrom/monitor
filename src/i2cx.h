#if 1	// <-

#if !defined(EXT)
#define EXT extern
#endif

#if defined(STM32F1) || defined(STM32F4)
void i2cWrite(uint8_t);
#endif	/* STM32F1 || STM32F4 */

void i2cWrite(uint8_t *data, uint32_t size);

void i2cWaitBusy();
void i2cPut(uint8_t ch);
void i2cPutString(uint8_t *p, int size);
void i2cSend();
void i2cControl();

enum I2C_STATES
{
 I_IDLE,                        /* 0 idle state */
 I_WAIT_START,			/* 1 wait for not busy to start */
 I_START,			/* 2 wait for start and send address */
 I_ADDRESS,			/* 3 wait for address and start data */
 I_SEND_DATA,			/* 4 send data */
 I_WAIT_DATA,			/* 5 wait for data to be sent */
 I_WAIT_STOP,			/* 6 wait for stop */
};

enum I2C_STATUS
{
 IS_DONE,			/* 0 done */
 IS_BUSY,			/* 1 busy */
 IS_TIMEOUT,			/* 2 timeout */
};

#define I2C_BUF_SIZE 256
#define I2CX_TIMEOUT 500U

typedef struct
{
 int state;
 int lastState;
 int status;
 int errCount;
 unsigned int startTime;
 unsigned int timeout;
 unsigned int maxTime;
 int fil;
 int emp;
 int count;
 uint8_t buffer[I2C_BUF_SIZE];
} T_I2C_CTL, *P_I2C_CTL;

EXT T_I2C_CTL i2cCtl;
EXT int i2cError;

#endif	// ->
