#if 1	// <-

#if !defined(I2C_DEV)
#define I2C_DEV I2C1
#endif

#if !defined(EXT)
#define EXT extern
#endif

int i2c_start(I2C_TypeDef* I2Cx, uint8_t address);

void initI2c(void);
void i2cWrite(uint8_t data);

void i2cPut(uint8_t ch);
void i2cPutString(uint8_t *p, int size);
void i2cSend(void);
void i2cControl(void);

enum I2C_STATES
{
 I_IDLE,                        /* 0 idle state */
 I_WAIT_START,			/* 1 wait for not busy to start */
 I_START,			/* 2 wait for start and send address */
 I_ADDRESS,			/* 3 wait for address and start data */
 I_SEND_DATA,			/* 4 send data */
 I_WAIT_DATA,			/* 5 wait for data to be sent */
};

enum I2C_STATUS
{
 IS_DONE,			/* 0 done */
 IS_BUSY,			/* 1 busy */
 IS_TIMEOUT,			/* 2 timeout */
};

#define I2C_BUF_SIZE 256
#define I2C_TIMEOUT 500U

typedef struct
{
 int state;
 int lastState;
 int status;
 unsigned int startTime;
 unsigned int timeout;
 unsigned int maxTime;
 int fil;
 int emp;
 int count;
 uint8_t buffer[I2C_BUF_SIZE];
} T_I2C_CTL, *P_I2C_CTL;

EXT T_I2C_CTL i2cCtl;

#define SLAVE_ADDRESS 0x27 // the slave address (example)

EXT char slaveAddress;

inline void i2c_SendData(I2C_TypeDef* I2Cx, uint8_t data)
{
 I2Cx->DR = data;
}

inline void i2c_stop(I2C_TypeDef* I2Cx)
{
 I2Cx->CR1 |= I2C_CR1_STOP;
}

#endif	// ->
