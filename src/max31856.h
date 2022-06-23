#if 1	// <-
#if !defined(__MAX31856_INC__)
#define __MAX31856_INC__

#define MX56_CR0    0x00	/* Config 0 register */
#define MX56_CR1    0x01	/* Config 1 register */
#define MX56_MASK   0x02	/* Fault Mask register */
#define MX56_CJHF   0x03	/* cold junction High fault */
#define MX56_CJLF   0x04	/* cold junction Low fault */
#define MX56_LTHFTH 0x05	/* msb high temp fault */
#define MX56_LTHFTL 0x06	/* lsb high temp fault */
#define MX56_LTLFTH 0x07	/* msb low temp fault */
#define MX56_LTLFTL 0x08	/* lsb low temp fault */
#define MX56_CJTO   0x09	/* cold junction offset */
#define MX56_CJH    0x0A	/* msb cold junction temp */
#define MX56_CJL    0x0B	/* lsb cold junction temp */
#define MX56_LTB2   0x0C	/* byte 2 temp */
#define MX56_LTB1   0x0D	/* byte 1 temp */
#define MX56_LTB0   0x0E	/* byte 0 temp */
#define MX56_SR     0x0F	/* fault status */

#define MX56_W 0x80		/* register write flag */

#define MX56_CR0_AUTOCONVERT 0x80 /* auto convert flag */
#define MX56_CR0_1SHOT       0x40 /* one shot convert flag */
#define MX56_CR0_OCFAULT1    0x20 /* open circuit fault 1 flag */
#define MX56_CR0_OCFAULT0    0x10 /* open circuit fault 0 flag */
#define MX56_CR0_CJ          0x08 /* cold junction disable flag */
#define MX56_CR0_FAULT       0x04 /* fault mode flag */
#define MX56_CR0_FAULTCLR    0x02 /* fault clear flag */
#define MX56_CR0_N50	     0x01 /* 50hz noise filter */

#define MX56_FAULT_CJRANGE 0x80 /* cold junction out of range */
#define MX56_FAULT_TCRANGE 0x40 /* temp out of range */
#define MX56_FAULT_CJHIGH  0x20 /* cold junction high */
#define MX56_FAULT_CJLOW   0x10 /* cold junction low */
#define MX56_FAULT_TCHIGH  0x08 /* temp high */
#define MX56_FAULT_TCLOW   0x04 /* temp low */
#define MX56_FAULT_OVUV    0x02 /* over or under voltage */
#define MX56_FAULT_OPEN    0x01 /* open circuit */

typedef enum			/* noise filtering */
{
  MX56_FILTER_50HZ,
  MX56_FILTER_60HZ
} mx56_filter_t;

typedef enum			/* thermocouple types */
{
  MX56_TCTYPE_B = 0b0000,
  MX56_TCTYPE_E = 0b0001,
  MX56_TCTYPE_J = 0b0010,
  MX56_TCTYPE_K = 0b0011,
  MX56_TCTYPE_N = 0b0100,
  MX56_TCTYPE_R = 0b0101,
  MX56_TCTYPE_S = 0b0110,
  MX56_TCTYPE_T = 0b0111,
  MX56_VMODE_G8 = 0b1000,
  MX56_VMODE_G32 = 0b1100,
} mx56_type_t;

#define MX56_CR1_THERMO_MASK 0xf /* thermocouple type mask */

typedef enum			/* conversion modes */
{
  MX56_ONESHOT,
  MX56_ONESHOT_NOWAIT,
  MX56_CONTINUOUS
} mx56_conversion_t;

void max56Init(int dev, mx56_type_t type, mx56_conversion_t mode);
void max56SetConversionType(int dev, mx56_conversion_t type);
void max56SetThermocoupleType(int dev, mx56_type_t type);

int32_t max56ReadTemp();
float max56ConvTemp(int t);
char *max56FmtTemp(char *buf, size_t bufLen);
char *max56FmtTemp(int32_t temp, char *buf, size_t bufLen);

int32_t max56ReadCJ();
float max56ConvCJ(int t);
char *max56FmtCJ(char *buf, size_t bufLen);
char *max56FmtCJ(int32_t temp, char *buf, size_t bufLen);

void max56Cmds(void);

#endif	/* __MAX31856_INC__ */
#endif	// ->
