#if 1	// <-
#if !defined(__MAX31865_INC__)
#define __MAX31865_INC__

#define MX65_CFG       0x00
#define MX65_RTD_MSB   0x01
#define MX65_RTD_LSB   0x02
#define MX65_H_FLT_MSB 0x03
#define MX65_H_FLT_LSB 0x04
#define MX65_L_FLT_MSB 0x05
#define MX65_L_FLT_LSB 0x06
#define MX65_FLT_STAT  0x07

#define MX65_WRITE     0x80

#define MX65_CFG_BIAS     0x80
#define MX65_CFG_MODEAUTO 0x40
#define MX65_CFG_MODEOFF  0x00
#define MX65_CFG_1SHOT    0x20
#define MX65_CFG_3WIRE    0x10
#define MX65_CFG_24WIRE   0x00
#define MX65_CFG_FLT_DET  0x0C
#define MX65_CFG_CLR_FLT  0x02
#define MX65_CFG_FILT50HZ 0x01
#define MX65_CFG_FILT60HZ 0x00

#define MX65_CFG_FLT_AUTO 0x04

#define MX65_FLT_HIGHTHRESH 0x80
#define MX65_FLT_LOWTHRESH  0x40
#define MX65_FLT_REFINLOW   0x20
#define MX65_FLT_REFINHIGH  0x10
#define MX65_FLT_RTDINLOW   0x08
#define MX65_FLT_OVUV       0x04

typedef enum mx65Wires_t
{
  MX65_2WIRE = 0,
  MX65_3WIRE = 1,
  MX65_4WIRE = 0
} MX65_WIRES;


#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

void max65Init(int dev, mx65Wires_t wires);
void max65SetWires(int dev, mx65Wires_t wires);
void max65EnableBias(int dev, bool bias);
void max65AutoConvert(int dev, bool convert);
uint8_t max65ReadFault(int dev);
void max65ClearFault(int dev);
uint8_t max65CheckFault(int dev);
unsigned int max65Temp(int dev);
uint16_t max65ReadRTD(int dev);

void max65Cmds();

#endif	/* __MAX31865_INC__ */
#endif	// ->
