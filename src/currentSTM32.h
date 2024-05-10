/* -------------------- monitor index 1 -------------------- */

#if (MONITOR_INDEX == 1)

#define MAX_CHAN_POWER 0
#define MAX_CHAN_RMS 1
#define MAX_CHAN (MAX_CHAN_POWER + MAX_CHAN_RMS)

#if defined(__CURRENT__)
#if MAX_CHAN_POWER > 0
T_RMSPWR rmsPower[MAX_CHAN_POWER];
#endif	/* MAX_CHAN_POWER */
#if MAX_CHAN_RMS > 0
T_RMSCHAN rmsData[MAX_CHAN_RMS];
#endif	/* MAX_CHAN_RMS */

/* {type, label, rmsAdc {adc, chan, voltAdc {adc, chan}, rmsAdc, voltAdc, 
   [curScale | rmsScale], voltScale, [pwr | rms]} */

T_CHANCFG chanCfg[MAX_CHAN] =
{
/* type, label, {cAdc, cChan}, {0, 0}, cScale, 0, cRmsData */
 {RMS_CHAN, 'c', {ADC1, ADC1_0} , {0, 0}, 30, 0, (P_RMSPWR) &rmsData[0]},
};
#endif	/* __CURRENT__ */

#endif	/* MONITOR_INDEX == 1 */

/* -------------------- monitor index 2 -------------------- */

#if (MONITOR_INDEX == 2)

#if defined(__CURRENT__)

#if MAX_CHAN_POWER > 0
T_RMSPWR rmsPower[MAX_CHAN_POWER];
#endif	/* MAX_CHAN_POWER */
#if MAX_CHAN_RMS > 0
T_RMSCHAN rmsData[MAX_CHAN_RMS];
#endif	/* MAX_CHAN_RMS */

/* {type, label, rmsAdc {adc, chan, voltAdc {adc, chan}, rmsAdc, voltAdc, 
   [curScale | rmsScale], voltScale, [pwr | rms]} */

T_CHANCFG chanCfg[MAX_CHAN] =
{
/* type, label, {cAdc, cChan}, {vAdc, vChan}, cScale, vScale, */
 {POWER_CHAN, 'p', {ADC1, ADC1_0}, {ADC2, ADC2_0}, 30, 0.15119 * VOLT_SCALE,
  (P_RMSPWR) &rmsPower[0]},

/* type, label, {cAdc, cChan}, {0, 0}, cScale, 0, cRmsData */
 {RMS_CHAN, 'c', {ADC1, ADC1_0}, {0, 0}, 30, 0, (P_RMSPWR) &rmsData[0]},

/* type, label, {vAdc, vChan}, {0, 0}, vScale, 0, vRmsData */
 {RMS_CHAN, 'v', {ADC2, ADC2_0} , {0, 0}, 0.15119 * VOLT_SCALE, 0,
  (P_RMSPWR) &rmsData[1]},
};

#endif	/* __CURRENT__ */

#endif	/* MONITOR_INDEX == 2 */

/* -------------------- monitor index 3 -------------------- */

#if (MONITOR_INDEX == 3)

#if defined(__CURRENT__)

#if MAX_CHAN_POWER > 0
T_RMSPWR rmsPower[MAX_CHAN_POWER];
#endif	/* MAX_CHAN_POWER */
#if MAX_CHAN_RMS > 0
T_RMSCHAN rmsData[MAX_CHAN_RMS];
#endif	/* MAX_CHAN_RMS */

/* {type, label, rmsAdc {adc, chan, voltAdc {adc, chan}, rmsAdc, voltAdc, 
   [curScale | rmsScale], voltScale, [pwr | rms]} */

T_CHANCFG chanCfg[MAX_CHAN] =
{
/* type, label, {cAdc, cChan}, {vAdc, vChan}, cScale, vScale, */
 {POWER_CHAN, 'p', {ADC1, ADC1_0}, {ADC2, ADC2_0}, 30, 0.15119 * VOLT_SCALE,
  (P_RMSPWR) &rmsPower[0]},

/* type, label, {cAdc, cChan}, {0, 0}, cScale, 0, cRmsData */
 {RMS_CHAN, 'c', {ADC1, ADC1_0}, {0, 0}, 30, 0, (P_RMSPWR) &rmsData[0]},

/* type, label, {vAdc, vChan}, {0, 0}, vScale, 0, vRmsData */
 {RMS_CHAN, 'v', {ADC2, ADC2_0} , {0, 0}, 0.15119 * VOLT_SCALE, 0,
  (P_RMSPWR) &rmsData[1]},
};

#endif	/* __CURRENT__ */

#endif	/* MONITOR_INDEX == 3 */

/* -------------------- monitor index 4 -------------------- */

#if (MONITOR_INDEX == 4)

#if defined(__CURRENT__)

#if MAX_CHAN_POWER > 0
T_RMSPWR rmsPower[MAX_CHAN_POWER];
#endif	/* MAX_CHAN_POWER */
#if MAX_CHAN_RMS > 0
T_RMSCHAN rmsData[MAX_CHAN_RMS];
#endif	/* MAX_CHAN_RMS */

/* {type, label, rmsAdc {adc, chan, voltAdc {adc, chan}, rmsAdc, voltAdc,
   [curScale | rmsScale], voltScale, [pwr | rms]} */

T_CHANCFG chanCfg[MAX_CHAN] =
{
/* type, label, {cAdc, cChan}, {vAdc, vChan}, cScale, vScale, */
 {POWER_CHAN, 'p', {ADC1, ADC1_0}, {ADC2, ADC2_0}, 30, 0.15119 * VOLT_SCALE,
  (P_RMSPWR) &rmsPower[0]},

/* type, label, {cAdc, cChan}, {0, 0}, cScale, 0, cRmsData */
 {RMS_CHAN, 'c', {ADC1, ADC1_0}, {0, 0}, 30, 0, (P_RMSPWR) &rmsData[0]},

/* type, label, {vAdc, vChan}, {0, 0}, vScale, 0, vRmsData */
 {RMS_CHAN, 'v', {ADC2, ADC2_0} , {0, 0}, 0.15119 * VOLT_SCALE, 0,
  (P_RMSPWR) &rmsData[1]},
};

#endif	/* __CURRENT__ */

#endif	/* MONITOR_INDEX == 4 */

/* -------------------- monitor end of definitions -------------------- */
