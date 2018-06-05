/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: px4_swashplateless_2016a.c
 *
 * Code generated for Simulink model 'px4_swashplateless_2016a'.
 *
 * Model version                  : 1.107
 * Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
 * C/C++ source code generated on : Mon Jun  4 14:17:10 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "px4_swashplateless_2016a.h"
#include "px4_swashplateless_2016a_private.h"
#include "px4_swashplateless_2016a_dt.h"

const char *g_pwm_device = PWM_OUTPUT0_DEVICE_PATH;
int g_pwm_fd = -1;
bool g_pwm_enabled = false;
const uint8_T px4_swashplateless_2016a_U8GND = 0U;/* uint8_T ground */

/* Block signals (auto storage) */
B_px4_swashplateless_2016a_T px4_swashplateless_2016a_B;

/* Block states (auto storage) */
DW_px4_swashplateless_2016a_T px4_swashplateless_2016a_DW;

/* Real-time model */
RT_MODEL_px4_swashplateless_2_T px4_swashplateless_2016a_M_;
RT_MODEL_px4_swashplateless_2_T *const px4_swashplateless_2016a_M =
  &px4_swashplateless_2016a_M_;
static void rate_scheduler(void);
real32_T sMultiWord2Single(const uint32_T u1[], int32_T n1, int32_T e1)
{
  real32_T y;
  int32_T i;
  int32_T exp_0;
  uint32_T u1i;
  uint32_T cb;
  y = 0.0F;
  exp_0 = e1;
  if ((u1[n1 - 1] & 2147483648U) != 0U) {
    cb = 1U;
    for (i = 0; i < n1; i++) {
      u1i = ~u1[i];
      cb += u1i;
      y -= (real32_T)ldexp((real32_T)cb, exp_0);
      cb = (uint32_T)(cb < u1i);
      exp_0 += 32;
    }
  } else {
    for (i = 0; i < n1; i++) {
      y += (real32_T)ldexp((real32_T)u1[i], exp_0);
      exp_0 += 32;
    }
  }

  return y;
}

void sMultiWordMul(const uint32_T u1[], int32_T n1, const uint32_T u2[], int32_T
                   n2, uint32_T y[], int32_T n)
{
  int32_T i;
  int32_T j;
  int32_T k;
  int32_T nj;
  uint32_T u1i;
  uint32_T yk;
  uint32_T a1;
  uint32_T a0;
  uint32_T b1;
  uint32_T w10;
  uint32_T w01;
  uint32_T cb;
  boolean_T isNegative1;
  boolean_T isNegative2;
  uint32_T cb1;
  uint32_T cb2;
  isNegative1 = ((u1[n1 - 1] & 2147483648U) != 0U);
  isNegative2 = ((u2[n2 - 1] & 2147483648U) != 0U);
  cb1 = 1U;

  /* Initialize output to zero */
  for (k = 0; k < n; k++) {
    y[k] = 0U;
  }

  for (i = 0; i < n1; i++) {
    cb = 0U;
    u1i = u1[i];
    if (isNegative1) {
      u1i = ~u1i + cb1;
      cb1 = (uint32_T)(u1i < cb1);
    }

    a1 = u1i >> 16U;
    a0 = u1i & 65535U;
    cb2 = 1U;
    k = n - i;
    nj = n2 <= k ? n2 : k;
    k = i;
    for (j = 0; j < nj; j++) {
      yk = y[k];
      u1i = u2[j];
      if (isNegative2) {
        u1i = ~u1i + cb2;
        cb2 = (uint32_T)(u1i < cb2);
      }

      b1 = u1i >> 16U;
      u1i &= 65535U;
      w10 = a1 * u1i;
      w01 = a0 * b1;
      yk += cb;
      cb = (uint32_T)(yk < cb);
      u1i *= a0;
      yk += u1i;
      cb += (yk < u1i);
      u1i = w10 << 16U;
      yk += u1i;
      cb += (yk < u1i);
      u1i = w01 << 16U;
      yk += u1i;
      cb += (yk < u1i);
      y[k] = yk;
      cb += w10 >> 16U;
      cb += w01 >> 16U;
      cb += a1 * b1;
      k++;
    }

    if (k < n) {
      y[k] = cb;
    }
  }

  /* Apply sign */
  if (isNegative1 != isNegative2) {
    cb = 1U;
    for (k = 0; k < n; k++) {
      yk = ~y[k] + cb;
      y[k] = yk;
      cb = (uint32_T)(yk < cb);
    }
  }
}

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (px4_swashplateless_2016a_M->Timing.TaskCounters.TID[1])++;
  if ((px4_swashplateless_2016a_M->Timing.TaskCounters.TID[1]) > 124) {/* Sample time: [0.5s, 0.0s] */
    px4_swashplateless_2016a_M->Timing.TaskCounters.TID[1] = 0;
  }
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0F) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = (real32_T)atan2((real32_T)u0_0, (real32_T)u1_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void px4_swashplateless_2016a_step(void)
{
  real32_T r;
  real32_T p;
  boolean_T p_0;
  uint8_T DataValid;
  boolean_T p_1;
  real32_T rtb_Sum2;
  real32_T rtb_Error;
  real32_T rtb_Error_i;
  real32_T rtb_Sum_b;
  real32_T rtb_Saturate;
  uint32_T tmp;
  uint32_T tmp_0;
  real_T u0;
  uint16_T u0_0;

  /* S-Function (sfun_px4_input_rc): '<Root>/input_rc' */
  {
    bool updated;
    orb_check(px4_swashplateless_2016a_DW.input_rc_input_rc_fd.fd, &updated);
    if (updated) {
      struct rc_input_values pwm_inputs;

      /* copy input_rc raw data into local buffer (uint16)*/
      orb_copy(ORB_ID(input_rc),
               px4_swashplateless_2016a_DW.input_rc_input_rc_fd.fd, &pwm_inputs);
      px4_swashplateless_2016a_B.input_rc_o1 = pwm_inputs.values[0];
      px4_swashplateless_2016a_B.input_rc_o2 = pwm_inputs.values[1];
      px4_swashplateless_2016a_B.input_rc_o3 = pwm_inputs.values[2];
      px4_swashplateless_2016a_B.input_rc_o4 = pwm_inputs.values[3];
      px4_swashplateless_2016a_B.input_rc_o5 = pwm_inputs.values[6];
    }
  }

  /* RelationalOperator: '<S9>/Compare' incorporates:
   *  Constant: '<S9>/Constant'
   */
  px4_swashplateless_2016a_B.Compare = (px4_swashplateless_2016a_B.input_rc_o5 >=
    1500);
  if (px4_swashplateless_2016a_M->Timing.TaskCounters.TID[1] == 0) {
    /* Switch: '<S10>/Switch' incorporates:
     *  Constant: '<S10>/Constant'
     *  Constant: '<S10>/Constant1'
     *  Constant: '<S10>/Constant2'
     *  Constant: '<S10>/Constant3'
     *  Switch: '<S10>/Switch1'
     */
    if (px4_swashplateless_2016a_B.Compare) {
      px4_swashplateless_2016a_B.Switch = MODE_BLINK_FAST;
      px4_swashplateless_2016a_B.Switch1 = COLOR_RED;
    } else {
      px4_swashplateless_2016a_B.Switch = MODE_BREATHE;
      px4_swashplateless_2016a_B.Switch1 = COLOR_GREEN;
    }

    /* End of Switch: '<S10>/Switch' */

    /* S-Function (sfun_px4_rgbled): '<S2>/RGB_LED' */
    ioctl(px4_swashplateless_2016a_DW.RGB_LED_rgbled_fd, RGBLED_SET_MODE,
          px4_swashplateless_2016a_B.Switch);
    ioctl(px4_swashplateless_2016a_DW.RGB_LED_rgbled_fd, RGBLED_SET_COLOR,
          px4_swashplateless_2016a_B.Switch1);
  }

  /* Switch: '<S11>/Switch1' incorporates:
   *  Constant: '<S11>/ARM_Tune'
   *  Constant: '<S11>/ARM_Tune1'
   */
  if (px4_swashplateless_2016a_B.Compare) {
    px4_swashplateless_2016a_B.Switch1_k = ARMING_WARNING_TUNE;
  } else {
    px4_swashplateless_2016a_B.Switch1_k = NOTIFY_POSITIVE_TUNE;
  }

  /* End of Switch: '<S11>/Switch1' */

  /* DataTypeConversion: '<S11>/Data Type Conversion' incorporates:
   *  Logic: '<S11>/Logical Operator'
   *  UnitDelay: '<S11>/Unit Delay'
   */
  px4_swashplateless_2016a_B.DataTypeConversion = (uint8_T)((int8_T)
    px4_swashplateless_2016a_DW.UnitDelay_DSTATE_o ^ (int8_T)
    px4_swashplateless_2016a_B.Compare);

  /* S-Function (sfun_px4_tune): '<S2>/Speaker_Tune' */
  if (px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd == -1) {
    px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd = open
      (TONEALARM0_DEVICE_PATH, O_WRONLY);
    if ((px4_swashplateless_2016a_B.DataTypeConversion > 0) &&
        (px4_swashplateless_2016a_B.DataTypeConversion !=
         px4_swashplateless_2016a_DW.Speaker_Tune_triggerState)) {
      if (px4_swashplateless_2016a_B.DataTypeConversion == 1) {
        ioctl(px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd, TONE_SET_ALARM,
              px4_swashplateless_2016a_B.Switch1_k);
      } else if (px4_swashplateless_2016a_B.DataTypeConversion == 2) {
        write(px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd, (uint8_T*)
              &px4_swashplateless_2016a_U8GND, 1);
      }
    }

    // save the state of the trigger
    px4_swashplateless_2016a_DW.Speaker_Tune_triggerState =
      px4_swashplateless_2016a_B.DataTypeConversion;
  } else {
    if ((px4_swashplateless_2016a_B.DataTypeConversion > 0) &&
        (px4_swashplateless_2016a_B.DataTypeConversion !=
         px4_swashplateless_2016a_DW.Speaker_Tune_triggerState)) {
      if (px4_swashplateless_2016a_B.DataTypeConversion == 1) {
        ioctl(px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd, TONE_SET_ALARM,
              px4_swashplateless_2016a_B.Switch1_k);
      } else if (px4_swashplateless_2016a_B.DataTypeConversion == 2) {
        write(px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd, (uint8_T*)
              &px4_swashplateless_2016a_U8GND, 1);
      }
    }

    // save the state of the trigger
    px4_swashplateless_2016a_DW.Speaker_Tune_triggerState =
      px4_swashplateless_2016a_B.DataTypeConversion;
  }

  /* Fcn: '<S6>/Fcn2' incorporates:
   *  DataTypeConversion: '<S6>/Data Type Conversion6'
   */
  rtb_Sum2 = ((real32_T)px4_swashplateless_2016a_B.input_rc_o1 - 1500.0F) /
    500.0F;

  /* Saturate: '<S6>/Saturation3' */
  if (rtb_Sum2 > 1.0F) {
    px4_swashplateless_2016a_B.Saturation3 = 1.0F;
  } else if (rtb_Sum2 < -1.0F) {
    px4_swashplateless_2016a_B.Saturation3 = -1.0F;
  } else {
    px4_swashplateless_2016a_B.Saturation3 = rtb_Sum2;
  }

  /* End of Saturate: '<S6>/Saturation3' */

  /* Fcn: '<S6>/Fcn1' incorporates:
   *  DataTypeConversion: '<S6>/Data Type Conversion5'
   */
  rtb_Sum2 = ((real32_T)px4_swashplateless_2016a_B.input_rc_o2 - 1500.0F) /
    500.0F;

  /* Saturate: '<S6>/Saturation2' */
  if (rtb_Sum2 > 1.0F) {
    px4_swashplateless_2016a_B.Saturation2 = 1.0F;
  } else if (rtb_Sum2 < -1.0F) {
    px4_swashplateless_2016a_B.Saturation2 = -1.0F;
  } else {
    px4_swashplateless_2016a_B.Saturation2 = rtb_Sum2;
  }

  /* End of Saturate: '<S6>/Saturation2' */

  /* Fcn: '<S6>/Fcn7' incorporates:
   *  DataTypeConversion: '<S6>/Data Type Conversion3'
   */
  rtb_Sum2 = ((real32_T)px4_swashplateless_2016a_B.input_rc_o4 - 1500.0F) /
    500.0F;

  /* Saturate: '<S6>/Saturation7' */
  if (rtb_Sum2 > 1.0F) {
    px4_swashplateless_2016a_B.Saturation7 = 1.0F;
  } else if (rtb_Sum2 < -1.0F) {
    px4_swashplateless_2016a_B.Saturation7 = -1.0F;
  } else {
    px4_swashplateless_2016a_B.Saturation7 = rtb_Sum2;
  }

  /* End of Saturate: '<S6>/Saturation7' */

  /* Fcn: '<S6>/Fcn6' incorporates:
   *  DataTypeConversion: '<S6>/Data Type Conversion2'
   */
  rtb_Sum2 = ((real32_T)px4_swashplateless_2016a_B.input_rc_o3 - 1000.0F) /
    1000.0F;

  /* Saturate: '<S6>/Saturation10' */
  if (rtb_Sum2 > 1.0F) {
    px4_swashplateless_2016a_B.Saturation10 = 1.0F;
  } else if (rtb_Sum2 < 0.0F) {
    px4_swashplateless_2016a_B.Saturation10 = 0.0F;
  } else {
    px4_swashplateless_2016a_B.Saturation10 = rtb_Sum2;
  }

  /* End of Saturate: '<S6>/Saturation10' */

  /* Sum: '<S3>/Sum' incorporates:
   *  Constant: '<S3>/Constant'
   *  Gain: '<S3>/Gain'
   */
  px4_swashplateless_2016a_B.Sum = 550.0F *
    px4_swashplateless_2016a_B.Saturation10 + 1400.0F;

  /* S-Function (sfun_px4_vehicle_attitude): '<Root>/vehicle_attitude' */
  {
    bool updated;
    orb_check(px4_swashplateless_2016a_DW.vehicle_attitude_vehicle_attitu.fd,
              &updated);
    if (updated) {
      struct vehicle_attitude_s raw;
      orb_copy(ORB_ID(vehicle_attitude),
               px4_swashplateless_2016a_DW.vehicle_attitude_vehicle_attitu.fd,
               &raw);

      /* read out the Roll value */
      px4_swashplateless_2016a_B.phi = raw.roll;// phi

      /* read out the Pitch value */
      px4_swashplateless_2016a_B.theta = raw.pitch;// theta
    }
  }

  /* Gain: '<S3>/Gain5' incorporates:
   *  Constant: '<S3>/Constant1'
   *  Sum: '<S3>/Sum1'
   */
  px4_swashplateless_2016a_B.Gain5 = (px4_swashplateless_2016a_B.phi + 0.85F) *
    3.0F;

  /* Sum: '<S13>/Sum' incorporates:
   *  Gain: '<S3>/Gain3'
   */
  rtb_Error = 3.14159274F * px4_swashplateless_2016a_B.Saturation3 -
    px4_swashplateless_2016a_B.Gain5;

  /* S-Function (sfun_px4_sensor_combined): '<Root>/sensor_combined1' */
  {
    bool updated;
    orb_check(px4_swashplateless_2016a_DW.sensor_combined1_sensor_fd.fd,
              &updated);
    if (updated) {
      /* obtained data sensor combined */
      struct sensor_combined_s raw;

      /* copy sensors raw data into local buffer */
      orb_copy(ORB_ID(sensor_combined),
               px4_swashplateless_2016a_DW.sensor_combined1_sensor_fd.fd, &raw);

      /* read out the gyro X,Y,Z */
      px4_swashplateless_2016a_B.p = (float)raw.gyro_rad_s[0];
      px4_swashplateless_2016a_B.q = (float)raw.gyro_rad_s[1];
      px4_swashplateless_2016a_B.r = (float)raw.gyro_rad_s[2];
    }
  }

  /* Sum: '<S13>/Sum1' incorporates:
   *  Gain: '<S13>/Kip'
   *  UnitDelay: '<S13>/Unit Delay'
   */
  px4_swashplateless_2016a_B.Sum1 = (0.0F *
    px4_swashplateless_2016a_DW.UnitDelay_DSTATE + rtb_Error) -
    px4_swashplateless_2016a_B.p;

  /* Gain: '<S3>/Gain6' incorporates:
   *  Constant: '<S3>/Constant2'
   *  Sum: '<S3>/Sum3'
   */
  px4_swashplateless_2016a_B.Gain6 = (px4_swashplateless_2016a_B.theta + -0.2F) *
    3.0F;

  /* Sum: '<S12>/Sum' incorporates:
   *  Gain: '<S3>/Gain4'
   */
  rtb_Error_i = 3.14159274F * px4_swashplateless_2016a_B.Saturation2 -
    px4_swashplateless_2016a_B.Gain6;

  /* Sum: '<S12>/Sum1' incorporates:
   *  Gain: '<S12>/Kit'
   *  UnitDelay: '<S12>/Unit Delay'
   */
  px4_swashplateless_2016a_B.Sum1_j = (0.0F *
    px4_swashplateless_2016a_DW.UnitDelay_DSTATE_k + rtb_Error_i) -
    px4_swashplateless_2016a_B.q;

  /* Gain: '<S3>/Gain2' */
  rtb_Sum2 = 0.25F * px4_swashplateless_2016a_B.r;

  /* Saturate: '<S3>/Saturation1' */
  if (rtb_Sum2 > 2.0F) {
    rtb_Sum2 = 2.0F;
  } else {
    if (rtb_Sum2 < 0.012F) {
      rtb_Sum2 = 0.012F;
    }
  }

  /* Sum: '<S3>/Sum2' incorporates:
   *  Saturate: '<S3>/Saturation1'
   */
  rtb_Sum2 = px4_swashplateless_2016a_B.Saturation7 - rtb_Sum2;

  /* Sum: '<S15>/Sum' incorporates:
   *  DiscreteIntegrator: '<S15>/Integrator'
   */
  rtb_Sum_b = rtb_Sum2 + px4_swashplateless_2016a_DW.Integrator_DSTATE;

  /* Saturate: '<S15>/Saturate' */
  if (rtb_Sum_b > 1.0F) {
    rtb_Saturate = 1.0F;
  } else if (rtb_Sum_b < -1.0F) {
    rtb_Saturate = -1.0F;
  } else {
    rtb_Saturate = rtb_Sum_b;
  }

  /* End of Saturate: '<S15>/Saturate' */

  /* Gain: '<S3>/Gain1' */
  px4_swashplateless_2016a_B.Gain1 = 50.0F * rtb_Saturate;

  /* Start for MATLABSystem: '<S4>/Read Encoder' incorporates:
   *  MATLABSystem: '<S4>/Read Encoder'
   */
  p_0 = false;
  p_1 = true;
  if (!(px4_swashplateless_2016a_DW.obj.SampleTime == -1.0)) {
    p_1 = false;
  }

  if (p_1) {
    p_0 = true;
  }

  if (!p_0) {
    px4_swashplateless_2016a_DW.obj.SampleTime = -1.0;
  }

  DataValid = 0U;
  MW_readADC(px4_swashplateless_2016a_DW.obj.fd_adc,
             px4_swashplateless_2016a_B.ArrayPassIn, &DataValid);

  /* Gain: '<S4>/Gain' incorporates:
   *  MATLABSystem: '<S4>/Read Encoder'
   *  Start for MATLABSystem: '<S4>/Read Encoder'
   */
  tmp = 1121499238U;
  tmp_0 = (uint32_T)px4_swashplateless_2016a_B.ArrayPassIn[2];
  sMultiWordMul(&tmp, 1, &tmp_0, 1, &px4_swashplateless_2016a_B.r0.chunks[0U], 2);

  /* Sum: '<S4>/Sum' incorporates:
   *  Constant: '<S4>/Constant'
   *  DataTypeConversion: '<S4>/Data Type Conversion2'
   */
  u0 = sMultiWord2Single(&px4_swashplateless_2016a_B.r0.chunks[0U], 2, 0) *
    1.8189894E-12F - 3.1415926535897931;

  /* Saturate: '<S4>/Saturation1' */
  if (u0 > 3.1415926535897931) {
    px4_swashplateless_2016a_B.Saturation1 = 3.1415926535897931;
  } else if (u0 < -3.1415926535897931) {
    px4_swashplateless_2016a_B.Saturation1 = -3.1415926535897931;
  } else {
    px4_swashplateless_2016a_B.Saturation1 = u0;
  }

  /* End of Saturate: '<S4>/Saturation1' */

  /* MATLAB Function: '<S7>/MATLAB Function' */
  /* MATLAB Function 'Motor control/MATLAB Function': '<S16>:1' */
  /* '<S16>:1:3' */
  r = (real32_T)sin(px4_swashplateless_2016a_B.Sum1);

  /* '<S16>:1:4' */
  p = (real32_T)sin(px4_swashplateless_2016a_B.Sum1_j);

  /* Gain: '<S7>/Gain' incorporates:
   *  MATLAB Function: '<S7>/MATLAB Function'
   */
  /* '<S16>:1:5' */
  /* '<S16>:1:7' */
  /* '<S16>:1:9' */
  px4_swashplateless_2016a_B.Gain = (real32_T)sqrt(r * r + p * p) * (real32_T)
    cos((real32_T)px4_swashplateless_2016a_B.Saturation1 - rt_atan2f_snf(p, r)) *
    50.0F;

  /* Sum: '<S7>/Sum1' */
  px4_swashplateless_2016a_B.Sum1_o = (px4_swashplateless_2016a_B.Sum -
    px4_swashplateless_2016a_B.Gain1) + px4_swashplateless_2016a_B.Gain;

  /* DataTypeConversion: '<S7>/Data Type Conversion1' */
  r = (real32_T)floor(px4_swashplateless_2016a_B.Sum1_o);
  if (rtIsNaNF(r) || rtIsInfF(r)) {
    r = 0.0F;
  } else {
    r = (real32_T)fmod(r, 65536.0F);
  }

  /* Saturate: '<S7>/Output_Limits2' incorporates:
   *  DataTypeConversion: '<S7>/Data Type Conversion1'
   */
  u0_0 = (uint16_T)(r < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)-r :
                    (int32_T)(uint16_T)r);
  if (u0_0 > 2000) {
    px4_swashplateless_2016a_B.topMotor = 2000U;
  } else if (u0_0 < 1400) {
    px4_swashplateless_2016a_B.topMotor = 1400U;
  } else {
    px4_swashplateless_2016a_B.topMotor = u0_0;
  }

  /* End of Saturate: '<S7>/Output_Limits2' */

  /* Sum: '<S7>/Sum3' */
  px4_swashplateless_2016a_B.Sum3 = px4_swashplateless_2016a_B.Gain1 +
    px4_swashplateless_2016a_B.Sum;

  /* DataTypeConversion: '<S7>/Data Type Conversion4' */
  r = (real32_T)floor(px4_swashplateless_2016a_B.Sum3);
  if (rtIsNaNF(r) || rtIsInfF(r)) {
    r = 0.0F;
  } else {
    r = (real32_T)fmod(r, 65536.0F);
  }

  /* Saturate: '<S7>/Output_Limits1' incorporates:
   *  DataTypeConversion: '<S7>/Data Type Conversion4'
   */
  u0_0 = (uint16_T)(r < 0.0F ? (int32_T)(uint16_T)-(int16_T)(uint16_T)-r :
                    (int32_T)(uint16_T)r);
  if (u0_0 > 2000) {
    px4_swashplateless_2016a_B.bottomMotor = 2000U;
  } else if (u0_0 < 1400) {
    px4_swashplateless_2016a_B.bottomMotor = 1400U;
  } else {
    px4_swashplateless_2016a_B.bottomMotor = u0_0;
  }

  /* End of Saturate: '<S7>/Output_Limits1' */

  /* S-Function (sfun_px4_pwm): '<Root>/PWM_output' */
  if (px4_swashplateless_2016a_B.Compare == true) {
    if (g_pwm_enabled == false) {
      int rc;

      /* arm system */
      rc = ioctl(g_pwm_fd, PWM_SERVO_ARM, 0);
      if (rc != OK)
        err(1,"PWM_SERVO_ARM");
      else {
        ioctl(g_pwm_fd, PWM_SERVO_SET(0), 900);
        ioctl(g_pwm_fd, PWM_SERVO_SET(1), 900);
        g_pwm_enabled = true;
        printf("***ARMED*** PWM fd = %d\n", g_pwm_fd);
      }
    }
  } else {
    if (g_pwm_enabled == true) {
      int rc;

      /* disarm system if enabled */
      ioctl(g_pwm_fd, PWM_SERVO_SET(0), 900);
      ioctl(g_pwm_fd, PWM_SERVO_SET(1), 900);
      rc = ioctl(g_pwm_fd, PWM_SERVO_DISARM, 0);
      g_pwm_enabled = false;
      if (rc != OK)
        err(1, "PWM_SERVO_DISARM");
      else
        printf("***DISARMED*** PWM fd = %d\n", g_pwm_fd);
    }
  }

  if (g_pwm_enabled) {
    /* output the PWM signals */
    ioctl(g_pwm_fd, PWM_SERVO_SET(0), (unsigned int)
          px4_swashplateless_2016a_B.topMotor);
    ioctl(g_pwm_fd, PWM_SERVO_SET(1), (unsigned int)
          px4_swashplateless_2016a_B.bottomMotor);
  }

  /* S-Function (sdspstatminmax): '<S4>/Maximum' */
  if (px4_swashplateless_2016a_B.Saturation1 >
      px4_swashplateless_2016a_DW.Maximum_Valdata) {
    px4_swashplateless_2016a_DW.Maximum_Valdata =
      px4_swashplateless_2016a_B.Saturation1;
  }

  px4_swashplateless_2016a_B.Maximum =
    px4_swashplateless_2016a_DW.Maximum_Valdata;

  /* End of S-Function (sdspstatminmax): '<S4>/Maximum' */

  /* Update for UnitDelay: '<S11>/Unit Delay' */
  px4_swashplateless_2016a_DW.UnitDelay_DSTATE_o =
    px4_swashplateless_2016a_B.Compare;

  /* Update for UnitDelay: '<S13>/Unit Delay' */
  px4_swashplateless_2016a_DW.UnitDelay_DSTATE = rtb_Error;

  /* Update for UnitDelay: '<S12>/Unit Delay' */
  px4_swashplateless_2016a_DW.UnitDelay_DSTATE_k = rtb_Error_i;

  /* Update for DiscreteIntegrator: '<S15>/Integrator' incorporates:
   *  Gain: '<S15>/Integral Gain'
   *  Sum: '<S15>/SumI1'
   *  Sum: '<S15>/SumI2'
   */
  px4_swashplateless_2016a_DW.Integrator_DSTATE += (0.0F * rtb_Sum2 +
    (rtb_Saturate - rtb_Sum_b)) * 0.004F;

  /* External mode */
  rtExtModeUploadCheckTrigger(2);

  {                                    /* Sample time: [0.004s, 0.0s] */
    rtExtModeUpload(0, px4_swashplateless_2016a_M->Timing.taskTime0);
  }

  if (px4_swashplateless_2016a_M->Timing.TaskCounters.TID[1] == 0) {/* Sample time: [0.5s, 0.0s] */
    rtExtModeUpload(1, ((px4_swashplateless_2016a_M->Timing.clockTick1) * 0.5));
  }

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.004s, 0.0s] */
    if ((rtmGetTFinal(px4_swashplateless_2016a_M)!=-1) &&
        !((rtmGetTFinal(px4_swashplateless_2016a_M)-
           px4_swashplateless_2016a_M->Timing.taskTime0) >
          px4_swashplateless_2016a_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(px4_swashplateless_2016a_M, "Simulation finished");
    }

    if (rtmGetStopRequested(px4_swashplateless_2016a_M)) {
      rtmSetErrorStatus(px4_swashplateless_2016a_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  px4_swashplateless_2016a_M->Timing.taskTime0 =
    (++px4_swashplateless_2016a_M->Timing.clockTick0) *
    px4_swashplateless_2016a_M->Timing.stepSize0;
  if (px4_swashplateless_2016a_M->Timing.TaskCounters.TID[1] == 0) {
    /* Update absolute timer for sample time: [0.5s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.5, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    px4_swashplateless_2016a_M->Timing.clockTick1++;
  }

  rate_scheduler();
}

/* Model initialize function */
void px4_swashplateless_2016a_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)px4_swashplateless_2016a_M, 0,
                sizeof(RT_MODEL_px4_swashplateless_2_T));
  rtmSetTFinal(px4_swashplateless_2016a_M, -1);
  px4_swashplateless_2016a_M->Timing.stepSize0 = 0.004;

  /* External mode info */
  px4_swashplateless_2016a_M->Sizes.checksums[0] = (43815335U);
  px4_swashplateless_2016a_M->Sizes.checksums[1] = (1772582824U);
  px4_swashplateless_2016a_M->Sizes.checksums[2] = (962582738U);
  px4_swashplateless_2016a_M->Sizes.checksums[3] = (1012703495U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[3];
    px4_swashplateless_2016a_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(px4_swashplateless_2016a_M->extModeInfo,
      &px4_swashplateless_2016a_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(px4_swashplateless_2016a_M->extModeInfo,
                        px4_swashplateless_2016a_M->Sizes.checksums);
    rteiSetTPtr(px4_swashplateless_2016a_M->extModeInfo, rtmGetTPtr
                (px4_swashplateless_2016a_M));
  }

  /* block I/O */
  (void) memset(((void *) &px4_swashplateless_2016a_B), 0,
                sizeof(B_px4_swashplateless_2016a_T));

  {
    px4_swashplateless_2016a_B.Switch = MODE_OFF;
    px4_swashplateless_2016a_B.Switch1 = COLOR_OFF;
    px4_swashplateless_2016a_B.Switch1_k = STOP_TUNE;
  }

  /* states (dwork) */
  (void) memset((void *)&px4_swashplateless_2016a_DW, 0,
                sizeof(DW_px4_swashplateless_2016a_T));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    px4_swashplateless_2016a_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 35;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;
  }

  {
    int32_T fd_adc_input;

    /* Start for S-Function (sfun_px4_input_rc): '<Root>/input_rc' */
    {
      /* S-Function Block: <Root>/input_rc */
      /* subscribe to PWM RC input topic */
      int fd = orb_subscribe(ORB_ID(input_rc));
      px4_swashplateless_2016a_DW.input_rc_input_rc_fd.fd = fd;
      px4_swashplateless_2016a_DW.input_rc_input_rc_fd.events = POLLIN;
      orb_set_interval(fd, 1);
      warnx("* Subscribed to input_rc topic (fd = %d)*\n", fd);
    }

    /* Start for S-Function (sfun_px4_rgbled): '<S2>/RGB_LED' */
    {
      // enable RGBLED, set intitial mode and color
      // more devices will be 1, 2, etc
      // #define RGBLED0_DEVICE_PATH "/dev/rgbled0"
      //px4_swashplateless_2016a_DW.RGB_LED_rgbled_fd = open("/dev/rgbled0", 0);
      px4_swashplateless_2016a_DW.RGB_LED_rgbled_fd = open(RGBLED0_DEVICE_PATH,
        0);
      ioctl(px4_swashplateless_2016a_DW.RGB_LED_rgbled_fd, RGBLED_SET_MODE,
            RGBLED_MODE_OFF);
      ioctl(px4_swashplateless_2016a_DW.RGB_LED_rgbled_fd, RGBLED_SET_COLOR,
            RGBLED_COLOR_OFF);
    }

    /* Start for S-Function (sfun_px4_tune): '<S2>/Speaker_Tune' */

    // enable tune driver
    // #define TONEALARM_DEVICE_PATH "/dev/tone_alarm"
    px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd = open
      (TONEALARM0_DEVICE_PATH, O_WRONLY);
    ioctl(px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd, TONE_SET_ALARM,
          TONE_STARTUP_TUNE);
    close(px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd);// close here since this may be used in another thread
    px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd = -1;

    /* Start for S-Function (sfun_px4_vehicle_attitude): '<Root>/vehicle_attitude' */
    {
      /* S-Function Block: <Root>/vehicle_attitude */
      /* subscribe to PWM RC input topic */
      int fd = orb_subscribe(ORB_ID(vehicle_attitude));
      px4_swashplateless_2016a_DW.vehicle_attitude_vehicle_attitu.fd = fd;
      px4_swashplateless_2016a_DW.vehicle_attitude_vehicle_attitu.events =
        POLLIN;
      orb_set_interval(fd, 1);
      warnx("* Subscribed to vehicle_attitude topic (fd = %d)*\n", fd);
    }

    /* Start for S-Function (sfun_px4_sensor_combined): '<Root>/sensor_combined1' */
    {
      /* S-Function Block: <Root>/sensor_combined1 */
      /* subscribe to sensor_combined topic */
      int fd = orb_subscribe(ORB_ID(sensor_combined));
      px4_swashplateless_2016a_DW.sensor_combined1_sensor_fd.fd = fd;
      px4_swashplateless_2016a_DW.sensor_combined1_sensor_fd.events = POLLIN;
      orb_set_interval(fd, 1);
      warnx("* Subscribed to sensor_combined topic (fd = %d)*\n", fd);
    }

    /* Start for MATLABSystem: '<S4>/Read Encoder' */
    px4_swashplateless_2016a_DW.obj.isInitialized = 0;
    px4_swashplateless_2016a_DW.obj.SampleTime = -1.0;
    px4_swashplateless_2016a_DW.obj.isInitialized = 1;
    MW_openADC(&fd_adc_input);
    px4_swashplateless_2016a_DW.obj.fd_adc = fd_adc_input;

    /* Start for S-Function (sfun_px4_pwm): '<Root>/PWM_output' */
    {
      int rc;
      int pwm_rate = 400;              /* default PWM Rate is 400Hz */
      int chMask = 0x00;               /* change channel mask based on which are used */

      /* channel group 0 */
      chMask |= 0x03;

      /* enable pwm outputs, set to disarm  */
      g_pwm_fd = open(g_pwm_device, 0);
      printf("OPEN PWM fd = %d\n", g_pwm_fd);
      rc = ioctl(g_pwm_fd, PWM_SERVO_SET_UPDATE_RATE, pwm_rate);
      if (rc != OK)
        err(1, "PWM_SERVO_SET_UPDATE_RATE");
      if (chMask > 0) {
        rc = ioctl(g_pwm_fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, chMask);
        if (rc != OK) {
          err(1, "PWM_SERVO_SET_SELECT_UPDATE_RATE");
        }

        printf("Set SERVO Rate (%dHz) Channel Mask:0x%08X\n", pwm_rate, chMask);
      }

      /* tell safety that its ok to disable it with the switch */
      rc = ioctl(g_pwm_fd, PWM_SERVO_SET_ARM_OK, 0);
      if (rc != OK)
        err(1, "PWM_SERVO_SET_ARM_OK");
      rc = ioctl(g_pwm_fd, PWM_SERVO_DISARM, 0);
      if (rc != OK)
        err(1, "PWM_SERVO_DISARM");
      g_pwm_enabled = false;
    }

    /* InitializeConditions for S-Function (sdspstatminmax): '<S4>/Maximum' */
    px4_swashplateless_2016a_DW.Maximum_Valdata = (rtMinusInf);
  }
}

/* Model terminate function */
void px4_swashplateless_2016a_terminate(void)
{
  /* Close uORB service used in the S-Function Block: <Root>/input_rc */
  close(px4_swashplateless_2016a_DW.input_rc_input_rc_fd.fd);

  /* Terminate for S-Function (sfun_px4_rgbled): '<S2>/RGB_LED' */
  ioctl(px4_swashplateless_2016a_DW.RGB_LED_rgbled_fd, RGBLED_SET_COLOR,
        RGBLED_COLOR_OFF);
  ioctl(px4_swashplateless_2016a_DW.RGB_LED_rgbled_fd, RGBLED_SET_MODE,
        RGBLED_MODE_OFF);
  usleep(100);

  /* Close uORB service used in the S-Function Block: <S2>/RGB_LED */
  close(px4_swashplateless_2016a_DW.RGB_LED_rgbled_fd);

  /* Terminate for S-Function (sfun_px4_tune): '<S2>/Speaker_Tune' */
  if (px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd != -1) {
    close(px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd);// this may be open in a different thread. close first.
    px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd = open
      (TONEALARM0_DEVICE_PATH, O_WRONLY);// re-open
    ioctl(px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd, TONE_SET_ALARM,
          TONE_STOP_TUNE);

    /* Close service used in the S-Function Block: <S2>/Speaker_Tune */
    close(px4_swashplateless_2016a_DW.Speaker_Tune_tune_fd);// finally close device here
  }

  /* Close uORB service used in the S-Function Block: <Root>/vehicle_attitude */
  close(px4_swashplateless_2016a_DW.vehicle_attitude_vehicle_attitu.fd);

  /* Close uORB service used in the S-Function Block: <Root>/sensor_combined1 */
  close(px4_swashplateless_2016a_DW.sensor_combined1_sensor_fd.fd);

  /* Start for MATLABSystem: '<S4>/Read Encoder' incorporates:
   *  Terminate for MATLABSystem: '<S4>/Read Encoder'
   */
  if (px4_swashplateless_2016a_DW.obj.isInitialized == 1) {
    px4_swashplateless_2016a_DW.obj.isInitialized = 2;
    close(px4_swashplateless_2016a_DW.obj.fd_adc);
  }

  /* End of Start for MATLABSystem: '<S4>/Read Encoder' */
  /* Terminate for S-Function (sfun_px4_pwm): '<Root>/PWM_output' */
  /* disable pwm outputs */
  ioctl(g_pwm_fd, PWM_SERVO_SET(0), 900);
  ioctl(g_pwm_fd, PWM_SERVO_SET(1), 900);
  ioctl(g_pwm_fd, PWM_SERVO_DISARM, 0);
  g_pwm_enabled = false;

  /* Close handle used in the S-Function Block: <Root>/PWM_output */
  close(g_pwm_fd);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
