//
// File: HAC_swingUp_controller.cpp
//
// Code generated for Simulink model 'HAC_swingUp_controller'.
//
// Model version                  : 5.0
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jul 28 18:26:28 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "HAC_swingUp_controller.h"
#include <math.h>
#include "rtwtypes.h"
#include <stddef.h>
#define NumBitsPerChar                 8U

static real_T look1_binlx(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);
extern "C"
{
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}

extern "C"
{
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetNaN(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T nan = 0.0;
    if (bitsPerReal == 32U) {
      nan = rtGetNaNF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
    }

    return nan;
  }

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetNaNF(void)
  {
    IEEESingle nanF = { { 0.0F } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}

extern "C"
{
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  static void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    boolean_T result = (boolean_T) 0;
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    if (bitsPerReal == 32U) {
      result = rtIsNaNF((real32_T)value);
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.fltVal = value;
      result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                           0x7FF00000 &&
                           ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                            (tmpVal.bitVal.words.wordL != 0) ));
    }

    return result;
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    IEEESingle tmp;
    tmp.wordL.wordLreal = value;
    return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                       (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
  }
}

extern "C"
{
  //
  // Initialize rtInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T inf = 0.0;
    if (bitsPerReal == 32U) {
      inf = rtGetInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      inf = tmpVal.fltVal;
    }

    return inf;
  }

  //
  // Initialize rtInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetMinusInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T minf = 0.0;
    if (bitsPerReal == 32U) {
      minf = rtGetMinusInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      minf = tmpVal.fltVal;
    }

    return minf;
  }

  //
  // Initialize rtMinusInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetMinusInfF(void)
  {
    IEEESingle minfF;
    minfF.wordL.wordLuint = 0xFF800000U;
    return minfF.wordL.wordLreal;
  }
}

static real_T look1_binlx(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T iLeft;

  // Column-major Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear point-slope'
  // Extrapolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    // Binary Search
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  // Column-major Interpolation 1-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'wrapping'

  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

// Model step function
void HAC_swingUp_controller::step()
{
  real_T rtb_Abs;
  real_T rtb_W_idx_1;
  real_T rtb_w1_2;
  real_T rtb_w4;
  real_T tmp;

  // If: '<Root>/If' incorporates:
  //   Abs: '<Root>/Abs'
  //   Inport: '<Root>/X'

  if (fabs(rtU.X_f[2]) < 0.43633231299858238) {
    // Outputs for IfAction SubSystem: '<Root>/HAC' incorporates:
    //   ActionPort: '<S1>/Action Port'

    // Abs: '<S1>/Abs' incorporates:
    //   Inport: '<Root>/error'

    rtb_Abs = fabs(rtU.error[2]);

    // If: '<S1>/If'
    if (rtb_Abs <= 0.087266462599716474) {
      // Outputs for IfAction SubSystem: '<S1>/If Action Subsystem' incorporates:
      //   ActionPort: '<S3>/Action Port'

      // SignalConversion generated from: '<S3>/W' incorporates:
      //   Constant: '<S3>/weight'
      //   Merge: '<S1>/Merge'

      rtb_w1_2 = 0.25;
      rtb_W_idx_1 = 0.25;
      rtb_Abs = 0.25;
      rtb_w4 = 0.25;

      // End of Outputs for SubSystem: '<S1>/If Action Subsystem'
    } else if (rtb_Abs >= 0.87266462599716477) {
      // Outputs for IfAction SubSystem: '<S1>/If Action Subsystem1' incorporates:
      //   ActionPort: '<S4>/Action Port'

      // SignalConversion generated from: '<S4>/W' incorporates:
      //   Constant: '<S4>/weights'
      //   Merge: '<S1>/Merge'

      rtb_w1_2 = 0.0;
      rtb_W_idx_1 = 0.0;
      rtb_Abs = 1.0;
      rtb_w4 = 0.0;

      // End of Outputs for SubSystem: '<S1>/If Action Subsystem1'
    } else {
      // Outputs for IfAction SubSystem: '<S1>/If Action Subsystem2' incorporates:
      //   ActionPort: '<S5>/Action Port'

      // Bias: '<S5>/Bias1' incorporates:
      //   Bias: '<S5>/Bias'
      //   Gain: '<S5>/Multiply'

      rtb_Abs = (rtb_Abs - 0.087266462599716474) * 0.954929658551372 + 0.25;

      // Gain: '<S5>/Gain' incorporates:
      //   Constant: '<S5>/Constant'
      //   Sum: '<S5>/Subtract'

      rtb_w4 = (1.0 - rtb_Abs) * 0.5;

      // Gain: '<S5>/Gain1' incorporates:
      //   Constant: '<S5>/Constant'
      //   Sum: '<S5>/Subtract1'

      rtb_w1_2 = ((1.0 - rtb_w4) - rtb_Abs) * 0.5;

      // SignalConversion generated from: '<S5>/W' incorporates:
      //   Merge: '<S1>/Merge'

      rtb_W_idx_1 = rtb_w1_2;

      // End of Outputs for SubSystem: '<S1>/If Action Subsystem2'
    }

    // End of If: '<S1>/If'

    // DotProduct: '<S1>/Dot Product' incorporates:
    //   Bias: '<S10>/Bias'
    //   Bias: '<S11>/Bias'
    //   Bias: '<S1>/Bias'
    //   Bias: '<S1>/Bias1'
    //   Bias: '<S8>/Bias'
    //   Bias: '<S9>/Bias'
    //   Gain: '<S10>/Gain'
    //   Gain: '<S11>/Gain'
    //   Gain: '<S1>/Gain'
    //   Gain: '<S1>/Gain1'
    //   Gain: '<S8>/Gain'
    //   Gain: '<S9>/Gain'
    //   Inport: '<Root>/error'
    //   Lookup_n-D: '<S1>/q lookup'
    //   Lookup_n-D: '<S1>/qd lookup'
    //   Lookup_n-D: '<S1>/x lookup'
    //   Lookup_n-D: '<S1>/xd lookup'
    //   MATLAB Function: '<S6>/MATLAB Function'
    //   MATLAB Function: '<S7>/MATLAB Function'
    //   Merge: '<S1>/Merge'

    rtb_Abs = (((look1_binlx((rtU.error[0] + 0.43) * 1.1627906976744187,
      rtConstP.pooled4, rtConstP.xlookup_tableData, 6U) * 58.8399 - 29.41995) *
                rtb_w1_2 + (look1_binlx((rtU.error[1] + 2.0) * 0.25,
      rtConstP.pooled7, rtConstP.xdlookup_tableData, 4U) * 58.8399 - 29.41995) *
                rtb_W_idx_1) + (look1_binlx(1.0 / (exp(-8.0 * rtU.error[2]) +
      1.0), rtConstP.pooled7, rtConstP.qlookup_tableData, 4U) * 58.8399 -
                29.41995) * rtb_Abs) + (look1_binlx(1.0 / (exp(-0.45 *
      rtU.error[3]) + 1.0), rtConstP.pooled4, rtConstP.qdlookup_tableData, 6U) *
      58.8399 - 29.41995) * rtb_w4;

    // End of Outputs for SubSystem: '<Root>/HAC'
  } else {
    // Outputs for IfAction SubSystem: '<Root>/energy swing up' incorporates:
    //   ActionPort: '<S2>/Action Port'

    // Trigonometry: '<S18>/Cos' incorporates:
    //   Trigonometry: '<S14>/Cos'

    rtb_w4 = cos(rtU.X_f[2]);

    // Sum: '<S18>/Sum' incorporates:
    //   Gain: '<S18>/Gain'
    //   Gain: '<S18>/Gain1'
    //   Math: '<S18>/Square'
    //   Trigonometry: '<S18>/Cos'

    rtb_Abs = rtU.X_f[3] * rtU.X_f[3] * 0.0017479050000000001 + 0.1714109256825 *
      rtb_w4;

    // Product: '<S14>/Product'
    rtb_w4 *= rtU.X_f[3];

    // Signum: '<S14>/Sign'
    if (rtIsNaN(rtb_w4)) {
      // Signum: '<S15>/Sign'
      rtb_w4 = (rtNaN);
    } else if (rtb_w4 < 0.0) {
      // Signum: '<S15>/Sign'
      rtb_w4 = -1.0;
    } else {
      // Signum: '<S15>/Sign'
      rtb_w4 = (rtb_w4 > 0.0);
    }

    // Signum: '<S16>/Sign'
    if (rtIsNaN(rtU.X_f[0])) {
      rtb_w1_2 = (rtNaN);
    } else if (rtU.X_f[0] < 0.0) {
      rtb_w1_2 = -1.0;
    } else {
      rtb_w1_2 = (rtU.X_f[0] > 0.0);
    }

    // Signum: '<S17>/Sign'
    if (rtIsNaN(rtU.X_f[1])) {
      rtb_W_idx_1 = (rtNaN);
    } else if (rtU.X_f[1] < 0.0) {
      rtb_W_idx_1 = -1.0;
    } else {
      rtb_W_idx_1 = (rtU.X_f[1] > 0.0);
    }

    // Signum: '<S15>/Sign1' incorporates:
    //   Constant: '<S15>/Constant'
    //   Sum: '<S15>/Sum'

    if (rtIsNaN(rtb_Abs - 0.1714109256825)) {
      tmp = (rtNaN);
    } else if (rtb_Abs - 0.1714109256825 < 0.0) {
      tmp = -1.0;
    } else {
      tmp = (rtb_Abs - 0.1714109256825 > 0.0);
    }

    // Sum: '<S2>/Sum' incorporates:
    //   Abs: '<S15>/Abs'
    //   Abs: '<S16>/Abs'
    //   Abs: '<S17>/Abs'
    //   Constant: '<S15>/Constant1'
    //   Constant: '<S16>/Constant'
    //   Constant: '<S17>/Constant'
    //   Gain: '<S14>/minus_k_su'
    //   Gain: '<S15>/k_em'
    //   Gain: '<S16>/Gain1'
    //   Gain: '<S16>/k_cw'
    //   Gain: '<S17>/Gain1'
    //   Gain: '<S17>/k_vw'
    //   Math: '<S15>/Exp'
    //   Math: '<S16>/Log'
    //   Math: '<S17>/Log'
    //   Product: '<S15>/Product1'
    //   Product: '<S16>/Product'
    //   Product: '<S17>/Product'
    //   Signum: '<S14>/Sign'
    //   Signum: '<S15>/Sign1'
    //   Signum: '<S16>/Sign'
    //   Signum: '<S17>/Sign'
    //   Sum: '<S15>/Subtract'
    //   Sum: '<S15>/Sum1'
    //   Sum: '<S16>/Sum'
    //   Sum: '<S17>/Sum'
    //
    //  About '<S15>/Exp':
    //   Operator: exp
    //
    //  About '<S16>/Log':
    //   Operator: log
    //
    //  About '<S17>/Log':
    //   Operator: log

    rtb_Abs = ((log(1.0 - 2.3255813953488373 * fabs(rtU.X_f[0])) * rtb_w1_2 *
                6.0 + log(1.0 - 0.5 * fabs(rtU.X_f[1])) * rtb_W_idx_1 * 2.5) +
               -2.0 * rtb_w4) + (exp(fabs(rtb_Abs - 0.22283420338725)) - 1.0) *
      (rtb_w4 * tmp) * 10.0;

    // End of Outputs for SubSystem: '<Root>/energy swing up'
  }

  // End of If: '<Root>/If'

  // Saturate: '<Root>/Saturation'
  if (rtb_Abs > 29.41995) {
    // Outport: '<Root>/u'
    rtY.u = 29.41995;
  } else if (rtb_Abs < -29.41995) {
    // Outport: '<Root>/u'
    rtY.u = -29.41995;
  } else {
    // Outport: '<Root>/u'
    rtY.u = rtb_Abs;
  }

  // End of Saturate: '<Root>/Saturation'
}

// Model initialize function
void HAC_swingUp_controller::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));
}

// Constructor
HAC_swingUp_controller::HAC_swingUp_controller():
  rtU(),
  rtY()
{
  // Currently there is no constructor body generated.
}

// Destructor
HAC_swingUp_controller::~HAC_swingUp_controller()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//
