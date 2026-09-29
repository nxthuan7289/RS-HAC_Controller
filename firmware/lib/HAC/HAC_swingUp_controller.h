//
// File: HAC_swingUp_controller.h
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
#ifndef HAC_swingUp_controller_h_
#define HAC_swingUp_controller_h_
#include "rtwtypes.h"
#include <stddef.h>

extern "C"
{
  static real_T rtGetNaN(void);
  static real32_T rtGetNaNF(void);
}                                      // extern "C"

#define NOT_USING_NONFINITE_LITERALS   1

extern "C"
{
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  static void rt_InitInfAndNaN(size_t realSize);
  static boolean_T rtIsInf(real_T value);
  static boolean_T rtIsInfF(real32_T value);
  static boolean_T rtIsNaN(real_T value);
  static boolean_T rtIsNaNF(real32_T value);
  struct BigEndianIEEEDouble {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  };

  struct LittleEndianIEEEDouble {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  };

  struct IEEESingle {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  };
}                                      // extern "C"

extern "C"
{
  static real_T rtGetInf(void);
  static real32_T rtGetInfF(void);
  static real_T rtGetMinusInf(void);
  static real32_T rtGetMinusInfF(void);
}                                      // extern "C"

// Class declaration for model HAC_swingUp_controller
class HAC_swingUp_controller
{
  // public data and function members
 public:
  // Constant parameters (default storage)
  struct ConstP {
    // Expression: u_x_sqm
    //  Referenced by: '<S1>/x lookup'

    real_T xlookup_tableData[7];

    // Pooled Parameter (Mixed Expressions)
    //  Referenced by:
    //    '<S1>/qd lookup'
    //    '<S1>/x lookup'

    real_T pooled4[7];

    // Expression: u_xd_sqm
    //  Referenced by: '<S1>/xd lookup'

    real_T xdlookup_tableData[5];

    // Pooled Parameter (Mixed Expressions)
    //  Referenced by:
    //    '<S1>/q lookup'
    //    '<S1>/xd lookup'

    real_T pooled7[5];

    // Expression: u_q_sqm
    //  Referenced by: '<S1>/q lookup'

    real_T qlookup_tableData[5];

    // Expression: u_qd_sqm
    //  Referenced by: '<S1>/qd lookup'

    real_T qdlookup_tableData[7];
  };

  // External inputs (root inport signals with default storage)
  struct ExtU {
    real_T X_f[4];                     // '<Root>/X'
    real_T error[4];                   // '<Root>/error'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY {
    real_T u;                          // '<Root>/u'
  };

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  HAC_swingUp_controller();

  // Destructor
  ~HAC_swingUp_controller();
};

// Constant parameters (default storage)
extern const HAC_swingUp_controller::ConstP rtConstP;

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'HAC_swingUp_controller'
//  '<S1>'   : 'HAC_swingUp_controller/HAC'
//  '<S2>'   : 'HAC_swingUp_controller/energy swing up'
//  '<S3>'   : 'HAC_swingUp_controller/HAC/If Action Subsystem'
//  '<S4>'   : 'HAC_swingUp_controller/HAC/If Action Subsystem1'
//  '<S5>'   : 'HAC_swingUp_controller/HAC/If Action Subsystem2'
//  '<S6>'   : 'HAC_swingUp_controller/HAC/Sigmoidal MF'
//  '<S7>'   : 'HAC_swingUp_controller/HAC/Sigmoidal MF1'
//  '<S8>'   : 'HAC_swingUp_controller/HAC/denorm_u'
//  '<S9>'   : 'HAC_swingUp_controller/HAC/denorm_u1'
//  '<S10>'  : 'HAC_swingUp_controller/HAC/denorm_u2'
//  '<S11>'  : 'HAC_swingUp_controller/HAC/denorm_u3'
//  '<S12>'  : 'HAC_swingUp_controller/HAC/Sigmoidal MF/MATLAB Function'
//  '<S13>'  : 'HAC_swingUp_controller/HAC/Sigmoidal MF1/MATLAB Function'
//  '<S14>'  : 'HAC_swingUp_controller/energy swing up/energy injection'
//  '<S15>'  : 'HAC_swingUp_controller/energy swing up/energy maintenance'
//  '<S16>'  : 'HAC_swingUp_controller/energy swing up/position potential well'
//  '<S17>'  : 'HAC_swingUp_controller/energy swing up/velocity potential well'
//  '<S18>'  : 'HAC_swingUp_controller/energy swing up/energy maintenance/Pend energy'

#endif                                 // HAC_swingUp_controller_h_

//
// File trailer for generated code.
//
// [EOF]
//
