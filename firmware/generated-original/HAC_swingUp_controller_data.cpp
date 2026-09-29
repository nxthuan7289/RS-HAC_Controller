//
// File: HAC_swingUp_controller_data.cpp
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

// Constant parameters (default storage)
const HAC_swingUp_controller::ConstP rtConstP = {
  // Expression: u_x_sqm
  //  Referenced by: '<S1>/x lookup'

  { 0.325, 0.43875000000000003, 0.4785625, 0.5, 0.5214375, 0.56125, 0.675 },

  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S1>/qd lookup'
  //    '<S1>/x lookup'

  { 0.25, 0.375, 0.4375, 0.5, 0.5625, 0.625, 0.75 },

  // Expression: u_xd_sqm
  //  Referenced by: '<S1>/xd lookup'

  { 0.099999999999999978, 0.17999999999999994, 0.5, 0.82000000000000006, 0.9 },

  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S1>/q lookup'
  //    '<S1>/xd lookup'

  { 0.25, 0.375, 0.5, 0.625, 0.75 },

  // Expression: u_q_sqm
  //  Referenced by: '<S1>/q lookup'

  { 0.1375, 0.2371875, 0.5, 0.7628125, 0.8625 },

  // Expression: u_qd_sqm
  //  Referenced by: '<S1>/qd lookup'

  { 0.099999999999999978, 0.17999999999999994, 0.24399999999999994, 0.5, 0.756,
    0.82000000000000006, 0.9 }
};

//
// File trailer for generated code.
//
// [EOF]
//
