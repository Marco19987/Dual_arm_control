//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 11-Apr-2025 10:25:57
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "jacobian_WRh_to_oTg1_oTg2.h"
#include "jacobian_WRh_to_oTg1_oTg2_terminate.h"
#include "jacobian_h_to_b2Tb1.h"
#include "jacobian_h_to_oTg1_oTg2.h"
#include "jacobian_h_to_x_state_not_ext.h"
#include "rt_nonfinite.h"

// Function Declarations
static void argInit_3x1_real_T(double result[3]);

static void argInit_4x1_real_T(double result[4]);

static void argInit_6x1_real_T(double result[6]);

static double argInit_real_T();

// Function Definitions
//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_3x1_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[4]
// Return Type  : void
//
static void argInit_4x1_real_T(double result[4])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 4; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[6]
// Return Type  : void
//
static void argInit_6x1_real_T(double result[6])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 6; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_jacobian_h_to_b2Tb1();
  main_jacobian_h_to_oTg1_oTg2();
  main_jacobian_h_to_x_state_not_ext();
  main_jacobian_WRh_to_oTg1_oTg2();
  // Terminate the application.
  // You do not need to do this more than one time.
  jacobian_WRh_to_oTg1_oTg2_terminate();
  return 0;
}

//
// Arguments    : void
// Return Type  : void
//
void main_jacobian_WRh_to_oTg1_oTg2()
{
  double b_jacobian_WRh_to_oTg1_oTg2[84];
  double in21_tmp[6];
  double in2_tmp[4];
  double in1_tmp[3];
  // Initialize function 'jacobian_WRh_to_oTg1_oTg2' input arguments.
  // Initialize function input argument 'in1'.
  argInit_3x1_real_T(in1_tmp);
  // Initialize function input argument 'in2'.
  argInit_4x1_real_T(in2_tmp);
  // Initialize function input argument 'in3'.
  // Initialize function input argument 'in4'.
  // Initialize function input argument 'in5'.
  // Initialize function input argument 'in6'.
  // Initialize function input argument 'in7'.
  // Initialize function input argument 'in8'.
  // Initialize function input argument 'in9'.
  // Initialize function input argument 'in10'.
  // Initialize function input argument 'in11'.
  // Initialize function input argument 'in12'.
  // Initialize function input argument 'in13'.
  // Initialize function input argument 'in14'.
  // Initialize function input argument 'in15'.
  // Initialize function input argument 'in16'.
  // Initialize function input argument 'in17'.
  // Initialize function input argument 'in18'.
  // Initialize function input argument 'in19'.
  // Initialize function input argument 'in20'.
  // Initialize function input argument 'in21'.
  argInit_6x1_real_T(in21_tmp);
  // Initialize function input argument 'in22'.
  // Initialize function input argument 'in23'.
  // Initialize function input argument 'in24'.
  // Call the entry-point 'jacobian_WRh_to_oTg1_oTg2'.
  jacobian_WRh_to_oTg1_oTg2(
      in1_tmp, in2_tmp, in1_tmp, in1_tmp, in1_tmp, in2_tmp, in1_tmp, in2_tmp,
      in1_tmp, in2_tmp, in1_tmp, in1_tmp, in1_tmp, in1_tmp, in1_tmp, in2_tmp,
      in1_tmp, in2_tmp, in1_tmp, in2_tmp, in21_tmp, in21_tmp, in21_tmp,
      in21_tmp, b_jacobian_WRh_to_oTg1_oTg2);
}

//
// Arguments    : void
// Return Type  : void
//
void main_jacobian_h_to_b2Tb1()
{
  double b_jacobian_h_to_b2Tb1[84];
  double in21_tmp[6];
  double in2_tmp[4];
  double in1_tmp[3];
  // Initialize function 'jacobian_h_to_b2Tb1' input arguments.
  // Initialize function input argument 'in1'.
  argInit_3x1_real_T(in1_tmp);
  // Initialize function input argument 'in2'.
  argInit_4x1_real_T(in2_tmp);
  // Initialize function input argument 'in3'.
  // Initialize function input argument 'in4'.
  // Initialize function input argument 'in5'.
  // Initialize function input argument 'in6'.
  // Initialize function input argument 'in7'.
  // Initialize function input argument 'in8'.
  // Initialize function input argument 'in9'.
  // Initialize function input argument 'in10'.
  // Initialize function input argument 'in11'.
  // Initialize function input argument 'in12'.
  // Initialize function input argument 'in13'.
  // Initialize function input argument 'in14'.
  // Initialize function input argument 'in15'.
  // Initialize function input argument 'in16'.
  // Initialize function input argument 'in17'.
  // Initialize function input argument 'in18'.
  // Initialize function input argument 'in19'.
  // Initialize function input argument 'in20'.
  // Initialize function input argument 'in21'.
  argInit_6x1_real_T(in21_tmp);
  // Initialize function input argument 'in22'.
  // Initialize function input argument 'in23'.
  // Initialize function input argument 'in24'.
  // Call the entry-point 'jacobian_h_to_b2Tb1'.
  jacobian_h_to_b2Tb1(in1_tmp, in2_tmp, in1_tmp, in1_tmp, in1_tmp, in2_tmp,
                      in1_tmp, in2_tmp, in1_tmp, in2_tmp, in1_tmp, in1_tmp,
                      in1_tmp, in1_tmp, in1_tmp, in2_tmp, in1_tmp, in2_tmp,
                      in1_tmp, in2_tmp, in21_tmp, in21_tmp, in21_tmp, in21_tmp,
                      b_jacobian_h_to_b2Tb1);
}

//
// Arguments    : void
// Return Type  : void
//
void main_jacobian_h_to_oTg1_oTg2()
{
  double b_jacobian_h_to_oTg1_oTg2[168];
  double in21_tmp[6];
  double in2_tmp[4];
  double in1_tmp[3];
  // Initialize function 'jacobian_h_to_oTg1_oTg2' input arguments.
  // Initialize function input argument 'in1'.
  argInit_3x1_real_T(in1_tmp);
  // Initialize function input argument 'in2'.
  argInit_4x1_real_T(in2_tmp);
  // Initialize function input argument 'in3'.
  // Initialize function input argument 'in4'.
  // Initialize function input argument 'in5'.
  // Initialize function input argument 'in6'.
  // Initialize function input argument 'in7'.
  // Initialize function input argument 'in8'.
  // Initialize function input argument 'in9'.
  // Initialize function input argument 'in10'.
  // Initialize function input argument 'in11'.
  // Initialize function input argument 'in12'.
  // Initialize function input argument 'in13'.
  // Initialize function input argument 'in14'.
  // Initialize function input argument 'in15'.
  // Initialize function input argument 'in16'.
  // Initialize function input argument 'in17'.
  // Initialize function input argument 'in18'.
  // Initialize function input argument 'in19'.
  // Initialize function input argument 'in20'.
  // Initialize function input argument 'in21'.
  argInit_6x1_real_T(in21_tmp);
  // Initialize function input argument 'in22'.
  // Initialize function input argument 'in23'.
  // Initialize function input argument 'in24'.
  // Call the entry-point 'jacobian_h_to_oTg1_oTg2'.
  jacobian_h_to_oTg1_oTg2(in1_tmp, in2_tmp, in1_tmp, in1_tmp, in1_tmp, in2_tmp,
                          in1_tmp, in2_tmp, in1_tmp, in2_tmp, in1_tmp, in1_tmp,
                          in1_tmp, in1_tmp, in1_tmp, in2_tmp, in1_tmp, in2_tmp,
                          in1_tmp, in2_tmp, in21_tmp, in21_tmp, in21_tmp,
                          in21_tmp, b_jacobian_h_to_oTg1_oTg2);
}

//
// Arguments    : void
// Return Type  : void
//
void main_jacobian_h_to_x_state_not_ext()
{
  double b_jacobian_h_to_x_state_not_ext[324];
  double in21_tmp[6];
  double in2_tmp[4];
  double in1_tmp[3];
  // Initialize function 'jacobian_h_to_x_state_not_ext' input arguments.
  // Initialize function input argument 'in1'.
  argInit_3x1_real_T(in1_tmp);
  // Initialize function input argument 'in2'.
  argInit_4x1_real_T(in2_tmp);
  // Initialize function input argument 'in3'.
  // Initialize function input argument 'in4'.
  // Initialize function input argument 'in5'.
  // Initialize function input argument 'in6'.
  // Initialize function input argument 'in7'.
  // Initialize function input argument 'in8'.
  // Initialize function input argument 'in9'.
  // Initialize function input argument 'in10'.
  // Initialize function input argument 'in11'.
  // Initialize function input argument 'in12'.
  // Initialize function input argument 'in13'.
  // Initialize function input argument 'in14'.
  // Initialize function input argument 'in15'.
  // Initialize function input argument 'in16'.
  // Initialize function input argument 'in17'.
  // Initialize function input argument 'in18'.
  // Initialize function input argument 'in19'.
  // Initialize function input argument 'in20'.
  // Initialize function input argument 'in21'.
  argInit_6x1_real_T(in21_tmp);
  // Initialize function input argument 'in22'.
  // Initialize function input argument 'in23'.
  // Initialize function input argument 'in24'.
  // Call the entry-point 'jacobian_h_to_x_state_not_ext'.
  jacobian_h_to_x_state_not_ext(
      in1_tmp, in2_tmp, in1_tmp, in1_tmp, in1_tmp, in2_tmp, in1_tmp, in2_tmp,
      in1_tmp, in2_tmp, in1_tmp, in1_tmp, in1_tmp, in1_tmp, in1_tmp, in2_tmp,
      in1_tmp, in2_tmp, in1_tmp, in2_tmp, in21_tmp, in21_tmp, in21_tmp,
      in21_tmp, b_jacobian_h_to_x_state_not_ext);
}

//
// File trailer for main.cpp
//
// [EOF]
//
