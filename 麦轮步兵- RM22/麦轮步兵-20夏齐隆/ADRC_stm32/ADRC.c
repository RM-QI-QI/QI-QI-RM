/*
 * File: ADRC.c
 *
 * Code generated for Simulink model :ADRC.
 *
 * Model version      : 1.2
 * Simulink Coder version    : 9.3 (R2020a) 18-Nov-2019
 * TLC version       : 9.3 (Jan 23 2020)
 * C/C++ source code generated on  : Sun Feb 27 20:42:18 2022
 *
 * Target selection: stm32.tlc
 * Embedded hardware selection: STM32CortexM
 * Code generation objectives: Unspecified
 * Validation result: Not run
 *
 *
 *
 * ******************************************************************************
 * * attention
 * *
 * * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * *
 * ******************************************************************************
 */

#include "ADRC.h"
#include "ADRC_private.h"

/* Block signals (default storage) */
B_ADRC ADRC_B;

/* Continuous states */
X_ADRC ADRC_X;

/* Block states (default storage) */
DW_ADRC ADRC_DW;

/* External outputs (root outports fed by signals with default storage) */
ExtY_ADRC ADRC_Y;

/* Real-time model */
RT_MODEL_ADRC ADRC_M_;
RT_MODEL_ADRC *const ADRC_M = &ADRC_M_;

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  ADRC_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  ADRC_step();
  ADRC_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  ADRC_step();
  ADRC_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  ADRC_step();
  ADRC_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void ADRC_step(void)
{
  if (rtmIsMajorTimeStep(ADRC_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&ADRC_M->solverInfo,((ADRC_M->Timing.clockTick0+1)*
      ADRC_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(ADRC_M)) {
    ADRC_M->Timing.t[0] = rtsiGetT(&ADRC_M->solverInfo);
  }

  {
    real_T *lastU;
    real_T lastTime;
    real_T rtb_deriva;
    real_T rtb_SUM0;
    int32_T tmp;
    rtb_deriva = 0.0 * ADRC_X.Transfer_CSTATE[0] + 20.0 *
      ADRC_X.Transfer_CSTATE[1];
    ADRC_Y.out = rtb_deriva;
    if (ADRC_M->Timing.t[0] < 1.0) {
      tmp = 0;
    } else {
      tmp = 8000;
    }

    rtb_SUM0 = (real_T)tmp - rtb_deriva;
    ADRC_B.KD = 20.0 * rtb_SUM0;
    rtb_deriva = ADRC_M->Timing.t[0];
    if ((ADRC_DW.TimeStampA >= rtb_deriva) && (ADRC_DW.TimeStampB >= rtb_deriva))
    {
      rtb_deriva = 0.0;
    } else {
      lastTime = ADRC_DW.TimeStampA;
      lastU = &ADRC_DW.LastUAtTimeA;
      if (ADRC_DW.TimeStampA < ADRC_DW.TimeStampB) {
        if (ADRC_DW.TimeStampB < rtb_deriva) {
          lastTime = ADRC_DW.TimeStampB;
          lastU = &ADRC_DW.LastUAtTimeB;
        }
      } else {
        if (ADRC_DW.TimeStampA >= rtb_deriva) {
          lastTime = ADRC_DW.TimeStampB;
          lastU = &ADRC_DW.LastUAtTimeB;
        }
      }

      rtb_deriva = (ADRC_B.KD - *lastU) / (rtb_deriva - lastTime);
    }

    ADRC_B.SUM1 = (rtb_SUM0 + ADRC_X.integat_CSTATE) + rtb_deriva;
    ADRC_B.KI = 60.0 * rtb_SUM0;
    if (rtmIsMajorTimeStep(ADRC_M)) {
    }
  }

  if (rtmIsMajorTimeStep(ADRC_M)) {
    real_T *lastU;
    if (ADRC_DW.TimeStampA == (rtInf)) {
      ADRC_DW.TimeStampA = ADRC_M->Timing.t[0];
      lastU = &ADRC_DW.LastUAtTimeA;
    } else if (ADRC_DW.TimeStampB == (rtInf)) {
      ADRC_DW.TimeStampB = ADRC_M->Timing.t[0];
      lastU = &ADRC_DW.LastUAtTimeB;
    } else if (ADRC_DW.TimeStampA < ADRC_DW.TimeStampB) {
      ADRC_DW.TimeStampA = ADRC_M->Timing.t[0];
      lastU = &ADRC_DW.LastUAtTimeA;
    } else {
      ADRC_DW.TimeStampB = ADRC_M->Timing.t[0];
      lastU = &ADRC_DW.LastUAtTimeB;
    }

    *lastU = ADRC_B.KD;
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(ADRC_M)) {
    rt_ertODEUpdateContinuousStates(&ADRC_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++ADRC_M->Timing.clockTick0;
    ADRC_M->Timing.t[0] = rtsiGetSolverStopTime(&ADRC_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      ADRC_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void ADRC_derivatives(void)
{
  XDot_ADRC *_rtXdot;
  _rtXdot = ((XDot_ADRC *) ADRC_M->derivs);
  _rtXdot->Transfer_CSTATE[0] = 0.0;
  _rtXdot->Transfer_CSTATE[0] += -10.0 * ADRC_X.Transfer_CSTATE[0];
  _rtXdot->Transfer_CSTATE[1] = 0.0;
  _rtXdot->Transfer_CSTATE[0] += -20.0 * ADRC_X.Transfer_CSTATE[1];
  _rtXdot->Transfer_CSTATE[1] += ADRC_X.Transfer_CSTATE[0];
  _rtXdot->Transfer_CSTATE[0] += ADRC_B.SUM1;
  _rtXdot->integat_CSTATE = ADRC_B.KI;
}

/* Model initialize function */
void ADRC_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&ADRC_M->solverInfo, &ADRC_M->Timing.simTimeStep);
    rtsiSetTPtr(&ADRC_M->solverInfo, &rtmGetTPtr(ADRC_M));
    rtsiSetStepSizePtr(&ADRC_M->solverInfo, &ADRC_M->Timing.stepSize0);
    rtsiSetdXPtr(&ADRC_M->solverInfo, &ADRC_M->derivs);
    rtsiSetContStatesPtr(&ADRC_M->solverInfo, (real_T **) &ADRC_M->contStates);
    rtsiSetNumContStatesPtr(&ADRC_M->solverInfo, &ADRC_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&ADRC_M->solverInfo,
      &ADRC_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&ADRC_M->solverInfo,
      &ADRC_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&ADRC_M->solverInfo,
      &ADRC_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&ADRC_M->solverInfo, (&rtmGetErrorStatus(ADRC_M)));
    rtsiSetRTModelPtr(&ADRC_M->solverInfo, ADRC_M);
  }

  rtsiSetSimTimeStep(&ADRC_M->solverInfo, MAJOR_TIME_STEP);
  ADRC_M->intgData.y = ADRC_M->odeY;
  ADRC_M->intgData.f[0] = ADRC_M->odeF[0];
  ADRC_M->intgData.f[1] = ADRC_M->odeF[1];
  ADRC_M->intgData.f[2] = ADRC_M->odeF[2];
  ADRC_M->intgData.f[3] = ADRC_M->odeF[3];
  ADRC_M->contStates = ((X_ADRC *) &ADRC_X);
  rtsiSetSolverData(&ADRC_M->solverInfo, (void *)&ADRC_M->intgData);
  rtsiSetSolverName(&ADRC_M->solverInfo,"ode4");
  rtmSetTPtr(ADRC_M, &ADRC_M->Timing.tArray[0]);
  ADRC_M->Timing.stepSize0 = 0.001;
  ADRC_X.Transfer_CSTATE[0] = 0.0;
  ADRC_X.Transfer_CSTATE[1] = 0.0;
  ADRC_DW.TimeStampA = (rtInf);
  ADRC_DW.TimeStampB = (rtInf);
  ADRC_X.integat_CSTATE = 0.0;
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] ADRC.c
 */
