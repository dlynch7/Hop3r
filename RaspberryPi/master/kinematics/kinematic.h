#ifndef __KINEMATIC__H__
#define __KINEMATIC__H__
// Header file for kinematic.c
// API for robot kinematic equations

// kinematic.c provides functions that were developed in MATLAB
// and can be found at
// https://github.com/dlynch7/Hop3r/tree/master/MATLAB/Kinematic

#include <stdio.h>
#include <stdint.h>

// Define link lengths
#define L1  0.0530
#define L2  0.1390
#define L3  0.0970
#define L4  0.0983
#define L5  0.0530
#define L6  0.1390
#define L7  0.0692
#define L8  0.1018
#define B1X 0.0573
#define B2X 0.0573
#define B1Y 0.0082
#define B2Y 0.0082

/******************************************************************************
* Function prototypes
*
* For each of these functions, the returned value indicates the error type.
******************************************************************************/

// forward kinematics:
int8_t geomFK(float *qa, float *qu, float *footPose, uint8_t solOption);
int8_t subchainFK(float *qa, float *qu, float *footPose, uint8_t chainOption);

// inverse kinematics:
int8_t subchainIK(float *qa, float *qu, float *footPose);

// Jacobians
int8_t actuatorJacobian(double *Ja, float *qa, float *qu, uint8_t chainOption);
int8_t constraintJacobian(double *Jc, float *qa, float *qu);
int8_t subchainJacobian(double *Js, float *qa, float *qu, uint8_t chainOption);

// task space to joint space conversions:
int8_t twist2vels(float *qa, float *qu, float *dqa_dt, float *twist);
int8_t wrench2torques(float *qa, float *qu, float *torques, float *wrench);

#endif
