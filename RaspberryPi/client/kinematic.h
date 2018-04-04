#ifndef __KINEMATIC__H__
#define __KINEMATIC__H__
// Header file for kinematic.c
// API for robot kinematic equations

// kinematic.c provides functions that were developed in MATLAB
// and can be found at
// https://github.com/dlynch7/Hop3r/tree/master/MATLAB/Kinematic

#include <stdio.h>
#include <stdint.h>

/******************************************************************************
* Function prototypes
******************************************************************************/

// forward kinematics:
int8_t geomFK(float *qa, float *qu, float *lengths, float *footPose, uint8_t solOption); // return value indicates error

// inverse kinematics:
int8_t subchainIK(float *qa, float *qu, float *footPose, float *lengths); // return value indicates error

// Jacobians
int8_t actuatorJacobian(float *J, float *qa, float *qu, float *lengths, uint8_t chainOption); // return value indicates error
int8_t constraintJacobian(float *J, float *qa, float *qu, float *lengths); // return value indicates error
int8_t subchainJacobian(float *J, float *qa, float *qu, float *lengths, uint8_t chainOption); // return value indicates error

#endif
