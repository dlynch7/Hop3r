// kinematic.c provides functions that were developed in MATLAB
// and can be found at
// https://github.com/dlynch7/Hop3r/tree/master/MATLAB/Kinematic

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "kinematic.h"

float qa[3] = {};
float dqa_dt[3] = {};
float qu[6] = {};
float footPose[3] = {};
float twist[3] = {};
float wrench[3] = {};

// forward kinematics:
int8_t geomFK(float *qa, float *qu, float *footPose, uint8_t solOption) {
  float xhth, yhth;                   // location of theta-chain "hip"
  float xAptheta, yAptheta;           // relative coordinates (intermediate vars)
  float betatheta, gammatheta         // relative angles (theta-chain)
  float xkth, ykth;                   // location of theta-chain "knee"
  float xkps, ykps;                   // location of psi-chain "knee"
  float xkph, ykph;                   // location of hi-chain "knee"
  float a,b,c,d,e,f;                  // intermediate variables
  float xA,yA,xA1,xA2,yA1,yA2;        // location of "lower ankle"
  float xuA,yuA,xuA1,xuA2,yuA1,yuA2;  // location of "upper ankle"

  /****************************************************************************
  * Calculate (xA, yA) using theta1 and psi1:
  ****************************************************************************/
  xkth = -B1X + L1*cos(qa[0]);  // x-coordinate of theta-chain "knee"
  ykth = B1Y + L1*sin(qa[0]);   // y-coordinate of theta-chain "knee"
  xkps = B2X + L5*cos(qa[2]);   // x-coordinate of psi-chain "knee"
  ykps = B2Y + L5*sin(qa[2]);   // y-coordinate of psi-chain "knee"

  a = sqrt((xkth - xkps)*(xkth - xkps) + (ykth - ykps)*(ykth - ykps));  // intermediate variable
  b = (L2*L2 - L6*L6 + a*a)/(2*a);                                      // intermediate variable
  c = sqrt(L2*L2 - b*b);                                                // intermediate variable

  xA1 = (b/a)*(xkps - xkth) + (c/a)*(ykps - ykth) + xkth;
  xA2 = (b/a)*(xkps - xkth) - (c/a)*(ykps - ykth) + xkth;
  yA1 = (b/a)*(ykps - ykth) - (c/a)*(xkps - xkth) + ykth;
  yA2 = (b/a)*(ykps - ykth) + (c/a)*(xkps - xkth) + ykth;

  if (solOption==1 || solOption==2) {
    xA = xA1;
    yA = yA1;
  } else {
    xA = xA2;
    yA = yA2;
  }

  /****************************************************************************
  * Calculate (xuA, yuA) using phi1 and (xA, yA):
  ****************************************************************************/
  xkph = L3*cos(qa[1]); // x-coordinate of phi-chain "knee"
  ykph = L3*sin(qa[1]); // y-coordinate of phi-chain "knee"

  d = sqrt((xkph - xA)*(xkph - xA) + (ykph - yA)*(ykph - yA));  // intermediate variable
  e = (L4*L4 - L7*L7 + d*d)/(2*d);                              // intermediate variable
  f = sqrt(L4*L4 - e*e);                                        // intermediate variable

  xuA1 = (e/d)*(xA - xkph) - (f/d)*(yA - ykph) + xkph;
  xuA2 = (e/d)*(xA - xkph) + (f/d)*(yA - ykph) + xkph;
  yuA1 = (e/d)*(yA - ykph) + (f/d)*(xA - xkph) + ykph;
  yuA2 = (e/d)*(yA - ykph) - (f/d)*(xA - xkph) + ykph;

  if (solOption==1 || solOption==3) {
    xuA = xuA1;
    yuA = yuA1;
  } else {
    xuA = xuA2;
    yuA = yuA2;
  }

  /****************************************************************************
  * Solve for "knee" angles (theta2, phi2, psi2):
  ****************************************************************************/
  xhth = -B1X;  // x-coordinate of theta-chain "hip" joint
  yhth = B1Y;   // y-coordinate of theta-chain "hip" joint

  xAptheta = -xhth + xA;  // x-coordinate of "lower ankle" w.r.t. theta-chain "hip"
  yAptheta = yhth - yA;  // y-coordinate of "lower ankle" w.r.t. theta-chain "hip"

  betatheta = 

}

int8_t subchainFK(float *qa, float *qu, float *footPose, uint8_t chainOption) {
  switch (chainOption) {
    case 0: // evaluate along theta-chain:
      footPose[0] = -B1X + L1*cos(qa[0]) + L2*cos(qa[0] + qu[0]) + L8*cos(qa[0] + qu[0] + qu[1]);
      footPose[1] = B1Y + L1*sin(qa[0]) + L2*sin(qa[0] + qu[0]) + L8*sin(qa[0] + qu[0] + qu[1]);
      footPose[2] = qa[0] + qu[0] + qu[1];
      return 0;
    case 1: // evaluate along phi-chain:
      footPose[0] = L3*cos(qa[1]) + L4*cos(qa[1] + qu[2]) + (L7+L8)*cos(qa[1] + qu[2] + qu[3]);
      footPose[1] = L3*sin(qa[1]) + L4*sin(qa[1] + qu[2]) + (L7+L8)*sin(qa[1] + qu[2] + qu[3]);
      footPose[2] = qa[1] + qu[2] + qu[3];
      return 0;
    case 2: // evaluate along psi-chain:
      footPose[0] = B2X + L5*cos(qa[2]) + L6*cos(qa[2] + qu[4]) + L8*cos(qa[2] + qu[4] + qu[5]);
      footPose[1] = B2Y + L5*sin(qa[2]) + L6*sin(qa[2] + qu[4]) + L8*sin(qa[2] + qu[4] + qu[5]);
      footPose[2] = qa[2] + qu[4] + qu[5];
      return 0;
    default:
      return -1;
  }
}

// inverse kinematics:
int8_t subchainIK(float *qa, float *qu, float *footPose) {

}

// Jacobians:
int8_t actuatorJacobian(float *J, float *qa, float *qu, uint8_t chainOption) {

}

int8_t constraintJacobian(float *J, float *qa, float *qu) {

}

int8_t subchainJacobian(float *J, float *qa, float *qu, uint8_t chainOption) {

}

// task space to joint space conversions
int8_t twist2vels(float *qa, float *qu, float *dqa_dt, float *twist) {

}

int8_t wrench2torques(float *qa, float *qu, float *torques, float *wrench)
