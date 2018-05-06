// kinematic.c provides functions that were developed in MATLAB
// and can be found at
// https://github.com/dlynch7/Hop3r/tree/master/MATLAB/Kinematic

#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include "kinematic.h"

#define PI 3.14159265

float qa[3] = {};
float dqa_dt[3] = {};
float qu[6] = {};
float footPose[3] = {};
float twist[3] = {};
float wrench[3] = {};

// int main(void) {
//   float qa[3] = {-1.6845,-2.6214,-1.4571};
//   float dqa_dt[3] = {0,0,0};
//   float qu[6] = {0.5867,-0.4729,2.0785,-1.0279,-0.5867,0.4729};
//   float footPose[3] = {0,-0.27,-PI/2};
//
//   geomFK(qa,qu,footPose,1);
//   subchainIK(qa,qu,footPose);
//
//   // printf("%f\t%f\t%f\t%f\t%f\t%f\n",qu[0],qu[1],qu[2],qu[3],qu[4],qu[5]);
// }

// forward kinematics:
int8_t geomFK(float *qa, float *qu, float *footPose, uint8_t solOption) {
  float xhth, yhth;                   // location of theta-chain "hip"
  float xhps, yhps;                   // location of psi-chain "hip"
  float xAptheta, yAptheta;           // relative coordinates (intermediate vars)
  float xAppsi, yAppsi;               // relative coordinates (intermediate vars)
  float betatheta, gammatheta;        // relative angles (theta-chain)
  float betaphi, gammaphi;            // relative angles (phi-chain)
  float betapsi, gammapsi;            // relative angles (psi-chain)
  float xkth, ykth;                   // location of theta-chain "knee"
  float xkps, ykps;                   // location of psi-chain "knee"
  float xkph, ykph;                   // location of hi-chain "knee"
  float a,b,c,d,e,f;                  // intermediate variables
  float xA,yA,xA1,xA2,yA1,yA2;        // location of "lower ankle"
  float xuA,yuA,xuA1,xuA2,yuA1,yuA2;  // location of "upper ankle"
  float footX, footY, footAngle;      // intermidiate vars for footPose vector

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

  betatheta = PI - acos((L1*L1 + L2*L2 - xAptheta*xAptheta - yAptheta*yAptheta)/(2*L1*L2));


  betaphi = PI - acos((L3*L3 + L4*L4 - xuA*xuA - yuA*yuA)/(2*L3*L4));

  //psi-chain
  xhps = B2X;
  yhps = B2Y;
  xAppsi = xhps - xA;
  yAppsi = yhps - yA;

  betapsi = -PI + acos((L5*L5 + L6*L6 - xAppsi*xAppsi - yAppsi*yAppsi)/(2*L5*L6));

  /****************************************************************************
  * Calculate the foot pose:
  ****************************************************************************/
  footAngle = PI + atan2(yuA - yA, xuA - xA);
  footX = xA + L8*cos(footAngle);
  footY = yA + L8*sin(footAngle);

  footPose[0] = footX;
  footPose[1] = footY;
  footPose[2] = footAngle;

  /****************************************************************************
  * Calculate the third angle in each open chain:
  ****************************************************************************/
  gammatheta = footAngle - qa[0] - betatheta - 2*PI;
  gammaphi = footAngle - qa[1] - betaphi - 2*PI;
  gammapsi = footAngle - qa[2] - betapsi - 2*PI;

  qu[0] = betatheta;
  qu[1] = gammatheta;
  qu[2] = betaphi;
  qu[3] = gammaphi;
  qu[4] = betapsi;
  qu[5] = gammapsi;

  // printf("%f\t%f\t%f\t%f\t%f\t%f\n",qu[0],qu[1],qu[2],qu[3],qu[4],qu[5]);

  return 1;
}

int8_t subchainFK(float *qa, float *qu, float *footPose, uint8_t chainOption) {
  switch (chainOption) {
    case 0: // evaluate along theta-chain:
      {
      footPose[0] = -B1X + L1*cos(qa[0]) + L2*cos(qa[0] + qu[0]) + L8*cos(qa[0] + qu[0] + qu[1]);
      footPose[1] = B1Y + L1*sin(qa[0]) + L2*sin(qa[0] + qu[0]) + L8*sin(qa[0] + qu[0] + qu[1]);
      footPose[2] = qa[0] + qu[0] + qu[1];
      return 0;
      }
    case 1: // evaluate along phi-chain:
      {
      footPose[0] = L3*cos(qa[1]) + L4*cos(qa[1] + qu[2]) + (L7+L8)*cos(qa[1] + qu[2] + qu[3]);
      footPose[1] = L3*sin(qa[1]) + L4*sin(qa[1] + qu[2]) + (L7+L8)*sin(qa[1] + qu[2] + qu[3]);
      footPose[2] = qa[1] + qu[2] + qu[3];
      return 0;
      }
    case 2: // evaluate along psi-chain:
      {
      footPose[0] = B2X + L5*cos(qa[2]) + L6*cos(qa[2] + qu[4]) + L8*cos(qa[2] + qu[4] + qu[5]);
      footPose[1] = B2Y + L5*sin(qa[2]) + L6*sin(qa[2] + qu[4]) + L8*sin(qa[2] + qu[4] + qu[5]);
      footPose[2] = qa[2] + qu[4] + qu[5];
      return 0;
      }
    default:
      return -1;
  }
}

// inverse kinematics:
int8_t subchainIK(float *qa, float *qu, float *footPose) {
  //extract foot pose
  float xF = footPose[0];
  float yF = footPose[1];
  float angF = footPose[2];

  // (x,y) location of lower ankle joint
  float xA = xF - L8*cos(angF);
  float yA = yF - L8*sin(angF);

  // (x,y) location of upper ankle joint
  float xAu = xF - (L7+L8)*cos(angF);
  float yAu = yF - (L7+L8)*sin(angF);

  /****************************************************************************
  * IK for theta-chain:
  ****************************************************************************/
  float xHtheta = -B1X;
  float yHtheta = B1Y;

  //calculate hip angle
  float xAptheta = -xHtheta + xA;
  float yAptheta = yHtheta - yA;
  float gammatheta = atan2(yAptheta,xAptheta);
  float alphatheta = acos((xAptheta*xAptheta + yAptheta*yAptheta + L1*L1 - L2*L2)/(2*L1*sqrt(xAptheta*xAptheta + yAptheta*yAptheta)));
  float theta1 = -gammatheta - alphatheta;

  // (x,y) location of "knee" joint
  float xKtheta = xHtheta + L1*cos(theta1);
  float yKtheta = yHtheta + L1*sin(theta1);

  // calculate knee angle
  float betatheta = acos((L1*L1 + L2*L2 - xAptheta*xAptheta - yAptheta*yAptheta)/(2*L1*L2));
  float theta2 = PI - betatheta;

  // (x,y,angle) of "lower ankle" joint using FK
  float xAtheta = xKtheta + L2*cos(theta1 + theta2);
  float yAtheta = yKtheta + L2*sin(theta1 + theta2);
  float theta3 = angF - theta1 - theta2 - 2*PI;

  printf("th1 = %f\tth2 = %f\tth3 = %f\n",theta1,theta2,theta3);
  /****************************************************************************
  * IK for phi-chain:
  ****************************************************************************/
  // Calculate (x,y) location of "hip" joint:
  float xHphi = 0;
  float yHphi = 0;

  // Calculate hip angle:
  float gammaphi = fabs(atan2(yAu,xAu)); //fabs() is absolute value of a float
  float alphaphi = acos((xAu*xAu + yAu*yAu + L3*L3 - L4*L4)/(2*L3*sqrt(xAu*xAu + yAu*yAu)));
  float phi1 = -gammaphi - alphaphi;

  // Calculate (x,y) location of "knee" joint:
  float xKphi = xHphi + L3*cos(phi1);
  float yKphi = yHphi + L3*sin(phi1);

  // Calculate knee angle:
  float betaphi = acos((L3*L3 + L4*L4 - xAu*xAu - yAu*yAu)/(2*L3*L4));
  float phi2 = PI - betaphi;

  // Calculate (x,y,angle) of "upper ankle" joint, using FK:
  float xAuphi = xKphi + L4*cos(phi1 + phi2);
  float yAuphi = yKphi + L4*sin(phi1 + phi2);
  float phi3 = angF - phi1 - phi2 - 2*PI;

  printf("ph1 = %f\tph2 = %f\tph3 = %f\n",phi1,phi2,phi3);

  /****************************************************************************
  * IK for psi-chain:
  ****************************************************************************/
  // Calculate (x,y) location of "hip" joint:
  float xHpsi = B2X;
  float yHpsi = B2Y;

  // Calculate hip angle:
  float xAppsi = xHpsi - xA;
  float yAppsi = yHpsi - yA;
  float gammapsi = fabs(atan2(yAppsi,xAppsi));
  float alphapsi = acos((xAppsi*xAppsi + yAppsi*yAppsi + L5*L5 - L6*L6)/(2*L5*sqrt(xAppsi*xAppsi + yAppsi*yAppsi)));
  float psi1 = -PI + gammapsi + alphapsi;

  // Calculate (x,y) location of "knee" joint:
  float xKpsi = xHpsi + L5*cos(psi1);
  float yKpsi = yHpsi + L5*sin(psi1);

  // Calculate knee angle:
  float betapsi = acos((L5*L5 + L6*L6 - xAppsi*xAppsi - yAppsi*yAppsi)/(2*L5*L6));
  float psi2 = -PI + betapsi;

  // Calculate (x,y,angle) of "lower ankle" joint, using FK:
  float xApsi = xKpsi + L6*cos(psi1 + psi2);
  float yApsi = yKpsi + L6*sin(psi1 + psi2);
  float psi3 = angF - psi1 - psi2 - 2*PI;

  printf("ps1 = %f\tps2 = %f\tps3 = %f\n",psi1,psi2,psi3);

  // Pack values into angle arrays
  qa[0] = theta1;
  qa[1] = phi1;
  qa[2] = psi1;

  qu[0] = theta2;
  qu[1] = theta3;
  qu[2] = phi2;
  qu[3] = phi3;
  qu[4] = psi2;
  qu[5] = psi3;

  return 0;
}

// Jacobians:
int8_t actuatorJacobian(double *Ja, float *qa, float *qu, uint8_t chainOption) {
  // first argument is the actuator Jacobian, Ja: 3x3 matrix
  double Jc[36]; // constraint Jacobian, 6x6
  double Jcinv[36]; // inverse of Jc, 6x6
  int s; // used with gsl_linalg_LU_decomp
  uint8_t i,j; // indices used to get Jcinv
  double Jtheta[9]; // subchain Jacobian (theta chain), 3x3
  double Jphi[9]; // subchain Jacobian (phi chain), 3x3
  double Jpsi[9]; // subchain Jacobian (psi chain), 3x3
  double Ha[18]; // another Jacobian, 6x3
  double B[9]; // an intermediate matrix, 6x3
  double C[9]; // another intermediate matrix, 3x3

  // compute the constraint Jacobian:
  if (constraintJacobian(Jc,qa,qu)) {
    return 1; // failure
  }
  // invert the constraint Jacobian:
  gsl_matrix_view m = gsl_matrix_view_array(Jc, 6, 6);
  gsl_matrix_view inv = gsl_matrix_view_array(Jcinv,6,6);
  gsl_permutation * p = gsl_permutation_alloc(6);
  gsl_linalg_LU_decomp(&m.matrix,p,&s);
  gsl_linalg_LU_invert(&m.matrix,p,&inv.matrix);
  for (i = 0; i < 6; ++i)
      for (j = 0; j < 6; ++j)
          Jcinv[(6*i)+j] = gsl_matrix_get(&inv.matrix,i,j);
  gsl_permutation_free(p);

  // compute all 3 open subchain Jacobians:
  if (subchainJacobian(Jtheta,qa,qu,0)) {
    return 1; // failure
  }
  if (subchainJacobian(Jphi,qa,qu,1)) {
    return 1; // failure
  }
  if (subchainJacobian(Jpsi,qa,qu,2)) {
    return 1; // failure
  }

  // form Ha from open subchain Jacobians:
  Ha[0] = Jtheta[0]; // 0x0 = 0
  Ha[1] = -Jphi[0]; // 0x0 = 0
  Ha[2] = 0;
  Ha[3] = Jtheta[3]; // 1x0 = 3
  Ha[4] = -Jphi[3]; // 1x0 = 3
  Ha[5] = 0;
  Ha[6] = Jtheta[6]; // 2x0 = 6
  Ha[7] = -Jphi[6]; // 2x0 = 6
  Ha[8] = 0;
  Ha[9] = 0;
  Ha[10] = -Jphi[0]; // 0x0 = 0
  Ha[11] = Jpsi[0]; // 0x0 = 0
  Ha[12] = 0;
  Ha[13] = -Jphi[3]; // 1x0 = 3
  Ha[14] = Jpsi[3]; // 1x0 = 3
  Ha[15] = 0;
  Ha[16] = -Jphi[6]; // 2x0 = 6
  Ha[17] = Jpsi[6]; // 2x0 = 6

  cblas_dgemm(CblasRowMajor,
              CblasNoTrans,CblasNoTrans, 6, 3, 6,
              -1.0, Jcinv, 6, Ha, 6, 0.0, B, 6);

  // Compute the actuator Jacobian using the specified subchain:
  switch (chainOption) {
    case 0:
      { // Ja = Jtheta*(a bunch of stuff)
        C[0] = 1;
        C[1] = 0;
        C[2] = 0;

        C[3] = B[0];
        C[4] = B[1];
        C[5] = B[2];

        C[6] = B[3];
        C[7] = B[4];
        C[8] = B[5];
        cblas_dgemm(CblasRowMajor,
                    CblasNoTrans,CblasNoTrans, 3, 3, 3,
                    1.0, Jtheta, 3, C, 3, 0.0, Ja, 3);
      }
    case 1:
      { // Ja = Jphi*(a bunch of stuff)
        C[0] = 0;
        C[1] = 1;
        C[2] = 0;

        C[3] = B[6];
        C[4] = B[7];
        C[5] = B[8];

        C[6] = B[9];
        C[7] = B[10];
        C[8] = B[11];
        cblas_dgemm(CblasRowMajor,
                    CblasNoTrans,CblasNoTrans, 3, 3, 3,
                    1.0, Jphi, 3, C, 3, 0.0, Ja, 3);
      }
    case 2:
      { // Ja = Jpsi*(a bunch of stuff)
        C[0] = 0;
        C[1] = 0;
        C[2] = 1;

        C[3] = B[12];
        C[4] = B[13];
        C[5] = B[14];

        C[6] = B[15];
        C[7] = B[16];
        C[8] = B[17];
        cblas_dgemm(CblasRowMajor,
                    CblasNoTrans,CblasNoTrans, 3, 3, 3,
                    1.0, Jpsi, 3, C, 3, 0.0, Ja, 3);
      }
    default:
      return 1; // failure
  }

  return 0;
}

int8_t constraintJacobian(double *J, float *qa, float *qu) { // 6x6 matrix
  // row 1:
  J[0] = -L2*sin(qa[0]+qu[0]) - L8*sin(qa[0]+qu[0]+qu[1]);
  J[1] = -L8*sin(qa[0]+qu[0]+qu[1]);
  J[2] = L4*sin(qa[1]+qu[2]) + (L7+L8)*sin(qa[1]+qu[2]+qu[3]);
  J[3] = (L7+L8)*sin(qa[1]+qu[2]+qu[3]);
  J[4] = 0;
  J[5] = 0;
  // row 2:
  J[6] = L2*cos(qa[0]+qu[0]) + L8*cos(qa[0]+qu[0]+qu[1]);
  J[7] = L8*cos(qa[0]+qu[0]+qu[1]);
  J[8] = -L4*cos(qa[1]+qu[2]) - (L7+L8)*cos(qa[1]+qu[2]+qu[3]);
  J[9] = -(L7+L8)*cos(qa[1]+qu[2]+qu[3]);
  J[10] = 0;
  J[11] = 0;
  // row 3:
  J[12] = 1;
  J[13] = 1;
  J[14] = -1;
  J[15] = -1;
  J[16] = 0;
  J[17] = 0;
  // row 4:
  J[18] = 0;
  J[19] = 0;
  J[20] = L4*sin(qa[1]+qu[2]) + (L7+L8)*sin(qa[1]+qu[2]+qu[3]);
  J[21] = (L7+L8)*sin(qa[1]+qu[2]+qu[3]);
  J[22] = -L6*sin(qa[2]+qu[4]) - L8*sin(qa[2]+qu[4]+qu[5]);
  J[23] = -L8*sin(qa[2]+qu[4]+qu[5]);
  // row 5:
  J[24] = 0;
  J[25] = 0;
  J[26] = -L4*cos(qa[1]+qu[2]) - (L7+L8)*cos(qa[1]+qu[2]+qu[3]);
  J[27] = -(L7+L8)*cos(qa[1]+qu[2]+qu[3]);
  J[28] = L6*cos(qa[2]+qu[4]) + L8*cos(qa[2]+qu[4]+qu[5]);
  J[29] = L8*cos(qa[2]+qu[4]+qu[5]);
  // row 6:
  J[30] = 0;
  J[31] = 0;
  J[32] = -1;
  J[33] = -1;
  J[34] = 1;
  J[35] = 1;

  return 0;
}

int8_t subchainJacobian(double *Js, float *qa, float *qu, uint8_t chainOption) { // 3x3 matrix
  switch (chainOption) {
    case 0: // theta chain
      {
      // row 1:
      Js[0] = -L1*sin(qa[0]) - L2*sin(qa[0]+qu[0]) - L8*sin(qa[0]+qu[0]+qu[1]);
      Js[1] = -L2*sin(qa[0]+qu[0]) - L8*sin(qa[0]+qu[0]+qu[1]);
      Js[2] = -L8*sin(qa[0]+qu[0]+qu[1]);
      // row 2:
      Js[3] = L1*cos(qa[0]) + L2*cos(qa[0]+qu[0]) + L8*cos(qa[0]+qu[0]+qu[1]);
      Js[4] = L2*cos(qa[0]+qu[0]) + L8*cos(qa[0]+qu[0]+qu[1]);
      Js[5] = L8*cos(qa[0]+qu[0]+qu[1]);
      // row 3:
      Js[6] = 1;
      Js[7] = 1;
      Js[8] = 1;
      return 0;
      }
    case 1: // phi chain
      {
      // row 1:
      Js[0] = -L3*sin(qa[1]) - L4*sin(qa[1]+qu[2]) - (L7+L8)*sin(qa[1]+qu[2]+qu[3]);
      Js[1] = -L4*sin(qa[1]+qu[2]) - (L7+L8)*sin(qa[1]+qu[2]+qu[3]);
      Js[2] = -(L7+L8)*sin(qa[1]+qu[2]+qu[3]);
      // row 2:
      Js[3] = L3*cos(qa[1]) + L4*cos(qa[1]+qu[2]) + (L7+L8)*cos(qa[1]+qu[2]+qu[3]);
      Js[4] = L4*cos(qa[1]+qu[2]) + (L7+L8)*cos(qa[1]+qu[2]+qu[3]);
      Js[5] = (L7+L8)*cos(qa[1]+qu[2]+qu[3]);
      // row 3:
      Js[6] = 1;
      Js[7] = 1;
      Js[8] = 1;
      return 0;
      }
    case 2: // psi chain
      {
      // row 1:
      Js[0] = -L5*sin(qa[2]) - L6*sin(qa[2]+qu[4]) - L8*sin(qa[2]+qu[4]+qu[5]);
      Js[1] = -L6*sin(qa[2]+qu[4]) - L8*sin(qa[2]+qu[4]+qu[5]);
      Js[2] = -L8*sin(qa[2]+qu[4]+qu[5]);
      // row 2:
      Js[3] = L5*cos(qa[2]) + L6*cos(qa[2]+qu[4]) + L8*cos(qa[2]+qu[4]+qu[5]);
      Js[4] = L6*cos(qa[2]+qu[4]) + L8*cos(qa[2]+qu[4]+qu[5]);
      Js[5] = L8*cos(qa[2]+qu[4]+qu[5]);
      // row 3:
      Js[6] = 1;
      Js[7] = 1;
      Js[8] = 1;
      return 0;
      }
    default:// unknown option
      return -1;
  }
}

// task space to joint space conversions
int8_t twist2vels(float *qa, float *qu, float *dqa_dt, float *twist) {
  return 0;
}

int8_t wrench2torques(float *qa, float *qu, float *torques, float *wrench) {
  return 0;
}
