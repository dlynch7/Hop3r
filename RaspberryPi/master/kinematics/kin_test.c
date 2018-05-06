// kin_test.c
// tests kinematic functions that will be put on the Pi
//
// compile with
// gcc -Wall kin_test.c kinematic.c -lgsl -lgslcblas -lm -o kin_test

#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include "kinematic.h"

int main(void) {
  int i,j;
  float qa[3] = {-1.6845,-2.6214,-1.4571};
  float qu[6];
  float footPose[3] = {};
  double wrench[3] = {1,1,1};
  double twist[3] = {1,1,1};
  double torques[3];
  double vels[3];
  double Ja[9];
  uint8_t didJaSucceed = 0;
  uint8_t didw2tSucceed = 0;
  uint8_t didt2vSucceed = 0;

  clock_t tic1 = clock();
  geomFK(qa,qu,footPose,1);
  subchainIK(qa,qu,footPose);

  didJaSucceed = actuatorJacobian(Ja, qa, qu, 0);
  clock_t toc1 = clock();

  printf("th2 = %f\tth3 = %f\nph2 = %f\tph3 = %f\nps2 = %f\tps3 = %f\n",qu[0],qu[1],qu[2],qu[3],qu[4],qu[5]);
  printf("Did Ja succeed? Yes (0) / No(1): %d\n",didJaSucceed);
  printf("Calculating FK, IK, and Ja took %f seconds\n", (double)(toc1 - tic1) / CLOCKS_PER_SEC);
  printf("Ja = \n");
  for (i = 0; i < 3; ++i)
     for (j = 0; j < 3; ++j)
         printf(j==2?"%6.3f\n":"%6.3f\t",Ja[(3*i)+j]);

  clock_t tic2 = clock();
  didw2tSucceed = wrench2torques(qa, qu, torques, wrench);
  clock_t toc2 = clock();

  printf("Did w2t succeed? Yes (0) / No(1): %d\n",didw2tSucceed);
  printf("Calculating w2t took %f seconds\n", (double)(toc2 - tic2) / CLOCKS_PER_SEC);
  printf("torques = [%6.3f, %6.3f, %6.3f]\n",torques[0],torques[1],torques[2]);

  clock_t tic3 = clock();
  didt2vSucceed = twist2vels(qa, qu, vels, twist);
  clock_t toc3 = clock();

  printf("Did t2v succeed? Yes (0) / No(1): %d\n",didt2vSucceed);
  printf("Calculating t2v took %f seconds\n", (double)(toc3 - tic3) / CLOCKS_PER_SEC);
  printf("vels = [%6.3f, %6.3f, %6.3f]\n",vels[0],vels[1],vels[2]);

  return 0;
}
