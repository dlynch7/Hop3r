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
  // double a_data[] = {1.0, 0.6, 0.0,
  //                    0.0, 1.5, 1.0,
  //                    0.0, 1.0, 1.0};
  int lda = 3;

  float A[] = {0.11, 0.12, 0.13,
               0.21, 0.22, 0.23};

  int ldb = 2;

  float B[] = {1011, 1012,
               1021, 1022,
               1031, 1032};

  int ldc = 2;

  float C[] = {0.00, 0.00,
               0.00, 0.00};

  // Compute C = A B
  cblas_sgemm(CblasRowMajor,CblasNoTrans,CblasNoTrans, 2, 2, 3, 1.0, A, lda, B, ldb, 0.0, C, ldc);

  // Output should be
  // [ 367.76, 368.12
  //   674.06, 674.72 ]

  printf("[ %g, %g\n", C[0], C[1]);
  printf("  %g, %g ]\n", C[2], C[3]);

  float qa[3] = {-1.6845,-2.6214,-1.4571};
  float qu[6];
  // float qu[6] = {0.5867,-0.4729,2.0785,-1.0279,-0.5867,0.4729};
  float footPose[3] = {};

  double Jc[36] = {};
  double Jcinv[36];

  clock_t tic0 = clock();
  geomFK(qa,qu,footPose,1);
  subchainIK(qa,qu,footPose);
  constraintJacobian(Jc, qa, qu);


  clock_t toc0 = clock();



  printf("th2 = %f\tth3 = %f\nph2 = %f\tph3 = %f\nps2 = %f\tps3 = %f\n",qu[0],qu[1],qu[2],qu[3],qu[4],qu[5]);
  printf("Calculation of qu, footPose, and Jc took %f seconds\n", (double)(toc0 - tic0) / CLOCKS_PER_SEC);

  printf("Jc = \n");
  printf("%f\t%f\t%f\t%f\t%f\t%f\n",Jc[0],Jc[1],Jc[2],Jc[3],Jc[4],Jc[5]);
  printf("%f\t%f\t%f\t%f\t%f\t%f\n",Jc[6],Jc[7],Jc[8],Jc[9],Jc[10],Jc[11]);
  printf("%f\t%f\t%f\t%f\t%f\t%f\n",Jc[12],Jc[13],Jc[14],Jc[15],Jc[16],Jc[17]);
  printf("%f\t%f\t%f\t%f\t%f\t%f\n",Jc[18],Jc[19],Jc[20],Jc[21],Jc[22],Jc[23]);
  printf("%f\t%f\t%f\t%f\t%f\t%f\n",Jc[24],Jc[25],Jc[26],Jc[27],Jc[28],Jc[29]);
  printf("%f\t%f\t%f\t%f\t%f\t%f\n",Jc[30],Jc[31],Jc[32],Jc[33],Jc[34],Jc[35]);

  /*double a_data[] = {1.0, 0.6, 0.0, 0.0, 0.0, 0.0,
                     0.0, 1.5, 1.0, 0.0, 0.0, 0.0,
                     0.0, 1.0, 1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 1.0, 0.6, 0.0,
                     0.0, 0.0, 0.0, 0.0, 1.5, 1.0,
                     0.0, 0.0, 0.0, 0.0, 1.0, 1.0};*/

  /*
   * Inverse is
   *    1   -1.2    1.2
   *    0    2.0   -2.0
   *    0   -2.0    3.0
   */
  //  double inva[36];
   int s, i, j;

   clock_t tic1 = clock();

   /* here, do your time-consuming job */

   gsl_matrix_view m = gsl_matrix_view_array(Jc, 6, 6);
   gsl_matrix_view inv = gsl_matrix_view_array(Jcinv,6,6);
   gsl_permutation * p = gsl_permutation_alloc(6);

   clock_t toc1 = clock();

   printf("The matrix is\n");
   for (i = 0; i < 6; ++i)
      for (j = 0; j < 6; ++j)
          printf(j==5?"%6.3f\n":"%6.3f\t", gsl_matrix_get(&m.matrix,i,j));

   clock_t tic2 = clock();

   gsl_linalg_LU_decomp(&m.matrix,p,&s);
   gsl_linalg_LU_invert(&m.matrix,p,&inv.matrix);

   clock_t toc2 = clock();

   printf("The inverse is\n");
   for (i = 0; i < 6; ++i)
       for (j = 0; j < 6; ++j)
           printf(j==5?"%6.3f\n":"%6.3f\t",gsl_matrix_get(&inv.matrix,i,j));

   gsl_permutation_free(p);

   printf("Inversion of Jc took %f seconds\n", ((double)(toc1 - tic1) / CLOCKS_PER_SEC) + ((double)(toc2 - tic2) / CLOCKS_PER_SEC));

   double Ja[9];
   uint8_t didJaSucceed = 0;
   clock_t tic3 = clock();
   didJaSucceed = actuatorJacobian(Ja, qa, qu, 0);
   clock_t toc3 = clock();

   printf("Did Ja succeed? Yes (0) / No(1): %d\n",didJaSucceed);
   printf("Calculation of Ja took %f seconds\n", (double)(toc3 - tic3) / CLOCKS_PER_SEC);
   printf("Ja = \n");
   for (i = 0; i < 3; ++i)
       for (j = 0; j < 3; ++j)
           printf(j==2?"%6.3f\n":"%6.3f\t",Ja[(3*i)+j]);

   return 0;
}
