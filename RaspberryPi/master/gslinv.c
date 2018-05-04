#include <stdio.h>
#include <gsl/gsl_linalg.h>
#include <time.h>

int main (void) {
  // double a_data[] = {1.0, 0.6, 0.0,
  //                    0.0, 1.5, 1.0,
  //                    0.0, 1.0, 1.0};

  double a_data[] = {1.0, 0.6, 0.0, 0.0, 0.0, 0.0,
                     0.0, 1.5, 1.0, 0.0, 0.0, 0.0,
                     0.0, 1.0, 1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 1.0, 0.6, 0.0,
                     0.0, 0.0, 0.0, 0.0, 1.5, 1.0,
                     0.0, 0.0, 0.0, 0.0, 1.0, 1.0};

  /*
   * Inverse is
   *    1   -1.2    1.2
   *    0    2.0   -2.0
   *    0   -2.0    3.0
   */
   double inva[36];
   int s, i, j;

   clock_t tic = clock();

   /* here, do your time-consuming job */

   gsl_matrix_view m = gsl_matrix_view_array(a_data, 6, 6);
   gsl_matrix_view inv = gsl_matrix_view_array(inva,6,6);
   gsl_permutation * p = gsl_permutation_alloc (6);

   printf("The matrix is\n");
   for (i = 0; i < 6; ++i)
      for (j = 0; j < 6; ++j)
          printf(j==5?"%6.3f\n":"%6.3f ", gsl_matrix_get(&m.matrix,i,j));

   gsl_linalg_LU_decomp(&m.matrix,p,&s);
   gsl_linalg_LU_invert(&m.matrix,p,&inv.matrix);

   clock_t toc = clock();

   printf("The inverse is\n");
   for (i = 0; i < 6; ++i)
       for (j = 0; j < 6; ++j)
           printf(j==5?"%6.3f\n":"%6.3f ",gsl_matrix_get(&inv.matrix,i,j));

   gsl_permutation_free (p);

   printf("Elapsed: %f seconds\n", (double)(toc - tic) / CLOCKS_PER_SEC);

   return 0;
}
