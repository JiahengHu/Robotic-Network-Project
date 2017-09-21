/*
  Generates numbers from a Gaussian distribution with zero mean and
  unit variance.

  Concept and code from NRC

  Modified to use drand48() and return doubles 
*/
#include <stdlib.h>
#include <math.h>

double gaussDist(void);
double gaussDist() {
  static int iset=0;
  static double gset;
  float fac,rsq,v1,v2;

  // generate a new pair of numbers and return the first
  if(iset == 0) {
    do {
      v1 = 2.0*drand48()-1.0;
      v2 = 2.0*drand48()-1.0;
      rsq = v1*v1+v2*v2;
    } while (rsq >= 1.0 || rsq == 0.0);

    fac = sqrt(-2.0*log(rsq)/rsq);
    gset = v1*fac;
    iset = 1;

    return(v2*fac);
  } 

  // return the last number we generated
  iset = 0;

  return(gset);
}
