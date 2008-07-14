#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "random_utils/random_utils.h"


int main(void)
{
  const int    N   = 10000000;
  const double tol = 1e-2;

  printf("Testing basic functions:\n");

  // initialization 
  random_utils::init();

  // uniform number generator
  double sum = 0.0;    
  for (int i = 0 ; i < N ; ++i)
    sum += random_utils::uniform(-1.0, 1.0);
  sum /= (double)N;    
  printf("should be ~0.0: %f\n", sum);
  if (fabs(sum) > tol)
    exit(1);

  sum = 0.0;    
  for (int i = 0 ; i < N ; ++i)
    sum += random_utils::uniform(0.0, 1.0);
  sum /= (double)N;
  printf("should be ~0.5: %f\n", sum);
  if (fabs(sum - 0.5) > tol)
    exit(1);

  // gaussian number generator
  double mean   = 5.0;
  double stddev = sqrt(2.0);

  sum = 0.0;
  double var = 0.0;    
  for (int i = 0 ; i < N ; ++i)
  {
    double v = random_utils::gaussian(mean, stddev);
    sum += v;
    v -= mean;
    var += v*v;
  }

  sum /= (double)N;
  var /= (double)N;
  printf("should be ~%f: %f\n", mean, sum);
  printf("should be ~%f: %f\n", stddev*stddev, var);
  if (fabs(sum - mean) > tol)
    exit(1);

  if (fabs(var - stddev*stddev) > tol)
    exit(1);


  ////////////////////////////////////////////////////////////

  printf("Testing state functions:\n");

  // initialization     
  random_utils::rngState s;
  random_utils::init(&s);

  // uniform number generation
  sum = 0.0;    
  for (int i = 0 ; i < N ; ++i)
    sum += random_utils::uniform(&s, -1.0, 1.0);
  sum /= (double)N;    
  printf("should be ~0.0: %f\n", sum);
  if (fabs(sum) > tol)
    exit(1);


  sum = 0.0;    
  for (int i = 0 ; i < N ; ++i)
    sum += random_utils::uniform(&s, 0.0, 1.0);
  sum /= (double)N;
  printf("should be ~0.5: %f\n", sum);
  if (fabs(sum - 0.5) > tol)
    exit(1);


  // gaussian number generator
  mean   = 5.0;
  stddev = sqrt(2.0);

  sum = 0.0;
  var = 0.0;    
  for (int i = 0 ; i < N ; ++i)
  {
    double v = random_utils::gaussian(&s, mean, stddev);
    sum += v;
    v -= mean;
    var += v*v;
  }

  sum /= (double)N;
  var /= (double)N;
  printf("should be ~%f: %f\n", mean, sum);
  printf("should be ~%f: %f\n", stddev*stddev, var);
  if (fabs(sum - mean) > tol)
    exit(1);

  if (fabs(var - stddev*stddev) > tol)
    exit(1);

  /////////////////////////////////////////////

  printf("Testing random ints:\n");

  for (int i = 0 ; i < N ; ++i)
    if (random_utils::uniformInt(&s, 1, 1) != 1)
    {
      printf("oops!\n");
      exit(1); 
    }

  sum = 0.0;
  for (int i = 0 ; i < N ; ++i)
    sum += random_utils::uniformInt(&s, 0, 1);
  sum /= (double)N;
  printf("should be ~0.5: %f\n", sum);
  if (fabs(sum - 0.5) > tol)
    exit(1);

  sum = 0.0;
  for (int i = 0 ; i < N ; ++i)
    sum += random_utils::uniformInt(&s, 1, 3);
  sum /= (double)N;
  printf("should be ~2: %f\n", sum);
  if (fabs(sum - 2.0) > tol)
    exit(1);
  // done

  printf("ALL OK\n");

  return 0;
}
