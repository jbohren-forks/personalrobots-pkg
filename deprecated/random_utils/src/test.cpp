/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \Author Ioan Sucan */

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
