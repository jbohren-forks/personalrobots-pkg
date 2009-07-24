#ifndef __MEAN_SHIFT_H__
#define __MEAN_SHIFT_H__
// This code is based off LGPL code

#include <math.h>
#include <string>
#include <iostream>
#include <vector>

#include <ANN/ANN.h>

namespace clustering
{
  void meanshiftGeneric(double *data,
                 int p,
                 int n,
                 double radius,
                 double rate,
                 int maxIter,
                 bool blur,
                 double *labels,
                 double *means);
}


#endif
