/*
 * mean_shift.h
 *
 *  Created on: Jul 23, 2009
 *      Author: dmunoz
 */

#ifndef __MEAN_SHIFT_H__
#define __MEAN_SHIFT_H__

#include <math.h>
#include <string.h>
#include <iostream>
#include <vector>

#include <ANN/ANN.h>

namespace meanshift
{
  void meanShift(double *data,
                 int p,
                 int n,
                 double radius,
                 double rate,
                 int maxIter,
                 bool blur,
                 double *labels,
                 double *means);
}


#endif /* MEAN_SHIFT_H_ */
