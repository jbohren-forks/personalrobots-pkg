#ifndef __L2_REG_H__
#define __L2_REG_H__

#include "features/FeatureMatrix.hh"
#include "objective/Regularization.hh"

#include "math.h"

/**
   @brief L2 regularization objective
 */
class L2Regularization : public Regularization {
public:
  L2Regularization() { };

  void subgradient(const Dvec &wvec, Dvec &wOut) const {
    wOut = 2 * wvec;
  }

  double objective(const Dvec &wvec) const {
    double norm = (norm_2(wvec));
    //    cout << "Vec " << wvec << " norm " << norm << endl;
    //    return norm*norm;
    return norm;
  }

};

#endif
