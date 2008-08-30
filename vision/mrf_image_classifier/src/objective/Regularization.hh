#ifndef __REGULARIZATION_H__
#define __REGULARIZATION_H__

#include "features/FeatureMatrix.hh"

/**
   @brief Abstract base class for a regularization function
*/
class Regularization {
public:
  virtual ~Regularization() {};

  /** 
      @brief Computes a subgradient of the regularization objective
      @param wvec The parameter vector
      @param wOut Output argument for subgradient
   */
  virtual void subgradient(const Dvec &wvec, Dvec &wOut) const = 0;

  /**
     @brief Computes value of regularization objective
     @param wvec The parameter vector
   */
  virtual double objective(const Dvec &wvec) const = 0;
};

#endif
