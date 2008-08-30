#ifndef __BINARY_SUB_MIN_H__
#define __BINARY_SUB_MIN_H__

#include "util/Dvec.hh"

/**
   @brief Abstract base class for things that can minimize a binary,
   submodular energy function.
 */
class BinarySubmodularMinimizer {
public:
  virtual ~BinarySubmodularMinimizer() { };
  
  /**
     @brief Computes ground state of a binary submodular energy
     @param e0 Vector of unary energies associated with 0-states
     @param e1 Vector of unary energies associated with 1-states
     @param e00 Vector of binary energies associated with 00-states
     @param e11 Vector of binary energies associated with 11-states
     @param labeling Output argument for energy ground state 
  */
  virtual double 
  energyGroundState(const Dvec& e0, const Dvec& e1,
		    const Dvec& e00, const Dvec& e11,
		    vector<int> &labeling) const = 0;  
};

#endif
