#ifndef __OBJECTIVE_FUNC_H__
#define __OBJECTIVE_FUNC_H__

#include "features/UndirectedFeatureGraph.hh"

/**
   @brief Abstract base class for objective functions that can be optimized
   via subgradient method.
 */
class ObjectiveFunction {
public:
  virtual ~ObjectiveFunction() {};

  // FIXME: ugly, used only for debugging
  virtual double loss(const Dvec& wvec) const = 0;
  virtual double augmentedLoss(const Dvec& wvec) const = 0;

  /**
     @brief Returns value of objective function.
     @param wvec The weight vector
  */
  virtual double objective(const Dvec& wvec) const = 0;

  /**
     @brief Calculates subgradient
     @param wvec The weight vector
     @param subg Output variable for subgradient
  */
  virtual void subgradient(const Dvec &wvec, Dvec &subg) const = 0;

  /**
     @brief Calculates subgradient with accumulation
     @param wvec The weight vector
     @param subg Output variable for subgradient

     This version adds the subgradient to whatever was previously in subg
  */
  virtual void subgradientAccum(const Dvec &wvec, Dvec &subg) const {
    Dvec stemp(subg.size());
    subgradient(wvec, stemp);
    subg = subg + stemp;
  }

  /**
     @brief Projects the weight vector onto the feasible set
     @param wvec The weight vector
  */
  virtual void projectFeasibleSet(Dvec &wvec) const = 0;

  /**
     @brief Gets dimension of weight vector
  */
  virtual int getWeightDim() const = 0;

  /// Gets dimension of state/label vector
  virtual int getStateDim() const = 0;

  // Allocates a new zero weight vector, returns a pointer to it 
  // Caller must free
  //  virtual Dvec* getZeroWeightVec() const = 0;

  /*
    // calculates ground state of loss-augmented energy
  virtual double lossAugEnergyGroundState(const vector<int>& targetState, 
					  const Dvec& wvec,
					  vector<int>& groundState) = 0;
  */
  
  /*
protected:
  UndirectedFeatureGraph *fgraph;
  */
};

#endif
