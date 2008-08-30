#ifndef __BINARY_SUB_NRG_H__
#define __BINARY_SUB_NRG_H__

#include "objective/BinarySubmodularEnergy.hh"
#include "objective/BinarySubmodularMinimizer.hh"
#include "features/FeatureMatrix.hh"
#include "objective/GraphCutMinimizer.hh"
#include "objective/ObjectiveFunction.hh"
#include "util/ThreadSafeLogger.hh"

#include <map>


/**
   @brief A max-log-probability MRF objective with unary/binary potentials,
   binary states, and submodular affinities.

   @tparam FM The type of feature matrix used (e.g., dense or sparse)
 */
template <class FM>
class BinarySubmodularObjective : public ObjectiveFunction {
public:
  /**
     @param anrg A binary submodular energy function
     @param anrgOpt A binary submodular energy minimizer
     @param atargLabels Binary target labeling
     @param alabelWeights Loss weights associated with each state
     @param alogger An optional logger
   */
  BinarySubmodularObjective(BinarySubmodularEnergy<FM> &anrg,
			    BinarySubmodularMinimizer &anrgOpt,	    
			    const std::vector<int> &atargLabels,
			    const std::vector<double> &alabelWeights,
			    ThreadSafeLogger* alogger = NULL) :
    nNodes(anrg.getFeatureGraph()->numNodes()),
    nEdges(anrg.getFeatureGraph()->numEdges()),
    targetState(atargLabels),
    labelWeights(alabelWeights),
    nrg(anrg),
    nrgOpt(anrgOpt),
    logger(alogger)
    //    debug("BinarySubObj.debug")
  {
  };  

  void subgradient(const Dvec &wvec, Dvec &subg) const;

  void subgradientAccum(const Dvec &wvec, Dvec &subg) const;

  void projectFeasibleSet(Dvec &wvec) const;

  int getWeightDim() const { return nrg.getWeightDim(); }

  int getStateDim() const { return nNodes; }

  double objective(const Dvec& wvec) const;

  // calculates loss 
  double loss(const std::vector<int>& targetState, 
	      const std::vector<int>& state) const;	   

  double loss(const Dvec& wvec) const;
  double augmentedLoss(const Dvec& wvec) const;

  void lossAugmentedEnergies(const std::vector<int> &targetState,
			     const Dvec &wvec, 
			     Dvec &e0, Dvec &e1,
			     Dvec &e00, Dvec &e11) const;

  /**
     @brief Retuns a binary vector representing the target state
   */
  std::vector<int> getTargetState() const { return targetState; }

private:
  int nNodes;
  int nEdges;

  std::vector<int> targetState;
  std::vector<double> labelWeights;

  BinarySubmodularEnergy<FM> nrg;
  BinarySubmodularMinimizer& nrgOpt;

  ThreadSafeLogger* logger;

  //  mutable ofstream debug;

  void 
  subgradientAccum(const std::vector<int> laugGroundState,
		   const DenseFeatureMatrix<double>* nfmat,
		   const DenseFeatureMatrix<double>* efmat,
		   DvecView sw0, DvecView sw1, 
		   DvecView sw00, DvecView sw11) const;

  /*
  void 
  subgradientAccum(const vector<int> laugGroundState,
		   const SparseFeatureMatrix<double>* nfmat,
		   const SparseFeatureMatrix<double>* efmat,
		   DvecView sw0, DvecView sw1, 
		   DvecView sw00, DvecView sw11) const;
  */

};

#include "objective/BinarySubmodularObjective.tcc"

#endif
