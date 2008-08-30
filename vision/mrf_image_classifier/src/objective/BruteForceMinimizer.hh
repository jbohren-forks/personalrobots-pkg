#ifndef __BRUTE_FORCE_MIN_H__
#define __BRUTE_FORCE_MIN_H__

#include <math.h>

#include "objective/BinarySubmodularMinimizer.hh"
#include "features/FeatureGraphExtractor.h"
#include "util/VectorPermutations.hh"

/**
   @brief A binary submodular minimizer that works via exhaustive enumeration.
 */
class BruteForceMinimizer : public BinarySubmodularMinimizer {
public:
  /**
     @param aedgeList A list of edges in the MRF graph
  */
  BruteForceMinimizer(const vector<pair<NodeId, NodeId> > *aedgeList) :
    edgeList(aedgeList) {
  };

  double energyGroundState(const Dvec& e0, const Dvec& e1,
			   const Dvec& e00, const Dvec& e11,
			   vector<int> &labeling) const {
    VectorPermutations perms(2, e0.size());

    vector<vector<int>*> *iperms = perms.getPerms();

    double minEnergy = HUGE_VAL;
    for (vector<vector<int>*>::iterator permPr = iperms->begin();
	 permPr != iperms->end();
	 permPr++) {
      vector<int>* perm = *permPr;
      double nrg = energyConfig(e0, e1, e00, e11, *perm);

      if (nrg < minEnergy) {
	labeling = *perm;
	minEnergy = nrg;
      }	
    }
    
    return minEnergy;
  }

private:
  const vector<pair<NodeId, NodeId> > *edgeList;

  double energyConfig(const Dvec& e0, const Dvec& e1,
		      const Dvec& e00, const Dvec& e11,
		      const vector<int> &labeling) const {
    double energy = 0;

    // FIXME: inefficient iteration
    // FIXME: node indices
    for (int ni = 0; ni < (int)e0.size(); ni++) {
      int label = labeling[ni];
      energy += (label == 0 ? e0(ni) : e1(ni));
    }
    
    for (int ei = 0; ei < (int)e00.size(); ei++) {
      pair<NodeId, NodeId> edge = (*edgeList)[ei];
      
      if (labeling[edge.first] == labeling[edge.second]) {
	if (labeling[edge.first] == 0) 
	  energy += e00(ei);
	else
	  energy += e11(ei);
      }
    }
    return energy;
  }


};

#endif
