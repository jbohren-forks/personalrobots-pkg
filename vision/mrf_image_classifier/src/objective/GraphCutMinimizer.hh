#ifndef __GRAPH_CUT_MIN_H__
#define __GRAPH_CUT_MIN_H__

#include "objective/BinarySubmodularMinimizer.hh"
#include "features/FeatureGraphExtractor.h"

#include "bk_maxflow/graph.h"
#include "bk_maxflow/graph.cpp"
#include "bk_maxflow/maxflow.cpp"

/**
   @brief A binary-submodular energy minimizer implemented using graph cuts
 */
class GraphCutMinimizer : public BinarySubmodularMinimizer {
public:
  GraphCutMinimizer(const vector<pair<int, int> > *aedgeList) :
    edgeList(aedgeList) {
  };

  double energyGroundState(const Dvec& e0, const Dvec& e1,
			   const Dvec& e00, const Dvec& e11,
			   vector<int> &labeling) const;

private:
  const vector<pair<int, int> > *edgeList;

  // Convert factor energies to directed graph weights 
  // such that min cut on graph minimizes total energy
  void energyCutWeights(const Dvec& e0, const Dvec& e1, 
			const Dvec& e00, const Dvec& e11, 
			Dvec &gw0, Dvec &gw1, Dvec &gwSS) const;

  // Calculates a minimum cut on directed graph with given weights.
  double directedMinCut(const Dvec &gw0, const Dvec &gw1, const Dvec &gwSS, 
			vector<int> &labeling) const;

};

#endif
