#ifndef __BINARY_SUBMODULAR_ENERGY_H__
#define __BINARY_SUBMODULAR_ENERGY_H__

#include "features/FeatureGraphExtractor.h"
#include "features/UndirectedFeatureGraph.hh"
#include "objective/BinarySubmodularMinimizer.hh"

#include <boost/numeric/ublas/vector_proxy.hpp>

#include <map>

//#include <flens/flens.h>

// binary submodular energy, parametrized by weight vector
/**
   @brief Binary submodular energy function for MRF learning
 */
template <class FM>
class BinarySubmodularEnergy {
public:
  /**
     @param agraph An undirected feature graph
   */
  BinarySubmodularEnergy(const UndirectedFeatureGraph<FM>* agraph) : 
    nNodes(agraph->numNodes()),
    nEdges(agraph->numEdges()),
    nodeDim(agraph->numNodeFeatures()),
    edgeDim(agraph->numEdgeFeatures()),
    fgraph(agraph) {
    assert(agraph != NULL);
  };

  void 
  weightRanges(Drange **r0, Drange **r1,
	       Drange **r00, Drange **r11) const;

  /*
  // Split weight vector out into component weight vectors
  void unpackWeights(const Dvec &wvec, 
		     Dvec &w0, Dvec &w1,
		     Dvec &w00, Dvec &w11) const;

  void packWeights(const Dvec &w0, const Dvec &w1,
		   const Dvec &w00, const Dvec &w11,
		   Dvec &wvec) const;
  */

  double groundState(const Dvec &wvec, vector<int> &labeling, 
		     BinarySubmodularMinimizer &opt) const;

  double groundState(const Dvec &wvec, vector<int> &labeling, 
		     BinarySubmodularMinimizer &opt,
		     map<int,int>& constrainedNodes) const;

  // computes energies associated with unary, binary factors
  void factorEnergies(const Dvec &wvec, 
		      Dvec &e0, Dvec &e1,
		      Dvec &e00, Dvec &e11) const;

  int getWeightDim() const { 
    return 2*nodeDim + 2*edgeDim; 
  }

  // evaluates the total energy of a configuration
  double evaluateEnergy(const vector<int> &labeling, const Dvec &wvec) const;

  const UndirectedFeatureGraph<FM>* getFeatureGraph() const { return fgraph; }

  static string stateAsString(const vector<int>& state) {
    string result;

    for (vector<int>::const_iterator it = state.begin();
	 it != state.end();
	 it++) {
      char buf[255];
      snprintf(buf, sizeof(buf), "%d", *it);
      result += buf;
    }

    return result;
  }

private:
  int nNodes;
  int nEdges;

  int nodeDim;
  int edgeDim;

  const UndirectedFeatureGraph<FM>* fgraph;
};

#include "objective/BinarySubmodularEnergy.hh"

template <class FM>
double BinarySubmodularEnergy<FM>::
evaluateEnergy(const vector<int>& labeling, const Dvec &wvec) const {
  double energy = 0;

  Dvec e0(nNodes), e1(nNodes);
  Dvec e00(nEdges), e11(nEdges);

  factorEnergies(wvec, e0, e1, e00, e11);

  // FIXME: inefficient iteration
  // FIXME: node indices
  for (int ni = 0; ni < nNodes; ni++) {
    int label = labeling[ni];
    energy += (label == 0 ? e0(ni) : e1(ni));
  }

  const vector<pair<NodeId, NodeId> >* edgeList = fgraph->getEdgeList();
  for (int ei = 0; ei < nEdges; ei++) {
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

template <class FM>
double BinarySubmodularEnergy<FM>::
groundState(const Dvec &wvec, vector<int> &labeling, 
	    BinarySubmodularMinimizer &opt) const {

  Dvec e0(nNodes), e1(nNodes), e00(nEdges), e11(nEdges);
  
  wvec.assertFinite();

  factorEnergies(wvec, e0, e1, e00, e11);

  return opt.energyGroundState(e0, e1, e00, e11, labeling);
}

template <class FM>
double BinarySubmodularEnergy<FM>::
groundState(const Dvec &wvec, vector<int> &labeling, 
	    BinarySubmodularMinimizer &opt,
	    map<int,int>& constrainedNodes) const {

  Dvec e0(nNodes), e1(nNodes), e00(nEdges), e11(nEdges);
  
  factorEnergies(wvec, e0, e1, e00, e11);

  for (map<int,int>::iterator it = constrainedNodes.begin();
       it != constrainedNodes.end();
       it++) {
    pair<int,int> keyval = *it;
    int node = keyval.first;
    int label = keyval.second;
    if (label == 0) {
      e0(node) = -HUGE_VAL;
      e1(node) = HUGE_VAL;
    } else {
      e0(node) = HUGE_VAL;
      e1(node) = -HUGE_VAL;
    }
  }

  return opt.energyGroundState(e0, e1, e00, e11, labeling);
}

/*
template <class FM>
void BinarySubmodularEnergy<FM>::
unpackWeights(const Dvec &wvec,
	      Dvec &w0, Dvec &w1,
	      Dvec &w00, Dvec &w11) const {
  w0 = wvec(_(0, nodeDim - 1));
  w1 = wvec(_(nodeDim, 2*nodeDim - 1));
  w00 = wvec(_(2*nodeDim, 2*nodeDim + edgeDim - 1));
  w11 = wvec(_(2*nodeDim + edgeDim, 2*nodeDim + 2*edgeDim - 1));
}

template <class FM>
void BinarySubmodularEnergy<FM>::
packWeights(const Dvec& w0, const Dvec &w1,
	    const Dvec& w00, const Dvec &w11,
	    Dvec &wvec) const {
  wvec(_(0, nodeDim - 1)) = w0;
  wvec(_(nodeDim, 2*nodeDim - 1)) = w1;
  wvec(_(2*nodeDim, 2*nodeDim + edgeDim - 1)) = w00;
  wvec(_(2*nodeDim + edgeDim, 2*nodeDim + 2*edgeDim - 1)) = w11;
}
*/

// returns ranges associated with component sub-vectors
// CALLER MUST FREE returned values
template <class FM>
void BinarySubmodularEnergy<FM>::
weightRanges(Drange** r0, Drange** r1,
	     Drange** r00, Drange** r11) const {

  assert(edgeDim > 0);

  *r0 = new Drange(0, nodeDim);
  *r1 = new Drange(nodeDim, 2*nodeDim);
  *r00 = new Drange(2*nodeDim, 2*nodeDim + edgeDim);
  *r11 = new Drange(2*nodeDim + edgeDim, 2*nodeDim + 2*edgeDim);
}

// All input vectors should be of the correct length
template <class FM>
void BinarySubmodularEnergy<FM>::
factorEnergies(const Dvec &wvec, 
	       Dvec &e0, Dvec &e1,
	       Dvec &e00, Dvec &e11) const {
  using namespace boost::numeric::ublas;
  using namespace boost::numeric;

  Drange *r0, *r1, *r00, *r11;
  weightRanges(&r0, &r1, &r00, &r11);

  /*
  const vector_range<const ublas::vector<double> > w0 = project(wvec, *r0);
  const vector_range<const ublas::vector<double> > w1 = project(wvec, *r1);
  const vector_range<const ublas::vector<double> > w00 = project(wvec, *r00);
  const vector_range<const ublas::vector<double> > w11 = project(wvec, *r11);
  */
  DvecViewConst w0(wvec, *r0);
  DvecViewConst w1(wvec, *r1);
  DvecViewConst w00(wvec, *r00);
  DvecViewConst w11(wvec, *r11);


  delete r0; 
  delete r1;
  delete r00;
  delete r11;

  //  if (nf != NULL) {
  const FeatureMatrix<double> *nf = fgraph->getNodeFeat();

    // calculate unary state energies
  nf->multiplyWeights(w0, e0);
  nf->multiplyWeights(w1, e1);

  e0 *= -1;
  e1 *= -1;
  //  }
  
    //  if (fgraph->getRows() > 0) {
  const FeatureMatrix<double> *ef = fgraph->getEdgeFeat();
  
  // calculate pairwise state energies
  ef->multiplyWeights(w00, e00);
  ef->multiplyWeights(w11, e11);

  e00 *= -1;
  e11 *= -1;  
  //  }

  wvec.assertFinite();
  nf->assertFinite();
  ef->assertFinite();



  //  assert(e0.firstIndex() == 0);
}

#endif
