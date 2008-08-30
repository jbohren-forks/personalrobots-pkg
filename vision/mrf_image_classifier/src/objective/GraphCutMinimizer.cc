#include "features/FeatureGraphExtractor.h"
#include "objective/GraphCutMinimizer.hh"

double GraphCutMinimizer::
energyGroundState(const Dvec& e0, const Dvec& e1,
		  const Dvec& e00, const Dvec& e11,
		  vector<int>& groundState) const {
  int nNodes = e0.size();
  int nEdges = e00.size();

  Dvec gw0(nNodes), gw1(nNodes), gwSS(nEdges);

  energyCutWeights(e0, e1, e00, e11, gw0, gw1, gwSS);

  return directedMinCut(gw0, gw1, gwSS, groundState);  
}

// See Kolmogorov and Zabih: "What energy functions can 
// be minimized via graph cuts" for details
void GraphCutMinimizer::
energyCutWeights(const Dvec& e0, const Dvec& e1,
		 const Dvec& e00, const Dvec& e11,
		 Dvec &gw0, Dvec &gw1, Dvec &gwSS) const {
  int nNodes = e0.size();
  int nEdges = e00.size();

  // NB: gw0/1/SS don't need to be cleared first
  // calculate unary graph weights 
  for (int ii = 0; ii < nNodes; ii++) {
    if (e1(ii) > e0(ii)) {
      gw0(ii) = e1(ii) - e0(ii);
      gw1(ii) = 0;
    } else {
      gw0(ii) = 0;
      gw1(ii) = e0(ii) - e1(ii);
    }
  }

  /*
  cout << "e0 " << e0 << endl;
  cout << "e1 " << e1 << endl;
  
  cout << "e00 " << e00 << endl;
  cout << "e11 " << e11 << endl;

  cout << "e all 0 = " << e0.sum() + e00.sum() << endl;
  cout << "e all 1 = " << e1.sum() + e11.sum() << endl;
  */

  gwSS = -(e00 + e11);

  for (int ei = 0; ei < nEdges; ei++) {
    pair<NodeId, NodeId> edge = (*edgeList)[ei];

    gw0(edge.first) -= e00(ei);
    gw1(edge.second) -= e11(ei);
  }
}

// all input graph weights should be nonnegative
double GraphCutMinimizer::
directedMinCut(const Dvec &gw0, const Dvec &gw1, const Dvec &gwSS, 
	       vector<int> &labeling) const {
  int nNodes = gw0.size();
  int nEdges = gwSS.size();

  // fill maxflow input data structures
  typedef Graph<double,double,double> GraphType;
  GraphType *graph = new GraphType(nNodes, nEdges);

  for (int ii = 0; ii < nNodes; ii++) {
    graph->add_node();
    graph->add_tweights(ii, gw0(ii), gw1(ii));

    //    printf("N %d S %f T %f\n", ii, gw0(ii), gw1(ii));
  }



  /*
    // DEBUGGING

  double min0 = HUGE_VAL, min1 = HUGE_VAL, minS = HUGE_VAL;
  double max0 = -HUGE_VAL, max1 = -HUGE_VAL, maxS = -HUGE_VAL;
  for (int ii = 0; ii < gw0.size(); ii++) {
    min0 = MIN(min0, gw0(ii));
    min1 = MIN(min1, gw1(ii));
    max0 = MAX(max0, gw0(ii));
    max1 = MAX(max1, gw1(ii));
    assert(finite(min0));
    assert(finite(min1));
    assert(finite(max0));
    assert(finite(max1));
  }
  for (int ii = 0; ii < gwSS.size(); ii++) {
    minS = MIN(minS, gwSS(ii));
    maxS = MAX(maxS, gwSS(ii));
    assert(isfinite(maxS));
    assert(isfinite(minS));
  }

  std::cout << "MIN 0: " << min0 << " 1: " << 
    min1 << " S: " << minS << std::endl;
  std::cout << "MAX 0: " << max0 << " 1: " << 
    max1 << " S: " << maxS << std::endl;
  */



  for (int ei = 0; ei < nEdges; ei++) {
    pair<NodeId, NodeId> edge = (*edgeList)[ei];

    //    printf("E %d -> %d = %f\n", edge.first, edge.second, gwSS(ei));

    //    graph->add_edge(edge.first, edge.second, gwSS(ei), gwSS(ei));
    graph->add_edge(edge.first, edge.second, gwSS(ei), 0);
  }

  double groundObjective = graph->maxflow();

  for (int ii = 0; ii < nNodes; ii++) {
    int label = graph->what_segment(ii) == GraphType::SOURCE ? 0 : 1;
    labeling.push_back(label);
  }

  delete graph;

  return groundObjective;
}

