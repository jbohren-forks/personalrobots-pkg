#include "highgui.h"

#include "objective/BinarySubmodularObjective.hh"
#include "features/FeatureMatrix.hh"

#include "objective/BruteForceMinimizer.hh"

#include <map>

#include <stdio.h>
#include <stdlib.h>

using namespace std;

template <class FM>
double BinarySubmodularObjective<FM>::
loss(const vector<int>& targetState, const vector<int>& state) const {
  double lossval = 0;
  for (unsigned int ii = 0; ii < targetState.size(); ii++) {
    if (targetState[ii] != state[ii]) {
      lossval += labelWeights[ii];
      //      lossval += 1;
    }
  }
  return lossval;
}

// FIXME: debugging code
template <class FM>
double BinarySubmodularObjective<FM>::
augmentedLoss(const Dvec& wvec) const {
  Dvec e0(nNodes), e1(nNodes), e00(nEdges), e11(nEdges);

  lossAugmentedEnergies(targetState, wvec, e0, e1, e00, e11);

  vector<int> groundState;
  nrgOpt.energyGroundState(e0, e1, e00, e11, groundState);

  return loss(groundState, targetState);
}

template <class FM>
double BinarySubmodularObjective<FM>::
loss(const Dvec& wvec) const {
  Dvec e0(nNodes), e1(nNodes), e00(nEdges), e11(nEdges);

  nrg.factorEnergies(wvec, e0, e1, e00, e11);

  vector<int> groundState;
  nrgOpt.energyGroundState(e0, e1, e00, e11, groundState);

  return loss(groundState, targetState);
}

template <class FM>
void BinarySubmodularObjective<FM>::
subgradient(const Dvec &wvec, Dvec &subg) const {
  subg.clear();
  subgradientAccum(wvec, subg);
}

/*
template <class FM>
void BinarySubmodularObjective<FM>::
subgradientAccum(const vector<int> laugGroundState,
		 const SparseFeatureMatrix<double>* nfmat,
		 const SparseFeatureMatrix<double>* efmat,
		 DvecView sw0, DvecView sw1, 
		 DvecView sw00, DvecView sw11) const {

  if (nfmat != NULL && nfmat->rows() > 0) {
    int ni0 = nfmat->firstIndex();
    SparseFeatureMatrix<double>::const_iterator itEnd = nfmat->end();
    for (SparseFeatureMatrix<double>::const_iterator it = nfmat->begin();
	 it != itEnd;
	 ++it) {
      pair<pair<int,int>, double> keyval = *it;
      pair<int,int> rowcol = keyval.first;
      int node = rowcol.first - ni0;
      int col = rowcol.second - ni0;
      if (laugGroundState[node] != targetState[node]) {
	sw0(col) += (targetState[node] - laugGroundState[node]) * keyval.second;
	sw1(col) -= (targetState[node] - laugGroundState[node]) * keyval.second;
      }
    }
  }

  if (efmat != NULL && efmat->rows() > 0) {
    int ei0 = efmat->firstIndex();
    const vector<pair<NodeId, NodeId> > *edgeList = 
      nrg.getFeatureGraph()->getEdgeList();

    SparseFeatureMatrix<double>::const_iterator itEnd = efmat->end();
    for (SparseFeatureMatrix<double>::const_iterator it = efmat->begin();
	 it != itEnd;
	 ++it) {
      pair<pair<int,int>, double> keyval = *it;
      pair<int,int> rowcol = keyval.first;
      pair<int,int> edge = (*edgeList)[rowcol.first - ei0];
      int col = rowcol.second - ei0;
      
      int t11 = targetState[edge.first] * targetState[edge.second];
      int g11 = laugGroundState[edge.first] * laugGroundState[edge.second];
      int t00 = (1 - targetState[edge.first]) * (1 - targetState[edge.second]);
      int g00 = (1 - laugGroundState[edge.first]) * (1 - laugGroundState[edge.second]);
      
      if ((t11 - g11) != 0) {
	sw11(col) -= (t11 - g11) * keyval.second;
      }
      
      if ((t00 - g00) != 0) {
	sw00(col) -= (t00 - g00) * keyval.second;
      }
    }
  }  
}
*/

template <class FM>
void BinarySubmodularObjective<FM>::
subgradientAccum(const std::vector<int> laugGroundState,
		 const DenseFeatureMatrix<double>* nfmat,
		 const DenseFeatureMatrix<double>* efmat,
		 DvecView sw0, DvecView sw1, 
		 DvecView sw00, DvecView sw11) const {

  assert(nNodes == nfmat->rows());
  assert(nEdges == efmat->rows());

  if (nfmat != NULL && nfmat->rows() > 0) {
  //  if (0) {
    for (int ii = 0; ii < nNodes; ii++) {
      if (laugGroundState[ii] != targetState[ii]) {
	nfmat->addRowToVec(sw0, targetState[ii] - laugGroundState[ii], ii);
	// FIXME: optimize. probably doing a very wasteful scalar-vector op
	//	w0 += (targetState[ii] - groundState[ii]) * (*features);
	// optimization: just set w1 to -w0 at end
	//	w1 -= (targetState[ii] - groundState[ii]) * (*features);
      }
    }
  }

  sw1 = -sw0;

  // calculate edge components of subgradient

  if (efmat != NULL && efmat->rows() > 0) {
    //  if (0) {
    const vector<pair<NodeId, NodeId> > *edgeList = 
      nrg.getFeatureGraph()->getEdgeList();
    for (int ii = 0; ii < nEdges; ii++) {
      pair<NodeId, NodeId> edge = (*edgeList)[ii];
      int t11 = targetState[edge.first] * targetState[edge.second];
      int g11 = laugGroundState[edge.first] * laugGroundState[edge.second];
      int t00 = (1 - targetState[edge.first]) * (1 - targetState[edge.second]);
      int g00 = (1 - laugGroundState[edge.first]) * (1 - laugGroundState[edge.second]);

      if ((t11 - g11) != 0) {
	efmat->addRowToVec(sw11, g11 - t11, ii);
      }

      if ((t00 - g00) != 0) {
	efmat->addRowToVec(sw00, g00 - t00, ii);
      }
    }
  }
}

template <class FM>
void BinarySubmodularObjective<FM>::
subgradientAccum(const Dvec &wvec, Dvec &subg) const {
  // calculate ground state of loss-augmented energy
  Dvec e0(nNodes), e1(nNodes), e00(nEdges), e11(nEdges);

  lossAugmentedEnergies(targetState, wvec, e0, e1, e00, e11);

  std::vector<int> groundState;
  nrgOpt.energyGroundState(e0, e1, e00, e11, groundState);

  Drange *r0, *r1, *r00, *r11;
  nrg.weightRanges(&r0, &r1, &r00, &r11);

  DvecView w0(subg, *r0),
    w1(subg, *r1),
    w00(subg, *r00),
    w11(subg, *r11);

  delete r0;
  delete r1;
  delete r00;
  delete r11;

  if (logger) {
    string state = BinarySubmodularEnergy<FM>::stateAsString(groundState);
    logger->log(__PRETTY_FUNCTION__, string("l. aug. ground: ") + state);
  }

  subgradientAccum(groundState, 
		   nrg.getFeatureGraph()->getNodeFeat(),
		   nrg.getFeatureGraph()->getEdgeFeat(),
		   w0, w1, w00, w11);

  /*
  cout << "subgradient w00 " << w00 << endl;
  cout << "subgradient w11 " << w11 << endl;

  cout << "subgradient w1 " << w1 << endl;
  cout << "subgradient w0 " << w0 << endl;
  */

}

template <class FM>
void BinarySubmodularObjective<FM>::
lossAugmentedEnergies(const vector<int>& targetState,
		      const Dvec &wvec, 
		      Dvec &e0, Dvec &e1,
		      Dvec &e00, Dvec &e11) const {

  nrg.factorEnergies(wvec, e0, e1, e00, e11);  

  //  cerr << "Factored energies " << endl;

  // adversarially decrease unary energies
  for (int ii = 0; ii < nNodes; ii++) {
    if (targetState[ii] == 0)
      e1(ii) -= labelWeights[ii];
    else
      e0(ii) -= labelWeights[ii];
  }
}

// FIXME: slow, precompute things
template <class FM>
double BinarySubmodularObjective<FM>::
objective(const Dvec& wvec) const {
  vector<int> laugGroundState;
  //  vector<int> groundState;

  Dvec e0(nNodes), e1(nNodes), e00(nEdges), e11(nEdges);

  lossAugmentedEnergies(targetState, wvec, e0, e1, e00, e11);

  nrgOpt.energyGroundState(e0, e1, e00, e11, laugGroundState);

  double lossval = loss(targetState, laugGroundState);
  double energyGround = nrg.evaluateEnergy(laugGroundState, wvec);
  double energyTarget = nrg.evaluateEnergy(targetState, wvec);

  double val = energyTarget - (energyGround - lossval);

  /*
  debug << energyGround << " " << energyTarget << " " << lossval << " " <<
    val << endl;
  */

  //  cvWaitKey();

  // TESTING
  // TESTING
  //if (val < 0) {

  if (0) {
    BruteForceMinimizer bfmin(nrg.getFeatureGraph()->getEdgeList());

    vector<int> laugBfGround;

    bfmin.energyGroundState(e0, e1, e00, e11, laugBfGround);

    cout << "BFGrnd state: ";
    for (int ii = 0; ii < (int)laugBfGround.size(); ii++) 
      cout << laugBfGround[ii];
    cout << endl;
    
    assert (laugBfGround == laugGroundState);
  }

  /*
    cout << "LssAug state: ";
    for (int ii = 0; ii < (int)laugGroundState.size(); ii++) 
      cout << laugGroundState[ii];
    cout << endl;
    
  cout << "Target state: ";
  for (unsigned int ii = 0; ii < targetState.size(); ii++) 
    cout << targetState[ii];
  cout << endl;
  */

  //  cvWaitKey();

  return val;
}

// clip negative components of edge weights
template <class FM>
void BinarySubmodularObjective<FM>::
projectFeasibleSet(Dvec &wvec) const {
  //  Dvec w0(nNodes), w1(nNodes), w00(nEdges), w11(nEdges);

  Drange *r0, *r1, *r00, *r11;
  nrg.weightRanges(&r0, &r1, &r00, &r11);

  DvecView 
    w00(wvec, *r00),
    w11(wvec, *r11);

  delete r0;
  delete r1;
  delete r00;
  delete r11;

  //  nrg.unpackWeights(wvec, w0, w1, w00, w11);

  if (getenv("oZeroContext")) {
    for (int ii = 0; ii < (int)w00.size(); ii++) {
      w00(ii) = 0;
      w11(ii) = 0;
    }
  } else {
    // FIXME: slow?
    for (int ii = 0; ii < (int)w00.size(); ii++) 
      if (w00(ii) < 0) w00(ii) = 0;
    
    for (int ii = 0; ii < (int)w11.size(); ii++) 
      if (w11(ii) < 0) w11(ii) = 0;
  }

  /*
  // FIXME: a little hacky...
  double *edgeFirst = wvec.data() + r00.firstIndex(),
    *edgeLast = wvec.data() + r11.lastIndex();
  for (double *pr = edgeFirst; pr <= edgeLast; pr++)
    if (*pr < 0) *pr = 0;
  */


  //  cout << "PROJECTION TESTING" << endl;
  //    for (double *pr = edgeFirst; pr <= edgeLast; pr++)
  //      *pr = 0;

  /*
  cout << "w0 " << w0 << endl;
  cout << "w1 " << w1 << endl;
  cout << "w00 " << w00 << endl;
  cout << "w11 " << w11 << endl;
  */
  
  //  nrg.packWeights(w0, w1, w00, w11, wvec);
}

