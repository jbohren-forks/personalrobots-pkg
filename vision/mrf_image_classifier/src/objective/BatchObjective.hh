#ifndef __BATCH_OBJECTIVE_H__
#define __BATCH_OBJECTIVE_H__

#include "objective/ObjectiveFunction.hh"
#include "util/Dvec.hh"
#include "util/ThreadSafeLogger.hh"

#include <vector>

using namespace std;

// FIXME: will shallow copies work if the objective objects have destructors?
// (double freeing could occur)

// FIXME: specify how to do feasible set projection in constructor

/**
   @brief An objective function that is the sum of multiple objective functions.
*/
class BatchObjective : public ObjectiveFunction {
public:
  /**
     @param ovec A vector of ObjectiveFunction's
     @param alogger An optional ThreadSafeLogger
   */
  BatchObjective(const vector<ObjectiveFunction*>& ovec, 
		 ThreadSafeLogger* alogger = NULL) : 
    objFuncs(ovec),
    logger(alogger)
  {
  };

  double objective(const Dvec& wvec) const {
    double val = 0;

    for (vector<ObjectiveFunction*>::const_iterator it = objFuncs.begin();
	 it != objFuncs.end();
	 it++) {
      double thisval = (*it)->objective(wvec);
      
      if (logger != NULL) {
	char buf[1000];
	snprintf(buf, sizeof(buf), "sub. objective: %f", thisval);
	logger->log(__PRETTY_FUNCTION__, buf);
      }

      val += thisval;
    }

    if (logger != NULL) {
      char buf[1000];
      snprintf(buf, sizeof(buf), "tot. objective: %f", val);
      logger->log(__PRETTY_FUNCTION__, buf);
    }

    return val;
  }

  double loss(const Dvec& wvec) const {
    double loss = 0;

    for (vector<ObjectiveFunction*>::const_iterator it = objFuncs.begin();
	 it != objFuncs.end();
	 it++) {
      loss += (*it)->loss(wvec);
    }

    return loss;
  }

  double augmentedLoss(const Dvec& wvec) const {
    double loss = 0;

    for (vector<ObjectiveFunction*>::const_iterator it = objFuncs.begin();
	 it != objFuncs.end();
	 it++) {
      loss += (*it)->augmentedLoss(wvec);
    }

    return loss;
  }

  // Gets dimension of weight vector
  int getWeightDim() const { 
    if (objFuncs.size() == 0)
      return 0;
    return objFuncs[0]->getWeightDim(); 
  }

  // Gets dimension of state/label vector
  int getStateDim() const {
    int dim = 0;		

    for (vector<ObjectiveFunction*>::const_iterator it = objFuncs.begin();
	 it != objFuncs.end();
	 it++) {
      dim += (*it)->getStateDim();
    }

    return dim;
  }

  // sums subgradients of component objectives
  void subgradient(const Dvec &wvec,
		   Dvec &subg) const {
    subg.clear();
    //    subg = 0;
    Dvec subSubg(wvec.size());

    for (vector<ObjectiveFunction*>::const_iterator it = objFuncs.begin();
	 it != objFuncs.end();
	 it++) {
      // FIXME: inefficient
      subSubg.clear();
      (*it)->subgradientAccum(wvec, subSubg);
      subg += subSubg;

      if (logger != NULL) {
	char buf[1000];
	snprintf(buf, sizeof(buf), "subSubg: %f", norm_2(subSubg));
	logger->log(__PRETTY_FUNCTION__, buf);
      }

      // FIXME: change this back
      //      (*it)->subgradientAccum(wvec, subg);
    }
  }

  // Project according to the first objective
  void projectFeasibleSet(Dvec &wvec) const {
    if (objFuncs.size() > 0)
      objFuncs[0]->projectFeasibleSet(wvec);
  }

private:
  vector<ObjectiveFunction*> objFuncs;
  ThreadSafeLogger* logger;
};

#endif
