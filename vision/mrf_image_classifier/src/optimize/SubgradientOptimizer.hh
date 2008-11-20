#ifndef __SUBGRAD_OPT_H__
#define __SUBGRAD_OPT_H__

#include "objective/ObjectiveFunction.hh"
#include "objective/Regularization.hh"
#include "util/ThreadSafeLogger.hh"

#include "highgui.h"

/**
  @brief Performs subgradient optiimzation
 */
class SubgradientOptimizer {
public:
  // FIXME: revise this
  // wvec0 is an initial guess for the solution
  // iirWeight is the coefficient of the IIR filter
  SubgradientOptimizer(const ObjectiveFunction *aobj,
		       const Regularization *areg,
		       const Dvec &wvec0, 
		       double aslackPenalty,
		       double aiirCoeff,
		       double astepsize,
		       ThreadSafeLogger* alogger = NULL) : 
    obj(aobj),
    reg(areg),
    wvec(wvec0),
    deltaPast(obj->getWeightDim()),
    lastSubg(obj->getWeightDim()),
    bestSoFar(wvec0),
    //    subgradient(obj->getWeightDim()),
    slackPenalty(aslackPenalty),
    iirCoeff(aiirCoeff),
    stepSize(astepsize),
    bestObjective(HUGE_VAL),
    iteration(0),
    logger(alogger)
  {  
    //    cerr << "Sub loaded " << wvec << endl;
    lastSubg.clear();
    deltaPast.clear();
    //    cvNamedWindow("testing");
  };

  /*
  SubgradientOptimizer(const ObjectiveFunction *aobj,
		       const Regularization *areg,
		       double aslackPenalty,
		       double aiirCoeff,
		       double astepsize) : 
    obj(aobj),
    reg(areg),
    wvec(obj->getWeightDim()),
    deltaPast(obj->getWeightDim()),
    subgradient(obj->getWeightDim()),
    slackPenalty(aslackPenalty),
    iirCoeff(aiirCoeff),
    stepSize(astepsize),
    iteration(0)
  {  
    //    deltaPast = 0; // redundant
    cvNamedWindow("testing");
  };
  */

  /**
     @brief Performs one iteration of subgradient method
   */
  void iterate() {
    iteration++;

    // calculate subgradients
    Dvec nrgSubg(obj->getWeightDim()), regSubg(obj->getWeightDim());

    // FIXME: inefficient...
    nrgSubg.clear();
    regSubg.clear();

    //    cerr << "Sub.opt. " << endl;

    obj->subgradient(wvec, nrgSubg);
    //cout << " ZERO REGULARIZATION " << endl;
    reg->subgradient(wvec, regSubg);

    Dvec delta(obj->getWeightDim());
    delta.clear();
    calcStepIIR(nrgSubg, regSubg, delta);
    //    calcStepAverage(nrgSubg, regSubg, delta);


    /*
    cerr << "Sub. norm " << norm_2(nrgSubg) << 
      " Reg. norm " << norm_2(regSubg) << 
      " Weight norm " << norm_2(wvec) << 
      " Delta norm " << norm_2(delta) << endl;
    */

    if (logger != NULL) {
      char logmess[1000];
      snprintf(logmess, sizeof(logmess), 
	       "Sub. norm %f, Reg. norm %f, Weight norm %f, Delta norm %f",
	       norm_2(nrgSubg), norm_2(regSubg), norm_2(wvec), norm_2(delta));
      
      //    cerr << logmess << endl;

      logger->log(__PRETTY_FUNCTION__, logmess);
    }

    wvec -= delta;

    obj->projectFeasibleSet(wvec);
  }

  void calcStepIIR(const Dvec& nrgSubg, const Dvec& regSubg, Dvec& delta) {
    //    static Dvec0 deltaPast(obj->getWeightDim());
    Dvec subg = slackPenalty*nrgSubg + regSubg;
    normalize(subg);
    delta = 
      (iirCoeff * iterationStepSize() * subg + (1 - iirCoeff) * deltaPast);
    deltaPast = delta;
  }

  void calcStepAverage(const Dvec& nrgSubg, const Dvec& regSubg, Dvec& delta) {
    //    static Dvec0 lastSubg(obj->getWeightDim());
    Dvec subg = slackPenalty*nrgSubg + regSubg;
    normalize(subg);
    delta = 0.5 * iterationStepSize() * (subg + lastSubg);
    lastSubg = subg;
  }

  /**
     @brief Returns current value of objective
     @return Current objective value
   */
  double objective() {
    double energyVal = obj->objective(wvec);

    double regObj = reg->objective(wvec);
    
    double augLoss = obj->augmentedLoss(wvec);
    double loss = obj->loss(wvec);

    cout << "Energy " << energyVal << " Reg " << regObj << 
      " Aug.Loss " << augLoss << " Loss " << loss << endl;
    //    cout << "Energy " << energyVal << ", Reg " << regObj;

    double obj = energyVal + regObj;

    if (obj < bestObjective) {
      bestObjective = obj;
      bestSoFar = wvec;
      cout << "NEW BEST: " << bestObjective << endl;
    }

    return obj;
  }

  /**
     @brief Returns current solution
     @param vec Output argument for current solution vector
   */
  void currentSolution(Dvec &vec) {
    vec = wvec;
  }

  /**
     @brief Returns best solution found so far (approximately)
   */
  double bestSolution(Dvec &outvec) {
    //    cout << "BEST SOL " << bestObjective << std::endl;
    outvec = bestSoFar;
    return bestObjective;
  }

  /**
     Manually sets best solution
   */
  void setBestSolution(Dvec& vec, double val) {
    bestSoFar = vec;
    bestObjective = val; 
  }

  /**
     @param aRate The learning rate to set
   */
  void setLearningRate(double aRate) {
    stepSize = aRate;
  }

  /**
     @param aCp The slack penalty to set
   */
  void setSlackPenalty(double aCp) {
    slackPenalty = aCp;
  }

  /**
     @param iir Sets the IIR filter parameter (should be in range (0,1])
  */
  void setIIRCoeff(double iir) {
    iirCoeff = iir;
  }

  
private:
  const ObjectiveFunction *obj;
  const Regularization *reg;

  // FIXME: is this ok?
  Dvec wvec;			// current solution
  Dvec deltaPast;		// used to do IIR filter
  //  Dvec objSubg;
  //  Dvec regSubg;
  //  Dvec subgradient;
  Dvec lastSubg;
  Dvec bestSoFar;

  double slackPenalty;
  double iirCoeff;
  double stepSize;
  double bestObjective;
  
  //  vector<int> targetState;

  int iteration;

  ThreadSafeLogger* logger;

  void normalize(Dvec& vec) { 
    // FIXME: check for divide by zero
    /*
    double norm = norm_2(vec); 
    assert(norm != 0);
    vec /= norm;
    */
  }

  double iterationStepSize() {
    //    return stepSize / (iteration+1);
    return stepSize;
  };


};

#endif

