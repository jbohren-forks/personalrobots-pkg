#ifndef __BINARY_SUBMOD_IMAGE_CLASSIFIER_H__
#define __BINARY_SUBMOD_IMAGE_CLASSIFIER_H__

#include "cv.h"

#include "objective/BatchObjective.hh"
#include "objective/BinarySubmodularObjective.hh"
#include "util/Dvec.hh"
#include "classifier/ImageClassifier.hh"
#include "objective/L2Regularization.hh"
#include "input/SegmentationLoader.hh"
#include "optimize/SubgradientOptimizer.hh"
#include "util/ThreadSafeLogger.hh"
#include "features/UndirectedFeatureGraph.hh"

#include <assert.h>
#include <stdlib.h>

/**
   @file
 */

/**
   @brief Default slack coefficient
 */
#define CP_DEFAULT 1e15

/**
   @brief Default learning rate
 */
#define RATE_DEFAULT 20e-16
//#define RATE_DEFAULT 20e-10
//#define RATE_DEFAULT 1e2  // with normalization
//#define RATE_DEFAULT 8e-5

//typedef SparseFeatureMatrix<double> tFeatureMatrix;
/**
   @brief The type of feature matrix used
 */
typedef DenseFeatureMatrix<double> tFeatureMatrix;

typedef UndirectedFeatureGraph<tFeatureMatrix> tUndirectedFeatureGraph;


/**
   @brief A binary image classifier based on binary MRFs with unary, submodular 
   binary potentials
   @param oBSICSlackPenalty (Environment variable) if 
   not provided in the constructor, the slack penalty parameter 
   is set to the value of this environment variable.
   @param oBSICLearningRate (Environment variable) if 
   not provided in the constructor, the learning rate 
   parameter is set to the value of this environment variable.
 */
class BinarySubmodularImageClassifier : public ImageClassifier {
public:
  /**
     @param avec An initial weight vector (is copied internally)
     @param aCp An optional slack coefficient
     @param arate An optional learning rate
   */
  BinarySubmodularImageClassifier(const Dvec* avec, 
				  double aCp = -1,
				  double arate = -1) :
    wvec(NULL),
    initialBestSolution(NULL),
    initialBestObjective(HUGE_VAL),
    objFunc(NULL),
    subgOpt(NULL),
    Cp(aCp),
    rate(arate),
    logger(NULL)
  {
    if (aCp < 0)
      if (getenv("oBSICSlackPenalty"))
	Cp = atof(getenv("oBSICSlackPenalty"));
      else 
	Cp = CP_DEFAULT;

    if (arate < 0)
      if (getenv("oBSICLearningRate"))
	rate = atof(getenv("oBSICLearningRate"));
      else
	rate = RATE_DEFAULT;
    
    if (avec != NULL && avec->size() > 0) {
      wvec = new Dvec(*avec);
      //      cout << "wvec: " << *wvec << endl;
    }
  };

  virtual ~BinarySubmodularImageClassifier();

  void loadTrainingData(const vector<IplImage*>& trainImages,
			const vector<IplImage*>& trainSeg);

  /**
     @brief "Raw" version of training data loader
     @param fgraphs Vector of feature graphs
     @param nodeWeights Vector of node weight vectors
     @param targetStates Vector of target states
   */
  void loadTrainingData(const vector<tUndirectedFeatureGraph*>& fgraphs,
			const vector<vector<double> >& nodeWeights,
			const vector<vector<int> >& targetStates);

  /**
     @brief Returns a vector of initial weights 
     @param trainImages A vector of at least one training image
     @return An initial weight vector
     
     This function exists because it is currently not possible to
     determine the dimension of the weight vector without instantiating
     the feature extraction machinery, which requires an image.

     @todo Think of a better way to get an initial weight vector
   */
  static Dvec getInitWeights(const vector<IplImage*>& trainImages);

  void train(int iterations);

  void evaluate(const IplImage* image, IplImage* segmented);

  /**
     @brief "Raw" version of evaluation function
     @param fgraph A feature graph
     @param labels Output argument for the predicted labels
   */
  void evaluate(const tUndirectedFeatureGraph* fgraph, vector<int>& labels);

  /**
     @return Returns the current weight vector
     @attention The returned vector points to internal storage, 
     and therefore may change.  It will also therefore be destroyed 
     when this object is destroyed.
   */
  const Dvec* getWeightVector() const { return wvec; }

  /**
     @return The best objective so far
     @param vec Output argument for best solution found so far
   */
  double getBestSolution(Dvec& vec) const { 
    if (subgOpt == NULL) {
      vec = *wvec;
      return HUGE_VAL;
    } else
      return subgOpt->bestSolution(vec); 
  }

  /** 
      Manually sets best solution found so far.
      Currently only used for deserialization
   */
  void setInitialBestSolution(const Dvec &wvec, double val) {
    initialBestSolution = new Dvec(wvec);
    initialBestObjective = val;
  }

  /**
     @param A logger to be set
   */
  void setLogger(ThreadSafeLogger* logger);

  /**
     @param aCp The slack penalty parameter
   */
  void setSlackPenalty(double aCp) {
    Cp = aCp;

    if (subgOpt != NULL)
      subgOpt->setSlackPenalty(Cp);
  }

  /**
     @param aRate The learning rate parameter
  */
  void setLearningRate(double aRate) {
    rate = aRate;

    if (subgOpt != NULL)
      subgOpt->setLearningRate(rate);
  }

  void viewLossAugmentedMincuts();

private:
  Dvec* wvec;
  Dvec* initialBestSolution;
  double initialBestObjective;

  vector<FeatureGraphExtractor<tFeatureMatrix>*> fgraphxs;
  vector<tUndirectedFeatureGraph*> fgraphs;
  vector<GraphCutMinimizer*> gcuts;
  vector<BinarySubmodularObjective<tFeatureMatrix>*> objFuncs;
  //  vector<int> targetState;
  BatchObjective* objFunc;
  L2Regularization l2reg;
  SubgradientOptimizer *subgOpt;

  double Cp;
  double rate;

  ThreadSafeLogger* logger;
};

#endif
