#include "classifier/BinarySubmodularImageClassifier.hh"
#include "display/LabelingViewer.hh"

#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <algorithm>

#define IIR_COEFF 0.25

using namespace boost::lambda;

BinarySubmodularImageClassifier::
~BinarySubmodularImageClassifier() {
  for (vector<tUndirectedFeatureGraph*>::iterator it = fgraphs.begin();
       it != fgraphs.end();
       it++) {
    delete *it;
  }

  for (vector<FeatureGraphExtractor<tFeatureMatrix>*>::iterator it = fgraphxs.begin();
       it != fgraphxs.end();
       it++) {
    delete *it;
  }

  for (vector<GraphCutMinimizer*>::iterator it = gcuts.begin();
       it != gcuts.end();
       it++) {
    delete *it;
  }

  for (vector<BinarySubmodularObjective<tFeatureMatrix>*>::iterator it = 
	 objFuncs.begin();
       it != objFuncs.end();
       it++) {
    delete *it;
  }

  if (wvec != NULL)
    delete wvec;

  if (initialBestSolution != NULL)
    delete initialBestSolution;

  delete objFunc;

  delete subgOpt;
}

/*
void BinarySubmodularImageClassifier::
loadTrainingData(const vector<IplImage*> trainImages,
		 const vector<IplImage*> trainSeg,
		 const vector<IplImage*> trainMasks) {
*/

/*
void BinarySubmodularImageClassifier::
loadTrainingData(const vector<IplImage*> trainImages,
		 const vector<IplImage*> trainSeg) {
  assert(trainImages.size() == trainSeg.size());

  //  vector<ObjectiveFunction> objFuncs;

  // create batch objective function
  for (int ii = 0; ii < (int)trainImages.size(); ii++) {
    // extract feature graph
    FeatureGraphExtractor fgx(trainImages[ii]);
    tUndirectedFeatureGraph *fg = fgx.getFeatureGraph();
    fgraphs.push_back(fg);

    // weights (blob sizes) for loss function
    vector<double> labelWeights;
    fgx.getBlobWeights(labelWeights);

    // get labeling vector
    vector<int> targetState;
    SegmentationLoader::
      loadBlobSegmentation(*fgx.getBlobber(), 
			   trainSeg[ii],
			   targetState);

    // choose graph cut minimizer
    GraphCutMinimizer *gcut = new GraphCutMinimizer(fg->getEdgeList());
    gcuts.push_back(gcut);

    // create objective function
    BinarySubmodularEnergy energy(fg);
    BinarySubmodularObjective *objFunc = 
      new BinarySubmodularObjective(energy, *gcut, targetState, labelWeights);

    objFuncs.push_back(objFunc);
    //    delete objFunc;
  }

  objFunc = new BatchObjective(objFuncs);

  subgOpt = new SubgradientOptimizer(objFunc, &l2reg, wvec,
				     Cp, 1.0, rate);
}
*/
void BinarySubmodularImageClassifier::
loadTrainingData(const vector<IplImage*>& trainImages,
		 const vector<IplImage*>& trainSeg) {

  vector<tUndirectedFeatureGraph*> fgraphs;
  vector<vector<double> > nodeWeights;
  vector<vector<int> > targetStates;

  for (int ii = 0; ii < (int)trainImages.size(); ii++) {
    // extract feature graph
    FeatureGraphExtractor<tFeatureMatrix> *fgx = 
      new FeatureGraphExtractor<tFeatureMatrix>(trainImages[ii]);
    tUndirectedFeatureGraph *fg = fgx->getFeatureGraph();

    fgraphxs.push_back(fgx);
    fgraphs.push_back(fg);

    // weights (blob sizes) for loss function
    vector<double> labelWeights;
    fgx->getBlobWeights(labelWeights);

    nodeWeights.push_back(labelWeights);

    // get labeling vector
    vector<int> targetState;
    SegmentationLoader::
      loadBlobSegmentation(*fgx->getBlobber(), 
			   trainSeg[ii],
			   targetState);

    targetStates.push_back(targetState);
  }

  loadTrainingData(fgraphs, nodeWeights, targetStates);
}

Dvec BinarySubmodularImageClassifier::
getInitWeights(const vector<IplImage*>& trainImages) {
  FeatureGraphExtractor<tFeatureMatrix> fgx(trainImages[0]);
  tUndirectedFeatureGraph* fg = fgx.getFeatureGraph();
  BinarySubmodularEnergy<tFeatureMatrix> nrg(fg);
  Dvec vec = Dvec(nrg.getWeightDim());
  vec.clear();
  delete fg;
  return vec;
}

// null feature graphs are skipped
void BinarySubmodularImageClassifier::
loadTrainingData(const vector<tUndirectedFeatureGraph*>& afgraphs,
		 const vector<vector<double> >& nodeWeights,
		 const vector<vector<int> >& targetStates) {
  //  vector<ObjectiveFunction> objFuncs;

  fgraphs = afgraphs;

  int weightDim = 0;		

  // create batch objective function
  for (int ii = 0; ii < (int)fgraphs.size(); ii++) {
    tUndirectedFeatureGraph* fg = fgraphs[ii];

    if (fg == NULL)
      continue;

    // weights (blob sizes) for loss function
    vector<double> labelWeights = nodeWeights[ii];

    // get labeling vector
    vector<int> targetState = targetStates[ii];

    // choose graph cut minimizer
    GraphCutMinimizer *gcut = new GraphCutMinimizer(fg->getEdgeList());
    gcuts.push_back(gcut);

    // create objective function
    BinarySubmodularEnergy<tFeatureMatrix> energy(fg);
    BinarySubmodularObjective<tFeatureMatrix> *tobjFunc = 
      new BinarySubmodularObjective<tFeatureMatrix>
      (energy, *gcut, targetState, labelWeights, logger);

    //    cout << "C " << tobjFunc << endl;

    weightDim = energy.getWeightDim();

    objFuncs.push_back(tobjFunc);
    //    delete objFunc;
  }

  vector<ObjectiveFunction*> genericObjFuncs;
  for_each(objFuncs.begin(), objFuncs.end(), 
	   bind(&vector<ObjectiveFunction*>::push_back, 
		&genericObjFuncs, 
		_1) );

  objFunc = new BatchObjective(genericObjFuncs, logger);

  if (wvec == NULL) {
    wvec = new Dvec(weightDim);
    wvec->clear();
    // randomize weight vector
    //    for (int ii = 0; ii < weightDim; ii++)
    //      (*wvec)(ii) = drand48();
  }  
    
  subgOpt = new SubgradientOptimizer(objFunc, &l2reg, *wvec, 
				     Cp, IIR_COEFF, rate,
				     logger);

  if (initialBestSolution != NULL)
    subgOpt->setBestSolution(*initialBestSolution, initialBestObjective);
}

void BinarySubmodularImageClassifier::
train(int iterations) {
  

  for (int ii = 0; ii < iterations; ii++) {

    subgOpt->iterate();

    //viewLossAugmentedMincuts();


    /// @fixme expose this parameter somewhere
    if ((ii % 10) == 0) {
    // FIXME: throttle this
    //    if ((ii % 1) == 0) {
      //      cout << "Iteration " << ii << endl;
      subgOpt->objective();
    }
  }
    
  subgOpt->currentSolution(*wvec);
}

/*
void BinarySubmodularImageClassifier::
evaluate(const IplImage* image, IplImage* segmented) {
  vector<int> groundState;

  for (int ii = 0; ii < objFuncs.size(); ii++) {
  objFuncs[ii]->energyGroundState(wvec, groundState, *gcuts[ii]);
  }
  }
*/

void BinarySubmodularImageClassifier::
evaluate(const IplImage* image, IplImage* segmented) {
  FeatureGraphExtractor<tFeatureMatrix> fgx(image);
  tUndirectedFeatureGraph *fg = fgx.getFeatureGraph();
  GraphCutMinimizer gcut(fg->getEdgeList());
  BinarySubmodularEnergy<tFeatureMatrix> energy(fg);

  //  wvec->assertFinite();
  Dvec wbest(wvec->size());
  subgOpt->bestSolution(wbest);

  vector<int> groundState;
  energy.groundState(wbest, groundState, gcut);
    
  SegmentationLoader::
    writeBlobSegmentation(*fgx.getBlobber(), groundState, segmented);
}

void BinarySubmodularImageClassifier::
evaluate(const tUndirectedFeatureGraph* fgraph, vector<int>& labels) {
  GraphCutMinimizer gcut(fgraph->getEdgeList());
  BinarySubmodularEnergy<tFeatureMatrix> energy(fgraph);

  Dvec wbest(wvec->size());
  subgOpt->bestSolution(wbest);

  energy.groundState(wbest, labels, gcut);
}

void BinarySubmodularImageClassifier::
viewLossAugmentedMincuts() {
  for (int im = 0; im < (int)fgraphs.size(); im++) {
    int nNodes = fgraphs[im]->numNodes();
    int nEdges = fgraphs[im]->numEdges();

    Dvec e0(nNodes), e1(nNodes), e00(nEdges), e11(nEdges);
    vector<int> groundState;
    vector<int> targetState = objFuncs[im]->getTargetState();
    objFuncs[im]->lossAugmentedEnergies(targetState, *wvec, e0, e1, e00, e11);
    gcuts[im]->energyGroundState(e0, e1, e00, e11, groundState);
    
    char buf[255];
    snprintf(buf, sizeof(buf), "%d", im);
    string winName("lossAug");
    winName += buf;
    FeatureGraphExtractor<DenseFeatureMatrix<double> > *fgx = fgraphxs[im];
    Blobber* blobber = fgx->getBlobber();
    LabelingViewer viewer(winName, *blobber, 2);
    viewer.viewLabeling(groundState);

    string winName2("mincut");
    vector<int> groundState2;
    winName2 += buf;
    LabelingViewer viewer2(winName2, *fgraphxs[im]->getBlobber(), 2);
    BinarySubmodularEnergy<tFeatureMatrix> energy(fgraphs[im]);
    energy.factorEnergies(*wvec, e0, e1, e00, e11);    
    gcuts[im]->energyGroundState(e0, e1, e00, e11, groundState2);
    viewer2.viewLabeling(groundState2);    
  }
  cvWaitKey();  
}

void BinarySubmodularImageClassifier::
setLogger(ThreadSafeLogger* alogger) {
  logger = alogger;
}
