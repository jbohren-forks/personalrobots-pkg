#include "classifier/NaryImageClassifier.hh"
#include "util/VectorPermutations.hh"
#include "objective/GraphCutMinimizer.hh"

#include <algorithm>
#include <functional>
#include <utility>
#include <vector>

#include <unistd.h>

//#include <boost/lambda/lambda.hpp>
#include <boost/thread.hpp>
#include <boost/timer.hpp>

using namespace boost::lambda;

using namespace std;

#define N_TRAINING_THREADS 4

class TrainingThreadSyncData {
public:
  TrainingThreadSyncData() :
    numThreads(0),
    threadWaiting(false)
  {
  };

  ~TrainingThreadSyncData() {
    assert(numThreads == 0 && threadWaiting == false);

    // FIXME: not sure why this is necessary
    numThreadsMutex.lock();
    numThreadsMutex.unlock();  
    ioMutex.lock();
    ioMutex.unlock();
  } 

  int numThreads;
  bool threadWaiting;
  boost::mutex numThreadsMutex;
  boost::mutex ioMutex;
  boost::condition freeSlotCond;
  boost::condition noRunningThreadCond;
};

// FIXME: make internal I/O thread-safe
class TrainingThread {
public:
  TrainingThread(NaryImageClassifier* aclassifier,
		 int aiterations, 
		 BinaryClassifierTree* aclassTree,
		 TrainingThreadSyncData* asyncData) :
    classifier(aclassifier),
    iterations(aiterations),
    classTree(aclassTree),
    syncData(asyncData)
  {
  }

  void operator()() {
    // first ensure that there is a slot open
    {
      boost::mutex::scoped_lock lock(syncData->numThreadsMutex);
      while (syncData->numThreads >= N_TRAINING_THREADS) {
	syncData->threadWaiting = true;
	syncData->freeSlotCond.wait(syncData->numThreadsMutex);	
	syncData->threadWaiting = false;
	//	sleep(2);
      }
      assert(syncData->numThreads < N_TRAINING_THREADS);
      syncData->numThreads++;
    }

    ClassifierNode* cnode = classTree->getTag();

    cerr << "Thread out" << endl;

    if (!cnode->isNull()) {
      // spawn children threads
      pair<BinaryClassifierTree*, BinaryClassifierTree*> children = 
	classTree->children();

      boost::thread thread0, thread1;

      if (children.first != NULL) {
	TrainingThread callable(classifier, iterations, 
				children.first, syncData);
	thread0 = boost::thread(callable);
      }

      if (children.second != NULL) {
	TrainingThread callable(classifier, iterations, 
				children.second, syncData);
	thread1 = boost::thread(callable);
      }

      cerr << "Spawned children" << endl;

      // train the classifier at this node
      { 
	boost::mutex::scoped_lock lock(syncData->ioMutex);
	cout << "Training node with scope... ";
	cnode->getScope().prettyPrint(cout);
	cout << endl;
      }

      cerr << "Thread is training ... " << endl;

      cnode->getClassifier()->train(iterations);
    }

    // free up this thread's slot
    { 
      boost::mutex::scoped_lock lock(syncData->numThreadsMutex);
      syncData->numThreads--;

      // should always be true...
      if (syncData->numThreads < N_TRAINING_THREADS)
	syncData->freeSlotCond.notify_one();

      if (syncData->numThreads == 0 && !syncData->threadWaiting) 
	syncData->noRunningThreadCond.notify_all();
    }
  }

  void waitAllThreads() {
    boost::mutex::scoped_lock lock(syncData->numThreadsMutex);
    if (syncData->numThreads > 0 || syncData->threadWaiting)
      syncData->noRunningThreadCond.wait(syncData->numThreadsMutex);
  }

private:
  NaryImageClassifier* classifier;
  int iterations;
  BinaryClassifierTree* classTree;
  TrainingThreadSyncData* syncData;
};

class TreeSetLogStream { 
public: 
  TreeSetLogStream(ThreadSafeLogger* alogger) : logger(alogger) {};

  void operator()(ClassifierNode* cnode) {
    BinarySubmodularImageClassifier* classifier = cnode->getClassifier();
    classifier->setLogger(logger);
  }

private:
  ThreadSafeLogger* logger;
};

NaryImageClassifier::
NaryImageClassifier(BinaryClassifierTree* classes,
		    ThreadSafeLogger* alogger) :
  classifierTree(classes),
  logger(alogger)
{
  if (alogger != NULL) {
    TreeSetLogStream treeSetter(logger);
    classifierTree->iter(treeSetter);
  }
}

NaryImageClassifier::
~NaryImageClassifier() {
  //  delete referenced classifiers
  //  BinaryClassTreeUtils::deleteClassifiers(&classTree);
  deleteClassifierNodes(*classifierTree);
  BinaryClassifierTree::deleteTree(classifierTree);

  for (int ii = 0; ii < (int)vFgraphEx.size(); ii++) 
    delete vFgraphEx[ii];

  for (int ii = 0; ii < (int)vFgraphs.size(); ii++) 
    delete vFgraphs[ii];

  for (int ii = 0; ii < (int)subGraphs.size(); ii++) 
    delete subGraphs[ii];
}

void NaryImageClassifier::
deleteClassifierNodes(BinaryClassifierTree& ctree) {
  ClassifierNode* cnode = ctree.getTag();

  delete cnode;

  pair<BinaryClassifierTree*, BinaryClassifierTree*> children = 
    ctree.children();
  
  if (children.first != NULL)
    deleteClassifierNodes(*children.first);
  if (children.second != NULL)
    deleteClassifierNodes(*children.second);
}

BinaryClassifierTree* NaryImageClassifier::
makeRandomBCTree(const ObjectSet& objset,
		 const Dvec& wvec0) {
  vector<ObjectClass> classes;
  objset.vectorize(classes);
  //  cout << "ROOT CLASSES: " << endl;
  //  objset.serialize(cout);

  BinaryClassifierTree* tree = 
    new BinaryClassifierTree(new ClassifierNode(wvec0, wvec0, HUGE_VAL, objset));

  if (classes.size() > 1) {
    vector<ObjectClass> permuted = 
      VectorPermutations::randomPermutation(classes, classes.size());
    
    int splitpos = permuted.size() / 2;
    
    vector<ObjectClass> left(permuted.begin(), 
			     permuted.begin() + splitpos);
    vector<ObjectClass> right(permuted.begin() + splitpos,
			      permuted.end());

    ObjectSet oleft(left);
    ObjectSet oright(right);
    
    //    cout << "LEFT RECURSION" << endl;
    tree->addChild0(makeRandomBCTree(oleft, wvec0));
    //    cout << "RIGHT RECURSION" << endl;
    tree->addChild1(makeRandomBCTree(oright, wvec0));
  }

  return tree;  
}

BinaryClassifierTree* NaryImageClassifier::
makeRandomBCTree(const ObjectSet& objset,
		 const vector<IplImage*>& images) {
  Dvec wvec0 = 
    BinarySubmodularImageClassifier::getInitWeights(images);

  return makeRandomBCTree(objset, wvec0);
}

// split nodes into equivalence classes according to which 
// scope they fall into.
// each output vector is a list of nodes in each scope
void NaryImageClassifier::
splitNodesByScope(const vector<vector<int> > &vlabels,
		  const ObjectSet& scope0, 
		  const ObjectSet& scope1,
		  vector<NodeSet>& vnodes0, 
		  vector<NodeSet>& vnodes1) {

  for (int vi = 0; vi < (int)vlabels.size(); vi++) {
    vector<int> labels = vlabels[vi];
    NodeSet nodes0, nodes1;

    for (int li = 0; li < (int)labels.size(); li++) {
      ObjectClass oclass(labels[li], "");

      if (scope0.find(oclass) != scope0.end())
	nodes0.insert(li);
      else if (scope1.find(oclass) != scope1.end())
	nodes1.insert(li);
      else
	throw "Node in neither scope";
    }
    vnodes0.push_back(nodes0);
    vnodes1.push_back(nodes1);
  }
}

/*
// split sets of vectors into two groups according to index sets
template <class T>
void NaryImageClassifier::
splitVectorsByIndices(const vector<vector<T> >& vecin,
		      const vector<vector<T> >& inds0,
		      const vector<vector<T> >& inds1,
		      vector<vector T> > &vecs0, 
		      vector<vector T> > &vecs1) {

  for (vector<vector<T> >::iterator it = inds0.begin();
       it != inds0.end();
       it++) {
    
  }
}
*/

void NaryImageClassifier::
train(int iterations) {
  /*
    // single-threaded version
  train(iterations, *classifierTree);

  return;
  */

  TrainingThreadSyncData syncData;
  TrainingThread callable(this, iterations, classifierTree, &syncData);
  boost::thread thread(callable);
  thread.join();
  callable.waitAllThreads();
}

// single-threaded version
void NaryImageClassifier::
train(int iterations, BinaryClassifierTree& classTree) {
  ClassifierNode* cnode = classTree.getTag();

  if (cnode->isNull()) 
    return;

  cout << "Training node with scope... ";
  cnode->getScope().prettyPrint(cout);
  cout << endl;

  cnode->getClassifier()->train(iterations);

  pair<BinaryClassifierTree*, BinaryClassifierTree*> children = 
    classTree.children();

  if (children.first != NULL) {
    train(iterations, *children.first);
  }

  if (children.second != NULL) {
    train(iterations, *children.second);
  }
}

// relabel to 0/1 according to equivalence classes in 
// the scope of this classifier
void NaryImageClassifier::
relabelNodesToBinary(const vector<vector<int> >& vlabels,
		     const ObjectSet& scope0,
		     const ObjectSet& scope1,
		     vector<vector<int> >& vnewLabels) {

  for (int vi = 0; vi < (int)vlabels.size(); vi++) {
    vector<int> oldLabels = vlabels[vi];
    vector<int> newLabels;
    newLabels.reserve(oldLabels.size());

    for (int li = 0; li < (int)oldLabels.size(); li++) {
      ObjectClass oclass(oldLabels[li], "");

      if (scope0.find(oclass) != scope0.end())
	newLabels.push_back(0);
      else if (scope1.find(oclass) != scope1.end())
	newLabels.push_back(1);
      else
	throw "Node in neither scope.  Incorrect # of objects in object set?";
    }

    vnewLabels.push_back(newLabels);
  }
}

// Load training data from IplImages
void NaryImageClassifier::
loadTrainingData(const vector<IplImage*>& trainImages,
		 const vector<IplImage*>& trainSeg) {
  assert(trainImages.size() == trainSeg.size());

  if (logger != NULL)
    logger->log(__PRETTY_FUNCTION__, "loading training data");
 
  // node weights, per-image and per-blob
  vector<vector<double> > vnodeWeights;

  // labels, per-image and per-blob
  vector<vector<int> > vlabels;

  // mapping from blob indices to node indices, which 
  // changes as we create subgraphs with different node numberings
  vector<Int2IntMap> vblob2NodeInds;

  for (int ii = 0; ii < (int)trainImages.size(); ii++) {
    FeatureGraphExtractor<tFeatureMatrix>* fgx = 
      new FeatureGraphExtractor<tFeatureMatrix>(trainImages[ii]);
    tUndirectedFeatureGraph* fg = fgx->getFeatureGraph();
    vFgraphs.push_back(fg);
    vFgraphEx.push_back(fgx);

    vector<int> labels;
    SegmentationLoader::loadBlobSegmentation(*fgx->getBlobber(),
					     trainSeg[ii],
					     labels);    
    vlabels.push_back(labels);
    
    vector<double> labelWeights;
    fgx->getBlobWeights(labelWeights);
    vnodeWeights.push_back(labelWeights);

    // initialize to the identity map
    Int2IntMap blob2NodeInds;
    for (int ii = 0; ii < fgx->getBlobber()->numBlobs(); ii++) {
      blob2NodeInds[ii] = ii;
    }
    vblob2NodeInds.push_back(blob2NodeInds);
  }

  loadTrainingData(vFgraphEx, vFgraphs, vnodeWeights, 
		   vlabels, vblob2NodeInds, classifierTree);
}

// maps each key in map0 to a value in map1 by composing the two maps
static Int2IntMap composeMaps(const Int2IntMap map0, const Int2IntMap map1) {
  Int2IntMap rmap;

  /*
  for (Int2IntMap::const_iterator it = map0.begin(); it != map0.end(); it++) {
    pair<int,int> keyval = *it;
    cout << "M0 " << keyval.first << " " << keyval.second << endl;
  }

  for (Int2IntMap::const_iterator it = map1.begin(); it != map1.end(); it++) {
    pair<int,int> keyval = *it;
    cout << "M1 " << keyval.first << " " << keyval.second << endl;
  }
  */

  for (Int2IntMap::const_iterator it = map0.begin();
       it != map0.end();
       it++) {
    pair<int,int> keyval = *it;
    Int2IntMap::const_iterator mapped1 = map1.find(keyval.second);

    if (mapped1 != map1.end()) {
      pair<int,int> keyval1 = *mapped1;
      rmap[keyval.first] = keyval1.second;
    }
  }

  return rmap;
}

/*
template<class T, size_t nIndices> 
static void splitVector(const vector<T>& vecIn,
			const unary_function<T, int>& indexer,
			vector<T> (&outVecs)[nIndices]) {
  for (typename vector<T>::const_iterator it = vecIn.begin();
       it != vecIn.end();
       it++) {
    int index = indexer(*it);
    assert(index >= 0 && index < nIndices);
    outVecs[index] = *it;
  }
}
*/

// Constructs classifiers at each node in the classifier tree,
// loads them with training data
// vblob2NodeInds is a mapping from blob indices to node indices
// used for debugging purposes
void NaryImageClassifier::
loadTrainingData(const vector<FeatureGraphExtractor<tFeatureMatrix>*>& vfgraphxs,
		 const vector<tUndirectedFeatureGraph*>& vfgraphs,
		 const vector<vector<double> >& vnodeWeights,
		 const vector<vector<int> >& vlabels,
		 const vector<Int2IntMap>& vblob2NodeInds,
		 BinaryClassifierTree *classTree) {
  
  ClassifierNode* cnode = classTree->getTag();

  pair<BinaryClassifierTree*, BinaryClassifierTree*> children = 
    classTree->children();

  // if either child is null, this is a leaf node (no more classifiers)
  if (children.first == NULL || children.second == NULL) {
    // should only happen if there is only one class left in this branch
    assert(cnode->getScope().size() == 1);

    cnode->setNull(true);
    return;
  }

  // kill this branch if there is no training data left in it
  bool haveTrainingData = false;
  for (vector<tUndirectedFeatureGraph*>::const_iterator it = vfgraphs.begin();
       it != vfgraphs.end();
       it++) {
    if (*it != NULL) 
      haveTrainingData = true;
  }

  if (!haveTrainingData) {
    cnode->setNull(true);
    return;
  }
	   
  // debugging
  if (0) {
    cout << "Scope: ";
    cnode->getScope().prettyPrint(cout);
    cout << endl;

    for (int vi = 0; vi < (int)vfgraphs.size(); vi++) {

      cout << "Image " << vi << endl;

      FeatureGraphExtractor<tFeatureMatrix>* fgx = vfgraphxs[vi];
      tUndirectedFeatureGraph* fg = vfgraphs[vi];

      if (fg == NULL) {
	cerr << "Null graph " << endl;
	continue;
      }

      const Int2IntMap* blob2NodeInds = &vblob2NodeInds[vi];

      fgx->displayGraph(*fg->getEdgeList(), blob2NodeInds);
      cvWaitKey();
    }
  }

  // create a classifier to separate classes in scope of 
  // left child from classes in scope of right child

  ObjectSet scope0 = children.first->getTag()->getScope();
  ObjectSet scope1 = children.second->getTag()->getScope();

  // relabel nodes to binary according to childrens' scopes
  // so we can use binary classifier
  vector<vector<int> > binaryLabels;
  relabelNodesToBinary(vlabels, scope0, scope1, binaryLabels);

  cnode->getClassifier()->
    loadTrainingData(vfgraphs, vnodeWeights, binaryLabels);

  if (getenv("oNSICSlackPenalty")) 
    cnode->getClassifier()->setSlackPenalty(atof(getenv("oNSICSlackPenalty")));

  if (getenv("oNSICLearningRate"))
    cnode->getClassifier()->setLearningRate(atof(getenv("oNSICLearningRate")));

  // split the training data into two groups as needed for the 
  // children classifiers
  vector<NodeSet> vnodes[2];
  splitNodesByScope(vlabels, 
		    children.first->getTag()->getScope(),
		    children.second->getTag()->getScope(),
		    vnodes[0], vnodes[1]);

  vector<tUndirectedFeatureGraph*> cfgraphs[2];
  vector<vector<double> > cnodeWeights[2];
  vector<vector<int> > clabels[2];
  vector<Int2IntMap> cblob2NodeInds[2];

  // create a subgraph for each equivalence class
  for (int vi = 0; vi < (int)vfgraphs.size(); vi++) {
    tUndirectedFeatureGraph* fgraph = vfgraphs[vi];
    const Int2IntMap* blob2NodeMap0 = &vblob2NodeInds[vi];

    for (int ci = 0; ci < 2; ci++) {
      NodeSet nodeSet = (vnodes[ci])[vi];

      tUndirectedFeatureGraph* subgraph = NULL;

      Int2IntMap old2NewMap;
      if (nodeSet.size() > 0) {
	subgraph = new tUndirectedFeatureGraph(*fgraph);
	subgraph->subgraph(nodeSet, old2NewMap);
	cblob2NodeInds[ci].push_back(composeMaps(*blob2NodeMap0, old2NewMap));
	subGraphs.push_back(fgraph);
      } else {
	cblob2NodeInds[ci].push_back(old2NewMap);
      }

      cfgraphs[ci].push_back(subgraph);
    }
  }

  // split up labels, weights 
  for (int vi = 0; vi < (int)vlabels.size(); vi++) {
    vector<int> labels = vlabels[vi];
    vector<double> nodeWeights = vnodeWeights[vi];
    vector<int> tclabels[2];
    vector<double> tcnodeWeights[2];
    NodeSet nodeSet0 = (vnodes[0])[vi];
    NodeSet nodeSet1 = (vnodes[1])[vi];
    
    for (int node = 0; node < (int)labels.size(); node++) {
      if (nodeSet0.find(node) != nodeSet0.end()) {
	tclabels[0].push_back(labels[node]);
	tcnodeWeights[0].push_back(nodeWeights[node]);
      } else if (nodeSet1.find(node) != nodeSet1.end()) {
	tclabels[1].push_back(labels[node]);
	tcnodeWeights[1].push_back(nodeWeights[node]);
      } else {
	throw "Node in neither scope";
      }
    }
    clabels[0].push_back(tclabels[0]);
    clabels[1].push_back(tclabels[1]);
    cnodeWeights[0].push_back(tcnodeWeights[0]);
    cnodeWeights[1].push_back(tcnodeWeights[1]);
  }

  loadTrainingData(vfgraphxs, cfgraphs[0], cnodeWeights[0], 
		   clabels[0], cblob2NodeInds[0], children.first);
  loadTrainingData(vfgraphxs, cfgraphs[1], cnodeWeights[1], 
		   clabels[1], cblob2NodeInds[1], children.second);
}

void NaryImageClassifier::
evaluate(const IplImage* image, IplImage* segmented) {

  FeatureGraphExtractor<tFeatureMatrix> fgx(image);
  tUndirectedFeatureGraph* fg = fgx.getFeatureGraph();

  vector<int> labeling = evaluate(fg, classifierTree);

  SegmentationLoader::
    writeBlobSegmentation(*fgx.getBlobber(), labeling, segmented);

  delete fg;  
}

// return proportion of correctly labeled pixels, 
// mask of errors (must be preallocated 8-bit image)
double NaryImageClassifier::
evaluateErrors(const IplImage* segmented, const IplImage* trueSegmentation,
	       IplImage* errors) {
  cvCmp(trueSegmentation, segmented, errors, CV_CMP_EQ);
  int nnz = cvCountNonZero(errors);
  CvSize size = cvGetSize(segmented); 
  return (double)nnz / (size.height * size.width);
}

// return proportion of correctly labeled pixels, 
// mask of errors (must be preallocated 8-bit image)
double NaryImageClassifier::
evaluate(const IplImage* image, const IplImage* trueSegmentation,
	 IplImage* segmented, IplImage* errors) {
  evaluate(image, segmented);
  return evaluateErrors(image, trueSegmentation, errors);
}

// also returns vector of per-blob labels and per-blob statistics
void NaryImageClassifier::
evaluate(const IplImage* image, IplImage* segmented, 
	 vector<int>& labeling, vector<blobStat>& blobStats) {

  boost::timer totalTimer;

  boost::timer featureTimer;

  if (getenv("oDebugOn"))
    std::cout << "Segmenting image: " << std::endl;

  FeatureGraphExtractor<tFeatureMatrix> fgx(image);
  tUndirectedFeatureGraph* fg = fgx.getFeatureGraph();

  if (getenv("oDebugOn"))
    std::cout << "Features took " << featureTimer.elapsed() << "s" << std::endl;

  boost::timer evalTimer;

  labeling = evaluate(fg, classifierTree);

  //  *blobStats = fgx.getBlobber()->getBlobStats();
  blobStats = fgx.getBlobber()->getBlobStatsCopy();

  SegmentationLoader::
    writeBlobSegmentation(*fgx.getBlobber(), labeling, segmented);

  if (getenv("oDebugOn")) {
    std::cout << "C.Tree ev. took " << evalTimer.elapsed() << "s" << std::endl;
    std::cout << "Total evaluation took " << totalTimer.elapsed() << "s"
	      << std::endl;
  }

  delete fg;
}

// evaluates classifier on given feature graph using classifier 
// at root of classTree, proceeds recursively
vector<int> NaryImageClassifier::
evaluate(const tUndirectedFeatureGraph* fgraph, 
	 const BinaryClassifierTree* classTree) {

  vector<int> labeling;

  if (fgraph->numNodes() == 0)
    return labeling;

  ClassifierNode* cnode = classTree->getTag();

  /*
  cout << "Scope " ;
  cnode.scope.prettyPrint(cout);
  cout << endl;
  */

  // if we arrived at a leaf node in the classifier tree,
  // label everything with its label
  if (cnode->getScope().size() == 1) {
    ObjectClass oclass = *(cnode->getScope().begin()); 
    for (int ii = 0; ii < fgraph->numNodes(); ii++) {
      labeling.push_back(oclass.id);
    }
    return labeling;
  }

  // if there is no classifier here, label everything with special label
  if (cnode->isNull()) {
    for (int ii = 0; ii < fgraph->numNodes(); ii++) {
      labeling.push_back(NULL_OBJECT_LABEL);
    }
    return labeling;
  }

  // otherwise, evaluate this classifier
  /*
  GraphCutMinimizer gcut(fgraph->getEdgeList());
  BinarySubmodularEnergy energy(fgraph);
  vector<int> metaLabels;
  energy.groundState(cnode.wvec, metaLabels, gcut);
  */
  vector<int> metaLabels;
  cnode->getClassifier()->evaluate(fgraph, metaLabels);

  // split indices into two sets according to labels
  NodeSet nodeSets[2];
  for (int ii = 0; ii < (int)metaLabels.size(); ii++) {
    if (metaLabels[ii] == 0)
      nodeSets[0].insert(ii);
    else
      nodeSets[1].insert(ii);
  }  

  // recursively evaluate descendant classifiers
  pair<const BinaryClassifierTree*, const BinaryClassifierTree*> pchildren = 
    classTree->const_children();
    
  const BinaryClassifierTree* children[] = 
    { pchildren.first, pchildren.second };
  vector<int> splitLabels[2];  // node labels of each subgraph
  Int2IntMap oldToNewInds[2]; // maps node indices in graph to subgraph indices

  // evaluate left child on everything labeled 0, 
  // evaluate right child on everything labeled 1
  for (int ii = 0; ii < 2; ii++) {
    const BinaryClassifierTree* child = children[ii];
    if (child == NULL) {
      assert(cnode->getScope().size() == 1);
      continue;
    } else {
      if (nodeSets[ii].size() != 0) {
	// evaluate child classifier on subgraph of nodes in its scope
	tUndirectedFeatureGraph* subgraph = 
	  new tUndirectedFeatureGraph(*fgraph);
	subgraph->subgraph(nodeSets[ii], oldToNewInds[ii]);

	//	cout << "Subgraph size " << subgraph->numNodes() << endl;
	
	splitLabels[ii] = evaluate(subgraph, child);
	
	delete subgraph;
      }
    }
  }

  // merge results of children classifiers
  for (int ii = 0; ii < (int)metaLabels.size(); ii++) {
    int metaLabel = metaLabels[ii];
    int subindex = (oldToNewInds[metaLabel])[ii];
    int label = (splitLabels[metaLabel])[subindex];
    labeling.push_back(label);
  }

  /*
  cout << "L ";
  for_each(labeling.begin(), labeling.end(), cout << _1 << "-" );
  cout << endl;
  */

  return labeling;
}

/*
void NaryImageClassifier::
viewTrainingData() {
  viewTrainingData(classTree);
}

void NaryImageClassifier::
viewTrainingData(const BinaryClassTree* classTree) {
  
}
*/

void BinaryClassTreeUtils::
serialize(const BinaryClassifierTree* classTree, std::ostream& ostr) 
{
  ostr << "ctree" << endl;

  if (classTree == NULL) {
    ostr << "null" << endl;
    return;
  } 

  ostr << "notnull" << endl;
  
  ClassifierNode* cnode = classTree->getTag();
  
  cnode->serialize(ostr);
  
  pair<const BinaryClassifierTree*, 
    const BinaryClassifierTree*> children = 
    classTree->const_children();
  
  serialize(children.first, ostr);
  serialize(children.second, ostr);
}

BinaryClassifierTree* BinaryClassTreeUtils::
deserialize(std::istream &istr) {
  string sctree;
  istr >> sctree;
  if (sctree.compare("ctree") != 0)
    throw DeserializationError();

  string nullness;
  istr >> nullness;
  if (nullness.compare("null") == 0)
    return NULL;
  else if (nullness.compare("notnull") != 0)
    throw DeserializationError();

  BinaryClassifierTree* ctree = 
    new BinaryClassifierTree(ClassifierNode::deserialize(istr));
  ctree->addChild0(deserialize(istr));
  ctree->addChild1(deserialize(istr));
  return ctree;
}

/*
void BinaryClassTreeUtils::
deleteClassifiers(BinaryClassTree* classTree) {
  if (classTree == NULL) return;

  BinarySubmodularImageClassifier* classifier = 
    classTree->getTag().classifier;
  
  if (classifier != NULL)
    delete classifier;
  
  pair<BinaryClassTree*, BinaryClassTree*> children = classTree->children(); 
  
  deleteClassifiers(children.first);
  deleteClassifiers(children.second);
}

*/
