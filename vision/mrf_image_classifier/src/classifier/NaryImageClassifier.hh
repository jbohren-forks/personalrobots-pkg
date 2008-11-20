#ifndef __NARY_IMAGE_CLASSIFIER_H__
#define __NARY_IMAGE_CLASSIFIER_H__

#include "cv.h"

#include "util/BinaryTree.hh"
#include "util/Dvec.hh"
#include "objective/BinarySubmodularEnergy.hh"
#include "classifier/BinarySubmodularImageClassifier.hh"
#include "classifier/ObjectSet.hh"
#include "util/ThreadSafeLogger.hh"

#include <cstdlib>
#include <stdexcept>

#define NULL_OBJECT_LABEL -1

// a node in the classifier tree
/**
   @brief A node in the classifier tree
 */
class ClassifierNode {
public:
  /**
     @param ascope The objects in the scope of this node
     @param aclassifier A classifier to discriminate between the 
     equivalence classes defined by the scopes of this node's children.
   */
  ClassifierNode(const ObjectSet& ascope,
		 const BinarySubmodularImageClassifier& aclassifier) :
    scope(ascope),
    classifier(aclassifier),
    nullity(false)
  {
  }
  
  /**
     @param awvec A weight vector for the classifier for this node
     @param abestvec The current best solution
     @param abestobj The curretn best objective value
     @param ascope The objects in the scope of this node
   */
  ClassifierNode(const Dvec& awvec, 
		 const Dvec& abestvec,
		 double abestobj,
		 const ObjectSet& ascope) :
    scope(ascope),
    classifier(&awvec),
    nullity(false)
  {
    classifier.setInitialBestSolution(abestvec, abestobj);
  }

  /**
     @param arg Sets nullity flag
   */
  void setNull(bool arg) { nullity = arg; }

  /**
     @return Value of nullity flag
   */
  bool isNull() const { return nullity; }

  /**
     @brief Deserializer
     @param istr A stream from which a serialized object can be read
     @return A pointer to the deserialized object
   */
  static ClassifierNode* deserialize(std::istream& istr) {
    string scnode;
    istr >> scnode;

    if (scnode.compare("cnode") != 0)
      throw DeserializationError();

    Dvec* wvec = DvecUtils::deserialize(istr);

    // best weight vector so far
    string best;
    istr >> best;

    if (best.compare("best") != 0)
      throw DeserializationError();

    // best objective value
    string bestObj;
    double bestObjVal;
    istr >> bestObj;

    cerr << "BEST " << bestObj << std::endl;

    if (bestObj.compare("inf") == 0 || getenv("oIgnoreBest"))
      bestObjVal = HUGE_VAL;
    else {
      std::istringstream iss(bestObj);
      iss >> bestObjVal;
    }

    Dvec *bestvec = DvecUtils::deserialize(istr);

    // parse scope
    ObjectSet scope(istr);

    ClassifierNode *cnode = 
      new ClassifierNode(*wvec, *bestvec, bestObjVal, scope);

    delete wvec;
    delete bestvec;

    return cnode;
  }

  /**
     @brief Serializer 
     @param ostr The stream to which the serialized object will 
     be written
   */
  void serialize(std::ostream& ostr) {
    ostr << "cnode" << endl;

    Dvec bestVec(classifier.getWeightVector()->size());
    double bestObj = classifier.getBestSolution(bestVec);

    /// @fixme make BinarySubmodularImageClassifier serialize itself
    DvecUtils::serialize(*classifier.getWeightVector(), ostr);

    ostr << "best " << bestObj << std::endl;
    DvecUtils::serialize(bestVec, ostr);

    scope.serialize(ostr);
  }

  /**
     @return A pointer to the classifier at this node
   */
  BinarySubmodularImageClassifier* getClassifier() { return &classifier; }

  /**
     @return The set of objects in the scope of this node
   */
  const ObjectSet& getScope() const { return scope; }

private:
  ObjectSet scope;	// set of classes in the scope of this classifier
  BinarySubmodularImageClassifier classifier;
  bool nullity;
};

typedef BinaryTree<ClassifierNode*> BinaryClassifierTree;

/**
   @brief Utilities for the BinaryClassifierTree class
 */
class BinaryClassTreeUtils {
public:
  static BinaryClassifierTree* deserialize(std::istream &istr);

  static void serialize(const BinaryClassifierTree* classTree, 
			std::ostream& ostr);

};

/**
   @brief N-ary image classifier
   @param oNSICSlackPenalty (Environment variable) if provided, 
   specifies the slack penalty parameter
   @param oNSICLearningRate (Environment variable) if provided, 
   specifies the learning rate parameter
   @attention Deletes referenced classifiers on destruction
   @attention All segmented images are of type IPL_IMAGE_32S,
   unless specified otherwise.
   @todo Rename to NarySubmodularImageClassifier, or template
   to employ different types of binary classifiers
 */
class NaryImageClassifier : public ImageClassifier {
public:
  /**
     @param tree A binary tree of binary classifiers
     @param logger An optional logger
   */
  NaryImageClassifier(BinaryClassifierTree* tree, 
		      ThreadSafeLogger* logger = NULL
		      );

  /**
     @attention Deletes referenced classifiers
   */
  virtual ~NaryImageClassifier();

  void loadTrainingData(const vector<IplImage*>& trainImages,
			const vector<IplImage*>& trainSeg);

  void train(int iterations);

  /** 
      @brief Recursively train given tree of classifiers
      @param iterations Number of iterations for which to train
   */
  void train(int iterations, BinaryClassifierTree& classTree);

  void evaluate(const IplImage* image, IplImage* segmented);

  /**
     @brief Evaluate the classifer on the given image
     @param image Image to be evaluated
     @param trueSegmentation True segmentation
     @param segmented Pre-allocated output argument for the segmented image
     @param errors Pre-allocated output argument for binary image of errors
     @return Accuracy on given image (in range [0,1])
   */
  double evaluate(const IplImage* image, const IplImage* trueSegmentation,
		  IplImage* segmented, IplImage* errors);


  /**
     @brief Evaluate the classifer on the given image
     @param image Image to be evaluated
     @param segmented Output argument for segmented image
     @param labeling Output argument for labeling
     @param blobStats Output argument for blob statistics
     @param edges Output argument for blob-blob edge list
   */
  void evaluate(const IplImage* image, IplImage* segmented, 
		vector<int>& labeling, vector<blobStat>& blobStats,
		vector<std::pair<int,int> >& edges);

  /**
     @param segmented Segmented image
     @param trueSegmentation True image segmentation
     @errors errors Output argument for binary error image 
     (works with IPL_IMAGE_8U)
     @return Accuracy on given image (in range [0,1])
   */
  double evaluateErrors(const IplImage* segmented, 
			const IplImage* trueSegmentation,
			IplImage* errors);

  /**
     @brief Makes a new object tree with random class splits
     @param objset Set of objects within the scope of the classifier
     @param images Vector of at least one training image (must be passed to 
     obtain correct-length weight vector for initialization)
   */
  static BinaryClassifierTree*
  makeRandomBCTree(const ObjectSet& objset,
		   const vector<IplImage*>& images);

  /**
     @brief Makes a new object tree with random class splits
     @param objset Set of objects within the scope of this classifier
     @param wvec0 Initial weight vector (applied to all classifiers)
   */
  static BinaryClassifierTree*
  makeRandomBCTree(const ObjectSet& objset,
		   const Dvec& wvec0);

  /**
     @brief Saves the state of the classifier
     @param ostr Stream to which to save the classifier
   */
  void saveClassifierTree(std::ostream& ostr) {
    BinaryClassTreeUtils::serialize(classifierTree, ostr);
  }

  //  const Dvec& getWeightVector() const { return wvec; }

private:
  //  BinaryTree<ObjectSet> classesTree;
  //  BinaryTree<Dvec> initWvecTree;
  BinaryClassifierTree* classifierTree;

  vector<tUndirectedFeatureGraph*> vFgraphs;
  vector<FeatureGraphExtractor<tFeatureMatrix>*> vFgraphEx;
  vector<tUndirectedFeatureGraph*> subGraphs;

  ThreadSafeLogger* logger;

  static void 
  deleteClassifierNodes(BinaryClassifierTree& ctree);

  static void
  splitNodesByScope(const vector<vector<int> > &vlabels,
		    const ObjectSet& scope0, 
		    const ObjectSet& scope1,
		    vector<NodeSet>& vnodes0, 
		    vector<NodeSet>& vnodes1);

  static void
  relabelNodesToBinary(const vector<vector<int> >& vlabels,
		       const ObjectSet& scope0,
		       const ObjectSet& scope1,
		       vector<vector<int> >& vnewLabels);

  void 
  loadTrainingData(const vector<FeatureGraphExtractor<tFeatureMatrix>*>& vfgraphxs,
		   const vector<tUndirectedFeatureGraph*>& vfgraphs,
		   const vector<vector<double> >& vnodeWeights,
		   const vector<vector<int> >& vlabels,
		   const vector<Int2IntMap>& vblob2NodeInds,
		   BinaryClassifierTree *classTree);

  vector<int> evaluate(const tUndirectedFeatureGraph* fgraph, 
		       const BinaryClassifierTree* classTree);

  static void 
  saveClassifierTree(const BinaryClassifierTree* classTree, std::ostream& ostr);

};

#endif
