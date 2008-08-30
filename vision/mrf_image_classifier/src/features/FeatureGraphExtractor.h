#ifndef __FEAT_GRAPH_H__
#define __FEAT_GRAPH_H__

#define H_BINS 50
#define S_BINS 50
#define V_BINS 50
#define THETA_BINS 30
#define MAG_BINS 30

#define IMAGE_MAX_VAL 255

#include "features/UndirectedFeatureGraph.hh"
#include "features/SuperpixelBlobber.h"

#include "cv.h"

#include "highgui.h"

#include "image.h"
#include "misc.h"
#include "segment-image.h"
#include "pnmfile.h"

#include <assert.h>
#include <ext/hash_set>
#include <set>

//using namespace std;

/**
   @brief Extracts a graph of features from an image
   @tparam FM The type of feature matrix used
 */
template <class FM>
class FeatureGraphExtractor {
 public:
  FeatureGraphExtractor(const IplImage *image);
  ~FeatureGraphExtractor();

  // FIXME: awkward free semantics
  // Caller must free 
  UndirectedFeatureGraph<FM> *getFeatureGraph();

  SuperpixelBlobber* getBlobber() { return &blobber; }

  void getBlobWeights(vector<double> &weights);

  void displayGraph(const Int2IntMap* blob2NodeInds = NULL);

  void displayGraph(const vector<pair<int,int> >& edgeList,
		    const Int2IntMap* blob2NodeInds);

  
  void testBlobWeights();

  const IplImage* getContourImage();

  /*
  static int numNodeFeatures() { return H_BINS*S_BINS*V_BINS + 1; }
  static int numEdgeFeatures() { return 2*numNodeFeatures() + 1; }
  */

 private:
  SuperpixelBlobber blobber;
  //  vector<NodeId> nodeList;
  int nNodes;
  vector<pair<int, int> > edgeList;
  IplImage *contourImage;
  UndirectedFeatureGraph<FM> *fgraph;

  void makeAdjacencyGraph();

  FM* getNodeFeatures(SuperpixelBlobber &blobber);

  FM* getNodeFeaturesSeparateHist(SuperpixelBlobber &blobber);

  FM* getNodeFeaturesSeparateHistOpenCv(SuperpixelBlobber &blobber);

  /*
  DenseFeatureMatrix<double>* 
    getNodeFeaturesTesting(const SuperpixelBlobber &blobber);
  */

  FM* getEdgeFeatures(const SuperpixelBlobber &blobber, 
		      FM* nodeFeat,
		      vector<pair<int,int> > &edgeList);

  void calcImGradientFeatures(const IplImage *src, 
			      IplImage *thetas,
			      IplImage *mags);
  
  static void 
    displayGraph(const IplImage* sourceImage,
		 const IplImage* contourImage, 
		 const vector<blobStat*>& blobStats,
		 const vector<pair<int,int> >& edgeList,
		 const Int2IntMap* blob2NodeInds = NULL);
};

#include "features/FeatureGraphExtractor.tcc"

#endif
