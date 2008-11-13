#ifndef __FEAT_GRAPH_H__
#define __FEAT_GRAPH_H__

#define H_BINS 20
#define S_BINS 20
#define V_BINS 10
#define THETA_BINS 10
#define MAG_BINS 10
//#define N_CHAN_FEAT 3		/* number of "channel" features */

#define IMAGE_MAX_VAL 255

#include "features/UndirectedFeatureGraph.hh"
#include "features/SuperpixelBlobber.h"
#include "util/ImageHistogram.hh"

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

  /**
     Returns a vector of blobs in which adjacent blobs 
     with the same labels are merged.
   */
  void getMergedBlobs(const std::vector<int>& labeling,
		      std::vector<blobStat>& blobStats);

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
  int nNodeFeat;

  void makeAdjacencyGraph();

  //  FM* getNodeFeatures(SuperpixelBlobber &blobber);

  FM* getNodeFeaturesSeparateHist(SuperpixelBlobber &blobber);

  FM* getNodeFeaturesSeparateHistOpenCv(SuperpixelBlobber &blobber);

  /*
  DenseFeatureMatrix<double>* 
    getNodeFeaturesTesting(const SuperpixelBlobber &blobber);
  */

  FM* getEdgeFeatures(const SuperpixelBlobber &blobber, 
		      FM* nodeFeat,
		      vector<pair<int,int> > &edgeList);

  FM* getEdgeFeaturesMult(const SuperpixelBlobber &blobber, 
			  FM* nodeFeat,
			  vector<pair<int,int> > &edgeList);

  void calcGradientImages(const IplImage *src, 
			  IplImage *thetas,
			  IplImage *mags);
  
  static void 
    displayGraph(const IplImage* sourceImage,
		 const IplImage* contourImage, 
		 const vector<blobStat*>& blobStats,
		 const vector<pair<int,int> >& edgeList,
		 const Int2IntMap* blob2NodeInds = NULL);

  int calculateHistogramFeatures(vector<ImageHistogram<unsigned char>::Params>& 
				 histParams,
				 int firstColFeatMat,
				 FM* featMat);
};

#include "features/FeatureGraphExtractor.tcc"

#endif
