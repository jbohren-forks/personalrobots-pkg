#ifndef __SP_BLOBBER_H__
#define __SP_BLOBBER_H__

#include "cv.h"
//#include "superpix/image.h"
#include "image.h"

#include "features/Blobber.hh"

#include <vector>
#include <utility> 
#include <ext/hash_map>

#define SP_SIGMA 1.0
#define SP_K 25
#define SP_MIN_SIZE 25

/* image width is rescaled to this before finding superpixels */
#define SUPERPIX_RESCALE_X 320

using namespace std;

//typedef vector<coord2d> coordList;
typedef int blobId;
typedef __gnu_cxx::hash_map<int, blobId> blobIdHash; /* FIXME: hash function */

/**
   @brief Blobber based on Felzenszwalb/Huttenlocher superpixel implementation
 */
class SuperpixelBlobber : public Blobber {
 public:
  SuperpixelBlobber(const IplImage *image, double kParam = SP_K, 
		    int minSize = SP_MIN_SIZE);
  ~SuperpixelBlobber();

  int numBlobs() const { return nBlobs; }

  const vector<coordList*> *getBlobCoords() const
    { return &blobCoords; }

  const vector<IplImage*> *getBlobMasks() 
  { if (blobMasks.size() == 0) makeMasks();
    return &blobMasks; }

  void releaseMasks();
  
  const IplImage* getSourceImage() const 
  { return sourceImage; }

  const IplImage* getLabeledImage(); 

  void releaseLabeledImage();
  
  const vector<blobStat*>* getBlobStats();
  vector<blobStat> getBlobStatsCopy();

  void testViewMasks();

  void testViewSegmentation();

 private:  
  void preSegmentImage(const IplImage *image, IplImage* components);
  //  void makeBlobs(image<int>* labeled);
  void makeBlobs(const IplImage* labeled);
  void makeMasks();
  void makeLabeledImage();
  void calcBlobStats();

  double kParam;
  int minSize;

  int nBlobs;
  /* maps non-sequential ids returned by superpixel code to sequential ids */
  blobIdHash blobIds;	

  const IplImage *sourceImage;
  vector<coordList*> blobCoords;
  vector<IplImage*> blobMasks;
  vector<blobStat*> blobStats;


  IplImage* labeledSPIds;
  //  image<int> *labeledSPIds;
  //  image<int> *labeledIds;	/* image labeled with sequential blob ids */

  IplImage* labeledIds;
};

#endif
