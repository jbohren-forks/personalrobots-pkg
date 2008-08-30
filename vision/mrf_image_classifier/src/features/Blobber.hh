#ifndef __BLOBBER_H__
#define __BLOBBER_H__

#include "cv.h"

#include <vector>

#include "util/Coordinates.hh"

//using namespace std;

/**
   @brief Blob statistics container
 */
class blobStat {
 public:
  int mx, my;			/* mean xy */
  int area;
};

/**
   @brief Abstract base class for blob generators
 */
class Blobber {
public:
  virtual ~Blobber() { };

  /**
     @return Number of blobs generated
   */
  virtual int numBlobs() const = 0;

  /**
     @return A vector of blob coordinate lists
   */
  virtual const std::vector<coordList*> *getBlobCoords() const = 0;

  /**
     @return A vector containing one binary mask per blob.
     Mask is nonzero if blob contains pixel.
   */
  virtual const std::vector<IplImage*> *getBlobMasks() = 0;

  /**
     @brief Releases cached masks
   */
  virtual void releaseMasks() = 0;

  /**
     @return Image from which blobs were created
   */
  virtual const IplImage* getSourceImage() const = 0;

  /**
     @return An image where each pixel is labeled with a blob id
   */
  virtual const IplImage* getLabeledImage() = 0;
  
  /**
     @brief Releases cached labeled image
   */
  virtual void releaseLabeledImage() = 0;

  /**
     @return A vector of blob statistics (pointer to internal storage)
   */
  virtual const std::vector<blobStat*>* getBlobStats() = 0;

  /**
     @return A vector of blob statistics
   */
  virtual std::vector<blobStat> getBlobStatsCopy() = 0;
};

#endif
