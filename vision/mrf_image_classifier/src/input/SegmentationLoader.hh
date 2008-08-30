#ifndef __SEG_LOADER_H__
#define __SEG_LOADER_H__

#include "cv.h"
#include "cxtypes.h"
#include "highgui.h"

#include "features/Blobber.hh"

#include <ext/hash_map>
#include <algorithm>
#include <assert.h>

using namespace std;

struct countComparator {
  bool operator()(pair<int,int> h0, pair<int,int> h1) {
    return h0.second < h1.second;
  }
};

typedef __gnu_cxx::hash_map<int,int> labelCountHash;

/**
   @brief Saves/loads segmentations
 */
class SegmentationLoader {
public:
  /**
     @param blobber The blobber with respect to which the labels are referenced
     @param labels The blob labels
     @param fname The name of the file to which to save the segmentation
   */
  static void writeBlobSegmentation(const Blobber& blobber,
				    const vector<int>& labels,
				    string fname) {
    IplImage *image = 
      cvCreateImage(cvGetSize(blobber.getSourceImage()),
		    IPL_DEPTH_32S, 1);

    writeBlobSegmentation(blobber, labels, image);

    cvSave(fname.c_str(), image);

    cvReleaseImage(&image);
  }

  /**
     @param blobber The blobber with respect to which the labels are referenced
     @param labels The blob labels
     @param image Pre-allocated output argument for the segmented image
     
     Writes the segmentation to a pre-allocated image.
   */
  static void writeBlobSegmentation(const Blobber& blobber,
				    const vector<int>& labels,
				    IplImage *image) {
    const vector<coordList*> *blobCoords = blobber.getBlobCoords();

    int bnum = 0;
    // iterate over blobs
    for (vector<coordList*>::const_iterator it = blobCoords->begin();
	 it != blobCoords->end();
	 it++) {
      const coordList* clist = *it;
      int label = labels[bnum];
      
      // label blobs' coordinates in destination image
      for (vector<coord2d>::const_iterator cit = clist->coords.begin();
	   cit != clist->coords.end();
	   cit++) {
	coord2d coord = *cit;

	getPixelRef(image, coord.x, coord.y) = label;
      }
      
      bnum++;
    }
  }

  /**
     @param fname File from which to load a segmentation.  Should be 
     of type IPL_DEPTH_32S.
     @return Loaded segmentation as an image of type IPL_DEPTH_32S.
     Caller must free.
   */
  static IplImage* loadSegmentation(string fname) {
    return (IplImage*)cvLoad(fname.c_str());
  }

  /**
     @brief Loads a segmentation from an image
     @param blobber The blobber with respect to which the 
     labels will be generated
     @param image The segmentation image
     @labels Output argument containing blob labels
     
     Outputs a label for each blob based on the per-pixel segmentation 
     contained in the given image.  The label of a blob is 
     the most-frequently-occurring label within that blob.
   */
  static void loadBlobSegmentation(const Blobber& blobber, 
				   IplImage *image, 
				   vector<int>& labels) {

    assert(image != NULL);

    labels.clear();

    //    vector<IplImage*> *blobMasks = blobber.getBlobMasks();
    const vector<coordList*> *blobCoords = blobber.getBlobCoords();
    int bnum = 0;
    for (vector<coordList*>::const_iterator it = blobCoords->begin();
	 it != blobCoords->end();
	 it++) {
      coordList* clist = *it;
      
      // count the number of occurrences of each label
      labelCountHash labelCounts;
      for (vector<coord2d>::const_iterator cit = clist->coords.begin();
	   cit != clist->coords.end();
	   cit++) {
	coord2d coord = *cit;
	int label = getPixelRef(image, coord.x, coord.y);
	labelCounts[label] = labelCounts[label] + 1;
	//	cout << "Label " << label << ", count " << labelCounts[label] << endl;
      }

      // find the most frequently occurring label
      pair<int,int> bestBin = 
	*max_element(labelCounts.begin(), labelCounts.end(), countComparator());

      int bestLabel = bestBin.first;

      //      cout << "Winning label " << bestLabel << endl;

      labels.push_back(bestLabel); 


      //      cvShowImage("testing", (*blobMasks)[bnum]);
      bnum++;
      //      cvWaitKey();
    }
  }

  /**
     @brief Loads a segmentation from a file
     @param blobber The blobber with respect to which the labels will 
     be generated
     @param fname The file from which to load the segmentation
     @param labels Output argument containing blob labels 
     @return 1 on error, 0 otherwise

     Outputs a label for each blob based on the per-pixel segmentation 
     contained in the given file.  The label of a blob is 
     the most-frequently-occurring label within that blob.
   */
  static int loadBlobSegmentation(const Blobber& blobber,
				  const string& fname,
				  vector<int>& labels) {

    //    IplImage *image = cvLoadImage(fname, CV_LOAD_IMAGE_GRAYSCALE);
    IplImage *image = (IplImage*)cvLoad(fname.c_str());

    if (image == NULL) 
      return 1;

    loadBlobSegmentation(blobber, image, labels);

    cvReleaseImage(&image);

    return 0;
  }

  // FIXME: add support for other formats
  static int& getPixelRef(IplImage *image, int ii, int jj) {
    switch (image->depth) {
    case IPL_DEPTH_32S:
      return ((int*)(image->imageData + jj * image->widthStep))[ii];
      break;
    default:
      cerr << "Unsupported image depth: " << image->depth << endl;
      abort();
    }
  }
  /*
  static int& getPixelRef(IplImage *image, int ii, int jj) {
    switch (image->depth) {
    case IPL_DEPTH_8U:
      return ((unsigned char*)(image->imageData + jj * image->widthStep))[ii];
      break;
    case IPL_DEPTH_16U:
      return ((unsigned short*)(image->imageData + jj * image->widthStep))[ii];
      break;
    case IPL_DEPTH_32S:
      return ((int*)(image->imageData + jj * image->widthStep))[ii];
      break;
    default:
      cerr << "Unsupported image depth: " << image->depth << endl;
      abort();
    }
  }
  */

};

#endif
