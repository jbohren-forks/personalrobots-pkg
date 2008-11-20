#include "image.h"
#include "misc.h"
#include "segment-image.h"
#include "pnmfile.h"

#include "highgui.h"

#include "features/SuperpixelBlobber.h"

#include <iostream>

#include <boost/timer.hpp>

SuperpixelBlobber::
SuperpixelBlobber(const IplImage *iplImage, double akParam, int aminSize) :
  kParam(akParam),
  minSize(aminSize),
  nBlobs(0),
  sourceImage(iplImage),
  labeledSPIds(NULL),
  labeledIds(NULL)
{
  boost::timer timer;

  labeledSPIds =
    cvCreateImage(cvGetSize(iplImage), IPL_DEPTH_32S, 1);
    //    new image<int>(iplImage->width, iplImage->height);

#ifdef SUPERPIX_RESCALE_X
  // scale down image for segmentation, then scale back up for speed
  double scaledHeight = 
    iplImage->height * double(SUPERPIX_RESCALE_X) / iplImage->width;

  /// @fixme: preallocate in constructor!
  IplImage* scaledSPIds = 
    cvCreateImage(cvSize(SUPERPIX_RESCALE_X, scaledHeight), IPL_DEPTH_32S, 1);
  IplImage* scaledImage = 
    cvCreateImage(cvSize(SUPERPIX_RESCALE_X, scaledHeight), IPL_DEPTH_8U, 3);

  cvResize(iplImage, scaledImage, CV_INTER_LINEAR);

  preSegmentImage(scaledImage, scaledSPIds);

  cvResize(scaledSPIds, labeledSPIds, CV_INTER_NN);

  makeBlobs(labeledSPIds);

  //  testViewSegmentation();

  if (getenv("oDebugOn")) 
    std::cout << "Segmentation+blobs took " << timer.elapsed() << std::endl;

  //  makeMasks();

  //  makeLabeledImage();

  cvReleaseImage(&scaledSPIds);
  cvReleaseImage(&scaledImage);
#else
  labeledSPIds =
    cvCreateImage(cvGetSize(iplImage), IPL_DEPTH_32S, 1);

  preSegmentImage(iplImage, labeledSPIds);

  makeBlobs(labeledSPIds);
#endif
}

SuperpixelBlobber::
~SuperpixelBlobber() {
  // delete blob masks
  releaseMasks();

  // delete blob coordinate lists
  for (std::vector<coordList*>::iterator it = blobCoords.begin();
       it != blobCoords.end();
       it++) {
    delete *it;
  }

  // delete blob stats
  for (std::vector<blobStat*>::iterator it = blobStats.begin();
       it != blobStats.end();
       it++) {
    delete *it;
  }

  if (labeledSPIds != NULL)
    cvReleaseImage(&labeledSPIds);

  // FIXME: memory leak!
  /*
  if (labeledIds != NULL)
    delete labeledIds;
  */
}

void SuperpixelBlobber::
releaseMasks() {
  for (std::vector<IplImage*>::iterator it = blobMasks.begin();
       it != blobMasks.end();
       it++) {
    cvReleaseImage(&(*it));
  }
}

void SuperpixelBlobber::
preSegmentImage(const IplImage* iplImage, IplImage* components) {
//preSegmentImage(const IplImage *iplImage, image<int> *components) {

  // allocate a spImage, copy data from iplImage
  image<rgb> *spImage = 
    new image<rgb>(iplImage->width, iplImage->height);

  // NB: rows may be padded, so must copy each row separately
  for (int row = 0; row < iplImage->height; row++) {
    memcpy((char*)imPtr(spImage,0,row), 
	   iplImage->imageData + row*iplImage->widthStep,
	   iplImage->width * 3);
  }

  image<int> spComponents(iplImage->width, iplImage->height);

  // run superpixel segmentation
  int nComponents;
  image<rgb> *segImage = 
    segment_image(spImage, SP_SIGMA, SP_K, SP_MIN_SIZE, 
		  &nComponents, &spComponents);

  assert(components->depth == IPL_DEPTH_32S);

  for (int yy = 0; yy < iplImage->height; yy++) {
    int* outPr = (int*)(components->imageData + yy * components->widthStep);
    for (int xx = 0; xx < iplImage->width; xx++) {
      // NB: parentheses necessary here...
      outPr[xx] = imRef((&spComponents), xx, yy);
    }
  }
 
  delete spImage;
  delete segImage;
}

void SuperpixelBlobber::
makeBlobs(const IplImage* labeled) {
  // iterate over the labeled image
  // for each pixel, check if its label is in the hash
  // if it is, look up the associated blob id
  // if it is not, make a new blob id, put in hash
  // store coordinates in vector at position given by blob id

  assert(labeled->depth == IPL_DEPTH_32S);

  for (int jj = 0; jj < labeled->height; jj++) {
    int* imPr = (int*)(labeled->imageData + jj * labeled->widthStep);
    for (int ii = 0; ii < labeled->width; ii++) {
      blobId id;

      int label = imPr[ii];

	// look up blob id in hash, or create hash entry if not there
	blobIdHash::iterator idPr = blobIds.find(label);

	if (idPr == blobIds.end()) {
	  blobIds[label] = nBlobs;
	  id = nBlobs;
	  nBlobs++;
	  blobCoords.push_back(new coordList());

	  /*
	  cout << "xy = " << ii << ", " << jj << 
	    " sp label = " << label << endl;
	  cout << "nBlobs " << nBlobs << endl;
	  */
	} else {
	  id = (*(idPr)).second;
	}

	// store coordinates in vector
	coord2d coord(ii,jj);
	blobCoords[id]->add(coord);
    }
  }

  cout << "Nblobs = " << blobCoords.size() << endl;
}

void SuperpixelBlobber::
makeMasks() {
  for (std::vector<coordList*>::iterator it = blobCoords.begin();
       it != blobCoords.end();
       it++) {
    IplImage* mask = 
      cvCreateImage(cvGetSize(sourceImage), IPL_DEPTH_8U, 1);
    coordList *cList = *it;

    cvSetZero(mask);

    for (std::vector<coord2d>::iterator cit = cList->coords.begin();
	 cit != cList->coords.end();
	 cit++) {
      coord2d coord = *cit;
      // FIXME: seems inefficient

      //      cout << "Setting " << coord.x << ", " << coord.y << endl;

      //      cvSet2D(mask, coord.x, coord.y, cvRealScalar(255)); 
      *(uchar*)(mask->imageData + 
		mask->widthStep*coord.y + 
		coord.x) = 255;
    }

    blobMasks.push_back(mask);
  }
}

void SuperpixelBlobber::
testViewMasks() {
  cvNamedWindow("Original image");
  cvShowImage("Original image", sourceImage);

  std::cout << "Total " << blobMasks.size() << " blobs" << std::endl;

  cvNamedWindow("Blob mask");
  int bnum = 0;
  for (std::vector<IplImage*>::iterator it = blobMasks.begin();
       it != blobMasks.end();
       it++) {
    IplImage *mask = *it;

    cvShowImage("Blob mask", mask);

    cout << "Blob num " << bnum << endl;

    cvWaitKey();
    bnum++;
  }
}

void SuperpixelBlobber::
testViewSegmentation() {
  cvNamedWindow("Segmentation");

  if (labeledIds == NULL)
    makeLabeledImage();

  int maxid = 0;
  for (int ii = 0; ii < labeledIds->width * labeledIds->height; ii++)
    maxid = MAX(maxid, ((int*)labeledIds->imageData)[ii]);

  IplImage* labeled8 = cvCreateImage(cvGetSize(labeledIds), IPL_DEPTH_8U, 1);
  cvConvertScale(labeledIds, labeled8, 255 / maxid, 0);

  cvShowImage("Segmentation", labeled8);

  cvSaveImage("labels.ppm", labeled8);

  cvReleaseImage(&labeled8);
}

void SuperpixelBlobber::
makeLabeledImage() {
  /*
  labeledIds = new image<int>(labeledSPIds->width(), 
			      labeledSPIds->height(), 
			      0);
  */
  // FIXME: make it easy to change the depth here 
  labeledIds = 
    cvCreateImage(cvGetSize(sourceImage), IPL_DEPTH_32S, 1);
  cvSetZero(labeledIds);

  for (int jj = 0; jj < labeledSPIds->height; jj++) {
    int* imPr = (int*)(labeledSPIds->imageData + jj * labeledSPIds->widthStep);
    for (int ii = 0; ii < labeledSPIds->width; ii++) {
      int label = imPr[ii];
    //      int label = imRef(labeledSPIds, ii, jj);
      blobIdHash::iterator idPr = blobIds.find(label);

      assert(idPr != blobIds.end());

      int id = (*idPr).second;
      //      imRef(labeledIds, ii, jj) = id;
      ((int*)(labeledIds->imageData + 
	      labeledIds->widthStep*jj))[ii] = id;

      /*
      cout << "Label " << id << " = " << 
	*(int*)(labeledIds->imageData + 
		labeledIds->widthStep*jj + ii) << endl;
      */
    }
  }
}

const IplImage* SuperpixelBlobber::
getLabeledImage() {
  if (labeledIds == NULL) 
    makeLabeledImage();

  return labeledIds;
}

void SuperpixelBlobber::
releaseLabeledImage() { 
  if (labeledIds != NULL) cvReleaseImage(&labeledIds);
  labeledIds = NULL;
}

const vector<blobStat*>* SuperpixelBlobber::
getBlobStats() {
  if (blobStats.size() == 0)
    calcBlobStats();
  return &blobStats;
}

vector<blobStat> SuperpixelBlobber::
getBlobStatsCopy() {
  if (blobStats.size() == 0)
    calcBlobStats();

  vector<blobStat> bstats;
  for (vector<blobStat*>::iterator it = blobStats.begin();
       it != blobStats.end();
       it++) {
    bstats.push_back(*(*it));
  }

  return bstats;
}

void SuperpixelBlobber::
calcBlobStats() {
  int bnum = 0;

  for (vector<coordList*>::iterator it = blobCoords.begin();
       it != blobCoords.end();
       it++) {
    coordList* coords = *it;

    blobStat *bstat = new blobStat();
    bstat->mx = bstat->my = 0;
    
    for (vector<coord2d>::iterator cit = coords->coords.begin();
	 cit != coords->coords.end();
	 cit++) {
      coord2d coord = *cit;
      bstat->mx += coord.x;
      bstat->my += coord.y;
    }
    // FIXME: precision issues w/ int?
    bstat->mx /= blobCoords[bnum]->coords.size();
    bstat->my /= blobCoords[bnum]->coords.size();
    
    bstat->area = coords->coords.size();

    blobStats.push_back(bstat);

    bnum++;
  }
}
