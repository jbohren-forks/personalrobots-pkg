#include "optimize/SubgradientOptimizer.hh"
#include "features/FeatureMatrix.hh"
#include "objective/L2Regularization.hh"
#include "features/FeatureGraphExtractor.h"
#include "objective/BinarySubmodularObjective.hh"
#include "input/LabelingViewer.hh"
#include "input/SegmentationLoader.hh"

#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "cv.h"
#include "highgui.h"

#include <map>
#include <vector>
#include <string>

#define OB_PER_SCENE 5

//#define OUTPUT_SIZE 250		// output image size (square)

class ImageInfo {
public:
  string name;
  int objId;
  int seq;

  /*
  IplImage* image;
  IplImage* segImage;
  */
  string imPath;
  string segPath;
};

void getImageInfo(DIR* imageDir, 
		  const string dirpath,
		  vector<ImageInfo>& infos);

void getRandomImages(const string& imageDir,
		     vector<ImageInfo> &infos);

template <class T>
vector<T> randomPermutation(const vector<T>& vec, int size);

void composeImages(const vector<ImageInfo>& infos, 
		   IplImage *composed, IplImage **labeled);

// randomly compose scenes
// arguments: input dir, output dir
int main(int argc, char *argv[]) {
  if (argc < 4) {
    cerr << "usage: composeScenes <image dir> <output dir> <background>" 
	 << endl;
    exit(1);
  }

  srand48(time(NULL));

  DIR* dir = opendir(argv[1]);
  const char *outputdir = argv[2];
  string background(argv[3]);
  
  assert(dir != NULL);

  vector<ImageInfo> infos;

  getImageInfo(dir, argv[1], infos);

  int counter = 0;
  while(1) {
    vector<ImageInfo> randIms = randomPermutation(infos, OB_PER_SCENE);
    
    for (vector<ImageInfo>::iterator it = randIms.begin();
	 it != randIms.end();
	 it++) {
      ImageInfo info = *it;
      cout << "I " << info.name << endl;
    }
    
    IplImage *composed, *labeled;

    composed = cvLoadImage(background.c_str());

    composeImages(randIms, composed, &labeled);
    
    //
    cvNamedWindow("Composed image", 0);
    cvShowImage("Composed image", composed);
    cvResizeWindow("Composed image", 1000, 1000);
    //
    
    // save images
    char namebuf[1000];

    snprintf(namebuf, sizeof(namebuf), "%s/composed%d.png", 
	     outputdir, counter);
    cvSaveImage(namebuf, composed);

    snprintf(namebuf, sizeof(namebuf), "%s/composed%d.seg", 
	     outputdir, counter);
    cvSave(namebuf, labeled);
    //

    /*
      // TESTING
    IplImage *foo = cvCreateImage(cvGetSize(labeled), IPL_DEPTH_8U, 1);
    cvConvert(labeled, foo);
    cvSaveImage("testing.jpg", foo);
    */

    char key = cvWaitKey();
    if (key == 'q')
      exit(0);

    //
    cvReleaseImage(&composed);
    cvReleaseImage(&labeled);
    counter++;
  }
}

// caller must free images
// FIXME: adapt to other types of images
void getImageInfo(DIR* imageDir, 
		  const string dirpath,
		  vector<ImageInfo>& infos) {
  struct dirent *dent = readdir(imageDir);

  if (dent == NULL) 
    return;

  // skip non-png's
  string fname = dent->d_name;
  string::size_type dotpos = fname.find(".png");

  if (dotpos != string::npos) {
    string segPath = dirpath + "/" + fname.substr(0,dotpos) + ".seg";
    string imPath = dirpath + "/" + fname;

    // skip images w/out segmentations
    struct stat statBuf;
    if (stat(segPath.c_str(), &statBuf) == 0) { // if segmentation file exists
      ImageInfo info;

      info.name = fname;
      //      info.segImage = segimage;
      info.imPath = imPath;
      info.segPath = segPath;

      sscanf(fname.c_str(), "obj%d__%d.png", &info.objId, &info.seq);

      infos.push_back(info);
    }
  }

  getImageInfo(imageDir, dirpath, infos);
}

// returns random permutation of length size
template <class T>
vector<T> randomPermutation(const vector<T>& vec, int size) {
  vector<T> newlist;

  if (size == 0 || vec.size() == 0) 
    return newlist;

  // pick an element out at random
  int randind = int(ceil(drand48() * vec.size())) - 1;

  vector<T> sublist = vec;
  sublist.erase(sublist.begin() + randind);

  // get random permutation of rest of elements
  newlist = randomPermutation(sublist, size - 1);

  // append randomly picked-out element to new list
  newlist.push_back(vec[randind]);

  return newlist;
}

// copies source pixel to target pixel in a semi-generic way
inline void copyPixel(IplImage *dest, const IplImage *src, 
		      int si, int sj, 
		      int ti, int tj) {
  if (src->depth == (int)IPL_DEPTH_8U && src->nChannels == 3) {
    for (int cc = 0; cc < 3; cc++)
      ((unsigned char*)(dest->imageData + tj*dest->widthStep))[3*ti + cc] = 
	((unsigned char*)(src->imageData + sj*src->widthStep))[3*si + cc];
  } else if (src->depth == (int)IPL_DEPTH_32S && src->nChannels == 1) {
    ((int*)(dest->imageData + tj*dest->widthStep))[ti] = 
      ((int*)(src->imageData + sj*src->widthStep))[si];
  } else {
    assert(0);
  }
}

// copies source image to destination image at specified location
// coordinates are coordinates of "top-left" corner of image
void copyToPos(IplImage* dest, 
	       const IplImage *src, const IplImage *mask,
	       int xc, int yc) {

  // FIXME: generalize
  assert(mask->depth == (int)IPL_DEPTH_32S);
  //  assert(src->depth == (int)IPL_DEPTH_8U);

  for (int si = 0; si < src->width; si++) {
    for (int sj = 0; sj < src->height; sj++) {

      int maskval = ((int*)(mask->imageData + sj*mask->widthStep))[si];

      if (maskval == 1) {
	int ti = si + xc;
	int tj = sj + yc;
	
	if (ti < dest->width && tj < dest->height && ti > 0 && tj > 0) {
	  /*
	  for (int cc = 0; cc < 3; cc++)
	  ((unsigned char*)(dest->imageData + tj*dest->widthStep))[3*ti + cc] = 
	    ((unsigned char*)(src->imageData + sj*src->widthStep))[3*si + cc];
	  */
	  copyPixel(dest, src, si, sj, ti, tj);
	}
      }
    }
  }
}

/*
void testErode(IplImage* maskImage) {
  int erodeWin = 10;
  IplConvKernel *stel = 
    cvCreateStructuringElementEx(erodeWin, erodeWin, 
				 erodeWin/2, erodeWin/2, 
				 CV_SHAPE_RECT);

  IplImage *down = cvCreateImage(cvGetSize(maskImage), IPL_DEPTH_8U, 1);

  cvConvertScale(maskImage, down, 255, 0);

  cvNamedWindow("original image", 0);
  cvShowImage("original image", down);

  //  cvErode(down, down, stel, 1);
  cvErode(maskImage, maskImage, stel, 1);
  cvConvertScale(maskImage, down, 255, 0);

  cvNamedWindow("eroded image", 0);
  cvShowImage("eroded image", down);

  cvWaitKey();

  cvReleaseStructuringElement(&stel);
}
*/

// FIXME: is there a bug in OpenCV?
void fixedErode(IplImage* maskImage) {
  int erodeWin = 5;
  IplConvKernel *stel = 
    cvCreateStructuringElementEx(erodeWin, erodeWin, 
				 erodeWin/2, erodeWin/2, 
				 CV_SHAPE_RECT);

  IplImage *down = cvCreateImage(cvGetSize(maskImage), IPL_DEPTH_8U, 1);

  cvConvert(maskImage, down);

  cvErode(down, down, stel, 1);

  cvReleaseStructuringElement(&stel);

  cvConvert(down, maskImage);

  cvReleaseImage(&down);
}


// composed should be preallocated
// caller must free returned image
void composeImages(const vector<ImageInfo>& infos, 
		   IplImage *composed, IplImage **labeled) {
  /*
  *composed = 
    cvCreateImage(cvSize(OUTPUT_SIZE, OUTPUT_SIZE), IPL_DEPTH_8U, 3);
  cvZero(*composed);
  */

  *labeled = 
    cvCreateImage(cvGetSize(composed), IPL_DEPTH_32S, 1);
  cvZero(*labeled);

  // FIXME: opencv bug?
  int erodeWin = 10;
  IplConvKernel *stel = 
    cvCreateStructuringElementEx(erodeWin, erodeWin, 
				 erodeWin/2, erodeWin/2, 
				 CV_SHAPE_RECT);

  for (vector<ImageInfo>::const_iterator it = infos.begin();
       it != infos.end();
       it++) {
    ImageInfo info = *it;

    IplImage* srcImage = cvLoadImage(info.imPath.c_str(), CV_LOAD_IMAGE_COLOR);
    IplImage* maskImage = SegmentationLoader::loadSegmentation(info.segPath);

    assert(srcImage != NULL);
    assert(maskImage != NULL);




    fixedErode(maskImage);



    // randomly pick upper-left coordinate of subimage in composed image
    double xc = -srcImage->width/4 + drand48() * 
      (composed->width - srcImage->width/4);
    double yc = -srcImage->height/4 + drand48() * 
      (composed->height - srcImage->height/4);

    copyToPos(composed, srcImage, maskImage, (int)xc, (int)yc);

    // make a labeled image and copy it into place
    IplImage *thisLabeled = 
      cvCreateImage(cvGetSize(maskImage), IPL_DEPTH_32S, 1);

    cvConvertScale(maskImage, thisLabeled, info.objId, 0);

    copyToPos(*labeled, thisLabeled, maskImage, (int)xc, (int)yc);

    cvReleaseImage(&srcImage);
    cvReleaseImage(&maskImage);
    cvReleaseImage(&thisLabeled);
  }

  cvReleaseStructuringElement(&stel);
}

