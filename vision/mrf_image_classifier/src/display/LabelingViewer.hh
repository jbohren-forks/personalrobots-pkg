#ifndef __LABEL_VIEWER_H__
#define __LABEL_VIEWER_H__

#include "cv.h"
#include "highgui.h"

#include "features/Blobber.hh"
#include "classifier/ObjectSet.hh"
#include <string>

//#define MAX(A,B) ((A) > (B) ? (A) : (B))

/**
   @brief Displays segmentations 
 */
class LabelingViewer {
public:
  /**
     @param name The name of the window in which to display output
     @param ablobber The blobber corresponding to the image whose
     segmentations this object will be displaying
     @param anLabels The maximum label
   */
  LabelingViewer(const std::string name, Blobber &ablobber, int anLabels) :
    blobber(ablobber),
    nLabels(anLabels),
    wName(name)
  {
    cvNamedWindow(wName.c_str(), 0);

    sourceImage = blobber.getSourceImage();

    hsvImage0 = cvCreateImage(cvGetSize(sourceImage), 8, 3);
    hsvImage1 = cvCreateImage(cvGetSize(sourceImage), 8, 3);

    rgbImage1 = cvCreateImage(cvGetSize(sourceImage), 8, 3);

    cvCvtColor(sourceImage, hsvImage0, CV_BGR2HSV);

    hPlane0 = cvCreateImage(cvGetSize(sourceImage), 8, 1);
    sPlane0 = cvCreateImage(cvGetSize(sourceImage), 8, 1);
    vPlane0 = cvCreateImage(cvGetSize(sourceImage), 8, 1);

    hPlane1 = cvCreateImage(cvGetSize(sourceImage), 8, 1);
    sPlane1 = cvCreateImage(cvGetSize(sourceImage), 8, 1);
    vPlane1 = cvCreateImage(cvGetSize(sourceImage), 8, 1);
  
    cvSplit(hsvImage0, hPlane0, sPlane0, vPlane0, 0);
  }

  ~LabelingViewer() {
    cvReleaseImage(&hPlane0);
    cvReleaseImage(&sPlane0);
    cvReleaseImage(&vPlane0);
    cvReleaseImage(&hPlane1);
    cvReleaseImage(&sPlane1);
    cvReleaseImage(&vPlane1);
    cvReleaseImage(&hsvImage0);
    cvReleaseImage(&hsvImage1);
    cvReleaseImage(&rgbImage1);
  }

  /**
     @param labels The labeling to be viewed
     
     Displays the labeling given the blobber given in the constructor
   */
  void viewLabeling(std::vector<int>& labels) {
    using std::vector;

    cvSplit(hsvImage0, hPlane1, sPlane1, vPlane1, 0);

    const vector<IplImage*> *masks = blobber.getBlobMasks();
    int ii = 0;
    //    for (unsigned int ii = 0; ii < masks->size(); ii++) {

    for (vector<int>::const_iterator labelPr = labels.begin();
	 labelPr != labels.end();
	 labelPr++) {

      cvSet(hPlane1, cvRealScalar(getHue(nLabels, *labelPr)), (*masks)[ii]);
      cvAddS(vPlane0, cvRealScalar(getVal(nLabels, labels[ii])), 
	     vPlane1, (*masks)[ii]);
      cvSet(sPlane1, cvRealScalar(getSat(nLabels, *labelPr)), (*masks)[ii]);
      //      cvSet(vPlane1, cvRealScalar(getVal(nLabels, *labelPr)), (*masks)[ii]);

      ii++;
    }

    //    cvMerge(hPlane1, sPlane1, vPlane1, 0, hsvImage);
    cvMerge(hPlane1, sPlane1, vPlane1, 0, hsvImage1);

    cvCvtColor(hsvImage1, rgbImage1, CV_HSV2BGR);

    cvShowImage(wName.c_str(), rgbImage1);
  }

  /**
     @param labels The labeling
     
     Displays the labeling as a binary image.
   */
  void viewLabelingBW(std::vector<int> labels) {
    const std::vector<IplImage*> *masks = blobber.getBlobMasks();
    for (unsigned int ii = 0; ii < masks->size(); ii++) {
      cvSet(vPlane1, cvRealScalar(getVal(nLabels,labels[ii])), (*masks)[ii]);
    }
    cvShowImage(wName.c_str(), vPlane1);
  }

  /**
     @param wname Name of the window
     @param nLabels The maximum label
     @param rgbImage The original RGB image
     @param segmentation Segmented image (must be of type IPL_DEPTH_32S)
     @todo template to allow different segmentations with different 
     storage types
   */
  static void 
  viewLabeling(const char* wname, 
	       int nLabels, 
	       const IplImage* rgbImage, const IplImage* segmentation) {
    IplImage* hsvResult = cvCreateImage(cvGetSize(rgbImage), 8, 3);

    cvCvtColor(rgbImage, hsvResult, CV_BGR2HSV);
    
    for (int xx = 0; xx < hsvResult->width; xx++) {
      for (int yy = 0; yy < hsvResult->height; yy++) {
	int label = ((int*)(segmentation->imageData + 
			    yy*segmentation->widthStep))[xx];
	unsigned char* pr = 
	  (unsigned char*)(hsvResult->imageData + yy*hsvResult->widthStep);
	pr[3*xx] =   getHue(nLabels, label);
	pr[3*xx+1] = getSat(nLabels, label);
	//	pr[3*xx+1] = 200;
	//	pr[3*xx+2] = getVal(nLabels, label);
	//	pr[3*xx+2] = 150;
      }
    }


    IplImage* rgbResult = cvCreateImage(cvGetSize(hsvResult), 8, 3);

    cvCvtColor(hsvResult, rgbResult, CV_HSV2BGR);

    cvNamedWindow(wname, 0);
    cvShowImage(wname, rgbResult);

    cvReleaseImage(&hsvResult);
    cvReleaseImage(&rgbResult);
  }

  /**
     @param wname The window name
     @param nLabels The maximum label
     @param rgbImage The original RGB image
     @param segmentation The segmented image (must be of type IPL_IMAGE_32S)
     @param objSet A set of objects containing the labeled objects 
     @param labeling The per-blob labeling
     @param blobStats A vector of blob statistics
     @todo template to allow different storage types for segmentation image

     Displays a string label per blob, in addition to a colorized 
     visualization of the segmentation.
   */
  static void 
  viewLabeling(const char* wname, 
	       int nLabels,
	       const IplImage* rgbImage, 
	       const IplImage* segmentation,
	       const ObjectSet& objSet,
	       const std::vector<int>& labeling, 
	       const std::vector<blobStat>& blobStats) {
    static int counter = 0;

    IplImage* hsvResult = cvCreateImage(cvGetSize(rgbImage), 8, 3);

    cvCvtColor(rgbImage, hsvResult, CV_BGR2HSV);
    
    for (int xx = 0; xx < hsvResult->width; xx++) {
      for (int yy = 0; yy < hsvResult->height; yy++) {
	int label = ((int*)(segmentation->imageData + 
			    yy*segmentation->widthStep))[xx];
	unsigned char* pr = 
	  (unsigned char*)(hsvResult->imageData + yy*hsvResult->widthStep);
	pr[3*xx] =   getHue(nLabels, label);
	pr[3*xx+1] = getSat(nLabels, label);
	//	pr[3*xx+1] = 200;
	//	pr[3*xx+2] = getVal(nLabels, label);
	//	pr[3*xx+2] = 150;
      }
    }

    IplImage* rgbResult = cvCreateImage(cvGetSize(hsvResult), 8, 3);

    cvCvtColor(hsvResult, rgbResult, CV_HSV2BGR);



    cvNamedWindow("Sans labels", 0);
    cvShowImage("Sans labels", rgbResult);



    // add text labels
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 1.0);

    for (int bi = 0; bi < (int)labeling.size(); bi++) {
      int label = labeling[bi];

      // FIXME: hacky
      if (label == 0)
	continue;

      std::string name = objSet.findName(label);
      CvPoint blobMean = 
	cvPoint(blobStats[bi].mx, blobStats[bi].my);
      cvPutText(rgbResult, name.c_str(), 
		blobMean, &font, cvScalar(255, 255, 255));
    }

    if (getenv("oSaveLabelings")) {
      char fname[255];
      snprintf(fname, sizeof(fname), "segmented%05d.jpg", counter);
      cvSaveImage(fname, rgbResult);
      snprintf(fname, sizeof(fname), "original%05d.jpg", counter);
      cvSaveImage(fname, rgbImage);
      counter++;
    }

    cvNamedWindow(wname, 0);
    cvShowImage(wname, rgbResult);

    cvReleaseImage(&hsvResult);
    cvReleaseImage(&rgbResult);
  }

  /**
     @brief Displays single-channel image with automatic hue scaling
     @param wname The window name
     @param image The image to be displayed
   */
  static void 
  imagesc(const char* wname, const IplImage* image) {
    double imin = HUGE_VAL, imax = -HUGE_VAL;
    cvMinMaxLoc(image, &imin, &imax);
    double range = imax - imin;

    assert((int)image->depth == (int)IPL_DEPTH_32S);

    std::cout << "RANGE = " << range << std::endl;
    
    IplImage* hsv = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);

    for (int yy = 0; yy < hsv->height; yy++) {
      unsigned int* prIn = 
	(unsigned int*)(image->imageData + yy*image->widthStep);
      unsigned char* prOut = 
	(unsigned char*)(hsv->imageData + yy*hsv->widthStep);
      for (int xx = 0; xx < hsv->width; xx++) {
	prOut[3*xx] = 255.0 * (prIn[xx] - imin) / range;
	prOut[3*xx+1] = 200.0;
	prOut[3*xx+2] = 200.0;
      }
    }

    IplImage* rgb = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);

    cvCvtColor(hsv, rgb, CV_HSV2BGR);

    cvShowImage(wname, rgb);

    //    cvReleaseImage(&hsv);
    //    cvReleaseImage(&rgb);
  }


  static unsigned char getHue(int nLabels, int label) {
    return (unsigned char)(180 * (double)label / (nLabels - 1));
  }

  static unsigned char getVal(int nLabels, int label) {
    return (unsigned char)(100 * (double)label / (nLabels - 1));
    //    return 255;
    //    return 0;
  }

  static unsigned char getSat(int nLabels, int label) {
    return (unsigned char)(100 + 150 * (double)label / (nLabels - 1));
  }


private:
  Blobber &blobber;
  int nLabels;

  const IplImage *sourceImage;
  IplImage *hsvImage0, *hsvImage1;
  IplImage *rgbImage1;

  IplImage *hPlane0, *sPlane0, *vPlane0;
  IplImage *hPlane1, *sPlane1, *vPlane1;

  std::string wName;
};

#endif
