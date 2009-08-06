#include <iostream>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.hpp"

using namespace std;

int main(int argc, char** argv) {
  // -- Get img.
  if(argc == 1) { 
    cout << "Usage: " << argv[0] << " imgname" << endl;
    return 1;
  }
  IplImage* img = cvLoadImage(argv[1]);
  if(!img) {
    cout << "Could not load image." << endl; 
    return 1;
  }

  // -- Get HSV.
  IplImage* hsv = cvCreateImage( cvGetSize(img), 8, 3 );
  IplImage* hue = cvCreateImage( cvGetSize(img), 8, 1 );
  IplImage* sat = cvCreateImage( cvGetSize(img), 8, 1 );
  IplImage* val = cvCreateImage( cvGetSize(img), 8, 1 );
  cvCvtColor(img, hsv, CV_BGR2HSV);
  cvSplit(hsv, hue, sat, val, 0);

  // -- Setup hist.
  int num_bins = 8;
  float huerange[] = {0, 180}; //hsv
  float satrange[] = {0, 255}; //hsv
  float* ranges[] = {huerange, satrange}; //hue and sat.
  int sizes[] = {num_bins, num_bins};
  CvHistogram* hist = cvCreateHist(2, sizes, CV_HIST_ARRAY, ranges, 1);
  IplImage* imgs[] = {hue, sat};

  // -- Compute.
  cvCalcHist(imgs, hist, 0, 0);

  // -- Output.
  cout << "cv hist (before norm): " << endl;
  for(int i=0; i<num_bins; i++) {
    for(int j=0; j<num_bins; j++) {
      cout << cvQueryHistValue_2D(hist, i, j) << " ";
    }
  }
  cout << endl;

}
