#include "descriptors.h"

using namespace std;
using namespace cv;

/* 
   Notes:
   vector == std::vector
   Vector == cv::Vector.  
   vvf == cv::Vector< cv::Vector<float> >
   
   You should not be able to screw yourself with precomputation sharing, 
   e.g. d.push_back(new SuperpixelColorHistogram(5, 0.25, 10, string("hue"), sch1, sch1));
   should die saying that the segmentation params do not match. 
*/

#define NSAMPLES 2

vector<ImageDescriptor*> setupImageDescriptors() {
  vector<ImageDescriptor*> d;

  //HogWrapper(Size winSize, Size blockSize, Size blockStride, Size cellSize,
  //           int nbins, int derivAperture=1, double winSigma=-1,
  //           int histogramNormType=L2Hys, double L2HysThreshold=0.2, bool gammaCorrection=false)
  d.push_back(new HogWrapper(Size(16,16), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(32,32), Size(16,16), Size(8,8), Size(8,8), 7, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(64,64), Size(32,32), Size(16,16), Size(16,16), 7, 1, -1, 0, 0.2, true));
  d.push_back(new HogWrapper(Size(128,128), Size(64,64), Size(32,32), Size(32,32), 7, 1, -1, 0, 0.2, true));


  //SuperpixelColorHistogram(int seed_spacing, float scale, int nBins, std::string type, SuperpixelStatistic* seg_provider=NULL,
  //                         SuperpixelColorHistogram* hsv_provider_=NULL);
  SuperpixelColorHistogram* sch1 = new SuperpixelColorHistogram(20, 0.5, 10, string("hue"));
  d.push_back(sch1);
  d.push_back(new SuperpixelColorHistogram(5, 0.5, 10, string("hue"), NULL, sch1));
  d.push_back(new SuperpixelColorHistogram(5, 1, 10, string("hue"), NULL, sch1));
  d.push_back(new SuperpixelColorHistogram(5, 0.25, 10, string("hue"), NULL, sch1));
 
  //SurfWrapper(bool extended = true, int size = 100)
  d.push_back(new SurfWrapper(true, 150));
  d.push_back(new SurfWrapper(true, 100));
  d.push_back(new SurfWrapper(true, 50));
  
  return d;
}

// void computeSegmentation(IplImage* img) {
//   SuperpixelStatistic ss(5, 0.5, NULL);
  

// Load an image and compute features at NSAMPLES random points.
int main(int argc, char** argv)  {

  // -- Get many descriptors.
  vector<ImageDescriptor*> descriptors = setupImageDescriptors();

  // -- Load an image.
  if(argc < 2) {
    cout << "usage: " << argv[0] << " [image]" << endl;
    return 1;
  }
  IplImage* img = cvLoadImage(argv[1]);
  if(!img) {
    cout << "Could not load image " << argv[1] << endl;
    return 1;
  }

  // -- Choose random locations and make keypoints.
  Vector<Keypoint> kp;
  kp.reserve(NSAMPLES);
  for(int i=0; i<NSAMPLES; i++)  {
    int r = rand() % img->height;
    int c = rand() % img->width;
    int size = 1;
    kp.push_back(Keypoint(c, r, size));      
  }
  
  // -- Call all descriptors, get vectorized results.
  //    results[i][j] is the jth feature vector for the ith descriptor.
  vector<vvf> results(descriptors.size());
  for(size_t i=0; i<descriptors.size(); i++) {
    descriptors[i]->compute(img, kp, results[i]);
  }

  // -- Print out the results.
  for(size_t j=0; j<NSAMPLES; j++) {
    for(size_t i=0; i<descriptors.size(); i++) {
      cout << endl << endl << descriptors[i]->getName() << " descriptor, sample number " << j << ": " << endl;
      for(size_t k=0; k<results[i][j].size(); k++) {
	cout << results[i][j][k] << " ";
      }
    }
  }
  cout << endl << endl;

  return 0;
}



  
