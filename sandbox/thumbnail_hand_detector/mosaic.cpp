#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv_latest/CvBridge.h>
#include <descriptors_2d/descriptors_2d.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {

  vector<string> files;
  getDir(argv[1], files);
  assert(files.size() > 0);

  double count=0;
  for(size_t i=0; i<files.size(); ++i) { 
    if(files[i].find(".jpg") != string::npos ||
       files[i].find(".png") == string::npos) {
      count++;
    }
  }


  size_t num_imgs_per_side = ceil(sqrt(count));
  size_t  num_pixels_per_side = num_imgs_per_side * 128;
  size_t idx = -1;
  IplImage* mosaic = cvCreateImage(cvSize(num_pixels_per_side, num_pixels_per_side), 8, 1);
  for(size_t i=0; i<num_imgs_per_side; ++i) { 
    for(size_t j=0; j<num_imgs_per_side; ++j) { 
      idx++;
      cout << files[idx] << endl;

      if(idx >= files.size())
	break;

      if(files[idx].find(".jpg") == string::npos || 
	 files[idx].find(".png") == string::npos) {
	j--;
	continue;
      }



      IplImage* img = cvLoadImage((string(argv[1]) + "/" + files[idx]).c_str());
      assert(img->nChannels==1);
      IplImage* small = cvCreateImage(cvSize(128, 128), 8, 1);
      cvResize(img, small, CV_INTER_NN);
      if(!img) {
	cout << (string(argv[1]) + "/" + files[idx]).c_str() << " could not be loaded." << endl;
	return 1;
      }

      cvSetImageROI(mosaic, cvRect(i*128, j*128, 128, 128));
      cvAdd(small, mosaic, mosaic);
      cvResetImageROI(mosaic);
      cvReleaseImage(&small);
      cvReleaseImage(&img);
    }
  }

  CVSHOW("mos", mosaic);
  cvWaitKey(0);
  cvSaveImage("mosaic.jpg", mosaic);
  return 0;
}
