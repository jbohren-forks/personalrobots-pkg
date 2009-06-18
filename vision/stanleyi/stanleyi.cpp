#include <iostream>
#include <image_msgs/Image.h>
#include <rosrecord/Player.h>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <opencv_latest/CvBridge.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include "descriptors.h"

using namespace NEWMAT;
using namespace std;


template <class T>
void copyMsg(string name, T* m, ros::Time t, ros::Time t_no_use, void* n)
{
  if (m != 0) {
    *((T*)(n)) = *((T*)(m));
  }
}
  

int getdir (string dir, vector<string> &files)
{
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL) {
    cout << "Error(" << errno << ") opening " << dir << endl;
    return errno;
  }

  while ((dirp = readdir(dp)) != NULL) {
    files.push_back(string(dirp->d_name));
  }
  closedir(dp);
  return 0;
}


IplImage* findLabelMask(double stamp) 
{
  vector<string> files;
  IplImage *mask = NULL;

  
  char buf[100];
  sprintf(buf, "%f", stamp); 
  string sbuf(buf);
  sbuf = sbuf.substr(0, sbuf.length()-1);   //Remove the last digit - it's been rounded.
  string dir = string("/tmp/turk_submit_images/images/");


  getdir(dir, files);
  for(unsigned int i=0; i<files.size(); i++)
    {
      if(files[i].find(sbuf) != string::npos && 
	 files[i].find(string(".png")) != string::npos)
	{
	  cout << "Found label for " << sbuf << ": " << files[i] << endl;
	  mask = cvLoadImage((dir + files[i]).c_str());
	  break;
	}
    }
  return mask;
}


class Stanleyi
{
public:
  ros::record::Player lp_;
  image_msgs::Image img_msg_;
  image_msgs::CvBridge img_bridge_;
  IplImage* img_;
  IplImage* mask_;

  Stanleyi() 
  {
    lp_.addHandler<image_msgs::Image>(string("/forearm/image_rect_color"), &copyMsg<image_msgs::Image>, (void*)(&img_msg_));
  }

  void viewLabels(string bagfile);
};

void test_watershed(IplImage *img) {
  int seed_step = 10;

  cout << img->depth << " " << img->nChannels << endl;
    

  // -- Create a grid of seed points.
  IplImage* markers = cvCreateImage( cvGetSize(img), IPL_DEPTH_32S, 1 );
  cvZero(markers);
  int label = 0;
  for(int r=0; r<markers->height; r++) {
    long* ptr = (long*)(markers->imageData + r * markers->widthStep);
    for(int c=0; c<markers->width; c++) {
      if(c%seed_step==0 && r%seed_step==0) {
	*ptr = label;
	label++;
      }
      ptr++;

    }
  }


  double t = (double)cvGetTickCount();
  cvWatershed( img, markers );
  t = (double)cvGetTickCount() - t;
  printf( "exec time = %gms\n", t/(cvGetTickFrequency()*1000.) );
  

  // -- Display the results.
  CvMat* color_tab;
  color_tab = cvCreateMat( 1, label, CV_8UC3 );
  CvRNG rng = cvRNG(-1);
  for(int i = 0; i < label; i++ )
    {
      uchar* ptr = color_tab->data.ptr + i*3;
      ptr[0] = (uchar)(cvRandInt(&rng)%180 + 50);
      ptr[1] = (uchar)(cvRandInt(&rng)%180 + 50);
      ptr[2] = (uchar)(cvRandInt(&rng)%180 + 50);
    }
  
  // paint the watershed image
  IplImage* wshed = cvCloneImage( img );
  IplImage* img_gray = cvCloneImage( img );
  cvZero( wshed );
  IplImage* marker_mask = cvCreateImage( cvGetSize(img), 8, 1 );
  cvCvtColor( img, marker_mask, CV_BGR2GRAY );
  cvCvtColor( marker_mask, img_gray, CV_GRAY2BGR );

  for(int i = 0; i < markers->height; i++ )
    for(int j = 0; j < markers->width; j++ )
      {
	int idx = CV_IMAGE_ELEM( markers, int, i, j );
	uchar* dst = &CV_IMAGE_ELEM( wshed, uchar, i, j*3 );
	if( idx == -1 )
	  dst[0] = dst[1] = dst[2] = (uchar)255;
	else if( idx <= 0 || idx > label )
	  dst[0] = dst[1] = dst[2] = (uchar)0; // should not get here
	else
	  {
	    uchar* ptr = color_tab->data.ptr + (idx-1)*3;
	    dst[0] = ptr[0]; dst[1] = ptr[1]; dst[2] = ptr[2];
	  }
      }

  cvAddWeighted( wshed, 0.5, img_gray, 0.5, 0, wshed );
  cvNamedWindow("Wshed", CV_WINDOW_AUTOSIZE);
  cvShowImage( "Wshed", wshed );



  cvShowImage("Feature", markers);
  cvWaitKey(0);
}  
  
void test_patch(IplImage* img) {
  Patch p(100, "intensity", .2);
  Matrix* r = NULL;
  p.compute(img, 300, 100, r,  true);
  if(r!=NULL) {
    cout << *r << endl;
  }
}

void Stanleyi::viewLabels(string bagfile) 
{

  cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Feature", CV_WINDOW_AUTOSIZE);

  lp_.open(bagfile, ros::Time());
  while(lp_.nextMsg()) {
    if (img_bridge_.fromImage(img_msg_, "bgr")) {
      img_ = img_bridge_.toIpl();
 
      //seg
      //      if(mask_ != NULL)
      //	cvReleaseImage(&mask_);
      mask_ = findLabelMask(img_msg_.header.stamp.toSec());

      if(mask_ != NULL) {
	cvShowImage("Image", img_);
	cvShowImage("Mask", mask_);
	test_watershed(img_);
	test_patch(img_);
	
	cvWaitKey(0);
      }
    }
  }
}
  
int main(int argc, char** argv) 
{
  Stanleyi s;
  
  if(argc > 2 && !strcmp(argv[1], "--viewlabels"))
    {
      cout << "Showing labels for " << argv[2] << endl;  
      s.viewLabels(string(argv[2]));
    }
  
  else 
    {
      cout << "usage: " << endl;
    }
  
}

