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

void Stanleyi::viewLabels(string bagfile) 
{

  cvNamedWindow("Image", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
  lp_.open(bagfile, ros::Time());
  while(lp_.nextMsg()) 
    {
      if (img_bridge_.fromImage(img_msg_, "bgr"))
	{
	  img_ = img_bridge_.toIpl();
	  mask_ = findLabelMask(img_msg_.header.stamp.toSec());
	  if(mask_ != NULL) {
	    cvShowImage("Image", img_);
	    cvShowImage("Mask", mask_);
	    cvWaitKey(0);
	  }
	}
      
      if(mask_ != NULL)
	cvReleaseImage(&mask_);
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

