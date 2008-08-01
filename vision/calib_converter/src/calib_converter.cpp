#include <ros/node.h>
#include <iostream>
#include <fstream>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "std_msgs/ImageArray.h"
#include "std_msgs/PointCloudFloat32.h"
#include "std_msgs/String.h"
#include "image_utils/cv_bridge.h"
#include <time.h>
#include "logging/LogPlayer.h"

using namespace std;

void copyImageArray(string name, ros::msg* m, ros::Time t, void* n)
{
  if (m != 0) {
    *((std_msgs::ImageArray*)(n)) = *((std_msgs::ImageArray*)(m));
  }
}

void copyPointcloud(string name, ros::msg* m, ros::Time t, void* n)
{
  if (m != 0) {
    *((std_msgs::PointCloudFloat32*)(n)) = *((std_msgs::PointCloudFloat32*)(m));
  }
}

template <class T>
void copyMsg(string name, ros::msg* m, ros::Time t, void* n)
{
  if (m != 0) {
    *((T*)(n)) = *((T*)(m));
  }
}

void SplitFilename (const string& str, string *dir, string *file)
{
  size_t found;
  found=str.find_last_of("/\\");
  *file = str.substr(found+1);
  *dir = str.substr(0,found);
}


struct imgData
{
  string label;
  IplImage *cv_image;
  CvBridge<std_msgs::Image> *bridge;
};


class calib_converter
{
public:
  
  std_msgs::ImageArray image_msg;
  ros::thread::mutex cv_mutex;  
  LogPlayer lp;
  map<string, imgData> images;
  std_msgs::String calparams;
  std_msgs::PointCloudFloat32 cloud;

  calib_converter(string fullname)
  {

    // -- Get the filenames to save to.
    string dir, filename;
    SplitFilename(fullname, &dir, &filename);
    string num = filename.substr(0, filename.find_first_of("-"));
    string filebase = filename.substr(0, filename.find_first_of("."));
    string outputdir = dir + string("/") + filebase; 
    mkdir(outputdir.c_str(), 0755);

    // -- Load the messages.
    lp.open(fullname, ros::Time(0));
    lp.addHandler<std_msgs::ImageArray>(string("videre/images"), &copyMsg<std_msgs::ImageArray>, (void*)(&image_msg), true);
    lp.addHandler<std_msgs::String>(string("videre/cal_params"), &copyMsg<std_msgs::String>, (void*)(&calparams), true);
    lp.addHandler<std_msgs::PointCloudFloat32>(string("full_cloud"), &copyMsg<std_msgs::PointCloudFloat32>, (void*)(&cloud), true);
    while(lp.nextMsg());



    // -- If we don't have all the expected messages, yell.

   
    // -- Save the camera calibration information file.
    string outputname = outputdir + string("/") + string("cal_params.txt");
    ofstream f(outputname.c_str());
    f << calparams.data;
    f.close();


    // -- Save the pointcloud.
    outputname = outputdir + string("/") + string("pointcloud.xml");
    CvMat *M = cvCreateMat(cloud.get_pts_size(), 4, CV_64FC1);
    for(unsigned int i=0; i<cloud.get_pts_size(); i++) {
      cvmSet(M, i, 0, cloud.pts[i].x);
      cvmSet(M, i, 1, cloud.pts[i].y);
      cvmSet(M, i, 2, cloud.pts[i].z);
      cvmSet(M, i, 3, cloud.chan[0].vals[i]);
    }
    cvSave(outputname.c_str(), M);

    // -- For each image in the message.
    cv_mutex.lock();
    for (uint32_t i = 0; i < image_msg.get_images_size(); i++)
    {
      // -- Get the image into openCV and display.
      string l = image_msg.images[i].label;
      cout << "Looking at image " << l << endl;
      map<string, imgData>::iterator j = images.find(l);

      if (j == images.end())
      {
	cout << "adding new keyval pair for " << l << endl;
        images[l].label = image_msg.images[i].label;
        images[l].bridge = new CvBridge<std_msgs::Image>(&image_msg.images[i], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
        cvNamedWindow(l.c_str(), CV_WINDOW_AUTOSIZE);
        images[l].cv_image = 0;
      } 
      
      j = images.find(l);

      //make sure we don't have an old image.
//       if (j->second.cv_image)
// 	cvReleaseImage(&j->second.cv_image);
      
      j->second.bridge->to_cv(&j->second.cv_image);


      // -- Make sure we've got the right image types.
      if(l.compare(string("left_rectified")) != 0 && l.compare(string("right_rectified")) != 0) {
	cerr << "This bag file has something other than a left and right rectified image in it!" << endl;
      }
	
      // -- Save the image.
      string outputname = outputdir + string("/") + l + string(".png");
      cout << "saving to " << outputname << endl;
      cvSaveImage(outputname.c_str(), j->second.cv_image);

    }
    cv_mutex.unlock();

  }
};

int main(int argc, char **argv) {
  for(int i=1; i<argc; i++) {
    string name = argv[i];
    calib_converter cc(name);
  }
}
  
