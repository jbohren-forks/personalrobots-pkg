/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <vector>
#include <fstream>
#include <sstream>

#include "image_msgs/CvBridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/node.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/Image.h"
#include "std_msgs/PointCloud.h"
#include "std_msgs/UInt8.h" //for projector status

#include "color_calib.h"

#include "topic_synchronizer.h"

using namespace std;

class StereoView : public ros::node
{
public:

  image_msgs::Image limage;
  image_msgs::Image rimage;
  image_msgs::Image dimage;
  image_msgs::StereoInfo stinfo;
  image_msgs::DisparityInfo dispinfo;

  image_msgs::CvBridge lbridge;
  image_msgs::CvBridge rbridge;
  image_msgs::CvBridge dbridge;

  std_msgs::PointCloud cloud;

  color_calib::Calibration lcal;
  color_calib::Calibration rcal;

  IplImage* lcalimage;
  IplImage* rcalimage;
  IplImage* lastDisparity;
  IplImage* lastScaledDisparity;
  IplImage* lastLeft;
  IplImage* lastRight;
  

  TopicSynchronizer<StereoView> sync;

  bool capture;
  std_msgs::UInt8 projector_status;

  ros::thread::mutex cv_mutex;

  string fileName;
  unsigned int fileNum;

  StereoView() : ros::node("stereo_view"), 
                 lcal(this), rcal(this), lcalimage(NULL), rcalimage(NULL), lastDisparity(NULL), lastScaledDisparity(NULL), lastLeft(NULL), lastRight(NULL),
                 sync(this, &StereoView::image_cb_all, ros::Duration().fromSec(0.05), &StereoView::image_cb_timeout),
		 capture(false)
  {
    //param("~file_name", fileName, "stereoImage");  //Base string to use, images will be [base][L/R/D].jpg (left, right, disparity)
    //param("~start_number", fileNum, 0);  //Number to start with (i.e. if program crashes start with n+1
                                         //where n is the number of the last image saved
    
    fileName = "data/stereoImage";
    fileNum = 0;

    cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);

    std::list<std::string> left_list;
    left_list.push_back(std::string("stereo/left/image_rect_color"));
    left_list.push_back(std::string("stereo/left/image_rect"));

    std::list<std::string> right_list;
    right_list.push_back(std::string("stereo/right/image_rect_color"));
    right_list.push_back(std::string("stereo/right/image_rect"));

    sync.subscribe(left_list,  limage, 1);
    sync.subscribe(right_list, rimage, 1);

    sync.subscribe("stereo/disparity", dimage, 1);
    sync.subscribe("stereo/stereo_info", stinfo, 1);
    sync.subscribe("stereo/disparity_info", dispinfo, 1);

    sync.subscribe("stereo/cloud", cloud, 1);

    sync.ready();

    subscribe("projector_status", projector_status, &StereoView::projector_status_change, 1);
  }

  ~StereoView()
  {
    if (lcalimage)
      cvReleaseImage(&lcalimage);
    if (rcalimage)
      cvReleaseImage(&rcalimage);
    if (lastDisparity)
      cvReleaseImage(&lastDisparity);
    if (lastScaledDisparity)
      cvReleaseImage(&lastScaledDisparity);
    if (lastLeft)
      cvReleaseImage(&lastLeft);
    if (lastRight)
      cvReleaseImage(&lastRight);
  }

  void projector_status_change()
  {
    //Nothing to see here, move along
  }

  void image_cb_all(ros::Time t)
  {
    cv_mutex.lock();

    cout<<"Project status is: "<<(int)projector_status.data<<endl;

    if (lbridge.fromImage(limage, "bgr"))
    {
      if(!projector_status.data)
	{
	  if(lastLeft != NULL)string s1, s2(fileName), s3(fileName), s4(fileName);
	    cvReleaseImage(&lastLeft);

	  lastLeft = cvCloneImage(lbridge.toIpl());
	}
      
      cvShowImage("left", lbridge.toIpl());
    }

    if (rbridge.fromImage(rimage, "bgr"))
    {
      if(!projector_status.data)
	{
	  if(lastRight != NULL)
	    cvReleaseImage(&lastRight);

	  lastRight = cvCloneImage(rbridge.toIpl());
	}

      cvShowImage("right", rbridge.toIpl());
    }

    if (dbridge.fromImage(dimage))
    {
      IplImage* disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
      cvCvtScale(dbridge.toIpl(), disp, 4.0/dispinfo.dpp);

      if(projector_status.data)
	{
	  if(lastDisparity != NULL)
	    cvReleaseImage(&lastDisparity);
	  lastDisparity = cvCloneImage(dbridge.toIpl());

	  if(lastScaledDisparity != NULL)
	    cvReleaseImage(&lastScaledDisparity);	  
	  lastScaledDisparity = cvCloneImage(disp);
	}
	
      
      cvShowImage("disparity", disp);
      cvReleaseImage(&disp);
    }
    
    if(capture)
      {
	stringstream ss1, ss2, ss3, ss4;
	ss1<<fileName<<"L"<<fileNum<<".jpg";
	ss2<<fileName<<"R"<<fileNum<<".jpg";
	ss3<<fileName<<"D"<<fileNum<<".jpg";
	ss4<<fileName<<"D"<<fileNum<<".u16";
	cout<<"Saving images "<<fileName<<" "<<ss1.str()<<" "<<ss2.str()<<" "<<ss3.str()<<" "<<ss4.str()<<endl;
	cvSaveImage(ss1.str().c_str(), lastLeft);
	cvSaveImage(ss2.str().c_str(), lastRight);
	cvSaveImage(ss3.str().c_str(), lastScaledDisparity);
	cvSave(ss4.str().c_str(), lastDisparity);
	
	fileNum++;
	capture = false;
      }

    cv_mutex.unlock();

  }

  void write_out_point_cloud(string filename)
  {
    ofstream fout("capturedCloud.txt");
    for(unsigned int i=0; i<cloud.pts.size(); i++)
      fout<<cloud.pts[i].x<<" "<<cloud.pts[i].y<<" "<<cloud.pts[i].z<<" 0.1 0.2 0.3"<<endl;
    fout.close();
  }

  void image_cb_timeout(ros::Time t)
  {
    if (limage.header.stamp != t)
      printf("Timed out waiting for left image\n");

    if (rimage.header.stamp != t)
      printf("Timed out waiting for right image\n");

    if (dimage.header.stamp != t)
      printf("Timed out waiting for disparity image\n");

    if (cloud.header.stamp != t)
      printf("Timed out waiting for point cloud\n");

    //Proceed to show images anyways
    //image_cb_all(t);
  }
  
  bool spin()
  {
    while (ok())
    {
      cv_mutex.lock();
      int key = cvWaitKey(3);
      
      switch (key) {
      case 'c':
        capture = true;
        break;
      }

      cv_mutex.unlock();
      usleep(10000);
    }

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  StereoView view;
  view.spin();
  ros::fini();
  return 0;
}

