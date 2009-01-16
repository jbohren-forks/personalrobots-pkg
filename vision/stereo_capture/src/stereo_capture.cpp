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

class StereoView : public ros::Node
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

  std_msgs::PointCloud cloud,cloudNoTex;

  color_calib::Calibration lcal;
  color_calib::Calibration rcal;

  IplImage* lcalimage;
  IplImage* rcalimage;
  IplImage* lastDisparity;
  IplImage* lastScaledDisparity;
  IplImage* lastLeft;
  IplImage* lastRight;
  IplImage* lastNonTexDisp; //This is the disparity from the non-textured light
  

  TopicSynchronizer<StereoView> sync;

  bool capture;
  bool captureNoTex;
  std_msgs::UInt8 projector_status;

  boost::mutex cv_mutex;

  string fileName;
  unsigned int fileNum;

  StereoView() : ros::Node("stereo_view"), 
                 lcal(this), rcal(this), lcalimage(NULL), rcalimage(NULL), lastDisparity(NULL), lastScaledDisparity(NULL), lastNonTexDisp(NULL), lastLeft(NULL), lastRight(NULL),
                 sync(this, &StereoView::image_cb_all, ros::Duration().fromSec(0.05), &StereoView::image_cb_timeout),
		 capture(false), captureNoTex(false)
  {
    //param("~file_name", fileName, "stereoImage");  //Base string to use, images will be [base][L/R/D].jpg (left, right, disparity)
    //param("~start_number", fileNum, 0);  //Number to start with (i.e. if program crashes start with n+1
                                         //where n is the number of the last image saved
    
    fileName = "data/stereoImage";
    fileNum = 10;

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
    if(lastNonTexDisp)
      cvReleaseImage(&lastNonTexDisp);
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
	} else //Save non textured light disparity too
        { 
          if(lastNonTexDisp != NULL)
            cvReleaseImage(&lastNonTexDisp);
          lastNonTexDisp = cvCloneImage(disp);
          cloudNoTex = cloud;
       }
      cvShowImage("disparity", disp);
      cvReleaseImage(&disp);
    }
    
    if(capture && (lastDisparity != NULL)) //Look here to save the non texture disparity image
      {
        cout << "Shouldn't get here" << endl;
	stringstream ss1, ss2, ss3, ss4, ss5, sscloud, sscloudNoTex;
	ss1<<fileName<<"L"<<fileNum<<".jpg";
	ss2<<fileName<<"R"<<fileNum<<".jpg";
	ss3<<fileName<<"D"<<fileNum<<".jpg";
	ss4<<fileName<<"D"<<fileNum<<".u16";
        ss5<<fileName<<"d"<<fileNum<<".jpg";
        sscloud<<fileName<<"_C"<<fileNum<<".txt";
        sscloudNoTex<<fileName<<"_CnoTex"<<fileNum<<".txt";
	cout<<"Saving images "<<fileName<<" "<<ss1.str()<<" "<<ss2.str()<<" "<<ss3.str()<<" "<<ss4.str()<< " " << ss5.str() << endl;
	cvSaveImage(ss1.str().c_str(), lastLeft);
	cvSaveImage(ss2.str().c_str(), lastRight);
	cvSaveImage(ss3.str().c_str(), lastScaledDisparity);
	cvSave(ss4.str().c_str(), lastDisparity);
        cvSaveImage(ss5.str().c_str(),lastNonTexDisp);
        write_out_point_cloud(sscloud.str(),cloud);
        write_out_point_cloud(sscloudNoTex.str(),cloudNoTex);
	
	fileNum++;
	capture = false;
      }
    if(captureNoTex)
    {
	stringstream ss1, ss2, ss3, ss4, ss5, sscloud, sscloudNoTex;
	ss1<<fileName<<"L"<<fileNum<<".jpg";
	ss2<<fileName<<"R"<<fileNum<<".jpg";
        ss5<<fileName<<"d"<<fileNum<<".jpg";
        sscloudNoTex<<fileName<<"_CnoTex"<<fileNum<<".txt";
	cout<<"Saving images "<<fileName<<" "<<ss1.str()<<" "<<ss2.str()<<" "<<ss3.str()<<" "<<ss4.str()<< " " << ss5.str() << endl;
	cvSaveImage(ss1.str().c_str(), lbridge.toIpl());
	cvSaveImage(ss2.str().c_str(), rbridge.toIpl());
        cvSaveImage(ss5.str().c_str(),lastNonTexDisp);
        write_out_point_cloud(sscloudNoTex.str(),cloudNoTex);
	
	fileNum++;
	captureNoTex = false;
    }

    cv_mutex.unlock();

  }


  /** \brief Transform a compact RGB color info into 3 floats */
  void
    transformRGB (float val, float &r, float &g, float &b)
  {
    int rgb = *reinterpret_cast<int*>(&val);
    r = ((rgb >> 16) & 0xff) / 255.0f;
    g = ((rgb >> 8) & 0xff) / 255.0f;
    b = (rgb & 0xff) / 255.0f;
  }
  
  void write_out_point_cloud(string filename,  std_msgs::PointCloud &cloud_ )
  {
    int c_idx = -1;
    for (unsigned int d = 0; d < cloud_.chan.size (); d++)
    {
      if (cloud_.chan[d].name == "rgb")
        c_idx = d;
    }
    
    
    ofstream fout(filename.c_str());

    float r = 0.0, g = 0.0, b = 0.0;
    
    for (unsigned int i=0; i<cloud_.pts.size(); i++)
    {
      if (c_idx != -1)
        transformRGB (cloud_.chan[c_idx].vals[i], r, g, b);
      fout << cloud_.pts[i].x << " " << cloud_.pts[i].y << " " << cloud_.pts[i].z << " " << r << " " << g << " " << b << endl;
    }
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
      case 's':
         captureNoTex = true;
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

