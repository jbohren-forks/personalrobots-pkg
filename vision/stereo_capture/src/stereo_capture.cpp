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
#include <time.h>
#include <iostream>
#include <iomanip>


#include "image_msgs/CvBridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/node.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/Image.h"
#include "robot_msgs/PointCloud.h"
#include "std_msgs/UInt8.h" //for projector status
#include <string>

#include "color_calib.h"

#include "topic_synchronizer.h"

#include <boost/thread.hpp>

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

  robot_msgs::PointCloud cloud,cloudNoTex;

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

  bool capture;      //Capture stereo with and without texture
  bool captureNoTex; //Capture stereo without texture
  bool captureByTime;     //Capture after a given delay with texture
  bool captureNoTexByTime; //Capture after a given delay without texture
  int sync_capture;           //In capture with textured light, make sure we've seen a no texture, then texture display before saving data
  bool always_on_capture;

  std_msgs::UInt8 projector_status;
  clock_t trigger;

  boost::mutex cv_mutex;

  string fileName;
  char fileNumString[20];
  string mydir,XXX,SR,NP,m,prompt_;
  unsigned int fileNum;

  StereoView() : ros::Node("stereo_view"), 
                 lcal(this), rcal(this), lcalimage(NULL), rcalimage(NULL), lastDisparity(NULL), lastScaledDisparity(NULL), lastNonTexDisp(NULL), 
                 lastLeft(NULL), lastRight(NULL), sync(this, &StereoView::image_cb_all, ros::Duration().fromSec(0.05), &StereoView::image_cb_timeout),
		 capture(false), captureNoTex(false), captureByTime(false), captureNoTexByTime(false), sync_capture(0), always_on_capture(false)

  {
    //param("~file_name", fileName, "stereoImage");  //Base string to use, images will be [base][L/R/D].jpg (left, right, disparity)
    //param("~start_number", fileNum, 0);  //Number to start with (i.e. if program crashes start with n+1
                                         //where n is the number of the last image saved
    
  #define TIMER_FREQ 300000.0
  #define CAPTURE_FEQ_SECS  (15.0*TIMER_FREQ)

    mydir = "../ros-pkg/vision/stereo_capture/data/"; //The base runs in ~/ros
    XXX = "001"; //My ID
    SR = "R";    //Simulated or Real
    NP = "N";    //Natural or Painted
    m = "1";     //Object letter code
    prompt_ = "Input code for objects: "; //Prompt the user
  
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
    if(lastNonTexDisp)
      cvReleaseImage(&lastNonTexDisp);
  }

//=========== Get text input from user ===========//
// Prompt is promt to the user, init is the default string
std::string cvGetString(std::string prompt, std::string init)
{
	std::string str=init;
   	CvPoint orgprompt = cvPoint(10,30);
	CvPoint org = cvPoint(10,60);
	printf("%s\n%s",prompt.c_str(),str.c_str());
	fflush(stdout);
	int c = 0, slen;
	 //COLLECT USER INPUT, IGNORE WHITESPACE, ESC OUT
	 while (1) 
	 {
		c = cvWaitKey(0) & 0xFF;
		if(c == 27) {str.erase(); break;}
		if((c == 0)||(c > 128)) continue;
		if((c == 13)||(c == 10)) break; //Carriage return and/or line feed => accept this label
		if((c == ' ')||(c == '\t')) continue; //Ignore white space
		slen = str.length();
		printf("\r");
		for(int u=0; u<slen; ++u)
			printf(" ");
		printf("\r");
		if(c == 8) //backspace
		{
			//OVERLAY MASK ONTO IMAGE
			printf("\r");
			for(int u=0; u<slen; ++u)
				printf(" ");
			printf("\r");
			if(slen)
				str.erase(slen - 1);
		}
		else
		{
			str.append(1, (char)c);
		}
		printf("%s\r",str.c_str());
		fflush(stdout);
	} //End while
		//CLEAN UP AND OUT
		printf("string %s\n",str.c_str());
		printf("\n");
	  return str;
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
      if((!projector_status.data) || always_on_capture)
			{
	  			if(lastLeft != NULL)
	    			cvReleaseImage(&lastLeft);
	  			lastLeft = cvCloneImage(lbridge.toIpl());
	  			if(sync_capture == 2) sync_capture = 1;
			}      
      cvShowImage("left", lbridge.toIpl());
    }

    if (rbridge.fromImage(rimage, "bgr"))
    {
      if((!projector_status.data)||always_on_capture)
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
		  if(sync_capture == 1) sync_capture = 0;
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
	//THIS IS CAPTURE WHEN THE TEXTURE LIGHT IS ALWAYS ON
     if( always_on_capture && capture )//&& (lastDisparity != NULL)) 
      {
        cout << "w store..." << endl;
		stringstream ss1,ss1jpg, ss2, ss3, ss4, ss5, sscloud, sscloudNoTex;
		sprintf(fileNumString,"%.4d",fileNum);
		fileName = mydir + XXX + "." + fileNumString + "." + SR + "." + NP + "." + m;
		ss1jpg<<fileName<<".L"<<".jpg";
		ss1<<fileName<<".L"<<".png";
		ss2<<fileName<<".R"<<".png";
		ss3<<fileName<<".D"<<".png";
	//	ss4<<fileName<<".D"<<fileNum<<".u16";
        sscloud<<fileName<<".C"<<".txt";
		cout<<"Saving images "<<fileName<<" "<<ss1.str()<<" "<<ss1jpg.str()<<" "<<ss2.str()<<" "<<ss3.str()<<" "<< sscloud.str() << endl;
		cvSaveImage(ss1.str().c_str(), lastLeft);
		cvSaveImage(ss2.str().c_str(), lastRight);
		cvSaveImage(ss3.str().c_str(), lastScaledDisparity);
	//	cvSave(ss4.str().c_str(), lastDisparity);
        write_out_point_cloud(sscloud.str(),cloud);
		fileNum++;
		capture = false;
		always_on_capture = false;
		cout << "fileNum = " << fileNum << " !!!!!!!! you can move !!!!!!!!!!!!!" << endl;
      }  
//	THIS IS THE ALTERNATING CAPTURE HANDLER
    if((!sync_capture) && capture && (lastDisparity != NULL) && !always_on_capture) //Look here to save the non texture disparity image
      {
        cout << "Alternating capture" << endl;
		stringstream ss1,ss1jpg, ss2, ss3, ss4, ss5, sscloud, sscloudNoTex;
		sprintf(fileNumString,"%.4d",fileNum);
		fileName = mydir + XXX + "." + fileNumString + "." + SR + "." + NP + "." + m;
		ss1jpg<<fileName<<".L"<<".jpg";
		ss1<<fileName<<".L"<<".png";
		ss2<<fileName<<".R"<<".png";
		ss3<<fileName<<".D"<<".png";
	//	ss4<<fileName<<".D"<<fileNum<<".u16";
        ss5<<fileName<<".d"<<".png";
        sscloud<<fileName<<".C"<<".txt";
        sscloudNoTex<<fileName<<".CnoTex"<<".txt";
		cout<<"Saving images "<<fileName<<" "<<ss1.str()<<" "<<ss1jpg.str()<<" "<<ss2.str()<<" "<<ss3.str()<<" "<< ss5.str() << endl;
		cvSaveImage(ss1.str().c_str(), lastLeft);
		cvSaveImage(ss2.str().c_str(), lastRight);
		cvSaveImage(ss3.str().c_str(), lastScaledDisparity);
	//	cvSave(ss4.str().c_str(), lastDisparity);
        cvSaveImage(ss5.str().c_str(),lastNonTexDisp);
        write_out_point_cloud(sscloud.str(),cloud);
        write_out_point_cloud(sscloudNoTex.str(),cloudNoTex);
		fileNum++;
		capture = false;
		cout << "!!!!!!!!!!!!!!!!!!! move move move !!!!!!!!!!!!!!!!!!!!!" << endl;
      }
// THIS IS CAPTURE WHEN TEXTURE IS NEVER ON
    if(captureNoTex && !always_on_capture)
    {
		stringstream ss1,ss1jpg, ss2, ss3, ss4, ss5, sscloud, sscloudNoTex;
		sprintf(fileNumString,"%.4d",fileNum);
		fileName = mydir + XXX + "." + fileNumString + "." + SR + "." + NP + "." + m;
		ss1jpg<<fileName<<".L"<<".jpg";
		ss1<<fileName<<".L"<<".png";
		ss2<<fileName<<".R"<<".png";
        ss5<<fileName<<".d"<<".png";
        sscloudNoTex<<fileName<<".CnoTex"<<".txt";
		cout<<"Saving images "<<fileName<<" "<<ss1.str()<<" "<<ss1jpg.str()<<" "<<ss2.str()<<" "<< ss5.str() << endl;
/*		cout << "special save ../ros-pkg/vision/stereo_capture/data/foo.png" << endl;
	IplImage *I;	
	I = cvCreateImage(cvSize(200,100), 8, 3);
	cvSaveImage("../ros-pkg/vision/stereo_capture/data/foo.png",I);
	cvReleaseImage(&I);
	printf("Done saving image ../ros-pkg/vision/stereo_capture/data/foo.png\n");		
*/		
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
  
  void write_out_point_cloud(string file_name,  robot_msgs::PointCloud &cloud_ )
  {
    int c_idx = -1;
    for (unsigned int d = 0; d < cloud_.chan.size (); d++)
    {
      if (cloud_.chan[d].name == "rgb")
        c_idx = d;
    }
    
    
    ofstream fout(file_name.c_str());

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
      int key = cvWaitKey(3)&0x00FF;
      if(key == 27) //ESC
      	break;
      
	  string myprompt_ = "foo";
      switch (key) {
      case 'c':         //With textured light
      	if(!captureNoTexByTime)  {
      		m = "";
      		m = cvGetString(prompt_, m);  //GET USER LABEL FOR SCENE ...
	        capture = true;
	        captureNoTex = false;
	        sync_capture = 2;
		}
 			always_on_capture = false;
        break;
      case 'w':         //When textured light is always on
      		m = "";
      		m = cvGetString(prompt_, m);  //GET USER LABEL FOR SCENE ...
			captureNoTexByTime = false;
			captureByTime = false;
			capture = true;
			captureNoTex = false;
			sync_capture = 0;
			always_on_capture = true;
        break;
      case 's':         //Without textured light
			if(!captureByTime) {
			    captureNoTex = true;
			    capture = false;
			}
 			always_on_capture = false;
			sync_capture = 0;
         break;
	  case 'i':  //Init file name string
			myprompt_ = "Input your ID (0-999) (return or ESC to keep the default):";
			XXX = cvGetString(myprompt_,XXX);
			myprompt_ = "N for natural object, P for painted (ESC or Ret to keep default):";
			NP = cvGetString(myprompt_,NP);
		break;
      case 'h':
         printf(
                "\nCapture images and point clouds to /data directory\n"
				"\ti -- Init file name string XXX, N\n"
                "\tc -- Capture if projected light is running\n"
                "\tC -- Timed capture toggel if projected light is running\n"
                "\t  ** Watch out, textured capture waits for no, light, light before writing out images **\n"
                "\ts -- Capture (save) if *not* running projected light\n"
                "\tS -- Timed caputre if *not* running projected light\n"
                "\tw -- Write when textured light is always on\n\n"
                );
         break;
      case 'C':		//Peroidic capture ith textured light
			captureByTime ^= 1;
			captureNoTexByTime = false;
			trigger = clock() + CAPTURE_FEQ_SECS;
			always_on_capture = false;
			cout << "On_off" << (int)captureByTime <<  ":  Time of first captureByTime = " << setprecision(12) << trigger << endl;
			break;
      case 'S':      //Periodic capture without textured light
			captureNoTexByTime ^= 1;
			captureByTime = false;
			trigger = clock() + CAPTURE_FEQ_SECS;
			always_on_capture = false;
			cout << "On_off" << (int)captureNoTexByTime <<  ":  Time of first captureNoTexByTime = " << setprecision(12) << trigger << endl;
			break;      	
      }
      //Timed delay
      if(captureByTime)  //With textured light
      {
      	clock_t timenow = clock();
      	if(timenow > trigger)
      	{
      		capture = true;
      		sync_capture = 2;
      		captureNoTex = false;
      		trigger = timenow + CAPTURE_FEQ_SECS;
      		cout << "CaptureByTime Tigger = " <<  setprecision(10) << (float)timenow << endl;
      	}
      }
      if(captureNoTexByTime) //without textured light
      {
      	clock_t timenow = clock();
      	if(timenow > trigger)
      	{
      		captureNoTex = true;
      		capture = false;
      		trigger = timenow + CAPTURE_FEQ_SECS;
      		cout << "CaptureNoTexByTime Tigger = " <<  setprecision(10) << (float)timenow << endl;
      	}
      }
      cv_mutex.unlock();
      usleep(10000);
    }

    return true;
  }
};

int main(int argc, char **argv)
{
	for(int i = 0; i<argc; ++i)
		cout << "(" << i << "): " << argv[i] << endl;

  ros::init(argc, argv);
  StereoView view;
  view.spin();
  
  return 0;
}

