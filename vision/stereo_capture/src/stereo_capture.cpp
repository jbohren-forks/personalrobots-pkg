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

//#include "ros/node.h"
//#include "image_msgs/StereoInfo.h"
//#include "image_msgs/DisparityInfo.h"
//#include "image_msgs/Image.h"


#include "ros/node.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/Image.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/ColoredLines.h"
#include "image_msgs/ColoredLine.h"

//#include "CvStereoCamModel.h"

#include "robot_msgs/PointCloud.h"
#include "std_msgs/UInt8.h" //for projector status
#include <string>


//  to get and use camera info ...
//#include "image_msgs/CamInfo.h"
//#include "CvStereoCamModel.h"
// ... //

#include "color_calib.h"
#include "topic_synchronizer/topic_synchronizer.h"
#include <boost/thread.hpp>

//DEFINES
 // For image type captured
#define GOT_LEFT_NO_TEXTURE 1
#define GOT_RIGHT_NO_TEXTURE 2
#define GOT_LEFT_TEXTURE 4
#define GOT_RIGHT_TEXTURE 8
#define GOT_D 16
#define GOT_d 32


using namespace std;

boost::mutex g_cv_mutex;

struct MouseCallbackParams {
  vector<float> *W;
  int width,height;
  int x,y;
  int draw_count;
};

/*!
 * \brief Click on a point in the left image to get 2d, color and 3d information.
 *
 *
 * Click on a point in the left image to 
 * 1) Draw a red dot;
 * 2) Get the (x,y,z) value in the stereo frame (the vision-version of x,y,z with z pointing forwards.)
 */
void on_mouse(int event, int x, int y, int flags, void *params) {
	MouseCallbackParams* p = (MouseCallbackParams *)params;
	float X,Y,Z,A;
	int width,height;
	 switch(event) {

	  case CV_EVENT_MOUSEMOVE:
		 break;

	  case CV_EVENT_LBUTTONUP:
		 g_cv_mutex.lock(); 
		 width = p->width;
		 height = p->height;
		 if((int)((*(p->W)).size()) == width*height*4){
		 	 int idx = (y*width + x)*4;
		 	 X =(*(p->W))[idx++];
		 	 Y =(*(p->W))[idx++];
		 	 Z =(*(p->W))[idx++];	
		 	 A	=(*(p->W))[idx];		 		 	 
			 p->x = x;
			 p->y = y;
			 p->draw_count = 200;
			 printf("Stereo (%d,%d): (%f, %f, %f)A:%f\n", x,y,X,Y,Z,A);	//	-Y,-Z,X); //
		 }
		 else
		 	printf("Stereo ... nothing to report\n");    	    
		 g_cv_mutex.unlock();
		 break;
		 
	  default:
		 break;
	 }
}    
    
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
  image_msgs::CamInfo rcaminfo; /**< Right camera info msg. */

  robot_msgs::PointCloud cloud, cloudIsTex, cloudNoTex;

  color_calib::Calibration lcal;
  color_calib::Calibration rcal;

  IplImage* lcalimage;
  IplImage* rcalimage;	
  
  IplImage* disparityIsTextured;
  IplImage* leftTextured;
  IplImage* rightTextured;
  IplImage* leftNotTextured;
  IplImage* rightNotTextured;
  IplImage* disparityNotTextured; //This is the disparity from the non-textured light
  vector<float> W;  				/**< Will hold the world X,Y,Z,A points for each pixel in order. */
//  CvStereoCamModel* cam_model;  /**< Camera model so we can convert disparity to depth. */
//  bool init_cam_model;          /**< triggers initialization of camera model. */
//  float ZnearMM, ZfarMM; 			/**< Ignore depths closer, further than this. */
  MouseCallbackParams mcbparams;
  int skip_convert_count;
  
  TopicSynchronizer<StereoView> sync;
  int capture_type;   //Uses the GOT_ defines above
  bool capture;      //Capture stereo with and without texture
  bool captureNoTex; //Capture stereo without texture
  bool captureByTime;     //Capture after a given delay with texture
  bool captureNoTexByTime; //Capture after a given delay without texture
  bool always_on_capture;
  
  int projStatus,projStatusPrev,projStatusCount;

  std_msgs::UInt8 projector_status;
  clock_t trigger;

  boost::mutex cv_mutex;

  string fileName;
  char fileNumString[20];
  string mydir,DESC,m,tn,prompt_; //SR,NP, //m is object code, tn is texture/no_texture
  unsigned int fileNum;

  StereoView() : ros::Node("stereo_view"), 
                 lcal(this), rcal(this), lcalimage(NULL), rcalimage(NULL), disparityIsTextured(NULL), disparityNotTextured(NULL), 
                 leftTextured(NULL), rightTextured(NULL), leftNotTextured(NULL), rightNotTextured(NULL), 
                 sync(this, &StereoView::image_cb_all, ros::Duration().fromSec(0.05), &StereoView::image_cb_timeout),
                 projStatus(0), projStatusPrev(0), projStatusCount(0), skip_convert_count(0),
		 capture(false), captureNoTex(false), captureByTime(false), captureNoTexByTime(false), 
		 capture_type(0), always_on_capture(false)

  {
    //param("~file_name", fileName, "stereoImage");  //Base string to use, images will be [base][L/R/D].jpg (left, right, disparity)
    //param("~start_number", fileNum, 0);  //Number to start with (i.e. if program crashes start with n+1
                                         //where n is the number of the last image saved
    
  #define TIMER_FREQ 300000.0
  #define CAPTURE_FEQ_SECS  (15.0*TIMER_FREQ)

    mydir = "../ros-pkg/vision/stereo_capture/data/"; //The base runs in ~/ros
    DESC = "cups"; //Default data description
//    SR = "R";    //Simulated or Real
//    NP = "N";    //Natural or Painted
    m = "1";     //Object letter code
    tn = "not_set"; //(T)exture or (N)o texture
    prompt_ = "Input code for objects: "; //Prompt the user
  
    fileNum = 0;
 

    

    cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
    mcbparams.W = &W;
    mcbparams.width = 1;
    mcbparams.height = 2;
    mcbparams.x = 0;
    mcbparams.y = 0;
    mcbparams.draw_count = 0;
    cvSetMouseCallback("left", on_mouse, &mcbparams);
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
    sync.subscribe("stereo/right/cam_info", rcaminfo, 1);
    sync.ready();

    subscribe("projector_status", projector_status, &StereoView::projector_status_change, 1);
  }

  ~StereoView()
  {
    if (lcalimage)
      cvReleaseImage(&lcalimage);
    if (rcalimage)
      cvReleaseImage(&rcalimage);
    if (disparityIsTextured)
      cvReleaseImage(&disparityIsTextured);
    if (leftTextured)
      cvReleaseImage(&leftTextured);
    if (rightTextured)
      cvReleaseImage(&rightNotTextured);
    if (leftNotTextured)
      cvReleaseImage(&leftNotTextured);
    if (rightNotTextured)
      cvReleaseImage(&rightTextured);
    if(disparityNotTextured)
      cvReleaseImage(&disparityNotTextured);
 //   if(cam_model){
 //   	delete cam_model;
 //   	cvReleaseImage(&IZ);
 //   	cvReleaseImage(&IX);
 //   	cvReleaseImage(&IY);
 //   }
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

//These defines simply are a count to de-bounce the projector on/off message since it's not always exactly sync'd
#define PROJCOUNT 30
#define PROJGREEN 27
#define PROJRED 26
void image_cb_all(ros::Time t)
{
	
   cv_mutex.lock();
	string greenstr = " ";
	//Track state change
	projStatus = (int)projector_status.data;
	if(projStatus != projStatusPrev){
		projStatusCount = PROJCOUNT; //Up to 30 fps count down, though really usually ~7
	}
	projStatusPrev = projStatus;
	if((PROJGREEN >= projStatusCount) && (projStatusCount > PROJRED))	 greenstr = "   (debounced)";
	cout << "projector: " << projStatus << greenstr << endl;
	
    if (lbridge.fromImage(limage, "bgr"))
    { 
		if(((PROJGREEN >= projStatusCount) && (projStatusCount > PROJRED))|| captureNoTex || always_on_capture)
		{    
		   if((!projector_status.data)) //Texture is off
				{
		  			if(leftNotTextured != NULL)
			 			cvReleaseImage(&leftNotTextured);
		  			leftNotTextured = cvCloneImage(lbridge.toIpl());
					if(capture || captureNoTex)
						capture_type |= GOT_LEFT_NO_TEXTURE;
				}
			else // (projector_status.data) //Texture is on
			{
				if(leftTextured != NULL)
		 			cvReleaseImage(&leftTextured);
				leftTextured = cvCloneImage(lbridge.toIpl());
				if(capture || captureNoTex)
					capture_type |= GOT_LEFT_TEXTURE;
			}      
 		}
 		if(leftTextured && mcbparams.draw_count)
 		{
 		   int dc = mcbparams.draw_count;
 			cvCircle(leftTextured, cvPoint(mcbparams.x,mcbparams.y), 2, cvScalar(dc,dc,dc), 4);
    		cvShowImage("left",leftTextured);
    		mcbparams.draw_count -= 100;
    		if(mcbparams.draw_count < 0) mcbparams.draw_count = 0;
    	}
    	else 
	     	cvShowImage("left", lbridge.toIpl());
    }

    if (rbridge.fromImage(rimage, "bgr"))
    {
		if(((PROJGREEN >= projStatusCount) && (projStatusCount > PROJRED))|| captureNoTex || always_on_capture)
		{      
			  if((!projector_status.data)) //Texture if off
			  {
				  if(rightNotTextured != NULL)
					cvReleaseImage(&rightNotTextured);
				  rightNotTextured = cvCloneImage(rbridge.toIpl());
	 			  if(capture || captureNoTex)
					capture_type |= GOT_RIGHT_NO_TEXTURE;
			  }
			  else //Texture is on
			  {
				  if(rightTextured != NULL)
					  cvReleaseImage(&rightTextured);
				  rightTextured = cvCloneImage(rbridge.toIpl());
				  if(capture || captureNoTex)
					  capture_type |= GOT_RIGHT_TEXTURE;
			  }	  
		}
      cvShowImage("right", rbridge.toIpl());
    }

    if (dbridge.fromImage(dimage))
    {
      IplImage* disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);
      cvCvtScale(dbridge.toIpl(), disp, 4.0/dispinfo.dpp);
		if(((PROJGREEN >= projStatusCount) && (projStatusCount > PROJRED))|| captureNoTex || always_on_capture)
		{   
/*		   if(init_cam_model)
		   {
		      printf("cam_model initialized! *********************************************\n");
				cam_model = new CvStereoCamModel(rcaminfo.P[0], rcaminfo.P[5], 
											-(rcaminfo.P[3])/(rcaminfo.P[0]), 
											rcaminfo.P[2], rcaminfo.P[2], 
											rcaminfo.P[6], 1.0/dispinfo.dpp);
				IZ  = cvCreateImage( cvGetSize(disp), IPL_DEPTH_32F, 3 );
				IX = cvCloneImage(IZ);
				IY = cvCloneImage(IZ);
		   	init_cam_model = false;
		   }   
*/			if(projector_status.data)
			{
			  if(disparityIsTextured != NULL)
				cvReleaseImage(&disparityIsTextured);	  
			  disparityIsTextured = cvCloneImage(disp);
			  cloudIsTex = cloud;
			  if(capture || captureNoTex)
				  capture_type |= GOT_D;
			} 
			else //Save non textured light disparity too
			{ 
			  if(disparityNotTextured != NULL)
				cvReleaseImage(&disparityNotTextured);
			  disparityNotTextured = cvCloneImage(disp);
			  cloudNoTex = cloud;
			  if(capture || captureNoTex)
				  capture_type |= GOT_d;
			}
 		}
// 		if(cam_model) cam_model->dispToCart(disp,IZ);//disp8UToCart32F(disp, ZnearMM, ZfarMM, IZ, IX, IY);
   
		skip_convert_count--;
		if(skip_convert_count <= 0)
		{
			mcbparams.width = disp->width;
	  	  	mcbparams.height = disp->height;
		   point_cloud_to_xyVector(cloud, W, disp->width, disp->height );
		   skip_convert_count = 10;
		}
		cvShowImage("disparity", disp);
		cvReleaseImage(&disp);
    }
	//THIS IS CAPTURE WHEN THE TEXTURE LIGHT IS ALWAYS ON
     if( always_on_capture && capture && (capture_type == (GOT_LEFT_TEXTURE+GOT_RIGHT_TEXTURE+GOT_D)))
      {
         cout << "w store..." << endl;
			stringstream ss1,ss1jpg, ss2, ss3, ss4, ss5, sscloud, sscloudNoTex;
			sprintf(fileNumString,"%.4d",fileNum);
			fileName = mydir + DESC + "." + fileNumString + "." + m; //+ SR + "." + NP + "."
			ss1jpg<<fileName<<".L"<<".jpg";
			ss1<<fileName<<".L"<<".png";
			ss2<<fileName<<".R"<<".png";
			ss3<<fileName<<".D"<<".png";
		//	ss4<<fileName<<".D"<<fileNum<<".u16";
		   sscloud<<fileName<<".C"<<".txt";
			cout<<"Saving images "<<fileName<<" "<<ss1.str()<<" "<<ss1jpg.str()<<" "<<ss2.str()<<" "<<ss3.str()<<" "<< sscloud.str() << endl;
			cvSaveImage(ss1.str().c_str(), leftTextured);
			cvSaveImage(ss2.str().c_str(), rightTextured);
			cvSaveImage(ss3.str().c_str(), disparityIsTextured);
		   write_out_point_cloud(sscloud.str(),cloudIsTex);
			fileNum++;
			capture = false;
			always_on_capture = false;
			capture_type = 0;
			cout << "fileNum = " << fileNum << " !!!!!!!! always on can move !!!!!!!!!!!!!" << endl;
      }  
//	THIS IS THE ALTERNATING CAPTURE HANDLER
    if( capture && (!always_on_capture) && (capture_type == (GOT_LEFT_NO_TEXTURE+GOT_RIGHT_NO_TEXTURE+GOT_LEFT_TEXTURE+GOT_RIGHT_TEXTURE+GOT_D+GOT_d))) //Look here to save the non texture disparity image
      {
	      cout << "Alternating capture" << endl;
			stringstream ss1,ss1jpg, ss2, ss3, ss4, ss5, sscloud, sscloudNoTex, ss_l,ss_r;
			sprintf(fileNumString,"%.4d",fileNum);
			fileName = mydir + DESC + "." + fileNumString + "." + m;//+ SR + "." + NP + "." 
			ss1jpg<<fileName<<".l"<<".jpg";
			ss1<<fileName<<".L"<<".png"; //left image texture
			ss_l<<fileName<<".l"<<".png"; //left image no texure
			ss2<<fileName<<".R"<<".png"; //right image texture
			ss_r<<fileName<<".r"<<".png"; //right image no texture
			ss3<<fileName<<".D"<<".png";
		//	ss4<<fileName<<".D"<<fileNum<<".u16";
			ss5<<fileName<<".d"<<".png";
			sscloud<<fileName<<".C"<<".txt";
			sscloudNoTex<<fileName<<".CnoTex"<<".txt";
			cout<<"Saving images "<<fileName<<" "<<ss1.str()<<" "<<ss1jpg.str()<<" "<<ss_l.str()<<" "<<ss2.str()<<" "<<ss_r.str()<<" "<<ss3.str()<<" "<< ss5.str() << endl;
			cout<<"  .L, .R, .D => with texture, .l, .r, .d => without texture\n"<< endl;
			cvSaveImage(ss1.str().c_str(), leftTextured);
			cvSaveImage(ss_l.str().c_str(),leftNotTextured);
			cvSaveImage(ss2.str().c_str(), rightTextured);
			cvSaveImage(ss_r.str().c_str(),rightNotTextured);
			cvSaveImage(ss3.str().c_str(), disparityIsTextured);
			cvSaveImage(ss5.str().c_str(),disparityNotTextured);
			write_out_point_cloud(sscloud.str(),cloudIsTex);
			write_out_point_cloud(sscloudNoTex.str(),cloudNoTex);
			fileNum++;
			capture = false;
			capture_type = 0;
			cout << "!!!!!!!!!!!!!!!!!!! alternate move move !!!!!!!!!!!!!!!!!!!!!" << endl;
      }
// THIS IS CAPTURE WHEN TEXTURE IS NEVER ON
    if(captureNoTex && (!always_on_capture) && (capture_type == (GOT_LEFT_NO_TEXTURE+GOT_RIGHT_NO_TEXTURE+GOT_d)))
    {
			stringstream ss1,ss1jpg, ss2, ss3, ss4, ss5, sscloud, sscloudNoTex;
      	if((tn == "T") || (tn == "N"))
      	{
      		if(tn == "N") fileNum--;
      	   cout << "TN string " << tn << endl;
 				sprintf(fileNumString,"%.4d",fileNum);
				fileName = mydir + DESC + "." + fileNumString + "." + tn + "." + m;// + SR + "." + NP + "."
				if(tn == "T"){
					ss1jpg<<fileName<<".L"<<".jpg";
					ss1<<fileName<<".L"<<".png";
					ss2<<fileName<<".R"<<".png";
					ss5<<fileName<<".D"<<".png";
					sscloudNoTex<<fileName<<".C"<<".txt";
				} else { //tn == "N"
					ss1jpg<<fileName<<".l"<<".jpg";
					ss1<<fileName<<".l"<<".png";
					ss2<<fileName<<".r"<<".png";
					ss5<<fileName<<".d"<<".png";
					sscloudNoTex<<fileName<<".c"<<".txt";		
				}
				cout<<"Saving images "<<fileName<<" "<<ss1.str()<<" "<<ss1jpg.str()<<" "<<ss2.str()<<" "<< ss5.str() << endl;
				cvSaveImage(ss1.str().c_str(), leftNotTextured);
				cvSaveImage(ss2.str().c_str(), rightNotTextured);
				cvSaveImage(ss5.str().c_str(),disparityNotTextured);
				write_out_point_cloud(sscloudNoTex.str(),cloudNoTex);
	     	}
      	else
      	{
				sprintf(fileNumString,"%.4d",fileNum);
				fileName = mydir + DESC + "." + fileNumString + "." + m;// + SR + "." + NP + "."
				ss1jpg<<fileName<<".L"<<".jpg";
				ss1<<fileName<<".L"<<".png";
				ss2<<fileName<<".R"<<".png";
				ss5<<fileName<<".d"<<".png";
				sscloudNoTex<<fileName<<".CnoTex"<<".txt";
				cout<<"Saving images "<<fileName<<" "<<ss1.str()<<" "<<ss1jpg.str()<<" "<<ss2.str()<<" "<< ss5.str() << endl;
				cvSaveImage(ss1.str().c_str(), leftNotTextured);
				cvSaveImage(ss2.str().c_str(), rightNotTextured);
				cvSaveImage(ss5.str().c_str(),disparityNotTextured);
				write_out_point_cloud(sscloudNoTex.str(),cloudNoTex);
		   }
			fileNum++;
			captureNoTex = false;
			capture_type = 0;
			cout <<"!!!!!!!!!!!!!!! no texture move move !!!!!!!!!!!!!!" << endl;
    }
    //State transition
	 projStatusCount -= 1;
	 if(projStatusCount < 0) projStatusCount = 0;
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
  
  /** \brief Writes out point cloud: X,Y,Z,r,g,b or X,Y,Z,r,g,b,img(x),img(y) if stereo/do_keep_coords launch param is true */
  void write_out_point_cloud(string file_name,  robot_msgs::PointCloud &cloud_ )
  {
    int c_idx = -1;
    int cx_idx = -1, cy_idx = -1;
    for (unsigned int d = 0; d < cloud_.chan.size (); d++)
    {
      if (cloud_.chan[d].name == "rgb")
        c_idx = d;
      if (cloud_.chan[d].name == "x")
       cx_idx = d;
      if (cloud_.chan[d].name == "y")
       cy_idx = d;
    }
     
    ofstream fout(file_name.c_str());

    float r = 0.0, g = 0.0, b = 0.0;
    int x = -1,y = -1;
    if(c_idx != -1)
    {
		 if((cx_idx >= 0)&&(cy_idx >=0)){
			 for (unsigned int i=0; i<cloud_.pts.size(); i++)
			 {
			  transformRGB (cloud_.chan[c_idx].vals[i], r, g, b);
			  x = (int)(cloud_.chan[cx_idx].vals[i]);
			  y = (int)(cloud_.chan[cy_idx].vals[i]);
				//-Y,-Z,X); //X,Y,Z,A); Warning, these points are rotated to head coords, thus I unrotate here
				fout << -cloud_.pts[i].y << " " << -cloud_.pts[i].z << " " << cloud_.pts[i].x << " " << r << " " << g << " " << b << " " << x << " " << y << endl;
			 }
		 }
		 else{
			for (unsigned int i=0; i<cloud_.pts.size(); i++)
			 {
				transformRGB (cloud_.chan[c_idx].vals[i], r, g, b);
				//-Y,-Z,X); //X,Y,Z,A); Warning, these points are rotated to head coords, thus I unrotate here
				fout << -cloud_.pts[i].y << " " << -cloud_.pts[i].z << " " << cloud_.pts[i].x << " " << r << " " << g << " " << b << endl;
			 }   
		 } 
	 }
    fout.close();
  }

  /** \brief Puts point cloud into 32F vector(width,height) of X, Y, Z and A assuming stereo/do_keep_coords launch param is true */
  void point_cloud_to_xyVector(robot_msgs::PointCloud &cloud_, vector<float> &W, int width, int height )
  {
    int c_idx = -1;
    int cx_idx = -1, cy_idx = -1;
    int sizeW = (int)W.size();
    int maxElems = 4*width*height;
    if(sizeW != maxElems)
    {
    	printf("WARNING: W.size()[%d] in point_cloud_to_xyVector mismatch with width(%d)*height(%d)*4 [%d], resizing\n",  
    	       sizeW,width,height,maxElems);
    	W.resize(maxElems);
    }
    W.assign(maxElems,-1.0); //Clear 
    //FIND THE CORRECT CHANNEL INDICES
    for (unsigned int d = 0; d < cloud_.chan.size (); d++)
    {
      if (cloud_.chan[d].name == "rgb")
        c_idx = d;
      if (cloud_.chan[d].name == "x")
       cx_idx = d;
      if (cloud_.chan[d].name == "y")
       cy_idx = d;
    }  
    //MAKE SURE WE ACTUALL FOUND THE CHANNELS
    if((cx_idx < 0)||(cy_idx < 0)||(c_idx < 0)) //If we don't have the right point cloud, give up
    {
    	printf("WARNING: Point cloud doesn't have image x,y coordinates. Set stereo/do_keep_coords in the launch file, RETURN\n");
    	return;  
    }
	//FILL W
    int x,y,idx;
	 for (unsigned int i=0; i<cloud_.pts.size(); i++)
	 {
	     x = (int)(cloud_.chan[cx_idx].vals[i]);
	     y = (int)(cloud_.chan[cy_idx].vals[i]); 
	     idx = (width*y + x)*4;
	     W[idx++] = -cloud_.pts[i].y;//-Y,-Z,X); //X,Y,Z,A); Warning, these points are rotated to head coords, thus I unrotate here
	     W[idx++] = -cloud_.pts[i].z;
	     W[idx++] = cloud_.pts[i].x;
	     W[idx] = 0.0;
	 }
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
  	 bool texture_on = true;
    while (ok())
    {
      cv_mutex.lock();
      int key = cvWaitKey(3)&0x00FF;
      if(key == 27) //ESC
      	break;
      
	  string myprompt_ = "foo";
	  string tm = m; 
      switch (key) {
      case 'c':         //With alternating textured light
      	if(!captureNoTexByTime)  {
      		m = "";
      		m = cvGetString(prompt_, m);  //GET USER LABEL FOR SCENE ...
	        capture = true;
	        captureNoTex = false;
			capture_type = 0;
			cout <<"db capture on! " << endl;
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
			always_on_capture = true;
			capture_type = 0;
        break;
      case 'n':   //Write when red textured light is off
      	texture_on = false;
      case 't':         //Write when red textured light is on
      	texture_on = true;
      	if(key == 't'){
      		tn = "T";      
     		}	
     		else {
      		tn = "N";
      	}
 			captureNoTex = true;
		   capture = false;
			capture_type = 0;
 			always_on_capture = false;     		     		
			captureNoTexByTime = false;
			captureByTime = false;
        break;    
     case 'e':  //Erase object type string
     		m = "";
     		cout << "Erased object type string" << endl;
     		break;   
     case 'm':  //Set object  
     		m = "";
    		m = cvGetString(prompt_, m);  //GET USER LABEL FOR SCENE ...
    		if(m == "") m = tm;
    		break;
	  case 'i':  //Init file name string
			myprompt_ = "Input data description (return or ESC to keep the default):";
			DESC = cvGetString(myprompt_,DESC);
//			myprompt_ = "N for natural object, P for painted (ESC or Ret to keep default):";
//			NP = cvGetString(myprompt_,NP);
			break;
		case '-': //Decrease file number
			fileNum--;
			if(fileNum < 0) fileNum = 0;
			cout << "fileNum-- = " << fileNum << endl;
			break;
		case '+': //Increase file number
		 	fileNum++;
			cout << "fileNum++ = " << fileNum << endl;
			break;		 	
      case 's':         //Without textured light
			if(!captureByTime) {
			    captureNoTex = true;
			    capture = false;
			}
			capture_type = 0;
 			always_on_capture = false;
         break;
      case 'h':
         printf(
                "\nCapture images and point clouds to /data directory\n"
		 		    "\ti -- Init file name string DESC (data description), N\n"
                "\tc -- Capture if projected light is alternating on and off\n"
                "\tC -- Timed capture toggel if projected light is alternating on and off\n"
                "\ts -- Capture (save) if *not* running projected light\n"
                "\tS -- Timed caputre if *not* running projected light\n"
                "\tt -- Write with red textured light on\n"
                "\tn -- Write with red textured light off\n"
                "\tm -- Set object type string\n"
                "\te -- Erase object type string 'm'\n"
                "\t+ -- File number ++\n"
                "\t- -- File number --\n"
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
      		captureNoTex = false;
      		trigger = timenow + CAPTURE_FEQ_SECS;
			capture_type = 0;
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
			capture_type = 0;
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

