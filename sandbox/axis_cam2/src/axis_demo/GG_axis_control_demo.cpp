///////////////////////////////////////////////////////////////////////////////
// The axis_cam package provides a library that talks to Axis IP-based cameras
// as well as ROS nodes which use these libraries
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//
// NOTES:
// 9/15/08  Modified by Gary Bradski to 
//           (1) Simplify the demo -- detached from axis_cam and axis_ptz
//           (2) Put up a slider controls
//
// Typically in this file, to make, you'll call
// rosmake axis_cam2
//
// To run, it uses demo.xml to configure, so:
// roslaunch demo.xml
//    This will launch a axis_ptz and axis_cam node
//
// 
//

#include <stdio.h>
#include <iostream>

#include "ros/node.h"
#include "std_msgs/Image.h"
#include "axis_cam/PTZActuatorCmd.h"
#include "axis_cam/PTZActuatorState.h"
#include "image_utils/cv_bridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

//INTERACTION GLOBALS, PUT OTHER STUFF HERE
int g_pan = 175; //From -175 to 175, since slider is zero based, subtract off 175
bool g_pan_go = 0;

int g_tilt = 45; //From -45 to 90. Since slider zero based, subtract off 45
bool g_tilt_go = 0;

int g_zoom = 0; //From 0 to ??
bool g_zoom_go = 0;

// ====================================

//Insert your code here
void left_mouse(int event, int x, int y, int flags, void *params)
{
//  printf("Mouse event %d at (%d, %d) with flag=%d\n",event,x,y,flags);
} 

void pan_callback(int pos)
{
	g_pan_go = true;
//	printf("g_pan=%d\n",g_pan);
}

void tilt_callback(int pos)
{
	g_tilt_go = true;
//	printf("g_tilt%d\n",g_tilt);
}

void zoom_callback(int pos)
{
	g_zoom_go = true;
//	printf("g_zoom=%d\n",g_zoom);
}


/////////////////////////////////////////////////////////////////////
// AxisControl
/////////////////////////////////////////////////////////////////////

class AxisControl : public ros::node
{
public:
  std_msgs::Image image_;
  CvBridge<std_msgs::Image> cv_bridge_;

  axis_cam::PTZActuatorCmd   ptz_cmd_;
  axis_cam::PTZActuatorState ptz_state_;
  bool quit;

  IplImage *cv_image_;
// PUT YOUR PUBLIC VARIABLES HERE

// - - - - - - - - - - - 

//CONSTRUCTOR THAT ESTABLISHES THIS AS A ROS NODE, INIT PUBLIC VARIABLES HERE
  AxisControl() : ros::node("axis_control_demo"), cv_bridge_(&image_, CvBridge<std_msgs::Image>::CORRECT_BGR), cv_image_(NULL), quit(false)
  {
    //This pops up an OpenCV highgui window and puts in mouse call back and 3 sliders
	cvNamedWindow("cv_view", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("cv_view", left_mouse, 0);
	cvCreateTrackbar("Pan", "cv_view", &g_pan, 350, pan_callback);
	cvCreateTrackbar("Tilt", "cv_view", &g_tilt, 135, tilt_callback);
	cvCreateTrackbar("Zoom", "cv_view", &g_zoom, 10000, zoom_callback);

	//Ros stuff, subscribe to image and ptz state nodes
    subscribe("image", image_, &AxisControl::image_cb, 1);
    subscribe("ptz_state", ptz_state_, &AxisControl::ptz_cb, 1);

	//Advertise that you'll be talking commands to the ptz
    advertise<axis_cam::PTZActuatorCmd>("ptz_cmd", 1);

	//I PRE SET THE COMMANDS HERE
        g_pan = 175;
	ptz_cmd_.pan.cmd   = g_pan - 175;
	ptz_cmd_.pan.rel   = 0;
	ptz_cmd_.pan.mode  = 0;
	ptz_cmd_.pan.valid = 1;
	g_pan_go = true;  //Start at 0

	g_tilt = 45;
	ptz_cmd_.tilt.cmd = g_tilt - 45;
	ptz_cmd_.tilt.rel = 0;
	ptz_cmd_.tilt.mode = 0;
	ptz_cmd_.tilt.valid=1;
	g_tilt_go = true;

	g_zoom = 0;
	ptz_cmd_.zoom.cmd = g_zoom;
	ptz_cmd_.zoom.rel = 0;
	ptz_cmd_.zoom.mode = 0;
	ptz_cmd_.zoom.valid = 1;
	g_zoom_go = true;
  }

  ~AxisControl()
  {
    if (cv_image_)
      cvReleaseImage(&cv_image_);
  }



//ENTER STUFF YOU WANT TO DO AFTER YOU GET A NEW IMAGE HERE
  void image_cb()
  {
 //   printf("Received %dx%d image\n", image_.width, image_.height);
	printf("IMAGE IN");
    if (cv_image_)
      cvReleaseImage(&cv_image_);

    if (cv_bridge_.to_cv(&cv_image_)){
	printf("Showing image\n");
      cvShowImage("cv_view", cv_image_);
}

    //HERE YOU CAN PUT IN KEYBOARD COMMANDS, "c" will have the ascii key code

//test for long processing:	usleep(3000000);
	printf(" ... LEAVING IMAGE IN\n");
  }

//ENTER STUFF YOU WANT TO DO AFTER GETTING A STATE UPDATE FROM THE PTZ HERE
  void ptz_cb()
  {
	if(g_pan_go || g_tilt_go || g_zoom_go){
   		printf("Axis state was:\n");
    		printf(" Pan:  %f\n", ptz_state_.pan.pos);
    		printf(" Tilt: %f\n", ptz_state_.tilt.pos);
    		printf(" Zoom: %f\n\n", ptz_state_.zoom.pos);
		if(g_pan_go) {
			g_pan_go = false;
	    		printf("Pan cmd is =%d; ",g_pan - 175);
			ptz_cmd_.pan.cmd = g_pan - 175;
		}
		if(g_tilt_go) {
			g_tilt_go = false;
			printf("Tilt cmd is =%d; ",g_tilt - 45);
			ptz_cmd_.tilt.cmd = g_tilt - 45;
		}
		if(g_zoom_go) {
			g_zoom_go = false;
			printf("Zoom cmd is = %d ",g_zoom);
			ptz_cmd_.zoom.cmd = g_zoom;
		}
		printf("\n");
		publish("ptz_cmd", ptz_cmd_);
	}

  }

//MOSTLY A NO OP LOOP PLUS KEYBOARD INTERACTION STATE AND IMAGE FUNCTIONS ARE ABOVE
  bool spin()
  {
    while (ok() && !quit)
    {
    int c = cvWaitKey(3);
	c &= 0xFF;
        //I quit on ESC, "q" or "Q"
	if((c == 27)||(c == 'q')||(c == 'Q'))
		quit = true;
 
//      ptz_cmd_.pan.cmd = -ptz_cmd_.pan.cmd;
//      ptz_cmd_.tilt.cmd= -ptz_cmd_.tilt.cmd;
//      publish("ptz_cmd", ptz_cmd_);
//      usleep(5000000);
    }
    return true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  AxisControl a;

  a.spin();

  ros::fini();

  return 0;
}
