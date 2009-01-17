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

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <time.h>

#include "opencv/cv.h"
#include "opencv/cvaux.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"

#include "ros/node.h"

#include "std_msgs/UInt8.h"

#include "limits.h"

#include <iostream>

#define WD 1180
#define HT 768
#define DELAY 30
#define BLOCK_SIZE 4

//#define COLOR
inline int min(int a, int b){
return (a<b)?a:b;
}


bool BlankScreen = true;
bool BlankScreenPrevious = false;
IplImage* Rbp;
IplImage* BlankImage;
IplImage *current;

std_msgs::UInt8 status_out;

using namespace std;

class LightProjector : public ros::Node
{
public:

    LightProjector() : ros::Node("light_projector") 
    {
      param("~frequency", frequency, 0.5);
      advertise<std_msgs::UInt8>("projector_status", 10);
    }

    void grayscale_rbp(IplImage *rbp){

    int w = rbp->width;
    int h = rbp->height;
    int ws = rbp->widthStep;
    unsigned char *drbp = ((unsigned char *)(rbp->imageData));

    int i,j,x,y, rn;
    int bs = BLOCK_SIZE;

    for(y=0; y<h; y+=bs)
    for(x=0; x<w; x+=bs){
            rn = (rand()%2)*128;// 256;
	    for(j=0; j<bs; j++)
	    for(i=0; i<bs; i++)
	    drbp[(y+j)*ws+x+i] = rn; 
    }


    }
    
    bool spin()
    {
      int key = 'a';
      while (ok())
	{
	  key = cvWaitKey(10);	
	    
	  if(!BlankScreen){
	    grayscale_rbp(Rbp);
	    cvShowImage("RBP Projector", Rbp);
	  }
	  else{
	    cvShowImage("RBP Projector", BlankImage);
	  }

	  status_out.data = BlankScreen ? 1 : 0;
	  publish("projector_status", status_out);
	  
	  usleep(1000000.0/frequency);	      
	  BlankScreen = !BlankScreen;
	}
      return true;
    }
    
private:
  double frequency;
  
};

int main(int argc, char **argv){

  ros::init(argc, argv);

  Rbp = cvCreateImage(cvSize(WD,HT), 8, 1);
  BlankImage = cvCreateImage(cvSize(WD,HT), 8, 1);
  for(int j=0; j<BlankImage->height; j++)
    for(int i=0; i<BlankImage->width; i++)
      ((unsigned char *)(BlankImage->imageData))[j*BlankImage->widthStep+i] = 0;

  cvNamedWindow("RBP Projector", 0);
  cvResizeWindow("RBP Projector", WD, HT);
  cvMoveWindow("RBP Projector", 0, 0);

  cvShowImage("RBP Projector", BlankImage);

  LightProjector LP;

  

  srand ( time(NULL) );

  LP.spin();

  ros::fini();

  cvReleaseImage(&Rbp);
  cvReleaseImage(&BlankImage);
  cvDestroyWindow("RBP Projector");

  return 0;

}


