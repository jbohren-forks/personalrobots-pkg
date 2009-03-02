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
// Modified by Gary Bradski 2/26/09 to allow for variable random pattern

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
	int alt2_on1_off0;   				//Turn screen flashing to alternate (2), always on (1) or off (0)
	int _offset, _range, _block_size;	//Add offset to random pattern, pattern range, square size of pattern
	bool _ramp;							//If true, add a ramp of decreasing brightness from top to bottom
	float ffactor;						//Speed up factor for alternation frequency
	
    LightProjector() : ros::Node("light_projector"), alt2_on1_off0(2), _offset(-180), _range(400), _block_size(4), _ramp(false), ffactor(1.0)
    {
      param("~frequency", frequency, 0.5);
      cout << "frequency = " << frequency << endl;
      advertise<std_msgs::UInt8>("projector_status", 10);
    }
    // MAKE RANDOM PATTERN IMAGE:
    // rbp			-- random block pattern image
    // offset		-- Add this possibly negative value to the number, clip [0,255]
    // range		-- Vary numbers this much, clip [0,255]
    // blocks_size	-- How big the random squares should be
    // ramp			-- if on, linear ramp from top (bright) to bottom (dimmer)
    void grayscale_rbp(IplImage *rbp, int offset = 0, int range = 128, int block_size = BLOCK_SIZE, bool ramp = false){
		 int w = rbp->width;
		 int h = rbp->height;
		 int ws = rbp->widthStep;
		 unsigned char *drbp = ((unsigned char *)(rbp->imageData));

		 int i,j,x,y, rn;
		 int bs = block_size;
		 float ramp_down_step = 0.5*(float)range/(float)h;
		 int ramp_down = 0;

		 //Step accross image
		for(y=0; y<h; y+=bs){
		for(x=0; x<w; x+=bs){
			 //	rn = (rand()%2)*128;// 256;
			 	rn = (rand()%range)+offset;
			 	if(ramp) {
			 		ramp_down = (int)(ramp_down_step*(float)y);
			 		rn -= ramp_down;
				}
				if(rn < 0) rn = 0;
				if(rn > 255) rn = 255;
			 		
			 	//Do a random block
				for(j=0; j<bs; j++){
				for(i=0; i<bs; i++){
						drbp[(y+j)*ws+x+i] = rn;
				}
				} 
		}
		}
    }
    
    bool spin()
    {
      int key = 'a';
      while (ok())
	{  
	  key = cvWaitKey(10)&0xFF;	
	  if(key == 27 ) //ESC
	     break;
	  switch(key)
	  {
	   case 'h':
			cout << "\nNode to flip projector pattern on and off\n" << endl;
			cout << "  a,1,0  a,2:Alternate projector pattern, 1:On, 0:Off" << endl;
			cout << "  o,O    offset down, up" << endl;
			cout << "  r,R    range down, up" << endl;
			cout << "  b,B    block_size down, up" << endl;
			cout << "  d      dim bottom of image toggle" << endl;
			cout << "--------\n" << endl;
			break;
	  	case '0': //Keep projector off
	  		alt2_on1_off0 = 0;
	  		ffactor = 40.0;
	  		break;
	  	case '1': //Keep projector on
	  		alt2_on1_off0 = 1;
	  		ffactor = 40.0;
	  		break;
	  	case 'a': //Alternate the projector
	  	case '2':
	  		alt2_on1_off0 = 2;
	  		ffactor = 1.0;
	  		break;
//void grayscale_rbp(IplImage *rbp, int offset = 0, int range = 128, int block_size = BLOCK_SIZE, bool ramp = false){
		case 'o':
			_offset -= 10;
			if(_offset < -255) _offset = -255;
			printf("New Pattern: offset(%d), range(%d), block_size(%d), ramp(%d)\n",_offset,_range,_block_size,_ramp);
			grayscale_rbp(Rbp, _offset, _range, _block_size, _ramp);
			break;
		case 'O':
			_offset += 10;
			if(_offset > 255) _offset = 255;
			printf("New Pattern: offset(%d), range(%d), block_size(%d), ramp(%d)\n",_offset,_range,_block_size,_ramp);
			grayscale_rbp(Rbp, _offset, _range, _block_size, _ramp);
			break;
		case 'r':
			_range -= 10;
			if(_range < 10) _range = 10;
			printf("New Pattern: offset(%d), range(%d), block_size(%d), ramp(%d)\n",_offset,_range,_block_size,_ramp);
			grayscale_rbp(Rbp, _offset, _range, _block_size, _ramp);
			break;
		case 'R':
			_range += 10;
			if(_range > 255) _range = 400;
			printf("New Pattern: offset(%d), range(%d), block_size(%d), ramp(%d)\n",_offset,_range,_block_size,_ramp);
			grayscale_rbp(Rbp, _offset, _range, _block_size, _ramp);
			break;
		case 'b':
			_block_size -= 2;
			if(_block_size < 0) _block_size = 2;
			printf("New Pattern: offset(%d), range(%d), block_size(%d), ramp(%d)\n",_offset,_range,_block_size,_ramp);
			grayscale_rbp(Rbp, _offset, _range, _block_size, _ramp);
			break;
		case 'B':
			_block_size += 2;
			if(_block_size > 500) _block_size = 60;
			printf("New Pattern: offset(%d), range(%d), block_size(%d), ramp(%d)\n",_offset,_range,_block_size,_ramp);
			grayscale_rbp(Rbp, _offset, _range, _block_size, _ramp);
			break;
		case 'd':
			_ramp = !_ramp;
			printf("New Pattern: offset(%d), range(%d), block_size(%d), ramp(%d)\n",_offset,_range,_block_size,_ramp);
			grayscale_rbp(Rbp, _offset, _range, _block_size, _ramp);
			break;
			
	  }

	  
	  if(alt2_on1_off0 == 2)
	  {  
		  if(!BlankScreen){
			 cvShowImage("RBP Projector", Rbp); //This won't show until function exit
		  }
		  else{
			 cvShowImage("RBP Projector", BlankImage); //This won't show until function exit
		  }
		}
		else if(alt2_on1_off0 == 1) {
			cvShowImage("RBP Projector", Rbp);
			BlankScreen = 1;
		}
		else { //alt2_on1_off0 == 0
			cvShowImage("RBP Projector", BlankImage);
			BlankScreen = 0;
		}
	  
	  usleep(1000000.0/(frequency*ffactor));	      
	  if(alt2_on1_off0 == 2) BlankScreen = !BlankScreen;
	  status_out.data = (int)BlankScreen;
	  publish("projector_status", status_out);
//	  printf("Speckle(1) Blank(0) = %d \n",(int)(BlankScreen));	  
	}
      return true;
    }
    
private:
  double frequency;
  
};

int main(int argc, char **argv){
	for(int i = 0; i<argc; ++i)
		cout << "(" << i << "): " << argv[i] << endl;
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

  LP.grayscale_rbp(Rbp); //Set initial image

  srand ( time(NULL) );

  LP.spin();

  cvReleaseImage(&Rbp);
  cvReleaseImage(&BlankImage);
  cvDestroyWindow("RBP Projector");

  return 0;

}


