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

//
// Simple stereo acquisition test of the dcam driver
// Set up for Videre stereo camera
//

#include "dcam.h"
#include "stereocam.h"
#include "imwin.h"
#include "im3Dwin.h"

double get_ms();		// for timing
double t1,t2,t3,t4;

int 
main(int argc, char **argv)
{
  // initialize the system
  printf("Initializing system\n");
  dcam::init();

  // get cameras
  int nc = dcam::numCameras();
  printf("Number of cameras: %d\n", nc);

  // print out GUIDs, vendor, model
  for (int i=0; i<nc; i++)
    printf("Camera %d GUID: %llx  Vendor: %s  Model: %s\n", 
	   i, dcam::getGuid(i), dcam::getVendor(i), dcam::getModel(i));


  // open the first camera, and start streaming video
  cam::StereoDcam *dev;		// camera object
  if (nc > 0)
    {
      // initializing camera
      printf("Initializing camera 0\n");
      dev = new cam::StereoDcam(dcam::getGuid(0));
      
      // find modes
      dc1394video_modes_t *modes;
      modes = dev->getModes();
      int nm = modes->num;
      printf("Found %d video modes\n", nm);
      if (nm > 0)
	for (int i=0; i<nm; i++)
	  printf("  Video mode: %d = %s\n", modes->modes[i], 
		 dcam::getModeString(modes->modes[i]));

      printf("Setting format to 640x480\n");
      dev->setFormat(DC1394_VIDEO_MODE_640x480_YUV422);
      dev->setFormat(VIDERE_STEREO_640x480);
      //      dev->setFormat(DC1394_VIDEO_MODE_320x240_YUV422); // should throw an error
      
      // check STOC mode
      printf("Checking for STOC...\n");
      if (dev->isSTOC)
	{
	  printf("  ...Setting STOC mode to PROC_MODE_NONE\n");
	  dev->setProcMode(PROC_MODE_NONE);
	}
      else
	printf("  ...not a STOC\n");

      // set up windows for display
      imWindow *win1 = new imWindow(640, 480, "Left image");
      win1->show(); // FLTK barfs if win1 isn't shown before win2 is created
      imWindow *win2 = new imWindow(640, 480, "right image");
      win2->show();
      fltk_check();		// process fltk events

      // set up OpenGL window
      im3DWindow *w3d = new im3DWindow(0,0,640,480,"3D Display");
      w3d->show();

      // start transmitting
      printf("Starting transmission\n");
      dev->start();
      uint64_t ftime = 0;

      // set companding
      //      dev->setCompanding(true);	// must be set after starting camera
      dev->setUniqueThresh(12);
      dev->setTextureThresh(10);

      for (int i=0; i<200; i++)
	{
	  bool res = dev->getImage(500);
	  if (res)
	    {
	      if (ftime == 0)
		ftime = dev->camIm->im_time;
	      printf("Got frame %d at time +%llu us\n", i, dev->camIm->im_time - ftime);
	      ftime = dev->camIm->im_time;


	      t1 = get_ms();
	      dev->doRectify();	// rectify if it's not done by the STOC
	      t2 = get_ms();
	      dev->doDisparity(); // perform stereo processing, if not done by STOC
	      t3 = get_ms();
	      dev->doCalcPts();	// get 3D points
	      t4 = get_ms();

	      printf("Timing - Rect %d ms, Disparity %d ms, Pts %d ms\n", (int)(t2-t1), (int)(t3-t2), (int)(t4-t3));

	      // try displaying 3D points
	      w3d->DisplayImage(dev->stIm);
	      fltk_check();
	      printf("Num points: %d\n", dev->stIm->numPts);

	      // left window display, try rect, then raw
	      if (dev->stIm->imLeft->imRectColorType != COLOR_CODING_NONE)
		{
		  win1->DisplayImage((unsigned char *)dev->stIm->imLeft->imRectColor, 640, 480, 640, RGB24);
		  win1->label("Left rectified image");
		}
	      else if (dev->stIm->imLeft->imRectType != COLOR_CODING_NONE)
		{
		  win1->DisplayImage((unsigned char *)dev->stIm->imLeft->imRect, 640, 480, 640);
		  win1->label("Left rectified image");
		}
	      else if (dev->stIm->imLeft->imColorType != COLOR_CODING_NONE)
		{
		  win1->DisplayImage((unsigned char *)dev->stIm->imLeft->imColor, 640, 480, 640, RGB24);
		  win1->label("Left original image");
		}
	      else if (dev->stIm->imLeft->imType != COLOR_CODING_NONE)
		{
		  win1->DisplayImage((unsigned char *)dev->stIm->imLeft->im, 640, 480, 640);
		  win1->label("Left original image");
		}

	      // right window display, try disp, then rect, then raw
	      if (dev->stIm->hasDisparity)
		{
		  win2->DisplayImage((unsigned char *)dev->stIm->imDisp, 640, 480, 640, DISPARITY, 64*16);
		  win2->label("Disparity image");
		}
	      else if (dev->stIm->imRight->imRectType != COLOR_CODING_NONE)
		{
		  win2->DisplayImage((unsigned char *)dev->stIm->imRight->imRect, 640, 480, 640);
		  win2->label("Right rectified image");
		}
	      else if (dev->stIm->imRight->imColorType != COLOR_CODING_NONE)
		{
		  win2->DisplayImage((unsigned char *)dev->stIm->imRight->imColor, 640, 480, 640, RGB24);
		  win2->label("Right original image");
		}
	      else if (dev->stIm->imRight->imType != COLOR_CODING_NONE)
		{
		  win2->DisplayImage((unsigned char *)dev->stIm->imRight->im, 640, 480, 640);
		  win2->label("Right original image");
		}
	    }
	}

      // freeing camera
      dev->stop();
      free(dev);
    }

  return 0;
}


#ifdef WIN32
LARGE_INTEGER freq={0,0};
double get_ms()
{
  double a;
  LARGE_INTEGER perf;
  //  if (freq.LowPart == 0 && freq.HighPart == 0)
  if (freq.QuadPart == 0)
    QueryPerformanceFrequency(&freq);
  QueryPerformanceCounter(&perf);
  a = (double)perf.QuadPart / (double)freq.QuadPart;
  return 1000*a;
}
#else
double get_ms()
{
  struct timeval t0;
  gettimeofday(&t0,NULL);
  double ret = t0.tv_sec * 1000.0;
  ret += ((double)t0.tv_usec)*0.001;
  return ret;
}
#endif
