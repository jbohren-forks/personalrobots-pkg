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
// Simple acquisition test of the dcam driver
// Set up for Videre stereo camera
//

#include "dcam.h"

using namespace dcam;

int 
main(int argc, char **argv)
{
  // initialize the system
  printf("Initializing system\n");
  dcam::init();

  // get cameras
  int nc = numCameras();
  printf("Number of cameras: %d\n", nc);

  // print out GUIDs, vendor, model
  for (int i=0; i<nc; i++)
    printf("Camera %d GUID: %llx  Vendor: %s  Model: %s\n", 
	   i, getGuid(i), getVendor(i), getModel(i));


  // open the first camera, and start streaming video
  Dcam *cam;		// camera object
  if (nc > 0)
    {
      // initializing camera
      printf("Initializing camera 0\n");
      cam = new Dcam(getGuid(0));
      
      // find modes
      dc1394video_modes_t *modes;
      modes = cam->getModes();
      int nm = modes->num;
      printf("Found %d video modes\n", nm);
      if (nm > 0)
	for (int i=0; i<nm; i++)
	  printf("  Video mode: %d = %s\n", modes->modes[i], 
		 getModeString(modes->modes[i]));

      printf("Setting format\n");
      cam->setFormat(DC1394_VIDEO_MODE_640x480_YUV422);
      cam->setFormat(VIDERE_STEREO_640x480);
      //      cam->setFormat(DC1394_VIDEO_MODE_320x240_YUV422); // should throw an error
      
      // start transmitting
      printf("Starting transmission\n");
      cam->start();

      for (int i=0; i<40; i++)
	{
	  bool res = cam->getImage(500);
	  if (res)
	    printf("Got frame %d at time %llu\n", i, cam->camIm->im_time/1000);
	}

      // freeing camera
      cam->stop();
      free(cam);
    }

  return 0;
}
