/*********************************************************************
* This class takes OpenCV images and tracks blobs using OpenCV's cvCamShift function.
* Basically, this is mean shift tracking with some scale space search. 
* Right now the tracking is happening for hue features. 
* In the future, the feature computation should be moved outside so that the 
* caller can choose which features to track.
*
**********************************************************************
*
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Caroline Pantofaru
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

#ifndef BLOB_TRACKER_H
#define BLOB_TRACKER_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <ctype.h>
#include <vector>
#include <string>

// ROS
#include "ros/node.h"
#include <std_msgs/Image.h>

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include "image_utils/cv_bridge.h"


// The blob structure.
// Todo: Add non-rectangle support. 
// Todo: Add 3D support.
struct Blob {
  CvPoint center;
  CvHistogram *hist;
  CvRect window;
  CvConnectedComp comp;
  CvBox2D box;
  bool ready;
};

class BTracker
{
 public:


  IplImage *feat_image_ptr_; // The feature image from which to compute the histogram.
  IplImage *hsv_image_ptr_; // The HSV image.
  IplImage *mask_ptr_; // A mask of the image marking all pixels within a reasonable range (in some feature space, for now S&V).
  IplImage *backproject_ptr_; // The backprojection. The probability of each pixel's hue given the histogram.

  // Constructor
  BTracker();

  // Destructor
  ~BTracker();

  /*!
   * \brief Track the blob in a new frame
   */
  bool processFrame(IplImage*, bool, CvRect, CvRect*);

 private:

  // Blob to track
  Blob blob_;

  // Place to store a temporary histogram. Esp useful for checking if the tracker is lost.
  CvHistogram *temp_hist_;

  // Number of frames that had windows with low similarity to the original histogram. 
  // If this is larger than a threshold, the tracker should declare itself lost.
  int num_bad_frames_;


  /*!
   * Initialize a blob.
   */
  void initBlob(IplImage);


  /*!
   * Display the instructions.
   */
  void instructions();

 
};



#endif
