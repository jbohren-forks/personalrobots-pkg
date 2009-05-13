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

//! \author Vijay Pradeep

#ifndef STEREO_CHECKERBOARD_DETECTOR_MONO_CHECKERBOARD_HELPER_H_
#define STEREO_CHECKERBOARD_DETECTOR_MONO_CHECKERBOARD_HELPER_H_

#include <vector>

#include "opencv/cv.h"
#include "opencv_latest/CvBridge.h"

#include "robot_msgs/Point.h"

namespace stereo_checkerboard_detector
{

/**
 * A light wrapper around the openCV calls needed in order to extract checkerboards from ROS Images.
 * This class uses the ROS datatypes, but does not have any subscription/advertising or Node code. This
 * should instead be instantiated as part of a node.
 */
class MonoCheckerboardHelper
{

public :
  /**
   * \param w Num checkerboard squares Wide
   * \param h Num checkerboard squares high
   */
  MonoCheckerboardHelper(int w=2, int h=2)
  {
    setSize(w,h) ;
    search_win_size_ = cvSize(5,5) ;            //!< \todo Make this settable
  }

  /**
   * Sets the size of the checkerboard we're looking for
   * \param w Num checkerboard squares Wide
   * \param h Num checkerboard squares high
   */
  void setSize(int w, int h)
  {
    board_size_ = cvSize(w,h) ;
  }

  /**
   * Finding checkerboard corner points in a ROS image message from a single camera
   * \param image The ROS image from which we need to get images
   * \param corners Stores the calculated corners in a vector of ROS Point messages. In the form (x,y,0).
   * \return True if found.  False otherwise
   * \todo make image a 'const reference' once openCV calls are const-ified
   */
  bool getCorners(image_msgs::Image& image, std::vector<robot_msgs::Point>& corners) ;

  /**
   * Finding checkerboard corner points in an IPL image from a single camera
   * \param image The IPL image from which we need to get images
   * \param corners Stores the calculated corners
   * \return True if found. False otherwise
   */
  bool getCorners(const IplImage* image, std::vector<CvPoint2D32f>& corners) ;

private :
  CvSize board_size_ ;                   //!< Size of the checkboard
  CvSize search_win_size_ ;             //!< Size of search window for subpixel corner finder
  image_msgs::CvBridge img_bridge_ ;    //!< Converts ROS image messages into IPL images
} ;


}



#endif // STEREO_CHECKERBOARD_DETECTOR_MONO_CHECKERBOARD_HELPER_H_
