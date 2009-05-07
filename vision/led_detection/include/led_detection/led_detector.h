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

#ifndef LED_DETECTION_LED_DETECTOR_H_
#define LED_DETECTION_LED_DETECTOR_H_


#include "opencv_latest/CvBridge.h"

#include "image_msgs/Image.h"
#include "image_msgs/CamInfo.h"
#include "kinematic_calibration/ImagePoint.h"
#include "robot_msgs/Pose.h"

namespace led_detection
{

class LedDetector
{
public:
  LedDetector() ;
  ~LedDetector() ;

  /**
   * \brief Very thin wrapper around the IplImage findLed call
   * Find the LED in an image
   * \param image       input: Image in which we need to find the LED
   * \param info        input: Contains camera intrinsics for image
   * \param led_pose    input: Pose of the LED in the camera's frame. Ignored if NULL.
   * \param led_pix     output: Detected location of the LED, in pixel coords
   * \param debug_image output: Image that can be displayed to screen, to help in debugging
   * \return True: LED was found
   *         False: LED was not found
   */
  bool findLed(image_msgs::Image& image, const image_msgs::CamInfo& info,
               const robot_msgs::Pose* led_pose,
               kinematic_calibration::ImagePoint& led_pix, image_msgs::Image& debug_image) ;

  /**
   * \brief Does the 'heavy lifting' and openCV calls for finding the LED in an image.
   * \param image       input: Image in which we need to find the LED
   * \param info        input: Contains camera intrinsics for image
   * \param led_pose    input: Pose of the LED in the camera's frame. Ignored if NULL.
   * \param led_pix     output: Detected location of the LED, in pixel coords
   * \param debug_image output: Image that can be displayed to screen, to help in debugging.
   *                            Must already be allocated to the correct size. Ignored if NULL
   * \return True: LED was found
   *         False: LED was not found
   */
  bool findLed(const IplImage* image, const image_msgs::CamInfo& info,
               const robot_msgs::Pose* led_pose,
               kinematic_calibration::ImagePoint& led_pix, IplImage* debug_image) ;

private:

  image_msgs::CvBridge img_bridge_ ;

} ;


}





#endif // LED_DETECTION_LED_DETECTOR_H_
