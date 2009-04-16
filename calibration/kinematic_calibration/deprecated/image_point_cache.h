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

#ifndef KINEMATIC_CALIBRATION_IMAGE_POINT_CACHE_H_
#define KINEMATIC_CALIBRATION_IMAGE_POINT_CACHE_H_


#include "kinematic_calibration/msg_cache.h"

#include "image_msgs/ImagePointStamped.h"
#include "kinematic_calibration/Interval.h"

#include "tinyxml/tinyxml.h"

namespace kinematic_calibration
{


class ImagePointCache : public MsgCache<image_msgs::ImagePointStamped>
{
public:
  ImagePointCache(unsigned int N=1) : MsgCache<image_msgs::ImagePointStamped>(N)
  {

  }

  ~ImagePointCache()
  {

  }

  bool isStable(Interval interval, unsigned int min_samples, double pos_tolerance)
  {

    std::deque<image_msgs::ImagePointStamped>::iterator it ;
    it = storage_.begin() ;

    double led_min_x, led_min_y ;
    double led_max_x, led_max_y ;
    led_min_x =  numeric_limits<double>::max() ;
    led_min_x =  numeric_limits<double>::max() ;
    led_max_y = -numeric_limits<double>::max() ;
    led_max_y = -numeric_limits<double>::max() ;

    // Walk along list to just inside beginning of interval
    while( it < storage_.end() && it->header.stamp < interval.start)
      ++it ;
    // Perform calcs on elems inside interval
    unsigned int sample_count = 0 ;
    while( it < storage_.end() && it->header.stamp < interval.end)
    {
      if (it->image_point.x < led_min_x)
        led_min_x = it->image_point.x ;
      if (it->image_point.y < led_min_y)
        led_min_y = it->image_point.y ;

      if (it->image_point.x > led_max_x)
        led_max_x = it->image_point.x ;
      if (it->image_point.y > led_max_y)
        led_max_y = it->image_point.y ;

      sample_count++ ;
    }

    printf("LED stats:\n") ;
    printf("%f -> %f\n", led_min_x, led_max_x) ;
    printf("%f -> %f\n", led_min_y, led_max_y) ;

    if (led_max_x - led_min_x > pos_tolerance)
      return false ;

    if (led_max_y - led_min_y > pos_tolerance)
      return false ;

    return true ;
  }


} ;




}


#endif // KINEMATIC_CALIBRATION_IMAGE_POINT_CACHE_H_
