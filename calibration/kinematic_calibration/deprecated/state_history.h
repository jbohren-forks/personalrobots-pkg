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

#include <vector>

#include "kinematic_calibration/msg_cache.h"

#include "robot_msgs/MechanismState.h"
#include "robot_msgs/PointStamped.h"
#include "image_msgs/Image.h"
#include "kinematic_calibration/CameraCalSample.h"

#include <limits>



namespace kinematic_calibration
{

struct ToleranceElem
{
  unsigned int index ;
  double tol ;
};

class StateHistory : public MsgCache<CameraCalSample>
{
public:
  StateHistory(unsigned int N) : MsgCache<CameraCalSample>(N)  {  }
  ~StateHistory()  {  }

  bool isStable(const ros::Time& start, std::map<std::string, ToleranceElem> joint_map, double cam_tolerance,
                const unsigned int min_samples)
  {
    printf("storage_.size()=%u\n", storage_.size()) ;

    deque<CameraCalSample>::reverse_iterator it ;
    it = storage_.rbegin() ;

    // Is our history empty?
    if (it == storage_.rend())
      return false ;

    printf("newest_stamp: %f\n", it->header.stamp.toSec()) ;
    printf("req_stamp:    %f\n", start.toSec()) ;

    std::vector<double> joint_min(joint_map.size(),  numeric_limits<double>::max()) ;
    std::vector<double> joint_max(joint_map.size(), -numeric_limits<double>::max()) ;

    std::vector<image_msgs::ImagePoint> led_min(storage_.begin()->get_points_size()) ;
    std::vector<image_msgs::ImagePoint> led_max(storage_.begin()->get_points_size()) ;

    for (unsigned int i=0; i<storage_.begin()->get_points_size(); i++)
    {
      led_min[i].x =  numeric_limits<double>::max() ;
      led_min[i].y =  numeric_limits<double>::max() ;
      led_max[i].x = -numeric_limits<double>::max() ;
      led_max[i].y = -numeric_limits<double>::max() ;
    }

    unsigned int sample_count = 0 ;

    std::map<std::string, ToleranceElem>::iterator map_it ;
    while( it < storage_.rend() && it->header.stamp > start)
    {
      // Extract the relevant joint positions
      for (unsigned int i=0; i < it->mech_state.get_joint_states_size(); i++)
      {
        map_it = joint_map.find(it->mech_state.joint_states[i].name) ;
        if (map_it != joint_map.end()) // We found a joint we care about!
        {
          double joint_pos = it->mech_state.joint_states[i].position ;

          // Update joint min
          if (joint_pos < joint_min[map_it->second.index])
            joint_min[map_it->second.index] = joint_pos ;

          // Update joint max
          if (joint_pos > joint_max[map_it->second.index])
            joint_max[map_it->second.index] = joint_pos ;
        }
      }

      for (unsigned int i=0; i < it->get_points_size(); i++)
      {
        if (it->points[i].image_point.x < led_min[i].x)
          led_min[i].x = it->points[i].image_point.x ;
        if (it->points[i].image_point.y < led_min[i].y)
          led_min[i].y = it->points[i].image_point.y ;

        if (it->points[i].image_point.x > led_max[i].x)
          led_max[i].x = it->points[i].image_point.x ;
        if (it->points[i].image_point.y > led_max[i].y)
          led_max[i].y = it->points[i].image_point.y ;
      }


      sample_count++ ;

      if (sample_count >= min_samples)
        break ;

      ++it ;
    }

    printf("Found %u samples\n", sample_count) ;

    printf("Joints:\n") ;
    for (unsigned int i=0; i<joint_min.size(); i++)
    {
      printf("%u) %f -> %f\n", i, joint_min[i], joint_max[i]) ;
    }

    printf("Cameras:\n") ;
    for (unsigned int i=0; i<led_min.size(); i++)
    {
      printf("%02u) %f -> %f\n", i, led_min[i].x, led_max[i].x) ;
      printf("    %f -> %f\n", led_min[i].y, led_max[i].y) ;
    }

    if (sample_count < min_samples)
      return false ;

    for (unsigned int i=0; i<led_min.size(); i++)
    {
      if (led_max[i].x - led_min[i].x > cam_tolerance)
        return false ;
      if (led_max[i].y - led_min[i].y > cam_tolerance)
        return false ;
    }

    map_it = joint_map.begin() ;
    while(map_it != joint_map.end())
    {
      printf("%s->%u\n", map_it->first.c_str(), map_it->second.index ) ;


      double range = joint_max[map_it->second.index] - joint_min[map_it->second.index] ;

      // Check if we're past the allowed tolerance for this joint
      if (range > map_it->second.tol)
        return false ;

      ++map_it ;
    }

    return true ;
  }

  CameraCalSample* getPastSample(unsigned int age)
  {
    if (age > storage_.size())
      return false ;

    return(&storage_[storage_.size()-1-age]) ;


  }  

} ;







}
