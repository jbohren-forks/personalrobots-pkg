/*********************************************************************
 *
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
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#include <costmap_2d/rate.h>

namespace costmap_2d {
  Rate::Rate(double frequency) : start_(ros::Time::now()), expected_cycle_time_(1.0 / frequency), actual_cycle_time_(0.0) {}

  bool Rate::sleep(){
    ros::Time expected_end = start_ + expected_cycle_time_;

    ros::Time actual_end = ros::Time::now();

    //calculate the time we'll sleep for
    ros::Duration sleep_time = expected_end - actual_end; 

    //set the actual amount of time the loop took in case the user wants to know
    actual_cycle_time_ = actual_end - start_;

    //if we've taken too much time we won't sleep
    if(sleep_time < ros::Duration(0.0)){
      //make sure to reset our start time
      start_ = ros::Time::now();
      return false;
    }

    sleep_time.sleep();

    //make sure to reset our start time
    start_ = ros::Time::now();
    return true;
  }

  void Rate::reset(){
    start_ = ros::Time::now();
  }

  ros::Duration Rate::cycleTime(){
    return actual_cycle_time_;
  }
};
