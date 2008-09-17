/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

#ifndef TF_TRANSFORMCLIENT_H
#define TF_TRANSFORMCLIENT_H

#include "std_msgs/PointCloud.h"
#include "tf/tfMessage.h"
#include "tf/tf.h"
#include "ros/node.h"

namespace tf{

class TransformClient : public Transformer { //subscribes to message and automatically stores incoming data
  
private: 
  ros::node& node_;

public:
  TransformClient(ros::node & rosnode, 
                  bool interpolating = true,
                  unsigned long long max_cache_time = DEFAULT_CACHE_TIME,
                  unsigned long long max_extrapolation_distance = DEFAULT_MAX_EXTRAPOLATION_DISTANCE):
    Transformer(interpolating,
                max_cache_time,
                max_extrapolation_distance),
    node_(rosnode)
  {
    //  printf("Constructed rosTF\n");
    node_.subscribe("/tfMessage", msg_in_, &TransformClient::subscription_callback, this,100); ///\todo magic number
  };
  
    //Use Transformer interface for Stamped data types

    void transformVector(const std::string& target_frame, const std_msgs::Vector3Stamped & vin, std_msgs::Vector3Stamped & vout);  //output to msg or Stamped< >??
    void transformQuaternion(const std::string& target_frame, const std_msgs::QuaternionStamped & qin, std_msgs::QuaternionStamped & oout);
    void transformPointCloud(const std::string& target_frame, const std_msgs::PointCloud& pcin, std_msgs::PointCloud& pcout);
    //Duplicate for time transforming (add target_time and fixed_frame)

    ///\todo move to high precision laser projector class  void projectAndTransformLaserScan(const std_msgs::LaserScan& scan_in, std_msgs::PointCloud& pcout); 


private:
  tfMessage msg_in_;
  void subscription_callback();
  
  
};
}

#endif //TF_TRANSFORMCLIENT_H
