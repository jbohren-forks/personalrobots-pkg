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

#ifndef TF_TRANSFORMLISTENER_H
#define TF_TRANSFORMLISTENER_H

#include "std_msgs/PointCloud.h"
#include "tf/tfMessage.h"
#include "tf/tf.h"
#include "ros/node.h"

/// \todo remove backward compatability only
#include "rosTF/TransformArray.h"
// end remove

namespace tf{

/** \brief This class inherits from Transformer and automatically subscribes to ROS transform messages */
class TransformListener : public Transformer { //subscribes to message and automatically stores incoming data
  
private: 
  ros::node& node_;

  /// \todo remove backward compatability only
  //Temporary storage for callbacks(todo check threadsafe? make scoped in call?)
  rosTF::TransformArray tfArrayIn;

public:
  TransformListener(ros::node & rosnode, 
                  bool interpolating = true,
                  int64_t max_cache_time = DEFAULT_CACHE_TIME,
                  int64_t max_extrapolation_distance = DEFAULT_MAX_EXTRAPOLATION_DISTANCE):
    Transformer(interpolating,
                max_cache_time,
                max_extrapolation_distance),
    node_(rosnode)
  {
    //  printf("Constructed rosTF\n");
    node_.subscribe("/tf_message", msg_in_, &TransformListener::subscription_callback, this,100); ///\todo magic number

    /// \todo remove backward compatability only
    node_.subscribe("/TransformArray", tfArrayIn, &TransformListener::receiveArray, this,100);

  };
  
  ~TransformListener()
  {
    node_.unsubscribe("/tf_message");
    /// \todo remove backward compatability only
    node_.unsubscribe("/TransformArray");
  };
  
  
    //Use Transformer interface for Stamped data types
  /** \brief Transform a std_msgs::Vector natively */
    void transformVector(const std::string& target_frame, const std_msgs::Vector3Stamped & vin, std_msgs::Vector3Stamped & vout);  //output to msg or Stamped< >??
  /** \brief Transform a std_msgs::Quaternion natively */
    void transformQuaternion(const std::string& target_frame, const std_msgs::QuaternionStamped & qin, std_msgs::QuaternionStamped & oout);
  /** \brief Transform a std_msgs::PointCloud natively */
    void transformPointCloud(const std::string& target_frame, const std_msgs::PointCloud& pcin, std_msgs::PointCloud& pcout);
    ///\todo Duplicate for time transforming (add target_time and fixed_frame)

    ///\todo move to high precision laser projector class  void projectAndTransformLaserScan(const std_msgs::LaserScan& scan_in, std_msgs::PointCloud& pcout); 


private:
  /// memory space for callback
  tfMessage msg_in_; 
  ///\todo Switch to std_msgs::Transform
  /// Callback function for ros message subscriptoin
  void subscription_callback();

  ///\todo Remove : for backwards compatability only
void receiveArray()
{
  for (unsigned int i = 0; i < tfArrayIn.get_eulers_size(); i++)
  {
    try{
      //      setWithEulers(tfArrayIn.eulers[i].header.frame_id, tfArrayIn.eulers[i].parent, tfArrayIn.eulers[i].x, tfArrayIn.eulers[i].y, tfArrayIn.eulers[i].z, tfArrayIn.eulers[i].yaw, tfArrayIn.eulers[i].pitch, tfArrayIn.eulers[i].roll, tfArrayIn.eulers[i].header.stamp.sec * 1000000000ULL + tfArrayIn.eulers[i].header.stamp.nsec);
      setTransform(Stamped<Transform>(Transform(Quaternion(tfArrayIn.eulers[i].yaw, tfArrayIn.eulers[i].pitch, tfArrayIn.eulers[i].roll), 
                                                    Vector3(tfArrayIn.eulers[i].x, tfArrayIn.eulers[i].y, tfArrayIn.eulers[i].z)), 
                                        tfArrayIn.eulers[i].header.stamp.sec * 1000000000ULL + tfArrayIn.eulers[i].header.stamp.nsec, 
                                        tfArrayIn.eulers[i].header.frame_id ), 
                   tfArrayIn.eulers[i].parent);
    }    
    catch (tf::TransformException &ex)
    {
      std::cerr << "receiveArray: setWithEulers failed with frame_id "<< tfArrayIn.eulers[i].header.frame_id << " parent " << tfArrayIn.eulers[i].parent << std::endl;
      std::cerr<< ex.what();
    };
  }
  //std::cout << "received euler frame: " << tfArrayIn.eulers[i].header.frame_id << " with parent:" << tfArrayIn.eulers[i].parent << "time " << tfArrayIn.eulers[i].header.stamp.sec * 1000000000ULL + eulerIn.header.stamp.nsec << std::endl;
  for (unsigned int i = 0; i < tfArrayIn.get_dhparams_size(); i++)
  {
    std::cerr << "receiveArray: setWithDH failed No longer supported" << std::endl;
  }
  
  for (unsigned int i = 0; i < tfArrayIn.get_quaternions_size(); i++)
  {
    try{
      //    setWithQuaternion(tfArrayIn.quaternions[i].header.frame_id, tfArrayIn.quaternions[i].parent, tfArrayIn.quaternions[i].xt, tfArrayIn.quaternions[i].yt, tfArrayIn.quaternions[i].zt, tfArrayIn.quaternions[i].xr, tfArrayIn.quaternions[i].yr, tfArrayIn.quaternions[i].zr, tfArrayIn.quaternions[i].w, tfArrayIn.quaternions[i].header.stamp.sec * 1000000000ULL + tfArrayIn.quaternions[i].header.stamp.nsec);
      setTransform(Stamped<Transform>(Transform(Quaternion(tfArrayIn.quaternions[i].xr, tfArrayIn.quaternions[i].yr, tfArrayIn.quaternions[i].zr, tfArrayIn.quaternions[i].w), 
                                                    Vector3(tfArrayIn.quaternions[i].xt, tfArrayIn.quaternions[i].yt, tfArrayIn.quaternions[i].zt)), 
                                        tfArrayIn.quaternions[i].header.stamp.sec * 1000000000ULL + tfArrayIn.quaternions[i].header.stamp.nsec, 
                                        tfArrayIn.quaternions[i].header.frame_id ), 
                   tfArrayIn.quaternions[i].parent);
    }    
    catch (tf::TransformException &ex)
    {
      std::cerr << "receiveArray: setWithQuaternion failed with frame_id "<< tfArrayIn.quaternions[i].header.frame_id << " parent " << tfArrayIn.quaternions[i].parent << std::endl;
      std::cerr<< ex.what();
    };
    //  std::cout << "recieved quaternion frame: " << tfArrayIn.quaternions[i].header.frame_id << " with parent:" << tfArrayIn.quaternions[i].parent << std::endl;
  }
  for (unsigned int i = 0; i < tfArrayIn.get_matrices_size(); i++)
  {
    std::cerr << "receiveArray: setWithMatrix failed No longer supported" << std::endl;
  }
};
  
  
};
}

#endif //TF_TRANSFORMLISTENER_H
