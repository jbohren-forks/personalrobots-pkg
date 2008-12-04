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
#include "laser_scan/laser_scan.h"
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
  /**@brief Constructor for transform listener
   * \param rosnode A reference to an instance of a ros::node for communication
   * \param interpolating Whether to interpolate or return the closest
   * \param max_cache_time How long to store transform information */
  TransformListener(ros::node & rosnode, 
                    bool interpolating = true,
                    int64_t max_cache_time = DEFAULT_CACHE_TIME):
    Transformer(interpolating,
                max_cache_time),
    node_(rosnode)
  {
    //  printf("Constructed rosTF\n");
    node_.subscribe("/tf_message", msg_in_, &TransformListener::subscription_callback, this,100); ///\todo magic number

    /// \todo remove backward compatability only
    node_.subscribe("/TransformArray", tfArrayIn, &TransformListener::receiveArray, this,100);

  };
  
  ~TransformListener()
  {
    node_.unsubscribe("/tf_message", &TransformListener::subscription_callback, this);
    /// \todo remove backward compatability only
    node_.unsubscribe("/TransformArray", &TransformListener::receiveArray, this);
  };

  /* Methods from transformer unhiding them here */
  using Transformer::transformQuaternion;
  using Transformer::transformVector;
  using Transformer::transformPoint;
  using Transformer::transformPose;

  
  /** \brief Transform a Stamped Quaternion Message into the target frame */
  void transformQuaternion(const std::string& target_frame, const std_msgs::QuaternionStamped& stamped_in, std_msgs::QuaternionStamped& stamped_out);
  /** \brief Transform a Stamped Vector Message into the target frame */
  void transformVector(const std::string& target_frame, const std_msgs::Vector3Stamped& stamped_in, std_msgs::Vector3Stamped& stamped_out);
  /** \brief Transform a Stamped Point Message into the target frame */
  void transformPoint(const std::string& target_frame, const std_msgs::PointStamped& stamped_in, std_msgs::PointStamped& stamped_out);
  /** \brief Transform a Stamped Pose Message into the target frame */
  void transformPose(const std::string& target_frame, const std_msgs::PoseStamped& stamped_in, std_msgs::PoseStamped& stamped_out);

  
  /** \brief Transform a Stamped Quaternion Message into the target frame*/
  void transformQuaternion(const std::string& target_frame, const ros::Time& target_time, 
                           const std_msgs::QuaternionStamped& qin, 
                           const std::string& fixed_frame, std_msgs::QuaternionStamped& qout);
  /** \brief Transform a Stamped Vector Message into the target frame and time */
  void transformVector(const std::string& target_frame, const ros::Time& target_time, 
                       const std_msgs::Vector3Stamped& vin, 
                           const std::string& fixed_frame, std_msgs::Vector3Stamped& vout);
  /** \brief Transform a Stamped Point Message into the target frame and time  */
  void transformPoint(const std::string& target_frame, const ros::Time& target_time, 
                           const std_msgs::PointStamped& pin, 
                           const std::string& fixed_frame, std_msgs::PointStamped& pout);
  /** \brief Transform a Stamped Pose Message into the target frame and time  */
  void transformPose(const std::string& target_frame, const ros::Time& target_time, 
                     const std_msgs::PoseStamped& pin, 
                     const std::string& fixed_frame, std_msgs::PoseStamped& pout);

  
  /** \brief Transform a std_msgs::PointCloud natively */
    void transformPointCloud(const std::string& target_frame, const std_msgs::PointCloud& pcin, std_msgs::PointCloud& pcout);

  /** @brief Transform a std_msgs::PointCloud in space and time */
  void transformPointCloud(const std::string& target_frame, const ros::Time& target_time, 
                           const std_msgs::PointCloud& pcin, 
                           const std::string& fixed_frame, std_msgs::PointCloud& pcout);

  /** \brief Transform a std_msgs::LaserScan into a PointCloud in target frame */
  void transformLaserScanToPointCloud(const std::string& target_frame, std_msgs::PointCloud & cloudOut, const std_msgs::LaserScan & scanIn);


    ///\todo move to high precision laser projector class  void projectAndTransformLaserScan(const std_msgs::LaserScan& scan_in, std_msgs::PointCloud& pcout); 


private:
  /// memory space for callback
  tfMessage msg_in_; 
  ///\todo Switch to std_msgs::Transform
  /// Callback function for ros message subscriptoin
  void subscription_callback();

  /** @brief a helper function to be used for both transfrom pointCloud methods */
  void transformPointCloud(const std::string & target_frame, const Transform& transform, const ros::Time& target_time, const std_msgs::PointCloud& pcin, std_msgs::PointCloud& pcout);


  /** @brief A helper class for projecting laser scans */
  laser_scan::LaserProjection projector_;

  ///\todo Remove : for backwards compatability only
void receiveArray()
{
  for (unsigned int i = 0; i < tfArrayIn.get_eulers_size(); i++)
  {
    try{
      //      setWithEulers(tfArrayIn.eulers[i].header.frame_id, tfArrayIn.eulers[i].parent, tfArrayIn.eulers[i].x, tfArrayIn.eulers[i].y, tfArrayIn.eulers[i].z, tfArrayIn.eulers[i].yaw, tfArrayIn.eulers[i].pitch, tfArrayIn.eulers[i].roll, tfArrayIn.eulers[i].header.stamp.sec * 1000000000ULL + tfArrayIn.eulers[i].header.stamp.nsec);
      setTransform(Stamped<Transform>(Transform(Quaternion(tfArrayIn.eulers[i].yaw, tfArrayIn.eulers[i].pitch, tfArrayIn.eulers[i].roll), 
                                                    Vector3(tfArrayIn.eulers[i].x, tfArrayIn.eulers[i].y, tfArrayIn.eulers[i].z)), 
                                        tfArrayIn.eulers[i].header.stamp.sec * (uint64_t)1000000000ULL + tfArrayIn.eulers[i].header.stamp.nsec, 
                                      tfArrayIn.eulers[i].header.frame_id , tfArrayIn.eulers[i].parent) );
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
                                        tfArrayIn.quaternions[i].header.stamp.sec * (uint64_t)1000000000ULL + tfArrayIn.quaternions[i].header.stamp.nsec, 
                                        tfArrayIn.quaternions[i].header.frame_id , tfArrayIn.quaternions[i].parent) 
                   );
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
