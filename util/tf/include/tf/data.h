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

#ifndef TF_DATA_H
#define TF_DATA_H

#include <iostream> //needed before newmat
#include "newmat10/newmat.h"

#include <string>
#include "std_msgs/Vector3Stamped.h"
#include "std_msgs/QuaternionStamped.h"
#include "std_msgs/TransformStamped.h"
#include "LinearMath/btTransform.h"

namespace tf
{
/** \brief The data type which will be cross compatable with std_msgs
 * this will require the associated rosTF package to convert */
template <typename T>
class Stamped{
 public:
  T data_;
  uint64_t stamp_;
  std::string frame_id_;

  Stamped() :stamp_ (0),frame_id_ ("NO_ID"){}; //Default constructor used only for preallocation

  Stamped(const T& input, const uint64_t& timestamp, const std::string & frame_id):
    data_ (input), stamp_ ( timestamp ), frame_id_ (frame_id){ };

  Stamped(const Stamped<T>& input):data_(input.data_), stamp_(input.stamp_), frame_id_(input.frame_id_){};

  Stamped& operator=(const Stamped<T>& input){data_ = input.data_; stamp_ = input.stamp_; frame_id_ = input.frame_id_; return *this;};

  void stripStamp(T & output) { output = data_;};
};


/** \brief convert Vector3 msg to btVector3 */
static inline void Vector3MsgToBt(const std_msgs::Vector3& msg_v, btVector3& bt_v) {bt_v = btVector3(msg_v.x, msg_v.y, msg_v.z);};
/** \brief convert btVector3 to Vector3 msg*/
static inline void Vector3BtToMsg(const btVector3& bt_v, std_msgs::Vector3& msg_v) {msg_v.x = bt_v.x(); msg_v.y = bt_v.y(); msg_v.z = bt_v.z();};

/** \brief convert Vector3Stamped msg to Stamped<btVector3> */
static inline void Vector3StampedMsgToBt(const std_msgs::Vector3Stamped & msg, Stamped<btVector3>& bt) 
{Vector3MsgToBt(msg.vector, bt.data_); bt.stamp_ = msg.header.stamp.to_ull(); bt.frame_id_ = msg.header.frame_id;}; 
/** \brief convert Stamped<btVector3> to Vector3Stamped msg*/
static inline void Vector3StampedBtToMsg(const Stamped<btVector3>& bt, std_msgs::Vector3Stamped & msg)
{Vector3BtToMsg(bt.data_, msg.vector); msg.header.stamp = ros::Time(bt.stamp_); msg.header.frame_id = bt.frame_id_;};


/** \brief convert Quaternion msg to btQuaternion */
static inline void QuaternionMsgToBt(const std_msgs::Quaternion& msg, btQuaternion& bt) {bt = btQuaternion(msg.x, msg.y, msg.z, msg.w);};
/** \brief convert btQuaternion to Quaternion msg*/
static inline void QuaternionBtToMsg(const btQuaternion& bt, std_msgs::Quaternion& msg) {msg.x = bt.x(); msg.y = bt.y(); msg.z = bt.z();  msg.w = bt.w();};

/** \brief convert QuaternionStamped msg to Stamped<btQuaternion> */
static inline void QuaternionStampedMsgToBt(const std_msgs::QuaternionStamped & msg, Stamped<btQuaternion>& bt) 
{QuaternionMsgToBt(msg.quaternion, bt.data_); bt.stamp_ = msg.header.stamp.to_ull(); bt.frame_id_ = msg.header.frame_id;}; 
/** \brief convert Stamped<btQuaternion> to QuaternionStamped msg*/
static inline void QuaternionStampedBtToMsg(const Stamped<btQuaternion>& bt, std_msgs::QuaternionStamped & msg)
{QuaternionBtToMsg(bt.data_, msg.quaternion); msg.header.stamp = ros::Time(bt.stamp_); msg.header.frame_id = bt.frame_id_;};

/** \brief convert Transform msg to btTransform */
static inline void TransformMsgToBt(const std_msgs::Transform& msg, btTransform& bt) 
{bt = btTransform(btQuaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), btVector3(msg.translation.x, msg.translation.y, msg.translation.z));};
/** \brief convert btTransform to Transform msg*/
static inline void TransformBtToMsg(const btTransform& bt, std_msgs::Transform& msg) 
{Vector3BtToMsg(bt.getOrigin(), msg.translation);  QuaternionBtToMsg(bt.getRotation(), msg.rotation);};

/** \brief convert TransformStamped msg to Stamped<btTransform> */
static inline void TransformStampedMsgToBt(const std_msgs::TransformStamped & msg, Stamped<btTransform>& bt) 
{TransformMsgToBt(msg.transform, bt.data_); bt.stamp_ = msg.header.stamp.to_ull(); bt.frame_id_ = msg.header.frame_id;}; 
/** \brief convert Stamped<btTransform> to TransformStamped msg*/
static inline void TransformStampedBtToMsg(const Stamped<btTransform>& bt, std_msgs::TransformStamped & msg)
{TransformBtToMsg(bt.data_, msg.transform); msg.header.stamp = ros::Time(bt.stamp_); msg.header.frame_id = bt.frame_id_;};


/** \brief Convert the transform to a Homogeneous matrix for large operations */
static inline NEWMAT::Matrix transformAsMatrix(const btTransform& bt)
{
  NEWMAT::Matrix outMat(4,4);
  
  double * mat = outMat.Store();
    
  btQuaternion  rotation = bt.getRotation();
  btVector3 origin = bt.getOrigin();

  // math derived from http://www.j3d.org/matrix_faq/matrfaq_latest.html
  double xx      = rotation.x() * rotation.x();
  double xy      = rotation.x() * rotation.y();
  double xz      = rotation.x() * rotation.z();
  double xw      = rotation.x() * rotation.w();
  double yy      = rotation.y() * rotation.y();
  double yz      = rotation.y() * rotation.z();
  double yw      = rotation.y() * rotation.w();
  double zz      = rotation.z() * rotation.z();
  double zw      = rotation.z() * rotation.w();
  mat[0]  = 1 - 2 * ( yy + zz );
  mat[1]  =     2 * ( xy - zw );
  mat[2]  =     2 * ( xz + yw );
  mat[4]  =     2 * ( xy + zw );
  mat[5]  = 1 - 2 * ( xx + zz );
  mat[6]  =     2 * ( yz - xw );
  mat[8]  =     2 * ( xz - yw );
  mat[9]  =     2 * ( yz + xw );
  mat[10] = 1 - 2 * ( xx + yy );
  mat[12]  = mat[13] = mat[14] = 0;
  mat[3] = origin.x();
  mat[7] = origin.y();
  mat[11] = origin.z();
  mat[15] = 1;
    
    
  return outMat;
};
  



}
#endif //TF_DATA_H
