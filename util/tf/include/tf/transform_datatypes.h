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

#ifndef TF_TRANSFORM_DATATYPES_H
#define TF_TRANSFORM_DATATYPES_H

#include <iostream> //needed before newmat
#include "newmat10/newmat.h"

#include <string>
#include "std_msgs/PointStamped.h"
#include "std_msgs/Vector3Stamped.h"
#include "std_msgs/QuaternionStamped.h"
#include "std_msgs/TransformStamped.h"
#include "std_msgs/PoseStamped.h"
#include "LinearMath/btTransform.h"
#include "ros/time.h"

namespace tf
{
/** \brief A representaton of orientation or rotation depending on context*/
typedef btQuaternion Quaternion; ///\todo differentiate?
/** \brief A representation of a translation */
typedef btVector3 Vector3;
/** \brief  The transform library representation of a point(Position)*/
typedef btVector3 Point;
/** \brief A representation of a translation and rotation */
typedef btTransform Transform;
/** \brief A representation of pose (A position and orientation)*/
typedef btTransform Pose;

/** \brief The data type which will be cross compatable with std_msgs
 * this will require the associated rosTF package to convert */
template <typename T>
class Stamped : public T{
 public:
  ros::Time stamp_;
  std::string frame_id_;
  std::string parent_id_; ///only used for transform

  Stamped() :stamp_ (0ULL),frame_id_ ("NO_ID"), parent_id_("NOT A TRANSFORM"){}; //Default constructor used only for preallocation

  Stamped(const T& input, const ros::Time& timestamp, const std::string & frame_id, const std::string & parent_id = "NOT A TRANSFORM"):
    T (input), stamp_ ( timestamp ), frame_id_ (frame_id), parent_id_(parent_id){ };

//Stamped(const Stamped<T>& input):data_(input.data_), stamp_(input.stamp_), frame_id_(input.frame_id_), parent_id_(input.parent_id_){};

//Stamped& operator=(const Stamped<T>& input){data_ = input.data_; stamp_ = input.stamp_; frame_id_ = input.frame_id_;
//  parent_id_ = input.parent_id_; return *this;};

  void setData(const T& input){*static_cast<T*>(this) = input;};
  //  void stripStamp(T & output) { output = data_;}; //just down cast it
};


/** \brief convert Quaternion msg to Quaternion */
static inline void QuaternionMsgToTF(const std_msgs::Quaternion& msg, Quaternion& bt) {bt = Quaternion(msg.x, msg.y, msg.z, msg.w);};
/** \brief convert Quaternion to Quaternion msg*/
static inline void QuaternionTFToMsg(const Quaternion& bt, std_msgs::Quaternion& msg) {msg.x = bt.x(); msg.y = bt.y(); msg.z = bt.z();  msg.w = bt.w();};

/** \brief convert QuaternionStamped msg to Stamped<Quaternion> */
static inline void QuaternionStampedMsgToTF(const std_msgs::QuaternionStamped & msg, Stamped<Quaternion>& bt)
{QuaternionMsgToTF(msg.quaternion, bt); bt.stamp_ = msg.header.stamp.to_ull(); bt.frame_id_ = msg.header.frame_id;};
/** \brief convert Stamped<Quaternion> to QuaternionStamped msg*/
static inline void QuaternionStampedTFToMsg(const Stamped<Quaternion>& bt, std_msgs::QuaternionStamped & msg)
{QuaternionTFToMsg(bt, msg.quaternion); msg.header.stamp = bt.stamp_; msg.header.frame_id = bt.frame_id_;};

/** \brief convert Vector3 msg to Vector3 */
static inline void Vector3MsgToTF(const std_msgs::Vector3& msg_v, Vector3& bt_v) {bt_v = Vector3(msg_v.x, msg_v.y, msg_v.z);};
/** \brief convert Vector3 to Vector3 msg*/
static inline void Vector3TFToMsg(const Vector3& bt_v, std_msgs::Vector3& msg_v) {msg_v.x = bt_v.x(); msg_v.y = bt_v.y(); msg_v.z = bt_v.z();};

/** \brief convert Vector3Stamped msg to Stamped<Vector3> */
static inline void Vector3StampedMsgToTF(const std_msgs::Vector3Stamped & msg, Stamped<Vector3>& bt)
{Vector3MsgToTF(msg.vector, bt); bt.stamp_ = msg.header.stamp.to_ull(); bt.frame_id_ = msg.header.frame_id;};
/** \brief convert Stamped<Vector3> to Vector3Stamped msg*/
static inline void Vector3StampedTFToMsg(const Stamped<Vector3>& bt, std_msgs::Vector3Stamped & msg)
{Vector3TFToMsg(bt, msg.vector); msg.header.stamp = bt.stamp_; msg.header.frame_id = bt.frame_id_;};


/** \brief convert Point msg to Point */
static inline void PointMsgToTF(const std_msgs::Point& msg_v, Point& bt_v) {bt_v = Vector3(msg_v.x, msg_v.y, msg_v.z);};
/** \brief convert Point to Point msg*/
static inline void PointTFToMsg(const Point& bt_v, std_msgs::Point& msg_v) {msg_v.x = bt_v.x(); msg_v.y = bt_v.y(); msg_v.z = bt_v.z();};

/** \brief convert PointStamped msg to Stamped<Point> */
static inline void PointStampedMsgToTF(const std_msgs::PointStamped & msg, Stamped<Point>& bt)
{PointMsgToTF(msg.point, bt); bt.stamp_ = msg.header.stamp.to_ull(); bt.frame_id_ = msg.header.frame_id;};
/** \brief convert Stamped<Point> to PointStamped msg*/
static inline void PointStampedTFToMsg(const Stamped<Point>& bt, std_msgs::PointStamped & msg)
{PointTFToMsg(bt, msg.point); msg.header.stamp = bt.stamp_; msg.header.frame_id = bt.frame_id_;};


/** \brief convert Transform msg to Transform */
static inline void TransformMsgToTF(const std_msgs::Transform& msg, Transform& bt)
{bt = Transform(Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), Vector3(msg.translation.x, msg.translation.y, msg.translation.z));};
/** \brief convert Transform to Transform msg*/
static inline void TransformTFToMsg(const Transform& bt, std_msgs::Transform& msg)
{Vector3TFToMsg(bt.getOrigin(), msg.translation);  QuaternionTFToMsg(bt.getRotation(), msg.rotation);};

/** \brief convert TransformStamped msg to Stamped<Transform> */
static inline void TransformStampedMsgToTF(const std_msgs::TransformStamped & msg, Stamped<Transform>& bt)
{TransformMsgToTF(msg.transform, bt); bt.stamp_ = msg.header.stamp.to_ull(); bt.frame_id_ = msg.header.frame_id; bt.parent_id_ = msg.parent_id;};
/** \brief convert Stamped<Transform> to TransformStamped msg*/
static inline void TransformStampedTFToMsg(const Stamped<Transform>& bt, std_msgs::TransformStamped & msg)
{TransformTFToMsg(bt, msg.transform); msg.header.stamp = bt.stamp_; msg.header.frame_id = bt.frame_id_; msg.parent_id = bt.parent_id_;};

/** \brief convert Pose msg to Pose */
static inline void PoseMsgToTF(const std_msgs::Pose& msg, Pose& bt)
{bt = Transform(Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), Vector3(msg.position.x, msg.position.y, msg.position.z));};
/** \brief convert Pose to Pose msg*/
static inline void PoseTFToMsg(const Pose& bt, std_msgs::Pose& msg)
{PointTFToMsg(bt.getOrigin(), msg.position);  QuaternionTFToMsg(bt.getRotation(), msg.orientation);};

/** \brief convert PoseStamped msg to Stamped<Pose> */
static inline void PoseStampedMsgToTF(const std_msgs::PoseStamped & msg, Stamped<Pose>& bt)
{PoseMsgToTF(msg.pose, bt); bt.stamp_ = msg.header.stamp.to_ull(); bt.frame_id_ = msg.header.frame_id;};
/** \brief convert Stamped<Pose> to PoseStamped msg*/
static inline void PoseStampedTFToMsg(const Stamped<Pose>& bt, std_msgs::PoseStamped & msg)
{PoseTFToMsg(bt, msg.pose); msg.header.stamp = bt.stamp_; msg.header.frame_id = bt.frame_id_;};


/** \brief Convert the transform to a Homogeneous matrix for large operations */
static inline NEWMAT::Matrix transformAsMatrix(const Transform& bt)
{
  NEWMAT::Matrix outMat(4,4);

  double * mat = outMat.Store();

  Quaternion  rotation = bt.getRotation();
  Vector3 origin = bt.getOrigin();

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
#endif //TF_TRANSFORM_DATATYPES_H
