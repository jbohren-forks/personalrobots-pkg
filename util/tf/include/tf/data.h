#ifndef TF_DATA_H
#define TF_DATA_H

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





}
#endif //TF_DATA_H
