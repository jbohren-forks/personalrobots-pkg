#ifndef TF_DATA_H
#define TF_DATA_H

#include <string>
#include "std_msgs/Vector3.h"
#include "LinearMath/btVector3.h"

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


static inline void Vector3MsgToBt(const std_msgs::Vector3& msg_v, btVector3& bt_v) {bt_v = btVector3(msg_v.x, msg_v.y, msg_v.z);};
static inline void Vector3BtToMsg(const btVector3& bt_v, std_msgs::Vector3& msg_v) {msg_v.x = bt_v.x(); msg_v.y = bt_v.y(); msg_v.z = bt_v.z();};



}
#endif //TF_DATA_H
