#ifndef TF_DATA_H
#define TF_DATA_H

#include <string>

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

  void stripStamp(T & output) { output = data_;};
};

}
#endif //TF_DATA_H
