#ifndef STEREO_BLOB_TRACKER_RECT2DSTAMPED_H
#define STEREO_BLOB_TRACKER_RECT2DSTAMPED_H

#include <string>
#include <string.h>
#include <vector>
#include "ros/msg.h"
#include "ros/time.h"

#include "rostools/Header.h"
#include "stereo_blob_tracker/Rect2D.h"

namespace stereo_blob_tracker
{

//! \htmlinclude Rect2DStamped.msg.html

class Rect2DStamped : public ros::msg
{
public:
  rostools::Header header;
  stereo_blob_tracker::Rect2D rect;

  Rect2DStamped() : ros::msg()
  {
  }
  Rect2DStamped(const Rect2DStamped &copy) : ros::msg(),
    header(copy.header),
    rect(copy.rect)
  {
    (void)copy;
  }
  Rect2DStamped &operator =(const Rect2DStamped &copy)
  {
    if (this == &copy)
      return *this;
    header = copy.header;
    rect = copy.rect;
    return *this;
  }
  virtual ~Rect2DStamped() 
  {
  }
  inline static std::string __s_get_datatype() { return std::string("stereo_blob_tracker/Rect2DStamped"); }
  inline static std::string __s_get_md5sum() { return std::string("89499fc21647a323f3afb6d0c46b57fa"); }
  inline virtual const std::string __get_datatype() const { return __s_get_datatype(); }
  inline virtual const std::string __get_md5sum() const { return __s_get_md5sum(); }
  inline uint32_t serialization_length()
  {
    unsigned l = 0;
    l += header.serialization_length(); // header
    l += rect.serialization_length(); // rect
    return l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr)
  {
    header.seq++;
    bool __reset_timestamp = header.stamp.is_zero();
    if (__reset_timestamp)
      header.stamp = ros::Time::now();
    write_ptr = header.serialize(write_ptr);
    write_ptr = rect.serialize(write_ptr);
    if (__reset_timestamp)
      header.stamp = ros::Time();
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    read_ptr = header.deserialize(read_ptr);
    read_ptr = rect.deserialize(read_ptr);
    return read_ptr;
  }
};


}

#endif
