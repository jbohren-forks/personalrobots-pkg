#ifndef STEREO_BLOB_TRACKER_RECT2D_H
#define STEREO_BLOB_TRACKER_RECT2D_H

#include <string>
#include <string.h>
#include <vector>
#include "ros/msg.h"
#include "ros/time.h"

namespace stereo_blob_tracker
{

//! \htmlinclude Rect2D.msg.html

class Rect2D : public ros::msg
{
public:
  double x;
  double y;
  double w;
  double h;

  Rect2D() : ros::msg(),
    x(0),
    y(0),
    w(0),
    h(0)
  {
  }
  Rect2D(const Rect2D &copy) : ros::msg(),
    x(copy.x),
    y(copy.y),
    w(copy.w),
    h(copy.h)
  {
    (void)copy;
  }
  Rect2D &operator =(const Rect2D &copy)
  {
    if (this == &copy)
      return *this;
    x = copy.x;
    y = copy.y;
    w = copy.w;
    h = copy.h;
    return *this;
  }
  virtual ~Rect2D() 
  {
  }
  inline static std::string __s_get_datatype() { return std::string("stereo_blob_tracker/Rect2D"); }
  inline static std::string __s_get_md5sum() { return std::string("bee5273224786e7afdb2689dc145f9d9"); }
  inline virtual const std::string __get_datatype() const { return __s_get_datatype(); }
  inline virtual const std::string __get_md5sum() const { return __s_get_md5sum(); }
  inline uint32_t serialization_length()
  {
    unsigned l = 0;
    l += 8; // x
    l += 8; // y
    l += 8; // w
    l += 8; // h
    return l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr)
  {
    SROS_SERIALIZE_PRIMITIVE(write_ptr, x);
    SROS_SERIALIZE_PRIMITIVE(write_ptr, y);
    SROS_SERIALIZE_PRIMITIVE(write_ptr, w);
    SROS_SERIALIZE_PRIMITIVE(write_ptr, h);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, x);
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, y);
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, w);
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, h);
    return read_ptr;
  }
};


}

#endif
