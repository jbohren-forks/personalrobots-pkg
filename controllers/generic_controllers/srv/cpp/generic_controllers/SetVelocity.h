#ifndef SRV_GENERIC_CONTROLLERS_SETVELOCITY_H
#define SRV_GENERIC_CONTROLLERS_SETVELOCITY_H

#include <string>
#include <vector>

namespace generic_controllers
{

namespace SetVelocity
{

//! \htmlinclude request.msg.html

class request : public ros::msg
{
public:
  double velocity;

  request() : ros::msg(),
    velocity(0)
  {
  }
  request(const request &copy) : ros::msg(),
    velocity(copy.velocity)
  {
  }
  request &operator =(const request &copy)
  {
    if (this == &copy)
      return *this;
    velocity = copy.velocity;
    return *this;
  }
  virtual ~request() 
  {
  }
  inline static std::string __s_get_datatype() { return std::string("generic_controllers/request"); }
  inline static std::string __s_get_md5sum() { return std::string(""); }
  inline virtual const std::string __get_datatype() const { return __s_get_datatype(); }
  inline virtual const std::string __get_md5sum() const { return __s_get_md5sum(); }
  inline static std::string __s_get_server_md5sum() {     return std::string("9e7e442fdb347c05b81d95bb5f422f95"); }
  inline virtual const std::string __get_server_md5sum() const { return __s_get_server_md5sum(); }
  inline uint32_t serialization_length()
  {
    unsigned l = 0;
    l += 8; // velocity
    return l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr)
  {
    SROS_SERIALIZE_PRIMITIVE(write_ptr, velocity);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, velocity);
    return read_ptr;
  }
};

//! \htmlinclude response.msg.html

class response : public ros::msg
{
public:
  double velocity;

  response() : ros::msg(),
    velocity(0)
  {
  }
  response(const response &copy) : ros::msg(),
    velocity(copy.velocity)
  {
  }
  response &operator =(const response &copy)
  {
    if (this == &copy)
      return *this;
    velocity = copy.velocity;
    return *this;
  }
  virtual ~response() 
  {
  }
  inline static std::string __s_get_datatype() { return std::string("generic_controllers/response"); }
  inline static std::string __s_get_md5sum() { return std::string(""); }
  inline virtual const std::string __get_datatype() const { return __s_get_datatype(); }
  inline virtual const std::string __get_md5sum() const { return __s_get_md5sum(); }
  inline static std::string __s_get_server_md5sum() {     return std::string("9e7e442fdb347c05b81d95bb5f422f95"); }
  inline virtual const std::string __get_server_md5sum() const { return __s_get_server_md5sum(); }
  inline uint32_t serialization_length()
  {
    unsigned l = 0;
    l += 8; // velocity
    return l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr)
  {
    SROS_SERIALIZE_PRIMITIVE(write_ptr, velocity);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, velocity);
    return read_ptr;
  }
};

}

}

#endif
