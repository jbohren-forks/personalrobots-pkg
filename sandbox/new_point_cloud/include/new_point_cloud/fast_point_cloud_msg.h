#ifndef NEW_POINT_CLOUD_FAST_POINTCLOUD_H
#define NEW_POINT_CLOUD_FAST_POINTCLOUD_H

#include <string>
#include <vector>
#include "ros/message.h"
#include "ros/time.h"

#include "roslib/Header.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "new_point_cloud/point_cloud.h"

namespace new_point_cloud
{

// \htmlinclude PointCloud.msg.html

class FastPointCloud : public ros::Message
{
public:
  typedef boost::shared_ptr<FastPointCloud> Ptr;
  typedef boost::shared_ptr<FastPointCloud const> ConstPtr;

  typedef roslib::Header _header_type;
  typedef PointCloud _point_cloud_type;
  typedef std::vector<sensor_msgs::ChannelFloat32> _channels_type;

  roslib::Header header;
  PointCloud point_cloud;
  std::vector<sensor_msgs::ChannelFloat32> channels;

  FastPointCloud() : ros::Message()
  {
  }
  
  FastPointCloud(const FastPointCloud &copy)
    : ros::Message(),
      header(copy.header),
      point_cloud(copy.point_cloud),
      channels(copy.channels)
  {
  }
  
  FastPointCloud &operator =(const FastPointCloud &copy)
  {
    if (this == &copy)
      return *this;
    header = copy.header;
    point_cloud = copy.point_cloud;
    channels = copy.channels;
    return *this;
  }
  virtual ~FastPointCloud() 
  {
  }
  
  inline static std::string __s_getDataType() { return std::string("new_point_cloud/PointCloud32"); }
  inline static std::string __s_getMD5Sum() { return std::string("640119d7348ebc1e1fc6f0fa453d6a19"); }
  inline static std::string __s_getMessageDefinition()
  {
    return std::string(
    "Header header\n"
    "\n"
    "# 2D structure of the point cloud. If the cloud is unordered,\n"
    "# height is 1 and width is the length of the point cloud.\n"
    "uint32 width\n"
    "uint32 height\n"
    "\n"
    "new_point_cloud/Point32[] points\n"
    "\n"
    "sensor_msgs/ChannelFloat32[] channels\n"
    "\n"
    "================================================================================\n"
    "MSG: roslib/Header\n"
    "#Standard metadata for higher-level flow data types\n"
    "#sequence ID: consecutively increasing ID \n"
    "uint32 seq\n"
    "#Two-integer timestamp that is expressed as:\n"
    "# * stamp.secs: seconds (stamp_secs) since epoch\n"
    "# * stamp.nsecs: nanoseconds since stamp_secs\n"
    "# time-handling sugar is provided by the client library\n"
    "time stamp\n"
    "#Frame this data is associated with\n"
    "# 0: no frame\n"
    "# 1: global frame\n"
    "string frame_id\n"
    "\n"
    "================================================================================\n"
    "MSG: new_point_cloud/Point32\n"
    "float32 x\n"
    "float32 y\n"
    "float32 z\n"
    "uint8   r\n"
    "uint8   g\n"
    "uint8   b\n"
    "uint8   w\n"
    "\n"
    "================================================================================\n"
    "MSG: sensor_msgs/ChannelFloat32\n"
    "string name\n"
    "float32[] vals\n"
    "\n"
    );
  }
  inline virtual const std::string __getDataType() const { return __s_getDataType(); }
  inline virtual const std::string __getMD5Sum() const { return __s_getMD5Sum(); }
  inline virtual const std::string __getMessageDefinition() const { return __s_getMessageDefinition(); }

  uint32_t calc_channels_array_serialization_len() const
  {
    uint32_t l = 0;
    uint32_t channels_size = channels.size();
    for (size_t i = 0; i < channels_size; i++)
      l += channels[i].serializationLength();
    return l;
  }

  inline uint32_t serializationLength() const
  {
    unsigned __l = 0;
    __l += header.serializationLength(); // header
    __l += 4; // width
    __l += 4; // height
    __l += 4 + point_cloud.width*point_cloud.height*sizeof(Point); // points
    __l += 4 + calc_channels_array_serialization_len(); // channels
    return __l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    roslib::Header _ser_header = header;
    bool __reset_seq = (header.seq == 0);
    if (__reset_seq) _ser_header.seq = seq;
    bool __reset_timestamp = header.stamp.is_zero();
    if (__reset_timestamp)
      _ser_header.stamp = ros::Time::now();
    write_ptr = _ser_header.serialize(write_ptr, seq);
    SROS_SERIALIZE_PRIMITIVE(write_ptr, point_cloud.width);
    SROS_SERIALIZE_PRIMITIVE(write_ptr, point_cloud.height);
    uint32_t __points_size = point_cloud.width*point_cloud.height;
    SROS_SERIALIZE_PRIMITIVE(write_ptr, __points_size);
    uint32_t __points_len = __points_size * sizeof(Point);
    memcpy(write_ptr, point_cloud.points.get(), __points_len);
    write_ptr += __points_len;
    uint32_t __channels_size = channels.size();
    SROS_SERIALIZE_PRIMITIVE(write_ptr, __channels_size);
    for (size_t i = 0; i < __channels_size; i++)
      write_ptr = channels[i].serialize(write_ptr, seq);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    read_ptr = header.deserialize(read_ptr);
    uint32_t width, height;
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, width);
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, height);
    uint32_t __points_size;
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, __points_size);
    point_cloud.allocate(width, height);
    uint32_t __points_len = __points_size * sizeof(Point);
    memcpy(point_cloud.points.get(), read_ptr, __points_len);
    uint32_t __channels_size;
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, __channels_size);
    channels.resize(__channels_size);
    for (size_t i = 0; i < __channels_size; i++)
      read_ptr = channels[i].deserialize(read_ptr);
    return read_ptr;
  }
};

typedef boost::shared_ptr<FastPointCloud> FastPointCloudPtr;
typedef boost::shared_ptr<FastPointCloud const> FastPointCloudConstPtr;

}

#endif
