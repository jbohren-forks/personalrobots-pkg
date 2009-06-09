#include <image_publisher/image_publisher.h>

#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

ImagePublisher::ImagePublisher(const std::string& topic, const ros::NodeHandle& node_handle,
                               bool republishing)
  : node_handle_(node_handle),
    republishing_(republishing)
{
  if (!republishing_)
    image_pub_ = node_handle_.advertise<image_msgs::Image>(topic, 1);
  thumbnail_pub_ = node_handle_.advertise<image_msgs::Image>(topic + "_thumbnail", 1);
  compressed_pub_ = node_handle_.advertise<sensor_msgs::CompressedImage>(topic + "_compressed", 1);
  
  node_handle_.param("~thumbnail_size", thumbnail_size_, 128);
  node_handle_.param("~compression_type", compression_type_, std::string("jpeg"));
  
  
  node_handle_.param("~compression_quality", compression_quality_, 80);
}

ImagePublisher::~ImagePublisher()
{
}

uint32_t ImagePublisher::getNumSubscribers() //const
{
  return image_pub_.getNumSubscribers() + thumbnail_pub_.getNumSubscribers()
    + compressed_pub_.getNumSubscribers();
}

std::string ImagePublisher::getTopic() //const
{
  return image_pub_.getTopic();
}

std::string ImagePublisher::getTopicThumbnail() //const
{
  return thumbnail_pub_.getTopic();
}

std::string ImagePublisher::getTopicCompressed() //const
{
  return compressed_pub_.getTopic();
}

void ImagePublisher::publish(const image_msgs::Image& message) //const
{
  if (!republishing_)
    image_pub_.publish(message);
  
  uint32_t thumb_subscribers = thumbnail_pub_.getNumSubscribers();
  uint32_t compressed_subscribers = compressed_pub_.getNumSubscribers();
  if (thumb_subscribers == 0 && compressed_subscribers == 0)
    return;

  // Convert to IPL image
  /** @todo: support depths other than 8-bit */
  if (message.depth != "uint8" && message.depth != "int8") {
    ROS_ERROR("Unsupported image depth: %s", message.depth.c_str());
    return;
  }

  int channels = message.uint8_data.layout.dim[2].size;
  std::string encoding;
  if (channels == 1)
    encoding = "mono";
  else if (channels == 3)
    encoding = "rgb"; /** @todo: avoid BGR->RGB conversion? */
  else {
    /** @todo: RGBA, BGRA. Can we do anything with other encodings? */
    ROS_ERROR("Unsupported number of image channels: %d", channels);
    return;
  }
  
  if (!cv_bridge_.fromImage(message, encoding)) {
    ROS_ERROR("Could not convert from %s to %s", message.encoding.c_str(), encoding.c_str());
    return;
  }
  
  if (thumb_subscribers > 0) {
    image_msgs::Image thumbnail;
    thumbnail.header = message.header;
    thumbnail.label = message.label;
    publishThumbnailImage(thumbnail);
  }

  if (compressed_subscribers > 0) {
    sensor_msgs::CompressedImage compressed;
    compressed.header = message.header;
    compressed.label = message.label;
    compressed.encoding = encoding;
    publishCompressedImage(compressed);
  }
}

void ImagePublisher::publish(const image_msgs::ImageConstPtr& message) //const
{
  publish(*message);
}

void ImagePublisher::shutdown()
{
  image_pub_.shutdown();
  thumbnail_pub_.shutdown();
  compressed_pub_.shutdown();
}

void ImagePublisher::publishThumbnailImage(image_msgs::Image& thumbnail) //const
{
  const IplImage* image = cv_bridge_.toIpl();
  int width = image->width;
  int height = image->height;
  float aspect = std::sqrt((float)width / height);
  int scaled_width  = thumbnail_size_ * aspect + 0.5;
  int scaled_height = thumbnail_size_ / aspect + 0.5;

  cv::WImageBuffer_b buffer(scaled_width, scaled_height, image->nChannels);
  cvResize(image, buffer.Ipl());

  if (image_msgs::CvBridge::fromIpltoRosImage(buffer.Ipl(), thumbnail)) {
    thumbnail_pub_.publish(thumbnail);
  } else {
    ROS_ERROR("Unable to create thumbnail image message");
  }
}

void ImagePublisher::publishCompressedImage(sensor_msgs::CompressedImage& compressed) //const
{
  const IplImage* image = cv_bridge_.toIpl();
  int jpeg_params[] = {CV_IMWRITE_JPEG_QUALITY, 80, 0};
  CvMat* buf = cvEncodeImage(".jpeg", image, jpeg_params);

  compressed.format = "jpeg";
  compressed.uint8_data.layout.dim.resize(2);
  compressed.uint8_data.layout.dim[0].label = "height";
  compressed.uint8_data.layout.dim[0].size = image->height;
  compressed.uint8_data.layout.dim[0].stride = 0;

  compressed.uint8_data.layout.dim[1].label = "width";
  compressed.uint8_data.layout.dim[1].size = image->width;
  compressed.uint8_data.layout.dim[1].stride = 0;

  compressed.uint8_data.data.resize(buf->width);
  memcpy(&compressed.uint8_data.data[0], buf->data.ptr, buf->width);

  compressed_pub_.publish(compressed);
}
