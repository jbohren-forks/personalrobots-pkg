#include <ros/ros.h>
#include <image_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv_latest/CvBridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

ros::Publisher g_decompressed_pub;

void imageCB(const sensor_msgs::CompressedImageConstPtr &msg)
{
  const CvMat compressed = cvMat(1, msg->uint8_data.data.size(), CV_8UC1,
                                 const_cast<unsigned char*>(&msg->uint8_data.data[0]));
  cv::WImageBuffer_b decompressed( cvDecodeImage(&compressed, CV_LOAD_IMAGE_ANYCOLOR) );
  image_msgs::Image image;
  if ( image_msgs::CvBridge::fromIpltoRosImage(decompressed.Ipl(), image) ) {
    image.header = msg->header;
    image.encoding = msg->encoding;
    g_decompressed_pub.publish(image);
  }
  else {
    ROS_ERROR("Unable to create image message");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "decoder", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  g_decompressed_pub = n.advertise<image_msgs::Image>("decompressed", 1);
  ros::Subscriber compressed_sub = n.subscribe("compressed", 1, &imageCB);

  ros::spin();

  return 0;
}
