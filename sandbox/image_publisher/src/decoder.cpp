#include <ros/ros.h>
#include <image_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv_latest/CvBridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

class Decompressor
{
  ros::NodeHandle n_;
  ros::Subscriber compressed_sub_;
  ros::Publisher decompressed_pub_;

public:
  Decompressor(ros::NodeHandle n)
    : n_(n),
      compressed_sub_( n_.subscribe("compressed", 1, &Decompressor::imageCB, this) ),
      decompressed_pub_( n_.advertise<image_msgs::Image>("decompressed", 1) )
  {}

  void imageCB(const sensor_msgs::CompressedImageConstPtr &msg)
  {
    const CvMat compressed = cvMat(1, msg->uint8_data.data.size(), CV_8UC1,
                                   const_cast<unsigned char*>(&msg->uint8_data.data[0]));
    cv::WImageBuffer_b decompressed( cvDecodeImage(&compressed, CV_LOAD_IMAGE_ANYCOLOR) );
    image_msgs::Image image;
    if ( image_msgs::CvBridge::fromIpltoRosImage(decompressed.Ipl(), image) ) {
      image.header = msg->header;
      image.encoding = msg->encoding;
      decompressed_pub_.publish(image);
    }
    else {
      ROS_ERROR("Unable to create image message");
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "decoder");
  ros::NodeHandle n;
  Decompressor d(n);

  ros::spin();

  return 0;
}
