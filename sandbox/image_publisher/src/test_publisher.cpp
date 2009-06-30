#include <image_publisher/image_publisher.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle n;
  //ImagePublisher image_pub("raw_image", n);
  ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("raw_image", 1);
  
  cv::WImageBuffer3_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
  sensor_msgs::Image msg;
  sensor_msgs::CvBridge::fromIpltoRosImage(image.Ipl(), msg);
  msg.encoding = "bgr";
  msg.header.frame_id = "base_link";
  
  ros::Rate loop_rate(5);
  while (n.ok()) {
    image_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
