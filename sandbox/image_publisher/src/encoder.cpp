#include <image_publisher/image_publisher.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "encoder");
  ros::NodeHandle n;

  std::string topic = n.mapName("image");
  ImagePublisher image_pub(topic, n, true);
  ros::Subscriber raw_sub = n.subscribe(topic, 1, &ImagePublisher::publish, &image_pub);

  ros::spin();

  return 0;
}
