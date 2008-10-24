
#include "ros/node.h"
#include "std_msgs/Point32.h"
#include <rosTF/rosTF.h>

using std::string;

/**
 * This is a hack to get the position of the robot. TfPy is not here yet, so this is just to get the tests working.
 */
class GroundTruthTransform : public ros::node {
public:
  std_msgs::Point32 msg;
  rosTFClient tf;

  GroundTruthTransform() : ros::node("GroundTruthTransform"), tf(*this, false) {
    advertise<std_msgs::Point32>("groundtruthposition");
  }

  void speak() {
    libTF::TFPose robotPose, global_pose;
    robotPose.x = 0;
    robotPose.y = 0;
    robotPose.z = 0;
    robotPose.yaw = 0;
    robotPose.pitch = 0;
    robotPose.roll = 0;
    robotPose.time = 0;
    try {
      robotPose.frame = "base";
    } catch(libTF::TransformReference::LookupException& ex) {
      std::cerr << "LookupException in lookup(\"base\"): " << ex.what() << "\n";
      std::cout << "LookupException in lookup(\"base\"): " << ex.what();
      return;
    } catch(libTF::TransformReference::ExtrapolateException& ex) {
      std::cerr << "ExtrapolateException in lookup(\"base\"): " << ex.what() << "\n";
      std::cout << "ExtrapolateException in lookup(\"base\"): " << ex.what();
      return;
    } catch(libTF::TransformReference::ConnectivityException& ex) {
      std::cerr << "ConnectivityException in lookup(\"base\"): " << ex.what() << "\n";
      std::cout << "ConnectivityException in lookup(\"base\"): " << ex.what();
      return;
    }


    try {
      global_pose = this->tf.transformPose("map", robotPose);
    } catch(libTF::TransformReference::LookupException& ex) {
      std::cerr << tf.viewFrames();
      std::cerr << "LookupException in transformPose(\"map\", robotPose): " << ex.what() << "\n";
      std::cout << "LookupException in transformPose(\"map\", robotPose): " << ex.what();
      return;
    } catch(libTF::TransformReference::ExtrapolateException& ex) {
      std::cerr << "ExtrapolateException in transformPose(\"map\", robotPose): " << ex.what() << "\n";
      std::cout << "ExtrapolateException in transformPose(\"map\", robotPose): " << ex.what();
      return;
    } catch(libTF::TransformReference::ConnectivityException& ex) {
      std::cerr << "ConnectivityException in transformPose(\"map\", robotPose): " << ex.what() << "\n";
      std::cout << "ConnectivityException in transformPose(\"map\", robotPose): " << ex.what();
      return;
    }
    msg.x = global_pose.x;
    msg.y = global_pose.y;
    msg.z = global_pose.z;
    std::cout << msg.x << ", " << msg.y << ", " << msg.z;
    publish("groundtruthposition", msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv);
  std::cout << "Starting...\n";
  GroundTruthTransform t;
  while (t.ok())
  {
    usleep(100000);
    std::cout << "Transform: ";
    t.speak();
    std::cout << std::endl;
  }
  ros::fini();
  return 0;
}
