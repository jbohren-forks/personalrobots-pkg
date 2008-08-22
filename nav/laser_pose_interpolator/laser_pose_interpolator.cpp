

#include "ros/node.h"
#include "laser_pose_interpolator/PoseLaserScan.h"
#include <rosTF/rosTF.h>

using std::string;

/**
 * This class provides laser scans with properly interpolated poses for each.
 */
class LaserPoseInterpolator : public ros::node {
public:
  laser_pose_interpolator::PoseLaserScan msg;
  std_msgs::LaserScan laserMsg;
  rosTFClient tf;

  LaserPoseInterpolator() : ros::node("LaserPoseInterpolator"), tf(*this, false) {
    advertise<laser_pose_interpolator::PoseLaserScan>("pose_scan", 0);
    subscribe("scan", laserMsg, &LaserPoseInterpolator::laserReceived, 0);
    
  }

  ~LaserPoseInterpolator() { ros::fini(); }
   
  void laserReceived() {
    libTF::TFPose2D robotPose, global_pose;
    robotPose.x = 0;
    robotPose.y = 0;
    robotPose.yaw = 0;
    robotPose.time = laserMsg.header.stamp.sec * 1000000000ULL + laserMsg.header.stamp.nsec;
    try {
      robotPose.frame = "FRAMEID_ROBOT";
    } catch(libTF::TransformReference::LookupException& ex) {
      std::cerr << "LookupException in lookup(\"FRAMEID_ROBOT\"): " << ex.what() << "\n";
      std::cout << "LookupException in lookup(\"FRAMEID_ROBOT\"): " << ex.what() << "\n";
      return;
    } catch(libTF::TransformReference::ExtrapolateException& ex) {
      std::cerr << "ExtrapolateException in lookup(\"FRAMEID_ROBOT\"): " << ex.what() << "\n";
      std::cout << "ExtrapolateException in lookup(\"FRAMEID_ROBOT\"): " << ex.what() << "\n";
      return;
    } catch(libTF::TransformReference::ConnectivityException& ex) {
      std::cerr << "ConnectivityException in lookup(\"FRAMEID_ROBOT\"): " << ex.what() << "\n";
      std::cout << "ConnectivityException in lookup(\"FRAMEID_ROBOT\"): " << ex.what() << "\n";
      return;
    }


    try {
      global_pose = this->tf.transformPose2D("FRAMEID_ODOM", robotPose);
    } catch(libTF::TransformReference::LookupException& ex) {
      std::cerr << tf.viewFrames();
      std::cerr << "LookupException in transformPose2D(\"FRAMEID_ODOM\", robotPose): " << ex.what() << "\n";
      std::cout << "LookupException in transformPose2D(\"FRAMEID_ODOM\", robotPose): " << ex.what() << "\n";
      return;
    } catch(libTF::TransformReference::ExtrapolateException& ex) {
      std::cerr << "ExtrapolateException in transformPose2D(\"FRAMEID_ODOM\", robotPose): " << ex.what() << "\n";
      std::cout << "ExtrapolateException in transformPose2D(\"FRAMEID_ODOM\", robotPose): " << ex.what() << "\n";
      return;
    } catch(libTF::TransformReference::ConnectivityException& ex) {
      std::cerr << "ConnectivityException in transformPose2D(\"FRAMEID_ODOM\", robotPose): " << ex.what() << "\n";
      std::cout << "ConnectivityException in transformPose2D(\"FRAMEID_ODOM\", robotPose): " << ex.what() << "\n";
      return;
    }
    msg.scan = laserMsg;
    msg.pose.x = global_pose.x;
    msg.pose.y = global_pose.y;
    msg.pose.th = global_pose.yaw;
    publish("pose_scan", msg);
  }

};


int main(int argc, char **argv) {
  ros::init(argc, argv);
  LaserPoseInterpolator laser;
  while (laser.ok()) {
    usleep(100000);
  }
  return 0;
}






