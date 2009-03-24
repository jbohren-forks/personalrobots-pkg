#include "ros/node.h"
#include "robot_msgs/PoseStamped.h"
#include "tf/tf.h"
#include <LinearMath/btTransform.h>
#include <boost/thread.hpp>

class PoseTracker
{
private:
  boost::mutex cb_mutex_;
  robot_msgs::PoseStamped plug_msg_;
  robot_msgs::PoseStamped outlet_msg_;
  tf::Transform plug_pose_; // raw board in CV camera frame transform
  tf::Transform outlet_pose_; // in high-def camera frame
  tf::Transform camera_in_cvcam_;
  enum {Plug, Outlet} current_target_;

public:
  PoseTracker()
  {
    camera_in_cvcam_.getOrigin().setValue(0.0, 0.0, 0.0);
    camera_in_cvcam_.getBasis().setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);

    current_target_ = Outlet;
    ROS_INFO("Waiting for outlet...");
    ros::Node::instance()->subscribe("/plug_detector/pose", plug_msg_,
                                     &PoseTracker::plug_cb, this, 1);
    ros::Node::instance()->subscribe("/outlet_detector/pose", outlet_msg_,
                                     &PoseTracker::outlet_cb, this, 1);
  }

  void plug_cb()
  {
    boost::mutex::scoped_lock lock(cb_mutex_);
    if (current_target_ != Plug)
      return;
    tf::PoseMsgToTF(plug_msg_.pose, plug_pose_);

    btTransform plug_in_board = plug_pose_.inverse() * camera_in_cvcam_.inverse() * outlet_pose_;
    btQuaternion R = plug_in_board.getRotation();
    btVector3 O = plug_in_board.getOrigin();
    ROS_INFO("plug_in_board: %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n",
             O.x(), O.y(), O.z(), R.x(), R.y(), R.z(), R.w());
    
    current_target_ = Outlet;
    ROS_INFO("Waiting for outlet...");
  }

  void outlet_cb()
  {
    boost::mutex::scoped_lock lock(cb_mutex_);
    if (current_target_ != Outlet)
      return;
    tf::PoseMsgToTF(outlet_msg_.pose, outlet_pose_);

    current_target_ = Plug;
    ROS_INFO("Waiting for plug...");
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv);
  ros::Node n("calibrate_plug");
  PoseTracker tracker;
  n.spin();
  
  return 0;
}
