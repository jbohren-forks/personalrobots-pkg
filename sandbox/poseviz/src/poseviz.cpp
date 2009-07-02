//
//   poseviz.cpp
//
//   Visualize a pose message
//

// ROS headers
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>



//
//   Variable Definitions
//
std::string pose_name;		// Name of the pose (to be visualized)

ros::Publisher  pub;		// Published (to visualizer)
ros::Subscriber sub;		// Pose message



//
//  Receiver (and Transmiter)
//
void receive_and_retransmit(const robot_msgs::PoseStampedConstPtr &msg)
{
  // Record the current pose
  tf::Pose tf_pose ;
  tf::poseMsgToTF(msg->pose, tf_pose) ;

  // Initialize the marker message
  visualization_msgs::Marker marker ;

  marker.header.frame_id = msg->header.frame_id ;
  marker.header.stamp = msg->header.stamp ;
  marker.ns = pose_name ;
  marker.type = visualization_msgs::Marker::ARROW ;
  marker.action = visualization_msgs::Marker::ADD ;
  marker.scale.x = .1 ;
  marker.scale.y = .1 ;
  marker.scale.z = .1 ;
  marker.lifetime = ros::Duration().fromSec(0.0) ;

  // Publish the X axis
  marker.pose = msg->pose ;

  marker.id = 0 ;
  marker.color.r = 1.0 ;
  marker.color.g = 0.0 ;
  marker.color.b = 0.0 ;
  marker.color.a = 1.0 ;
  pub.publish(marker) ;

  // Publish the Y axis
  btQuaternion x_to_y(btVector3(0.0, 0.0, 1.0), M_PI/2) ;
  
  tf::poseTFToMsg( tf_pose*btTransform(x_to_y), marker.pose) ;
  marker.id = 1 ;
  marker.color.r = 0.0 ;
  marker.color.g = 1.0 ;
  marker.color.b = 0.0 ;
  marker.color.a = 1.0 ;
  pub.publish(marker) ;

  // Publish the Z axis
  btQuaternion x_to_z(btVector3(0.0,-1.0, 0.0), M_PI/2) ;

  tf::poseTFToMsg( tf_pose*btTransform(x_to_z), marker.pose) ;
  marker.id = 2 ;
  marker.color.r = 0.0 ;
  marker.color.g = 0.0 ;
  marker.color.b = 1.0 ;
  marker.color.a = 1.0 ;
  pub.publish(marker) ;
}



//
//   Clean up marker
//
void clear(void)
{
  // Initialize the marker message
  visualization_msgs::Marker marker ;

  marker.ns = pose_name ;
  marker.type = visualization_msgs::Marker::ARROW ;
  marker.action = visualization_msgs::Marker::ADD ;
  marker.lifetime = ros::Duration().fromSec(0.1) ;	 // time???

  // Publish the X axis
  marker.id = 0 ;
  pub.publish(marker) ;

  // Publish the Y axis
  marker.id = 1 ;
  pub.publish(marker) ;

  // Publish the Z axis
  marker.id = 2 ;
  pub.publish(marker) ;
}



//
// Main
//
int main(int argc, char **argv) 
{
  // Now initialize ROS.
  ros::init(argc, argv, "poseviz");
  ros::NodeHandle n;

  // Get the name for the visualized pose.
  n.param("pose_name", pose_name, std::string("poseviz_pose"));

  // Initialize the publisher and then the subscriber.
  pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 3);
  sub = n.subscribe("pose", 1, receive_and_retransmit);

  // Run.
  ros::spin();

  // HOW TO CLEAR THE FRAME IN RVIZ???
  //clear();
  return(0);
}
