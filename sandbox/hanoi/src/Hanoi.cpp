#include <pr2_mechanism_msgs/JointStates.h>
#include <pr2_mechanism_msgs/JointState.h>

#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/intersections.h>

#include "LinearMath/btQuaternion.h"
#include "move_arm_tools/ArmCtrlCmd.h"

#include "Hanoi.h"

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Hanoi::Hanoi(ros::NodeHandle &nh)
  : nodeHandle_(nh), tf_(nodeHandle_)
{
  std::string cloud_topic;

  
  if (!nodeHandle_.getParam("parameter_frame",parameter_frame_))
    parameter_frame_ = "base_link";

  // Subscribe to the blobs topic
  blobSubscriber_ = nodeHandle_.subscribe("blobs", 100, &Hanoi::BlobCB, this);

  headPublisher_ = nodeHandle_.advertise<pr2_mechanism_msgs::JointStates>("pan_tilt", 1, true);

  // Create the publisher for cylinder data
  cylinderPublisher_ = nodeHandle_.advertise<hanoi::Cylinders>("cylinders", 1);

  cloudSubscriber_ = nodeHandle_.subscribe("cloud_data",1,&Hanoi::CloudCB,this);

  ros::service::waitForService("/auto_arm_cmd_server");
  moveArmService_ = nodeHandle_.serviceClient<move_arm_tools::ArmCtrlCmd::Request, move_arm_tools::ArmCtrlCmd::Response> ("/auto_arm_cmd_server");


  // Get the point cloud
  /*tf::MessageNotifier<sensor_msgs::PointCloud> *message_notifier = 
    new tf::MessageNotifier<sensor_msgs::PointCloud>( tf_, 
       boost::bind(&Hanoi::CloudCB,this,_1), "cloud_data", parameter_frame_, 1);
       */

  this->CommandArm(0.5, 0.0, 0.8, 0, 0 ,0);
  this->CommandHead(0.0, 0.4);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Hanoi::~Hanoi()
{
}

////////////////////////////////////////////////////////////////////////////////
// Pan and tilt the head
void Hanoi::CommandHead(float pan, float tilt)
{
  pr2_mechanism_msgs::JointStates jointStatesMsg;
  pr2_mechanism_msgs::JointState panJoint, tiltJoint;

  panJoint.name = "head_pan_joint";
  panJoint.position = pan;

  tiltJoint.name = "head_tilt_joint";
  tiltJoint.position = tilt;

  jointStatesMsg.joints.push_back(panJoint);
  jointStatesMsg.joints.push_back(tiltJoint);

  headPublisher_.publish( jointStatesMsg );
}

////////////////////////////////////////////////////////////////////////////////
// Command arm
void Hanoi::CommandArm(float x, float y, float z, 
                       float roll, float pitch, float yaw)
{
  move_arm_tools::ArmCtrlCmd::Request req;
  move_arm_tools::ArmCtrlCmd::Response res;

  std::stringstream cmd;

  btQuaternion quat(yaw, pitch, roll);

  cmd << "move " << x << " " << y << " " << z << " " 
    << quat.getX() << " " << quat.getY() << " " << quat.getZ() 
    << " " << quat.getW();

  req.cmd = cmd.str();
  req.allowed_time = 60.0;

  printf("Moving arm...\r\n");
  moveArmService_.call(req, res);

  if (res.success)
    printf("success\n");
  else
    printf("failure\n");
}

////////////////////////////////////////////////////////////////////////////////
// Got a point cloud
void Hanoi::CloudCB(const sensor_msgs::PointCloudConstPtr &cloud)
{
  pointcloud_ = *cloud;

  //ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)pointcloud_.points.size (), pointcloud_.header.frame_id.c_str (), (int)pointcloud_.channels.size (), cloud_geometry::getAvailableChannels (pointcloud_).c_str ());

}


////////////////////////////////////////////////////////////////////////////////
// Blob callback
void Hanoi::BlobCB(const cmvision::BlobsConstPtr &msg)
{
  printf("Blob Count[%d]\n", msg->blobCount);

  redBlob_.area = 0;
  greenBlob_.area = 0;
  blueBlob_.area = 0;

  for (unsigned int i = 0; i < msg->blobCount; i++)
  {
    unsigned int r,g,b;
    unsigned int area, x, y;

    r = msg->blobs[i].red;
    g = msg->blobs[i].green;
    b = msg->blobs[i].blue;

    area = msg->blobs[i].area;

    x = msg->blobs[i].x;
    y = msg->blobs[i].y;

    if (r == 255 && area > redBlob_.area)
      redBlob_ = msg->blobs[i];
    if (g == 255 && area > greenBlob_.area)
      greenBlob_ = msg->blobs[i];
    if (b == 255 && area > blueBlob_.area)
      blueBlob_ = msg->blobs[i];

   //printf("Blob[%d] RGB[%d %d %d] Size[%d]\n", i, r,g,b, area);
  }

  printf("Red Size[%d] Pos[%d %d]\n", redBlob_.area, redBlob_.x, redBlob_.y);
  printf("Green Size[%d] Pos[%d %d]\n", greenBlob_.area, greenBlob_.x, greenBlob_.y);
  printf("Blue Size[%d] Pos[%d %d]\n", blueBlob_.area, blueBlob_.x, blueBlob_.y);
  this->CalculateGraspPoints();

  cylinderMessage_.redArea = redBlob_.area;
  cylinderMessage_.redX = redBlob_.x;
  cylinderMessage_.redY = redBlob_.y;
  cylinderMessage_.redLeft = redBlob_.left;
  cylinderMessage_.redRight = redBlob_.right;
  cylinderMessage_.redTop = redBlob_.top;
  cylinderMessage_.redBottom = redBlob_.bottom;

  cylinderMessage_.greenArea = greenBlob_.area;
  cylinderMessage_.greenX = greenBlob_.x;
  cylinderMessage_.greenY = greenBlob_.y;
  cylinderMessage_.greenLeft = greenBlob_.left;
  cylinderMessage_.greenRight = greenBlob_.right;
  cylinderMessage_.greenTop = greenBlob_.top;
  cylinderMessage_.greenBottom = greenBlob_.bottom;

  cylinderMessage_.blueArea = blueBlob_.area;
  cylinderMessage_.blueX = blueBlob_.x;
  cylinderMessage_.blueY = blueBlob_.y;
  cylinderMessage_.blueLeft = blueBlob_.left;
  cylinderMessage_.blueRight = blueBlob_.right;
  cylinderMessage_.blueTop = blueBlob_.top;
  cylinderMessage_.blueBottom = blueBlob_.bottom;

  cylinderPublisher_.publish(cylinderMessage_);
}

////////////////////////////////////////////////////////////////////////////////
// Calculate the 3D grasp points
bool Hanoi::CalculateGraspPoints()
{
  std::cout << "FrameId:" << pointcloud_.header.frame_id << std::endl;

  return true;
}
