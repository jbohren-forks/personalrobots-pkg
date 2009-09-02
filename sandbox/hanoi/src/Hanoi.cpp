#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/intersections.h>

#include "move_arm_tools/ArmCtrlCmd.h"

#include "Hanoi.h"

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Hanoi::Hanoi(ros::NodeHandle &nh)
  : nodeHandle_(nh), tf_(nodeHandle_)
{
  std::string cloud_topic;
  std::string start_arm;
  float x,y,z;

  if (!nodeHandle_.getParam("parameter_frame",parameter_frame_))
    parameter_frame_ = "base_link";

  if (!nodeHandle_.getParam("~start_arm", start_arm))
    start_arm = "0.1 0.2 0.3";

  int index1 = start_arm.find(" ");
  int index2 = start_arm.rfind(" ");

  x = atof(start_arm.substr(0, index1).c_str());
  y = atof(start_arm.substr(index1+1, index2-index1+1).c_str());
  z = atof(start_arm.substr(index2).c_str());

  armGoal_.header.frame_id = parameter_frame_;

  std::cout << "XYZ[" << x << "," << y << "," << z << "]\n";

  // Subscribe to the blobs topic
  blobSubscriber_ = nodeHandle_.subscribe("blobs", 100, &Hanoi::BlobCB, this);

  //moveArmResultSubscriber_ = nodeHandle_.subscribe("/move_right_arm/result", 10, &Hanoi::MoveArmResultCB, this);

  colorTrackerStatusSubscriber_ = nodeHandle_.subscribe("/color_tracker/status", 10, &Hanoi::ColorTrackStatusCB, this);

  armStatusSubscriber_ = nodeHandle_.subscribe("/arm_controller/status", 10, &Hanoi::ArmStatusCB, this);

  // Create the publisher for cylinder data
  cylinderPublisher_ = nodeHandle_.advertise<hanoi::Cylinders>("~/cylinders", 1);

  // Create the publisher that controls head tracking
  colorTrackPublisher_ = nodeHandle_.advertise<hanoi::ColorTrackCmd>("/color_tracker/cmd", 1);

  // Create the publisher that controls head tracking
  armCmdPublisher_ = nodeHandle_.advertise<hanoi::ArmCmd>("/arm_controller/cmd", 1);

  // Subscribe to the point cloud data
  //cloudSubscriber_ = nodeHandle_.subscribe("cloud_data",1,&Hanoi::CloudCB,this);

  ros::service::waitForService("/auto_arm_cmd_server");

  /*updateTimer_ = nodeHandle_.createTimer(ros::Duration(0.5), 
        &Hanoi::UpdateCB, this);
        */

  // Get the point cloud
  tf::MessageNotifier<sensor_msgs::PointCloud> *message_notifier = 
    new tf::MessageNotifier<sensor_msgs::PointCloud>( tf_, 
       boost::bind(&Hanoi::CloudCB,this,_1), "cloud_data", parameter_frame_, 1);

  // Move arm to starting position
  //this->CommandArm("ik", x,y,z, M_PI/2.0, 0 , 0);

  actions_["grasp"].push_back(MOVE_ARM);
  actions_["grasp"].push_back(MOVING_ARM);
  actions_["grasp"].push_back(OPEN_GRIPPER);
  actions_["grasp"].push_back(OPENING_GRIPPER);
  actions_["grasp"].push_back(MOVE_OFFSET);
  actions_["grasp"].push_back(MOVING_ARM);
  actions_["grasp"].push_back(CLOSE_GRIPPER);
  actions_["grasp"].push_back(CLOSING_GRIPPER);

  //this->ActivateAction("grasp");

  hanoi::ColorTrackCmd ctcmd;
  ctcmd.red = 1;
  ctcmd.green = 0;
  ctcmd.blue = 0;
  colorTrackPublisher_.publish(ctcmd);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Hanoi::~Hanoi()
{
}

////////////////////////////////////////////////////////////////////////////////
// Activate an action to perform
void Hanoi::ActivateAction(const std::string actionName)
{
  std::map< std::string, std::vector<State> >::iterator iter;

  iter = actions_.find(actionName);

  if (iter != actions_.end())
  {
    stateIter_ = iter->second.begin();
    currentAction_ = actionName;
  }
  else
    currentAction_ = "";
}


////////////////////////////////////////////////////////////////////////////////
// Update callback
void Hanoi::UpdateCB( const ros::TimerEvent &e )
{
  std::map<std::string, std::vector<State> >::iterator iter;

  iter = actions_.find(currentAction_);

  printf("Update\n");

  if (iter != actions_.end())
  {
    switch (*stateIter_)
    {
      case MOVE_ARM:
        printf("MOVE_ARM\n");
        if (this->MoveArm())
          stateIter_++;
        break;

      case MOVING_ARM:
        printf("MOVING_ARM\n");
        if (this->ArmAtGoal())
          stateIter_++;
        break;

      case OPEN_GRIPPER:
        /*printf("OPEN_GRIPPER\n");
        this->OpenGripper();
        stateIter_++;
        */
        break;

      case OPENING_GRIPPER:
        /*printf("OPENING_GRIPPER\n");
        if (this->GripperIsOpen())
          stateIter_++;
          */
        break;

      case CLOSE_GRIPPER:
        /*printf("CLOSE_GRIPPER\n");
        this->CloseGripper();
        stateIter_++;
        */
        break;

      case CLOSING_GRIPPER:
        /*printf("CLOSING_GRIPPER\n");
        if (this->GripperIsClosed())
          stateIter_++;
          */
        break;

      case MOVE_OFFSET:
        printf("MOVE_OFFSET\n");
        this->CommandArm("ik", armGoal_.point.x + HAND_OFFSET, 
            armGoal_.point.y, armGoal_.point.z, M_PI/2.0, 0 , 0);
        stateIter_++;
        break;
    };

    // Check to see if the stateIter_ has reached the end of the action list
    if (stateIter_ == iter->second.end())
    {
      std::cout << "Action[" << currentAction_ << " Is Complete!\n";
      currentAction_ = "";
    }
  }
}

/*
////////////////////////////////////////////////////////////////////////////////
// Open the gripper
void Hanoi::OpenGripper()
{
  move_arm::ActuateGripperGoal g;
  g.data = 50.0;

  gripperAction_->sendGoal(g);

  bool status = gripperAction_->waitForGoalToFinish(ros::Duration(5));
  if (status)
    std::cout << "Gripper State:" << gripperAction_->getTerminalState().toString() << "\n";
  else
    std::cout << "Gripper unable to reach goal\n";
}*/

////////////////////////////////////////////////////////////////////////////////
/// Close the gripper
/*void Hanoi::CloseGripper()
{
  move_arm::ActuateGripperGoal g;
  g.data = -50.0;

  gripperAction_->sendGoal(g);

  bool status = gripperAction_->waitForGoalToFinish(ros::Duration(5));
  if (status)
    std::cout << "Gripper State:" << gripperAction_->getTerminalState().toString() << "\n";
  else
    std::cout << "Gripper unable to reach goal\n";
}*/

////////////////////////////////////////////////////////////////////////////////
// Return true if the arm has reached its goal
bool Hanoi::ArmAtGoal()
{
  geometry_msgs::PointStamped point;
  float dist;

  point.header.frame_id = "r_wrist_roll_link";
  point.header.stamp = ros::Time();

  point.point.x = 0;
  point.point.y = 0;
  point.point.z = 0;

  tf_.transformPoint( "base_link", point, point );

  dist = sqrt( pow(point.point.x-armGoal_.point.x,2) + 
               pow(point.point.y-armGoal_.point.y,2) +
               pow(point.point.z-armGoal_.point.z,2));

  printf("Distance to goal[%f]\n",dist);
  if (dist < 0.001)
    return true;
 
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// Command arm
void Hanoi::CommandArm(const std::string &mode, float x, float y, float z, 
                       float roll, float pitch, float yaw)
{
  hanoi::ArmCmd armcmd;

  armcmd.mode = mode;

  armcmd.x = x;
  armcmd.y = y;
  armcmd.z = z;

  armcmd.roll = roll;
  armcmd.pitch = pitch;
  armcmd.yaw = yaw;

  armCmdPublisher_.publish(armcmd);
}

////////////////////////////////////////////////////////////////////////////////
// Got a point cloud
void Hanoi::CloudCB(const tf::MessageNotifier<sensor_msgs::PointCloud>::MessagePtr& cloud)
{
  pointcloud_ = *cloud;

  //ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)pointcloud_.points.size (), pointcloud_.header.frame_id.c_str (), (int)pointcloud_.channels.size (), cloud_geometry::getAvailableChannels (pointcloud_).c_str ());
}

////////////////////////////////////////////////////////////////////////////////
// Move arm to a goal
bool Hanoi::MoveArm()
{
  if (pointcloud_.points.size() <= 0)
    return false;

  float dist, minDist;
  int minIndex = 0;
  int closestIndex = 0;
  float minPointX;
  float xvalue, yvalue;

  minDist = 1000;
  minPointX = 1000;

  tf_.transformPointCloud(parameter_frame_, pointcloud_, pointcloud_ );

  for (unsigned int y = redBlob_.y; y < redBlob_.bottom; y++)
  {
    for (unsigned int i=0; i < pointcloud_.channels[1].values.size(); i++)
    {
      yvalue = pointcloud_.channels[2].values[i];
      xvalue = pointcloud_.channels[1].values[i];

      dist = sqrt( pow(xvalue-redBlob_.x, 2) + pow(yvalue-y,2) );
      if (dist < minDist)
      {
        minDist = dist;
        minIndex = i;
      }
    }

    if (pointcloud_.points[minIndex].x < minPointX)
    {
      minPointX = pointcloud_.points[minIndex].x;
      closestIndex = minIndex;
    }

  }

  printf("MinDist[%f] I[%d] XYZ[%f %f %f]\n",minDist, closestIndex, 
      pointcloud_.points[closestIndex].x,
      pointcloud_.points[closestIndex].y, 
      pointcloud_.points[closestIndex].z );

  this->CommandArm("plan", 
      pointcloud_.points[closestIndex].x - HAND_OFFSET,
      pointcloud_.points[closestIndex].y, 
      pointcloud_.points[closestIndex].z, M_PI/2.0,0,0 );

  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Blob callback
void Hanoi::BlobCB(const cmvision::BlobsConstPtr &msg)
{
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

  /*printf("Red Size[%d] Pos[%d %d]\n", redBlob_.area, redBlob_.x, redBlob_.y);
  printf("Green Size[%d] Pos[%d %d]\n", greenBlob_.area, greenBlob_.x, greenBlob_.y);
  printf("Blue Size[%d] Pos[%d %d]\n", blueBlob_.area, blueBlob_.x, blueBlob_.y);
  */
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
  //std::cout << "FrameId:" << pointcloud_.header.frame_id << std::endl;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Color track status callback
void Hanoi::ColorTrackStatusCB(const hanoi::ColorTrackStatusConstPtr &msg)
{
  colorTrackStatus_ = msg->status;
}

////////////////////////////////////////////////////////////////////////////////
// Arm status callback
void Hanoi::ArmStatusCB(const hanoi::ArmStatusConstPtr &msg)
{
  armStatus_ = *msg;
}
