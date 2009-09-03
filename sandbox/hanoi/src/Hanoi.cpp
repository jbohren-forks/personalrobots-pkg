#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/intersections.h>

#include <visualization_msgs/Marker.h>

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

  armStatusSubscriber_ = nodeHandle_.subscribe("/arm_controller/status", 10, 
      &Hanoi::ArmStatusCB, this);

  // Create the publisher for cylinder data
  cylinderPublisher_ = nodeHandle_.advertise<hanoi::Cylinders>("~/cylinders", 1);

  // Create the publisher that controls head tracking
  colorTrackPublisher_ = nodeHandle_.advertise<hanoi::ColorTrackCmd>(
      "/color_tracker/cmd", 1);

  // Create the publisher that controls head tracking
  armCmdPublisher_ = nodeHandle_.advertise<hanoi::ArmCmd>(
      "/arm_controller/cmd", 1);

  visPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>( 
      "visualization_marker", 0 );

  // Subscribe to the point cloud data
  //cloudSubscriber_ = nodeHandle_.subscribe("cloud_data",1,&Hanoi::CloudCB,this);

  //ros::service::waitForService("/auto_arm_cmd_server");

  updateTimer_ = nodeHandle_.createTimer(ros::Duration(0.5), 
        &Hanoi::UpdateCB, this);

  // Get the point cloud
  tf::MessageNotifier<sensor_msgs::PointCloud> *message_notifier = 
    new tf::MessageNotifier<sensor_msgs::PointCloud>( tf_, 
       boost::bind(&Hanoi::CloudCB,this,_1), "cloud_data", parameter_frame_, 1);

  // Move arm to starting position
  //this->CommandArm("ik", x,y,z, M_PI/2.0, 0 , 0);

  actions_["calibrate"].push_back(CALIBRATE);

  actions_["grasp"].push_back(MOVE_ARM);
  actions_["grasp"].push_back(MOVING_ARM);
  actions_["grasp"].push_back(OPEN_GRIPPER);
  actions_["grasp"].push_back(OPENING_GRIPPER);
  actions_["grasp"].push_back(MOVE_OFFSET);
  actions_["grasp"].push_back(MOVING_ARM);
  actions_["grasp"].push_back(CLOSE_GRIPPER);
  actions_["grasp"].push_back(CLOSING_GRIPPER);
  actions_["grasp"].push_back(LIFT);
  actions_["grasph"].push_back(MOVING_ARM);

  actions_["left"].push_back(MOVE_POST0);
  actions_["left"].push_back(MOVING_ARM);

  actions_["middle"].push_back(MOVE_POST1);
  actions_["middle"].push_back(MOVING_ARM);

  actions_["right"].push_back(MOVE_POST2);
  actions_["right"].push_back(MOVING_ARM);

  actions_["release"].push_back(LOWER);
  actions_["release"].push_back(MOVING_ARM);
  actions_["release"].push_back(OPEN_GRIPPER);
  actions_["release"].push_back(OPENING_GRIPPER);
  actions_["release"].push_back(MOVE_NEG_OFFSET);
  actions_["release"].push_back(MOVING_ARM);
  actions_["release"].push_back(MOVE_DEFAULT_POS);
  actions_["release"].push_back(MOVING_ARM);
  actions_["release"].push_back(CLOSE_GRIPPER);
  actions_["release"].push_back(CLOSING_GRIPPER);

  plan_.push_back(std::pair<std::string, std::string>("grasp","green"));
  plan_.push_back(std::pair<std::string, std::string>("right","green"));
  plan_.push_back(std::pair<std::string, std::string>("release","green"));
  plan_.push_back(std::pair<std::string, std::string>("grasp","red"));
  plan_.push_back(std::pair<std::string, std::string>("middle","red"));
  plan_.push_back(std::pair<std::string, std::string>("release","red"));

  this->ActivateAction("calibrate");

  /*hanoi::ColorTrackCmd ctcmd;
  ctcmd.red = 1;
  ctcmd.green = 0;
  ctcmd.blue = 0;
  colorTrackPublisher_.publish(ctcmd);
  */

  calibrated_ = false;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Hanoi::~Hanoi()
{
}

////////////////////////////////////////////////////////////////////////////////
// Calibrate the post locations before playing the game
bool Hanoi::Calibrate()
{
  if (pointcloud_.points.size() <= 0)
  {
    printf("No point cloud.\n");
    return false;
  }

  if (colorTrackStatus_ != "tracking")
  {
    printf("Hasn't seen the target\n");
    return false;
  }

  if (redPos_.x > 0.1)
  {
    postY[0] = redPos_.y;
    postY[1] = postY[0] - 0.5;
    postY[2] = postY[2] - 0.7;
    return true;
  }

  return false;
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
// Start the plan
void Hanoi::StartPlan()
{
  planIter_ = plan_.begin();

  this->SetDisk( (*planIter_).second );
  this->ActivateAction( (*planIter_).first );
}

////////////////////////////////////////////////////////////////////////////////
// Update the plan
void Hanoi::UpdatePlan()
{
  planIter_++;

  if (planIter_ == plan_.end())
  {
    std::cout << "Plan complete\n";
    return;
  }

  this->SetDisk( (*planIter_).second );
  this->ActivateAction( (*planIter_).first );
}

////////////////////////////////////////////////////////////////////////////////
// Set which disk is active
void Hanoi::SetDisk( const std::string &clr) 
{
  activeDisk_ = clr;
}

////////////////////////////////////////////////////////////////////////////////
// Update callback
void Hanoi::UpdateCB( const ros::TimerEvent &e )
{
  std::map<std::string, std::vector<State> >::iterator iter;

  iter = actions_.find(currentAction_);

  // Update the pose of the end effector
  geometry_msgs::PointStamped point;
  point.header.frame_id = "r_wrist_roll_link";
  point.header.stamp = ros::Time();

  point.point.x = 0;
  point.point.y = 0;
  point.point.z = 0;

  tf_.transformPoint( "base_link", point, point );

  curEEPos.x = point.point.x;
  curEEPos.y = point.point.y;
  curEEPos.z = point.point.z;

  if (iter != actions_.end())
  {
    switch (*stateIter_)
    {
      case CALIBRATE:
        {
          printf("CALIBRATE\n");
          if (this->Calibrate())
          {
            calibrated_ = true;
            this->StartPlan();
          }
          break;
        }

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
        {
          printf("OPEN GRIPPER\n");
          hanoi::ArmCmd armcmd;
          armcmd.action = "open";
          armCmdPublisher_.publish(armcmd);
          stateIter_++;
          break;
        }

      case OPENING_GRIPPER:
        printf("OPENING GRIPPER\n");
        if (armStatus_.status == "open" )
          stateIter_++;
        break;

      case CLOSE_GRIPPER:
        {
          printf("CLOSE GRIPPER\n");
          hanoi::ArmCmd armcmd;
          armcmd.action = "close";
          armCmdPublisher_.publish(armcmd);
          stateIter_++;
          break;
        }

      case CLOSING_GRIPPER:
        {
          printf("CLOSING GRIPPER\n");
          if (armStatus_.status == "closed" )
            stateIter_++;
          break;
        }

      case MOVE_OFFSET:
        {
          printf("MOVE_OFFSET\n");
          printf("Curr Pos[%f %f %f]\n", curEEPos.x, curEEPos.y, curEEPos.z);
          this->CommandArmDelta("ik", HAND_OFFSET*0.5, 0, 0, M_PI/2.0, 0 , 0);
          stateIter_++;
          break;
        }

      case MOVE_DEFAULT_POS:
        {
          printf("MOVE_DEFAULT_POS\n");
          this->CommandArm("plan", 0.4, -0.5, 0.8, M_PI/2.0, 0 , 0);
          stateIter_++;
          break;
        }

      case MOVE_NEG_OFFSET:
        {
          printf("MOVE_NEG_OFFSET\n");
          this->CommandArmDelta("ik", -0.02, -0.1, 0, M_PI/2.0, 0 , 0);
          stateIter_++;
          break;
        }

      case LIFT:
        {
          printf("LIFT\n");
          this->CommandArmDelta("ik", 0, 0, 0.3, M_PI/2.0, 0 , 0);
          stateIter_++;
          break;
        }

      case LOWER:
        {
          printf("LOWER\n");
          this->LowerDisk();
          stateIter_++;
          break;
        }

      case MOVE_POST0:
        {
          this->CommandArm("ik", curEEPos.x, postY[0], 0.9,
              M_PI/2.0, 0 , 0);
          stateIter_++;
          break;
        }
      case MOVE_POST1:
        {
          this->CommandArm("ik", curEEPos.x, postY[1], 0.9,
              M_PI/2.0, 0 , 0);
          stateIter_++;
          break;
        }
      case MOVE_POST2:
        {
          this->CommandArm("ik", curEEPos.x, postY[2], 0.9,
              M_PI/2.0, 0 , 0);
          stateIter_++;
          break;
        }

    };

    // Check to see if the stateIter_ has reached the end of the action list
    if (stateIter_ == iter->second.end())
    {
      std::cout << "Action[" << currentAction_ << " Is Complete!\n";
      currentAction_ = "";
      this->UpdatePlan();
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
  //geometry_msgs::PointStamped point;
  float dist;

  /*point.header.frame_id = "r_wrist_roll_link";
  point.header.stamp = ros::Time();

  point.point.x = 0;
  point.point.y = 0;
  point.point.z = 0;

  tf_.transformPoint( "base_link", point, point );

  dist = sqrt( pow(point.point.x - armGoal_.point.x,2) + 
               pow(point.point.y - armGoal_.point.y,2) +
               pow(point.point.z - armGoal_.point.z,2));
  */

  dist = sqrt( pow(curEEPos.x - armGoal_.point.x,2) + 
               pow(curEEPos.y - armGoal_.point.y,2) +
               pow(curEEPos.z - armGoal_.point.z,2));

  //printf("Distance to goal[%f]\n",dist);
  if (dist < 0.07)
    return true;
 
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// Command arm
void Hanoi::CommandArm(const std::string &mode, float x, float y, float z, 
                       float roll, float pitch, float yaw)
{
  hanoi::ArmCmd armcmd;

  armcmd.action = "move";
  armcmd.mode = mode;

  armcmd.x = x;
  armcmd.y = y;
  armcmd.z = z;

  armcmd.roll = roll;
  armcmd.pitch = pitch;
  armcmd.yaw = yaw;

  armGoal_.point.x = x;
  armGoal_.point.y = y;
  armGoal_.point.z = z;

  printf("Sending arm command[%s to %f %f %f]\n",mode.c_str(), x,y,z);
  armCmdPublisher_.publish(armcmd);
}

////////////////////////////////////////////////////////////////////////////////
// Command arm delta
void Hanoi::CommandArmDelta(const std::string &mode, 
    float dx, float dy, float dz, float roll, float pitch, float yaw)
{
  this->CommandArm(mode, curEEPos.x + dx, curEEPos.y + dy, curEEPos.z + dz,
     roll, pitch, yaw );
}

////////////////////////////////////////////////////////////////////////////////
// Got a point cloud
void Hanoi::CloudCB(const tf::MessageNotifier<sensor_msgs::PointCloud>::MessagePtr& cloud)
{
  printf("Got point cloud\n");
  pointcloud_ = *cloud;

  tf_.transformPointCloud(parameter_frame_, pointcloud_, pointcloud_ );

  //ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)pointcloud_.points.size (), pointcloud_.header.frame_id.c_str (), (int)pointcloud_.channels.size (), cloud_geometry::getAvailableChannels (pointcloud_).c_str ());
}

////////////////////////////////////////////////////////////////////////////////
// Move arm to a goal
bool Hanoi::MoveArm()
{
  Point3d pos;

  if (pointcloud_.points.size() <= 0)
  {
    printf("No point cloud.\n");
    return false;
  }

  if (colorTrackStatus_ != "tracking")
  {
    printf("Hasn't seen the target\n");
    return false;
  }


  if (activeDisk_ == "red")
    pos = redPos_;
  else if (activeDisk_ == "green")
    pos = greenPos_;
  else
    pos = bluePos_;

  printf("Planning to[%f %f %f]\n", pos.x, pos.y, pos.z);

  this->CommandArm("plan", pos.x - HAND_OFFSET, pos.y, pos.z + 0.006, M_PI/2.0,0,0 );

  return true;
}



////////////////////////////////////////////////////////////////////////////////
// Blob callback
void Hanoi::BlobCB(const cmvision::BlobsConstPtr &msg)
{
  if (pointcloud_.points.size() <= 0)
    return;

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

  }

  /*printf("Red Size[%d] Pos[%d %d]\n", redBlob_.area, redBlob_.x, redBlob_.y);
  printf("Green Size[%d] Pos[%d %d]\n", greenBlob_.area, greenBlob_.x, greenBlob_.y);
  printf("Blue Size[%d] Pos[%d %d]\n", blueBlob_.area, blueBlob_.x, blueBlob_.y);
  */

  redPos_ = this->GetPoint3d(redBlob_);
  greenPos_ = this->GetPoint3d(greenBlob_);
  bluePos_ = this->GetPoint3d(blueBlob_);

  /*printf("Red Pos[%f %f %f]\n",redPos_.x, redPos_.y, redPos_.z);
  printf("Green Pos[%f %f %f]\n",greenPos_.x, greenPos_.y, greenPos_.z);
  printf("Blue Pos[%f %f %f]\n",bluePos_.x, bluePos_.y, bluePos_.z);
  */

  this->SendMarker(1.0,0.0,0.0, redPos_ );
  this->SendMarker(0.0,1.0,0.0, greenPos_ );
  this->SendMarker(0.0,0.0,1.0, bluePos_ );

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

////////////////////////////////////////////////////////////////////////////////
/// Get the 3d location of a color blob
Point3d Hanoi::GetPoint3d(cmvision::Blob &blob)
{
  Point3d result;

  float dist, minDist;
  int minIndex = 0;
  int closestIndex = 0;
  float minPointX;
  float xvalue, yvalue;

  minPointX = 1000;

  // Use the center of the blob in the x-direction
  unsigned int x = blob.x;

  for (unsigned int y = blob.top; y < blob.bottom; y++)
  {
    minDist = 1000;
    for (unsigned int i=0; i < pointcloud_.channels[1].values.size(); i++)
    {
      yvalue = pointcloud_.channels[2].values[i];
      xvalue = pointcloud_.channels[1].values[i];

      dist = sqrt( pow(xvalue-x, 2) + pow(yvalue-y,2) );
      if (dist < minDist)
      {
        minDist = dist;
        minIndex = i;
      }
    }

    if (minDist < 1.0 && pointcloud_.points[minIndex].x < minPointX)
    {
      minPointX = pointcloud_.points[minIndex].x;
      closestIndex = minIndex;
    }
  }

  result.x = pointcloud_.points[closestIndex].x;
  result.y = pointcloud_.points[closestIndex].y;
  result.z = pointcloud_.points[closestIndex].z;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Send a sphere visualization marker to rviz
void Hanoi::SendMarker(float r, float g, float b, Point3d pos)
{
  int id = 0;

  id = ( ((unsigned char)(r*255.0)) << 16 ) |
       ( ((unsigned char)(g*255.0)) << 8 )  |
       ( ((unsigned char)(b*255.0)));

  printf("ID[%d]\n",id);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "hanoi";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pos.x;
  marker.pose.position.y = pos.y;
  marker.pose.position.z = pos.z;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  visPublisher_.publish( marker );
}

////////////////////////////////////////////////////////////////////////////////
/// Lower a disk into place
void Hanoi::LowerDisk()
{
  float dz = 0;
  float maxZ = 0;
  float minZ = 0.78;//redPos_.z + 0.05;

  if ( (redPos_.y - curEEPos.y) < 0.1 && redPos_.z < curEEPos.z)
    maxZ = redPos_.z + 0.05;

  if ( (greenPos_.y - curEEPos.y) < 0.1 && greenPos_.z < curEEPos.z)
    if (greenPos_.z > maxZ)
      maxZ = greenPos_.z + 0.05;

/*  if ( (bluePos_.y - curEEPos.y) < 0.1 && bluePos_.z < curEEPos.z)
    if (greenPos_.z > maxZ)
      maxZ = bluePos.z + 0.05;
      */

  if (maxZ != 0)
    dz = curEEPos.z - maxZ;
  else
    dz = curEEPos.z - minZ;

  printf("MaxZ[%f] MinZ[%f] Dz[%f]\n", maxZ, minZ, dz);

  this->CommandArmDelta("ik", 0, 0, -dz, M_PI/2.0, 0 , 0);
}
