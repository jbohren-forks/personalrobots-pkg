#ifndef HANOI_H
#define HANOI_H

#include <map>
#include <vector>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm/MoveArmAction.h>
#include <move_arm/ActuateGripperAction.h>

#include <cmvision/Blobs.h>
#include <tf/message_notifier.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>
#include <pr2_mechanism_msgs/JointStates.h>
#include <pr2_mechanism_msgs/JointState.h>

#include "hanoi/Cylinders.h"
#include "hanoi/ColorTrackCmd.h"
#include "hanoi/ColorTrackStatus.h"

#define HAND_OFFSET 0.25

class Hanoi
{
  enum State {MOVE_ARM, MOVING_ARM, OPEN_GRIPPER, OPENING_GRIPPER,
              MOVE_OFFSET,
              CLOSE_GRIPPER, CLOSING_GRIPPER};

  /// \brief Constructor
  public: Hanoi(ros::NodeHandle &nh);

  /// \brief Destructor
  public: virtual ~Hanoi();

  /// \brief Activate an action to perform
  public: void ActivateAction(const std::string actionName);

  /// \brief Update callback
  private: void UpdateCB( const ros::TimerEvent &e ); 

  /// \brief Got a point cloud
  private: void CloudCB(const tf::MessageNotifier<sensor_msgs::PointCloud>::MessagePtr& cloud);
 
  /// \brief Blob callback
  public: void BlobCB(const cmvision::BlobsConstPtr &msg);
          
  /// \brief Joint state callback
  public: void JointStatesCB(const pr2_mechanism_msgs::JointStatesConstPtr &msg);

  /// \brief Move arm result callback
  public: void MoveArmResultCB(const move_arm::MoveArmResultConstPtr &msg);

  /// \brief Color track status callback
  public: void ColorTrackStatusCB(const hanoi::ColorTrackStatusConstPtr &msg);

  /// \brief Calculate the 3D grasp points
  private: bool CalculateGraspPoints();

  /// \brief Command arm
  private: bool CommandArm(const std::string &mode, float x, float y, float z, 
                           float roll, float pitch, float yaw);

  /// \brief Move arm to a goal
  private: bool MoveArm();

  /// \brief Return true if the arm has reached its goal
  private: bool ArmAtGoal();

  /// \brief Open the gripper
  private: void OpenGripper();

  /// \brief Close the gripper
  private: void CloseGripper();

  /// \brief Return true if the gripper is open
  private: bool GripperIsOpen();

  /// \brief Return true if the gripper is closed
  private: bool GripperIsClosed();


  private: ros::NodeHandle nodeHandle_;
  private: tf::TransformListener tf_;

  private: ros::Subscriber blobSubscriber_;
  private: ros::Subscriber cloudSubscriber_;
  private: ros::Subscriber jointStateSubscriber_;
  private: ros::Subscriber moveArmResultSubscriber_;
  private: ros::Subscriber colorTrackerStatusSubscriber_;

  private: ros::Publisher cylinderPublisher_;
  private: ros::Publisher positionArmPublisher_;
  private: ros::Publisher colorTrackPublisher_;

  private: ros::ServiceClient moveArmService_;

  private: cmvision::Blob redBlob_;
  private: cmvision::Blob greenBlob_;
  private: cmvision::Blob blueBlob_;

  private: bool hasPointCloud_;
  private: sensor_msgs::PointCloud pointcloud_;
  private: pr2_mechanism_msgs::JointStates jointstates_;

  private: std::string cloud_topic_;
  private: std::string parameter_frame_, fixed_frame_;

  private: hanoi::Cylinders cylinderMessage_;

  private: geometry_msgs::PointStamped armGoal_;

  private: ros::Timer updateTimer_;

  private: std::string currentAction_;
  private: std::vector<State>::iterator stateIter_;
  private: std::map< std::string, std::vector<State> > actions_;

  private: actionlib::SimpleActionClient<move_arm::ActuateGripperAction> *gripperAction_;

  private: move_arm::MoveArmResult moveArmResult_;
  private: ros::Time moveArmGoalId_;

  private: unsigned int colorTrackStatus_;
};

#endif
