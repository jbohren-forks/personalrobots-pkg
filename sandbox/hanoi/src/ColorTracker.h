#ifndef COLORTRACKER_H
#define COLORTRACKER_H

#include <ros/ros.h>

#include <pr2_mechanism_msgs/JointStates.h>
#include <pr2_mechanism_msgs/JointState.h>
#include <robot_mechanism_controllers/JointControllerState.h>
#include <cmvision/Blobs.h>

#include "hanoi/ColorTrackCmd.h"
#include "hanoi/ColorTrackStatus.h"

class ColorTracker
{
  /// \brief Constructor
  public: ColorTracker(ros::NodeHandle &nh);

  /// \brief Destructor
  public: virtual ~ColorTracker();

  /// \brief Blob callback
  public: void BlobCB(const cmvision::BlobsConstPtr &msg);

  /// \brief Color track callback
  public: void CmdCB(const hanoi::ColorTrackCmdConstPtr &msg);

  public: void PanStateCB(const robot_mechanism_controllers::JointControllerStateConstPtr &msg);

  public: void TiltStateCB(const robot_mechanism_controllers::JointControllerStateConstPtr &msg);

  /// \brief Pan and tilt the head
  private: void MoveHead(float pan, float tilt);

  /// \brief Velocity control of the head
  private: void VelCmd(float pan, float tilt);

  private: ros::NodeHandle nodeHandle_;

  private: ros::Subscriber blobSubscriber_;
  private: ros::Subscriber headCmdSubscriber_;
  private: ros::Subscriber panStateSubscriber_;
  private: ros::Subscriber tiltStateSubscriber_;

  private: ros::Publisher statusPublisher_;
  private: ros::Publisher headPublisher_;

  private: cmvision::Blob redBlob_;
  private: cmvision::Blob greenBlob_;
  private: cmvision::Blob blueBlob_;

  private: unsigned int trackRed_, trackGreen_, trackBlue_;

  private: float panPos_, tiltPos_;

};

#endif
