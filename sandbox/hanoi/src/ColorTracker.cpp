#include <sensor_msgs/JointState.h>
#include "ColorTracker.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor
ColorTracker::ColorTracker(ros::NodeHandle &nh)
  : nodeHandle_(nh)
{
  double start_pan, start_tilt;

  // Subscribe to the blobs topic
  blobSubscriber_ = nodeHandle_.subscribe("blobs", 100, 
      &ColorTracker::BlobCB, this);

  // Subscribe to the blobs topic
  headCmdSubscriber_ = nodeHandle_.subscribe("~/cmd", 10, 
      &ColorTracker::CmdCB, this);

  panStateSubscriber_ = nodeHandle_.subscribe(
      "/head_controller/pan_controller/state",10, 
      &ColorTracker::PanStateCB, this);

  tiltStateSubscriber_ = nodeHandle_.subscribe(
      "/head_controller/tilt_controller/state",10, 
      &ColorTracker::TiltStateCB, this);

  // Create a publisher for outputting status of the tracker
  statusPublisher_ = nodeHandle_.advertise<hanoi::ColorTrackStatus>(
      "~/status",1,true);

  // Publisher that controls the head
  headPublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>(
      "pan_tilt", 1, true);

  if (!nodeHandle_.getParam("~/pan_gain", panGain_))
    panGain_ = 0.05;

  if (!nodeHandle_.getParam("~/tilt_gain", tiltGain_))
    tiltGain_ = 0.05;

  if (!nodeHandle_.getParam("~/start_tilt", start_tilt))
    start_tilt = 0.5;

  if (!nodeHandle_.getParam("~/start_pan", start_pan))
    start_pan = 0.5;

  this->PosHead(start_pan, start_tilt);

  trackRed_ = 0;
  trackGreen_ = 0;
  trackBlue_ = 0;

  hasRed_ = hasGreen_ = hasBlue_ = false;

  updateTimer_ = nodeHandle_.createTimer(ros::Duration(0.1), 
        &ColorTracker::UpdateCB, this);

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ColorTracker::~ColorTracker()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update callback
void ColorTracker::UpdateCB( const ros::TimerEvent &e )
{
  float x, y;
  float newPan = panPos_;
  float newTilt = tiltPos_;

  x = y = -1;
  if (trackRed_ == 1 && trackGreen_ == 0 && trackBlue_ == 0 && hasRed_)
  {
    x = redBlob_.x;
    y = redBlob_.y;
  }
  else if (trackRed_ == 0 && trackGreen_ == 1 && trackBlue_ == 0 && hasGreen_)
  {
    x = greenBlob_.x;
    y = greenBlob_.y;
  }
  else if (trackRed_ == 0 && trackGreen_ == 0 && trackBlue_ == 1 && hasBlue_)
  {
    x = blueBlob_.x;
    y = blueBlob_.y;
  }

  x = (x - imageWidth_/2.0) / (imageWidth_*0.5);
  y = (y - imageHeight_/2.0) / (imageHeight_*0.5);

  hanoi::ColorTrackStatus msg;
  msg.r = trackRed_;
  msg.g = trackGreen_;
  msg.b = trackBlue_;

  if (x > -1.0 && x < 1.0)
  {
    newPan -= x * panGain_;
    newTilt += y * tiltGain_;

    msg.status = "tracking";

    printf("New[%f %f]\n",newPan, newTilt);
    this->PosHead(newPan, newTilt);
  }
  else
  {
    msg.status = "tracking";
  }

  statusPublisher_.publish(msg);
}


////////////////////////////////////////////////////////////////////////////////
// Color track callback
void ColorTracker::CmdCB(const hanoi::ColorTrackCmdConstPtr &msg)
{
  printf("Got Cmd[%d %d %d]\n",msg->red, msg->green, msg->blue);
  trackRed_ = msg->red;
  trackGreen_ = msg->green;
  trackBlue_ = msg->blue;
}

////////////////////////////////////////////////////////////////////////////////
// Blob callback
void ColorTracker::BlobCB(const cmvision::BlobsConstPtr &msg)
{
  if (trackRed_ == 0 && trackGreen_ == 0 && trackBlue_ ==0)
    return;

  hasRed_ = hasBlue_ = hasGreen_ = false;

  redBlob_.area = 0;
  greenBlob_.area = 0;
  blueBlob_.area = 0;

  imageWidth_ = msg->imageWidth;
  imageHeight_ = msg->imageHeight;

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
    {
      redBlob_ = msg->blobs[i];
      hasRed_ = true;
    }
    if (g == 255 && area > greenBlob_.area)
    {
      greenBlob_ = msg->blobs[i];
      hasRed_ = true;
    }
    if (b == 255 && area > blueBlob_.area)
    {
      blueBlob_ = msg->blobs[i];
      hasRed_ = true;
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
// Velocity control of head
void ColorTracker::VelHead(float pan, float tilt)
{
  sensor_msgs::JointState jointstate;

  jointstate.name.push_back("head_pan_joint");
  jointstate.name.push_back("head_tilt_joint");

  jointstate.velocity.push_back(pan);
  jointstate.velocity.push_back(tilt);

  jointstate.effort.push_back(pan);
  jointstate.effort.push_back(tilt);

  headPublisher_.publish( jointstate );
}

////////////////////////////////////////////////////////////////////////////////
/// Pan and tilt the head
void ColorTracker::PosHead(float pan, float tilt)
{
  sensor_msgs::JointState jointstate;

  jointstate.name.push_back("head_pan_joint");
  jointstate.name.push_back("head_tilt_joint");

  jointstate.position.push_back(pan);
  jointstate.position.push_back(tilt);

  headPublisher_.publish( jointstate );
}

////////////////////////////////////////////////////////////////////////////////
void ColorTracker::PanStateCB(const robot_mechanism_controllers::JointControllerStateConstPtr &msg)
{
  panPos_ = msg->set_point;
}

////////////////////////////////////////////////////////////////////////////////
void ColorTracker::TiltStateCB(const robot_mechanism_controllers::JointControllerStateConstPtr &msg)
{
  tiltPos_ = msg->set_point;
}

