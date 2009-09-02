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

  panStateSubscriber_ = nodeHandle_.subscribe("/head_controller/pan_controller/state",10, &ColorTracker::PanStateCB, this);

  tiltStateSubscriber_ = nodeHandle_.subscribe("/head_controller/tilt_controller/state",10, &ColorTracker::TiltStateCB, this);

  // Create a publisher for outputting status of the tracker
  statusPublisher_ = nodeHandle_.advertise<hanoi::ColorTrackStatus>("~/status",1,true);

  // Publisher that controls the head
  headPublisher_ = nodeHandle_.advertise<pr2_mechanism_msgs::MechanismState>("pan_tilt", 1, true);

  if (!nodeHandle_.getParam("~/start_tilt", start_tilt))
    start_tilt = 0.5;

  if (!nodeHandle_.getParam("~/start_pan", start_pan))
    start_pan = 0.5;

  this->PosHead(start_pan, start_tilt);

  trackRed_ = 0;
  trackGreen_ = 0;
  trackBlue_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ColorTracker::~ColorTracker()
{
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

  float x,y;
  float newPan = panPos_;
  float newTilt = tiltPos_;

  redBlob_.x = redBlob_.y = -1;
  greenBlob_.x = greenBlob_.y = -1;
  blueBlob_.x = blueBlob_.y = -1;

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

  x = y = -1;
  if (trackRed_ == 1 && trackGreen_ == 0 && trackBlue_ == 0)
  {
    x = (redBlob_.x - msg->imageWidth/2.0) / (msg->imageWidth*0.5);
    y = (redBlob_.y - msg->imageHeight/2.0) / (msg->imageHeight*0.5);
  }
  else if (trackRed_ == 0 && trackGreen_ == 1 && trackBlue_ == 0)
  {
    x = (greenBlob_.x - msg->imageWidth/2.0) / (msg->imageWidth*0.5);
    y = (greenBlob_.y - msg->imageHeight/2.0) / (msg->imageHeight*0.5);
  }
  else if (trackRed_ == 0 && trackGreen_ == 0 && trackBlue_ == 1)
  {
    x = (blueBlob_.x - msg->imageWidth/2.0) / (msg->imageWidth*0.5);
    y = (blueBlob_.y - msg->imageHeight/2.0) / (msg->imageHeight*0.5);
  }

  //printf("At[%d %d] WH[%d %d] XY[%f %f] XYN[%f %f]\n",redBlob_.x, redBlob_.y, msg->imageWidth, msg->imageHeight, x,y, x/320, y/240);

  if (x > -1.0 && x < 1.0)
  {
    newPan -= x * .2;
    newTilt += y * .2;

    printf("New[%f %f]\n",newPan, newTilt);
    //this->PosHead(newPan, newTilt);
    this->VelHead(x*.2, y*.2);
  }
  else
    this->PosHead(newPan, newTilt+0.2);
}

////////////////////////////////////////////////////////////////////////////////
// Velocity control of head
void ColorTracker::VelHead(float pan, float tilt)
{
  pr2_mechanism_msgs::MechanismState mechState;
  pr2_mechanism_msgs::JointState panState, tiltState;

  panState.name = "head_pan_joint";
  panState.velocity = pan;

  tiltState.name = "head_tilt_joint";
  tiltState.velocity = tilt;

  mechState.joint_states.push_back(panState);
  mechState.joint_states.push_back(tiltState);

  headPublisher_.publish( mechState );
}

////////////////////////////////////////////////////////////////////////////////
/// Pan and tilt the head
void ColorTracker::PosHead(float pan, float tilt)
{
  pr2_mechanism_msgs::MechanismState jointStatesMsg;
  pr2_mechanism_msgs::JointState panState, tiltState;

  panState.name = "head_pan_joint";
  panState.position = pan;

  tiltState.name = "head_tilt_joint";
  tiltState.position = tilt;

  jointStatesMsg.joint_states.push_back(panState);
  jointStatesMsg.joint_states.push_back(tiltState);

  headPublisher_.publish( jointStatesMsg );
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

