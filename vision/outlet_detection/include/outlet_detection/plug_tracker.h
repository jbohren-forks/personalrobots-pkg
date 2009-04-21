#warning License agreement
#ifndef PLUG_TRACKER_H
#define PLUG_TRACKER_H

#include <ros/node.h>
#include <image_msgs/Image.h>
#include <image_msgs/CamInfo.h>
#include <opencv_latest/CvBridge.h>
#include <robot_msgs/PoseStamped.h>
#include <prosilica_cam/PolledImage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <opencv/cv.h>
#include <opencv/cvwimage.h>

#include <boost/thread.hpp>

class PlugTracker
{
public:
  PlugTracker(ros::Node &node);
  ~PlugTracker();

  void activate();
  void deactivate();

  void spin();
  
private:
  void processCamInfo();
  void processImage();

  // TODO: separate out these functions, shared with outlet_node
  CvRect fitToFrame(CvRect roi);
  void setRoi(CvRect roi);
  void setRoiToGripperPosition();

  ros::Node &node_;
  boost::thread active_thread_;
  
  prosilica_cam::PolledImage::Request req_;
  prosilica_cam::PolledImage::Response res_;
  image_msgs::Image& img_;
  image_msgs::CamInfo& cam_info_;
  image_msgs::CvBridge img_bridge_;
  robot_msgs::PoseStamped pose_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  std::string gripper_frame_id_;
  int board_w_, board_h_;
  CvMat *K_, *grid_pts_;

  tf::Transform plug_in_board_, camera_in_cvcam_;

  enum { WholeFrame, LastImageLocation, GripperPosition } roi_policy_;
  int frame_w_, frame_h_;
  static const float RESIZE_FACTOR_FAILED = 1.2f;
  static const float RESIZE_FACTOR_FOUND = 3.0f;
  
  bool display_;
  cv::WImageBuffer3_b display_img_;

  static const char wndname[];
};

#endif
