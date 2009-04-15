#include <ros/node.h>
#include <image_msgs/Image.h>
#include <image_msgs/CamInfo.h>
#include <image_msgs/CvBridge.h>
#include <robot_msgs/PoseStamped.h>
#include <prosilica_cam/PolledImage.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>


// TODO: don't inherit from Node
class PlugTracker : public ros::Node
{
public:
  PlugTracker();
  ~PlugTracker();

  void caminfo_cb();
  void image_cb();

  void spin();

  
private:
  CvRect fitToFrame(CvRect roi);
  void setRoi(CvRect roi);

  prosilica_cam::PolledImage::Request req_;
  prosilica_cam::PolledImage::Response res_;
  image_msgs::Image& img_;
  image_msgs::CamInfo& cam_info_;
  image_msgs::CvBridge img_bridge_;
  robot_msgs::PoseStamped pose_;
  tf::TransformBroadcaster tf_broadcaster_;
  int board_w_, board_h_;
  CvMat *K_, *grid_pts_;

  tf::Transform plug_in_board_, camera_in_cvcam_;

  // TODO: policy to project from current gripper location
  enum { WholeFrame, LastImageLocation } roi_policy_;
  int frame_w_, frame_h_;
  static const float RESIZE_FACTOR_FAILED = 1.2f;
  static const float RESIZE_FACTOR_FOUND = 3.0f;
  
  bool display_;
  IplImage* display_img_;

  static const char wndname[];
};
