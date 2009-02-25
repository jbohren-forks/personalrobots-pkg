#include <ros/node.h>
#include <image_msgs/Image.h>
#include <image_msgs/CamInfo.h>
#include <image_msgs/CvBridge.h>
#include <robot_msgs/PoseStamped.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread.hpp>

#include "outlet_detector.h"

class OutletDetector : public ros::Node
{
private:
  static const char wndname[];
  
  image_msgs::Image img_;
  image_msgs::CamInfo cam_info_;
  image_msgs::CvBridge img_bridge_;
  robot_msgs::PoseStamped pose_;
  CvMat* K_;
  boost::mutex cb_mutex_;
  bool display_;

public:
  OutletDetector() : ros::Node("outlet_detector"), K_(NULL)
  {
    param("~display", display_, false);
    if (display_) {
      cvNamedWindow(wndname, CV_WINDOW_AUTOSIZE);
      cvStartWindowThread();
    }
    
    subscribe(mapName("prosilica") + "/image_rect", img_,
              &OutletDetector::image_cb, this, 1);
    subscribe(mapName("prosilica") + "/cam_info", cam_info_,
              &OutletDetector::caminfo_cb, this, 1);
    advertise<robot_msgs::PoseStamped>("~outlet_pose", 1);
  }

  ~OutletDetector()
  {
    cvReleaseMat(&K_);
    if (display_)
      cvDestroyWindow(wndname);
  }

  void caminfo_cb()
  {
    boost::mutex::scoped_lock lock(cb_mutex_);
    if (K_ == NULL)
      K_ = cvCreateMat(3, 3, CV_64FC1);
    memcpy((char*)(K_->data.db), (char*)(&cam_info_.K[0]), 9 * sizeof(double));
  }

  void image_cb()
  {
    boost::mutex::scoped_lock lock(cb_mutex_);
    if (K_ == NULL) {
      ROS_WARN("Need calibration info to process image");
      return;
    }

    if (!img_bridge_.fromImage(img_, "bgr")) {
      ROS_ERROR("Failed to convert image");
      return;
    }

    IplImage* image = img_bridge_.toIpl();
    std::vector<outlet_t> outlets;
    if (!detect_outlet_tuple(image, K_, NULL, outlets)) {
      ROS_WARN("Failed to detect outlet");
      return;
    }

    CvPoint3D32f holes[12];
    get_outlet_coordinates(outlets[0], holes);
    get_outlet_coordinates(outlets[1], holes + 3);
    get_outlet_coordinates(outlets[2], holes + 6);
    get_outlet_coordinates(outlets[3], holes + 9);

    // Transform into expected coordinate frame
    for (CvPoint3D32f* hole = holes; hole != holes + 12; ++hole) {
      float new_x = hole->z;
      hole->z = -hole->y;
      hole->y = -hole->x;
      hole->x = new_x;
    }

    // TODO: fit plane to all holes, publish normal
    
    //pose_.header.stamp =
    pose_.pose.position.x = holes[0].x;
    pose_.pose.position.y = holes[0].y;
    pose_.pose.position.z = holes[0].z;

    publish("~outlet_pose", pose_);
    
    if (display_) {
      draw_outlets(image, outlets);
      cvShowImage(wndname, image);
    }
  }
};

const char OutletDetector::wndname[] = "Outlet detector";

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  OutletDetector detector;
  detector.spin();

  return 0;
}
