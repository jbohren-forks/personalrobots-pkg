#include <ros/node.h>
#include <image_msgs/Image.h>
#include <image_msgs/CamInfo.h>
#include <image_msgs/CvBridge.h>
#include <robot_msgs/PoseStamped.h>
#include <prosilica_cam/PolledImage.h>
#include <tf/transform_broadcaster.h>

#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread.hpp>

extern "C"
int cvFindChessboardCorners_ex( const void* arr, CvSize pattern_size,
                                CvPoint2D32f* out_corners, int* out_corner_count,
                                int flags );

// TODO: make these parameters
static const double SQUARE_SIZE = 0.004;
static const int BOARD_W = 3;
static const int BOARD_H = 4;

class PlugDetector : public ros::Node
{
private:
  static const char wndname[];

  prosilica_cam::PolledImage::Request req_;
  prosilica_cam::PolledImage::Response res_;
  image_msgs::Image& img_;
  image_msgs::CamInfo& cam_info_;
  image_msgs::CvBridge img_bridge_;
  robot_msgs::PoseStamped pose_;
  tf::TransformBroadcaster tf_broadcaster_;
  CvMat *K_, *grid_pts_;
  btQuaternion rotation_;
  //boost::mutex cb_mutex_;
  bool display_;
  IplImage* display_img_;

public:
  PlugDetector()
    : ros::Node("plug_detector"), img_(res_.image), cam_info_(res_.cam_info),
      tf_broadcaster_(*this), K_(NULL), display_img_(NULL)
  {
    param("display", display_, false);
    if (display_) {
      cvNamedWindow(wndname, CV_WINDOW_AUTOSIZE);
      cvStartWindowThread();
    }

    grid_pts_ = cvCreateMat(BOARD_W * BOARD_H, 3, CV_64FC1);
    int j = 0;
    for (int y = 0; y < BOARD_H; ++y) {
      for (int x = 0; x < BOARD_W; ++x) {
        cvSetReal2D(grid_pts_, j, 0, x*SQUARE_SIZE);
        cvSetReal2D(grid_pts_, j, 1, y*SQUARE_SIZE);
        cvSetReal2D(grid_pts_, j, 2, 0.0);
        ++j;
      }
    }

    btMatrix3x3 mat( 0,  0,  1,
                    -1,  0,  0,
                     0, -1,  0 );
    mat.getRotation(rotation_);
    
    //subscribe("Image", img_, &PlugDetector::image_cb, this, 1);
    //subscribe("CamInfo", cam_info_, &PlugDetector::caminfo_cb, this, 1);
    advertise<robot_msgs::PoseStamped>("pose", 1);
  }

  ~PlugDetector()
  {
    cvReleaseMat(&K_);
    cvReleaseMat(&grid_pts_);
    cvReleaseImage(&display_img_);
    if (display_)
      cvDestroyWindow(wndname);
  }

  void caminfo_cb()
  {
    //boost::mutex::scoped_lock lock(cb_mutex_);
    if (K_ == NULL)
      K_ = cvCreateMat(3, 3, CV_64FC1);
    memcpy((char*)(K_->data.db), (char*)(&cam_info_.K[0]), 9 * sizeof(double));
  }

  void image_cb()
  {
    /*
    boost::mutex::scoped_lock lock(cb_mutex_);
    if (K_ == NULL) {
      ROS_WARN("Need calibration info to process image");
      return;
    }
    */

    if (!img_bridge_.fromImage(img_, "mono")) {
      ROS_ERROR("Failed to convert image");
      return;
    }

    IplImage* image = img_bridge_.toIpl();
    int ncorners;
    std::vector<CvPoint2D32f> corners(BOARD_W * BOARD_H);
    int found = cvFindChessboardCorners_ex(image, cvSize(BOARD_W, BOARD_H),
                                           &corners[0], &ncorners,
                                           CV_CALIB_CB_ADAPTIVE_THRESH);
    if (!found) {
      ROS_WARN("Failed to detect plug, found %d corners", ncorners);
      if (display_) {
        // TODO: draw
        cvShowImage(wndname, image);
      }
      return;
    }

    cvFindCornerSubPix(image, &corners[0], ncorners, cvSize(11,11), cvSize(-1,-1),
                       cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

    double rot[3], trans[3];
    CvMat R3, T3, D, img_pts;
    cvInitMatHeader(&R3, 3, 1, CV_64FC1, rot);
    cvInitMatHeader(&T3, 3, 1, CV_64FC1, trans);
    // Assume image already rectified, so distortion coefficients are 0
    double zeros[4] = {0};
    cvInitMatHeader(&D, 1, 4, CV_64FC1, zeros);
    cvInitMatHeader(&img_pts, corners.size(), 2, CV_32FC1, &corners[0]);

    // Find checkerboard pose
    cvFindExtrinsicCameraParams2(grid_pts_, &img_pts, K_, &D, &R3, &T3);

    // Convert from Rodriguez to quaternion
    btQuaternion orientation;
    double fang = sqrt(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2]);
    if( fang < 1e-6 )
      orientation.setValue(0, 0, 0, 1);
    else {
      double fmult = sin(fang/2)/fang;
      orientation.setValue(rot[0]*fmult, rot[1]*fmult, rot[2]*fmult, cos(fang/2));
    }

    // Rotate into x-forward frame
    btVector3 position(trans[2], -trans[0], -trans[1]);
    orientation *= rotation_;

    memcpy(&pose_.pose.position.x, &position[0], 3*sizeof(double));
    pose_.pose.orientation.w = orientation.w();
    pose_.pose.orientation.x = orientation.x();
    pose_.pose.orientation.y = orientation.y();
    pose_.pose.orientation.z = orientation.z();
    
    pose_.header.frame_id = "high_def_frame";
    publish("pose", pose_);
    tf_broadcaster_.sendTransform(tf::Transform(orientation, position),
                                  ros::Time::now(), "plug_frame",
                                  "high_def_frame");
    
    ROS_INFO("Plug: %.5f %.5f %.5f", pose_.pose.position.x,
             pose_.pose.position.y, pose_.pose.position.z);
    
    if (display_) {
      if (!display_img_ || display_img_->width != image->width ||
          display_img_->height != image->height) {
        cvReleaseImage(&display_img_);
        display_img_ = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
      }
      cvCvtColor(image, display_img_, CV_GRAY2BGR);
      cvDrawChessboardCorners(display_img_, cvSize(BOARD_W, BOARD_H),
                              &corners[0], ncorners, 1);      
      cvShowImage(wndname, display_img_);
    }
  }

  bool spin()
  {
    while (ok())
    {
      req_.timeout_ms = 100;
      if (ros::service::call("/prosilica/poll", req_, res_)) {
        caminfo_cb();
        image_cb();
      } else {
        ROS_WARN("Service call failed");
        usleep(100000);
      }
    }

    return true;
  }
};

const char PlugDetector::wndname[] = "Plug detector";

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  PlugDetector detector;
  detector.spin();

  return 0;
}
