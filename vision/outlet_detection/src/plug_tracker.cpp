#include "outlet_detection/plug_tracker.h"

extern "C"
int cvFindChessboardCorners_ex( const void* arr, CvSize pattern_size,
                                CvPoint2D32f* out_corners, int* out_corner_count,
                                int flags );


PlugTracker::PlugTracker(ros::Node &node)
  : TrackerBase(node, "plug"), grid_pts_(NULL)
{
  // Read plug-specific parameters
  double square_size;
  if (!node_.getParam("~square_size", square_size)) {
    ROS_FATAL("Square size unspecified");
    node_.shutdown();
    return;
  }
  
  if (!node_.getParam("~board_width", board_w_)) {
    ROS_FATAL("Board width unspecified");
    node_.shutdown();
    return;
  }

  if (!node_.getParam("~board_height", board_h_)) {
    ROS_FATAL("Board height unspecified");
    node_.shutdown();
    return;
  }

  corners_.resize(board_w_ * board_h_);

  // Set up "true" grid of corner points
  grid_pts_ = cvCreateMat(board_w_ * board_h_, 3, CV_64FC1);
  int j = 0;
  for (int y = 0; y < board_h_; ++y) {
    for (int x = 0; x < board_w_; ++x) {
      cvSetReal2D(grid_pts_, j, 0, x*square_size);
      cvSetReal2D(grid_pts_, j, 1, y*square_size);
      cvSetReal2D(grid_pts_, j, 2, 0.0);
      ++j;
    }
  }

  // Set up transforms
  // TODO: set these through parameters?
  //plug_in_board_.getOrigin().setValue(0.003, -0.01, 0.005);
  //plug_in_board_.getBasis().setValue(0, -1, 0, -1, 0, 0, 0, 0, -1);
  plug_in_board_.getOrigin().setValue(0.00398, -0.01252, 0.00659);
  plug_in_board_.setRotation(btQuaternion(-0.70607, 0.70787, 0.01876, -0.00651));
  camera_in_cvcam_.getOrigin().setValue(0.0, 0.0, 0.0);
  camera_in_cvcam_.getBasis().setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);

  activate();
}

PlugTracker::~PlugTracker()
{
  cvReleaseMat(&grid_pts_);
}

bool PlugTracker::detectObject(tf::Transform &pose)
{
  if (!img_bridge_.fromImage(img_, "mono")) {
    ROS_ERROR("Failed to convert image");
    return false; // throw instead?
  }
  IplImage* image = img_bridge_.toIpl();
  
  if ( !cvFindChessboardCorners_ex(image, cvSize(board_w_, board_h_),
                                   &corners_[0], &ncorners_,
                                   CV_CALIB_CB_ADAPTIVE_THRESH) )
    return false;

  //static const int RADIUS = 5;
  static const int RADIUS = 11;
  cvFindCornerSubPix(image, &corners_[0], ncorners_, cvSize(RADIUS,RADIUS), cvSize(-1,-1),
                     cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

  double rot[3], trans[3];
  CvMat R3, T3, D, img_pts;
  cvInitMatHeader(&R3, 3, 1, CV_64FC1, rot);
  cvInitMatHeader(&T3, 3, 1, CV_64FC1, trans);
  // Assume image already rectified, so distortion coefficients are 0
  double zeros[4] = {0};
  cvInitMatHeader(&D, 1, 4, CV_64FC1, zeros);
  cvInitMatHeader(&img_pts, corners_.size(), 2, CV_32FC1, &corners_[0]);
  
  // Find checkerboard pose
  cvFindExtrinsicCameraParams2(grid_pts_, &img_pts, K_, &D, &R3, &T3);

  // Convert from Rodriguez to quaternion
  double rot3x3_arr[9];
  CvMat rot3x3_cv;
  cvInitMatHeader(&rot3x3_cv, 3, 3, CV_64FC1, rot3x3_arr);
  cvRodrigues2(&R3, &rot3x3_cv);
  btMatrix3x3 rot3x3(rot3x3_arr[0], rot3x3_arr[1], rot3x3_arr[2],
                     rot3x3_arr[3], rot3x3_arr[4], rot3x3_arr[5],
                     rot3x3_arr[6], rot3x3_arr[7], rot3x3_arr[8]);

  tf::Transform board_in_cvcam(rot3x3, tf::Vector3(trans[0], trans[1], trans[2]));

  // Calculate plug pose in the camera frame
  pose = camera_in_cvcam_ * board_in_cvcam * plug_in_board_;
  //pose = board_in_cvcam; // for calibration only!!

  return true;
}

CvRect PlugTracker::getBoundingBox()
{
  CvPoint2D32f board_corners[4];
  board_corners[0] = corners_[0];
  board_corners[1] = corners_[board_w_ - 1];
  board_corners[2] = corners_[(board_h_ - 1) * board_w_];
  board_corners[3] = corners_[corners_.size() - 1];
  float min_x = frame_w_, min_y = frame_h_, max_x = 0, max_y = 0;
  for (int i = 0; i < 4; ++i) {
    min_x = std::min(min_x, board_corners[i].x);
    min_y = std::min(min_y, board_corners[i].y);
    max_x = std::max(max_x, board_corners[i].x);
    max_y = std::max(max_y, board_corners[i].y);
  }
  
  return cvRect(min_x + 0.5f, min_y + 0.5f,
                max_x - min_x + 0.5f, max_y - min_y + 0.5f);
}

IplImage* PlugTracker::getDisplayImage(bool success)
{
  IplImage* image = img_bridge_.toIpl();
  display_img_.Allocate(image->width, image->height);
  cvCvtColor(image, display_img_.Ipl(), CV_GRAY2BGR);
  cvDrawChessboardCorners(display_img_.Ipl(), cvSize(board_w_, board_h_),
                          &corners_[0], ncorners_, success);
  return display_img_.Ipl();
}
