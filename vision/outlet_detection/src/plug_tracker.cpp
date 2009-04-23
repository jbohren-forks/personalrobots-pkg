#include "outlet_detection/plug_tracker.h"
#include "outlet_detection/outlet_detector.h" // TODO: separate out ROI stuff

#include <opencv/highgui.h>

#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

#include <boost/bind.hpp>

// TODO: don't use "high_def_frame" explicitly

extern "C"
int cvFindChessboardCorners_ex( const void* arr, CvSize pattern_size,
                                CvPoint2D32f* out_corners, int* out_corner_count,
                                int flags );


PlugTracker::PlugTracker(ros::Node &node)
  : node_(node), img_(res_.image), cam_info_(res_.cam_info),
    tf_broadcaster_(node), tf_listener_(node),
    K_(NULL), grid_pts_(NULL)
{
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

  node_.param("~image_service", image_service_, std::string("/prosilica/poll"));

  std::string policy;
  node_.param("~roi_policy", policy, std::string("WholeFrame"));
  if (policy == std::string("WholeFrame"))
    roi_policy_ = WholeFrame;
  else if (policy == std::string("LastImageLocation"))
    roi_policy_ = LastImageLocation;
  else if (policy == std::string("GripperPosition")) {
    roi_policy_ = GripperPosition;
    node_.param("~gripper_frame", gripper_frame_id_, std::string(""));
  } else {
    ROS_FATAL("Unknown ROI policy setting");
    node_.shutdown();
    return;
  }

  req_.timeout_ms = 100;

  node_.param("~display", display_, true);
  if (display_) {
    cvNamedWindow(wndname, 0); // no autosize
    cvStartWindowThread();
  }

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
  
  //plug_in_board_.getOrigin().setValue(0.003, -0.01, 0.005);
  //plug_in_board_.getBasis().setValue(0, -1, 0, -1, 0, 0, 0, 0, -1);
  plug_in_board_.getOrigin().setValue(0.00398, -0.01252, 0.00659);
  plug_in_board_.setRotation(btQuaternion(-0.70607, 0.70787, 0.01876, -0.00651));
  camera_in_cvcam_.getOrigin().setValue(0.0, 0.0, 0.0);
  camera_in_cvcam_.getBasis().setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);
  
  node_.advertise<robot_msgs::PoseStamped>("~pose", 1);

  activate();
}

PlugTracker::~PlugTracker()
{
  node_.unadvertise("~pose");
  
  if (active_thread_.joinable()) {
    active_thread_.interrupt();
    active_thread_.join();
  }
  
  cvReleaseMat(&K_);
  cvReleaseMat(&grid_pts_);
  if (display_)
    cvDestroyWindow(wndname);
}

void PlugTracker::activate()
{
  boost::thread t(boost::bind(&PlugTracker::spin, this));
  active_thread_.swap(t);
}

void PlugTracker::deactivate()
{
  active_thread_.interrupt();
}

void PlugTracker::processCamInfo()
{
  if (K_ == NULL)
    K_ = cvCreateMat(3, 3, CV_64FC1);
  memcpy((char*)(K_->data.db), (char*)(&cam_info_.K[0]), 9 * sizeof(double));
  frame_w_ = cam_info_.width;
  frame_h_ = cam_info_.height;
}

void PlugTracker::processImage()
{
  if (!img_bridge_.fromImage(img_, "mono")) {
    ROS_ERROR("Failed to convert image");
    return;
  }

  IplImage* image = img_bridge_.toIpl();
  int ncorners;
  std::vector<CvPoint2D32f> corners(board_w_ * board_h_);
  int found = cvFindChessboardCorners_ex(image, cvSize(board_w_, board_h_),
                                         &corners[0], &ncorners,
                                         CV_CALIB_CB_ADAPTIVE_THRESH);
  if (!found) {
    //ROS_WARN("Failed to detect plug, found %d corners", ncorners);
    
    // Expand ROI for next image
    if (roi_policy_ == LastImageLocation) {
      CvRect outlet_roi = cvRect(req_.region_x, req_.region_y,
                                 req_.width, req_.height);
      outlet_roi = fitToFrame(resize_rect(outlet_roi, RESIZE_FACTOR_FAILED));
      setRoi(outlet_roi);
    }
    
    if (display_) {
      // TODO: draw found corners
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
  double rot3x3_arr[9];
  CvMat rot3x3_cv;
  cvInitMatHeader(&rot3x3_cv, 3, 3, CV_64FC1, rot3x3_arr);
  cvRodrigues2(&R3, &rot3x3_cv);
  btMatrix3x3 rot3x3(rot3x3_arr[0], rot3x3_arr[1], rot3x3_arr[2],
                     rot3x3_arr[3], rot3x3_arr[4], rot3x3_arr[5],
                     rot3x3_arr[6], rot3x3_arr[7], rot3x3_arr[8]);

  tf::Transform board_in_cvcam(rot3x3, tf::Vector3(trans[0], trans[1], trans[2]));

  // Plug pose in the camera frame
  tf::Transform plug_in_camera = camera_in_cvcam_ * board_in_cvcam * plug_in_board_;
  //tf::Transform plug_in_camera = board_in_cvcam; // for calibration only!!

  tf::PoseTFToMsg(plug_in_camera, pose_.pose);
  pose_.header.frame_id = "high_def_frame";
  pose_.header.stamp = img_.header.stamp;
  node_.publish("~pose", pose_);
  tf_broadcaster_.sendTransform(plug_in_camera,
                                ros::Time::now(), "plug_frame",
                                "high_def_frame");

  // Calculate ROI for next image request
  if (roi_policy_ == LastImageLocation) {
    CvPoint2D32f board_corners[4];
    board_corners[0] = corners[0];
    board_corners[1] = corners[board_w_ - 1];
    board_corners[2] = corners[(board_h_ - 1) * board_w_];
    board_corners[3] = corners[corners.size() - 1];
    float min_x = frame_w_, min_y = frame_h_, max_x = 0, max_y = 0;
    for (int i = 0; i < 4; ++i) {
      min_x = std::min(min_x, board_corners[i].x);
      min_y = std::min(min_y, board_corners[i].y);
      max_x = std::max(max_x, board_corners[i].x);
      max_y = std::max(max_y, board_corners[i].y);
    }
    CvRect outlet_roi = cvRect(min_x + 0.5f, min_y + 0.5f,
                               max_x - min_x + 0.5f, max_y - min_y + 0.5f);
    outlet_roi.x += req_.region_x;
    outlet_roi.y += req_.region_y;
    outlet_roi = fitToFrame(resize_rect(outlet_roi, RESIZE_FACTOR_FOUND));
    setRoi(outlet_roi);
  }
  /*
  ROS_INFO("Plug: %.5f %.5f %.5f", pose_.pose.position.x,
           pose_.pose.position.y, pose_.pose.position.z);
  */
  if (display_) {
    display_img_.Allocate(image->width, image->height);
    cvCvtColor(image, display_img_.Ipl(), CV_GRAY2BGR);
    cvDrawChessboardCorners(display_img_.Ipl(), cvSize(board_w_, board_h_),
                            &corners[0], ncorners, 1);
    cvShowImage(wndname, display_img_.Ipl());
  }
}

void PlugTracker::spin()
{
  while (node_.ok() && !boost::this_thread::interruption_requested())
  {
    if (roi_policy_ == GripperPosition)
      setRoiToGripperPosition();
    
    if (ros::service::call(image_service_, req_, res_)) {
      processCamInfo();
      processImage();
      usleep(100000);
    } else {
      //ROS_WARN("Service call failed");
      // TODO: wait for service to be available
      usleep(100000);
    }
  }
}

CvRect PlugTracker::fitToFrame(CvRect roi)
{
  CvRect fit;
  fit.x = std::max(roi.x, 0);
  fit.y = std::max(roi.y, 0);
  fit.width = std::min(roi.x + roi.width, frame_w_) - fit.x;
  fit.height = std::min(roi.y + roi.height, frame_h_) - fit.y;
  return fit;
}

inline void PlugTracker::setRoi(CvRect roi)
{
  req_.region_x = roi.x;
  req_.region_y = roi.y;
  req_.width = roi.width;
  req_.height = roi.height;
}

void PlugTracker::setRoiToGripperPosition()
{
  // Get gripper position in high def frame
  robot_msgs::PointStamped origin, gripper_in_high_def;
  origin.header.frame_id = gripper_frame_id_;
  origin.header.stamp = img_.header.stamp;
  tf_listener_.transformPoint("high_def_frame", origin, gripper_in_high_def);

  // Get intrinsic matrix of full frame (translate principal point)
  K_->data.db[2] += req_.region_x;
  K_->data.db[5] += req_.region_y;
  
  // Project to image coordinates
  CvPoint3D64f object_pt = cvPoint3D64f(-gripper_in_high_def.point.y,
                                        -gripper_in_high_def.point.z,
                                        gripper_in_high_def.point.x);
  CvMat object_points = cvMat(1, 1, CV_64FC3, &object_pt.x);
  double zeros[] = {0, 0, 0};
  CvMat rotation_vector = cvMat(3, 1, CV_64FC1, zeros);
  CvMat translation_vector = cvMat(3, 1, CV_64FC1, zeros);
  double point[2];
  CvMat image_points = cvMat(1, 1, CV_64FC2, point);
  cvProjectPoints2(&object_points, &rotation_vector, &translation_vector,
                   K_, NULL, &image_points);
  
  // Request ROI around projected point
  static const int ROI_SIZE = 400;
  CvRect outlet_roi = cvRect(point[0] - ROI_SIZE/2, point[1] - ROI_SIZE/2,
                             ROI_SIZE, ROI_SIZE);
  setRoi( fitToFrame(outlet_roi) );
}

const char PlugTracker::wndname[] = "Plug tracker";
