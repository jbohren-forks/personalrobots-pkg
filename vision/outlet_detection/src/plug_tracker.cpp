#include "outlet_detection/plug_tracker.h"
#include <visualization_msgs/Marker.h>

// FIXME: move back to OpenCV versions whenever they get updated
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

  // Set up transforms
  // Coordinate frame (looking down on plug): (right, -forward, down)
  double pos[3], ori[4];
  if (node_.getParam("~plug_position_x", pos[0]) &&
      node_.getParam("~plug_position_y", pos[1]) &&
      node_.getParam("~plug_position_z", pos[2]) &&
      node_.getParam("~plug_orientation_x", ori[0]) &&
      node_.getParam("~plug_orientation_y", ori[1]) &&
      node_.getParam("~plug_orientation_z", ori[2]) &&
      node_.getParam("~plug_orientation_w", ori[3]) )
  {
    plug_in_board_.getOrigin().setValue(pos[0], pos[1], pos[2]);
    plug_in_board_.setRotation(tf::Quaternion(ori[0], ori[1], ori[2], ori[3]));
  } else {
    ROS_WARN("Plug in board position unspecified, using defaults");
    
    // Measured with caliper
    //plug_in_board_.setOrigin(tf::Vector3(0.007, -0.029, 0.008)); // to tip
    plug_in_board_.setOrigin(tf::Vector3(0.007, -0.0085, 0.015));
    plug_in_board_.setRotation(tf::Quaternion(0.71428, -0.69958, 0.00588, 0.01906));
  }
  prong_in_board_.setOrigin(tf::Vector3(0.007, -0.029, 0.008)); // to tip
  prong_in_board_.setBasis(plug_in_board_.getBasis());

  camera_in_cvcam_.getOrigin().setValue(0.0, 0.0, 0.0);
  camera_in_cvcam_.getBasis().setValue(0, 0, 1, -1, 0, 0, 0, -1, 0);

  // Set up "true" grid of corner points
  corners_.resize(board_w_ * board_h_);
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

  node_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
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

  double rot[3], trans[3];
  CvMat R3, T3, D, img_pts;
  cvInitMatHeader(&R3, 3, 1, CV_64FC1, rot);
  cvInitMatHeader(&T3, 3, 1, CV_64FC1, trans);
  // Assume image already rectified, so distortion coefficients are 0
  double zeros[4] = {0};
  cvInitMatHeader(&D, 1, 4, CV_64FC1, zeros);
  cvInitMatHeader(&img_pts, corners_.size(), 2, CV_32FC1, &corners_[0]);
  double rot3x3_arr[9];
  CvMat rot3x3_cv;
  cvInitMatHeader(&rot3x3_cv, 3, 3, CV_64FC1, rot3x3_arr);

  // Determining the pose of a checkerboard with so few and such small squares is a poorly
  // conditioned problem, so we take some extra precautions against instability.
  // We do a rough estimate without subpixel refinement of the corners, which seems to
  // contribute to the instability. This gives a reliable position, but the orientation
  // may still be wrong due to ambiguity. If TF information from a target frame is
  // available, we substitute the orientation of the target frame in our estimate.
  // Then we refine our estimate by finding subpixel corners and repeating the pose
  // optimization using our rough estimate as the starting point.

  // Find rough checkerboard pose
  //ROS_WARN("Calculating plug pose");
  cvFindExtrinsicCameraParams2(grid_pts_, &img_pts, K_, &D, &R3, &T3, false);
  //ROS_WARN("Initial: T = (%f, %f, %f), R = (%f, %f, %f)", trans[0], trans[1], trans[2], rot[0], rot[1], rot[2]);

  if (roi_policy_ == TargetFrame) {
    // Use target tool frame to inform initial estimate
    robot_msgs::Pose target;
    try {
      target = getTargetInHighDef();
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("Transform Exception %s", ex.what());
      return false;
    }
    tf::Transform target_tf;
    tf::PoseMsgToTF(target, target_tf);

    // Transform to OpenCV coordinate system
    target_tf = camera_in_cvcam_.inverse() * target_tf * plug_in_board_.inverse();
    //trans[0] = target_tf.getOrigin().x();
    //trans[1] = target_tf.getOrigin().y();
    //trans[2] = target_tf.getOrigin().z();

    // Convert to Rodrigues rotation
    btMatrix3x3 &basis = target_tf.getBasis();
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        rot3x3_arr[3*i + j] = basis[i][j];
    cvRodrigues2(&rot3x3_cv, &R3);
    //ROS_WARN("Gripper ori: T = (%f, %f, %f), R = (%f, %f, %f)", trans[0], trans[1], trans[2], rot[0], rot[1], rot[2]);
  } else {
    ROS_WARN("Not using target frame info, orientation may be bogus");
  }

  //static const int RADIUS = 5;
  static const int RADIUS = 11;
  cvFindCornerSubPix(image, &corners_[0], ncorners_, cvSize(RADIUS,RADIUS), cvSize(-1,-1),
                     cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

  // Refine checkerboard pose
  cvFindExtrinsicCameraParams2(grid_pts_, &img_pts, K_, &D, &R3, &T3, true);
  //ROS_WARN("Refined: T = (%f, %f, %f), R = (%f, %f, %f)", trans[0], trans[1], trans[2], rot[0], rot[1], rot[2]);

  // Convert from Rodrigues to quaternion
  cvRodrigues2(&R3, &rot3x3_cv);
  btMatrix3x3 rot3x3(rot3x3_arr[0], rot3x3_arr[1], rot3x3_arr[2],
                     rot3x3_arr[3], rot3x3_arr[4], rot3x3_arr[5],
                     rot3x3_arr[6], rot3x3_arr[7], rot3x3_arr[8]);

  tf::Transform board_in_cvcam(rot3x3, tf::Vector3(trans[0], trans[1], trans[2]));

  // Calculate plug pose in the camera frame
  tf::Transform board_in_cam = camera_in_cvcam_ * board_in_cvcam;
  pose = board_in_cam * plug_in_board_;
  //pose = board_in_cvcam; // for calibration only!!

  publishBoardMarker(board_in_cam);
  publishBoardRayMarker(board_in_cam);
  publishPlugRayMarker(board_in_cam, pose);

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

void PlugTracker::publishBoardMarker(const tf::Transform &board_in_cam)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "high_def_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "plug_detector";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.005;
  marker.scale.y = 0.005;
  marker.scale.z = 0.005;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  int offsets[] = {0, board_w_ - 1, board_h_*board_w_ - 1, (board_h_ - 1)*board_w_, 0};
  for (unsigned i = 0; i < sizeof(offsets)/sizeof(int); ++i) {
    double *grid_pt = grid_pts_->data.db + 3*offsets[i];
    tf::Point corner(grid_pt[0], grid_pt[1], grid_pt[2]);
    tf::Point pt = board_in_cam * corner;
    robot_msgs::Point pt_msg;
    tf::PointTFToMsg(pt, pt_msg);
    marker.points.push_back(pt_msg);
  }
  
  node_.publish("visualization_marker", marker);
}

void PlugTracker::publishBoardRayMarker(const tf::Transform &board_in_cam)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "high_def_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "plug_detector";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.005;
  marker.scale.y = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.points.resize(2); // first is origin
  tf::PointTFToMsg(board_in_cam.getOrigin(), marker.points[1]);

  node_.publish("visualization_marker", marker);
}

void PlugTracker::publishPlugRayMarker(const tf::Transform &board_in_cam,
                                       const tf::Transform &plug_pose)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "high_def_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "plug_detector";
  marker.id = 2;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.005;
  marker.scale.y = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  marker.points.resize(2);
  tf::PointTFToMsg(board_in_cam.getOrigin(), marker.points[0]);
  tf::PointTFToMsg(plug_pose.getOrigin(), marker.points[1]);

  node_.publish("visualization_marker", marker);

  marker.id = 3;
  tf::PointTFToMsg((board_in_cam * prong_in_board_).getOrigin(), marker.points[1]);
  node_.publish("visualization_marker", marker);
}
