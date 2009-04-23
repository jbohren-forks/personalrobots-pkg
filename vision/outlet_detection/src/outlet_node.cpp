#include "outlet_detection/outlet_tracker.h"

#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

#include <opencv/highgui.h>

#include <Eigen/Core>
#include <Eigen/QR>

static void changeAxes(CvPoint3D32f src, btVector3 &dst);
static btVector3 fitPlane(btVector3 *holes, int num_holes);

OutletTracker::OutletTracker(ros::Node &node)
  : node_(node), img_(res_.image), cam_info_(res_.cam_info),
    tf_broadcaster_(node_), K_(NULL)
{
  node_.param("~image_service", image_service_, std::string("/prosilica/poll"));
  node_.param("~display", display_, true);

  std::string policy;
  node_.param("~roi_policy", policy, std::string("WholeFrame"));
  if (policy == std::string("WholeFrame"))
    roi_policy_ = WholeFrame;
  else if (policy == std::string("LastImageLocation"))
    roi_policy_ = LastImageLocation;
  else {
    ROS_FATAL("Unknown ROI policy setting");
    node_.shutdown();
  }

  req_.timeout_ms = 100;

  if (display_) {
    cvNamedWindow(wndname, 0); // no autosize
    cvStartWindowThread();
  }
  
  node_.advertise<robot_msgs::PoseStamped>("~pose", 1);

  activate();
}

OutletTracker::~OutletTracker()
{
  node_.unadvertise("~pose");
  
  cvReleaseMat(&K_);
  if (display_)
    cvDestroyWindow(wndname);
}

void OutletTracker::activate()
{
  boost::thread t(boost::bind(&OutletTracker::spin, this));
  active_thread_.swap(t);
}

void OutletTracker::deactivate()
{
  active_thread_.interrupt();
}

void OutletTracker::processCamInfo()
{
  if (K_ == NULL)
    K_ = cvCreateMat(3, 3, CV_64FC1);
  memcpy(K_->data.db, &cam_info_.K[0], 9 * sizeof(double));
  
  frame_w_ = cam_info_.width;
  frame_h_ = cam_info_.height;
}

void OutletTracker::processImage()
{
  robot_msgs::PoseStamped pose;
  
  if (detectOutlet(pose)) {
    node_.publish("~pose", pose);
    
    // Recenter ROI for next image request
    if (roi_policy_ == LastImageLocation) {
      CvRect tuple_roi[4];
      for (int i = 0; i < 4; i++)
        tuple_roi[i] = outlet_rect(outlets_[i]);
      CvRect outlet_roi;
      calc_bounding_rect(4, tuple_roi, outlet_roi);
      outlet_roi.x += req_.region_x;
      outlet_roi.y += req_.region_y;
      outlet_roi = fitToFrame(resize_rect(outlet_roi, RESIZE_FACTOR));
      setRoi(outlet_roi);
    }
  }
  else {
    // Expand ROI for next image
    if (roi_policy_ == LastImageLocation) {
      CvRect outlet_roi = cvRect(req_.region_x, req_.region_y,
                                 req_.width, req_.height);
      outlet_roi = fitToFrame(resize_rect(outlet_roi, RESIZE_FACTOR));
      setRoi(outlet_roi);
    }
  }
}
  
bool OutletTracker::detectOutlet(robot_msgs::PoseStamped &pose)
{
  if (!img_bridge_.fromImage(img_, "bgr")) {
    ROS_ERROR("Failed to convert image");
    return false;
  }

  IplImage* image = img_bridge_.toIpl();
  outlets_.clear();
  if (!detect_outlet_tuple(image, K_, NULL, outlets_)) {
    //ROS_WARN("Failed to detect outlet");
    if (display_)
      cvShowImage(wndname, image);
    return false;
  }

  // Change data representation and coordinate frame
  btVector3 holes[12];
  for (int i = 0; i < 4; ++i) {
    changeAxes(outlets_[i].coord_hole_ground, holes[3*i]);
    changeAxes(outlets_[i].coord_hole1, holes[3*i+1]);
    changeAxes(outlets_[i].coord_hole2, holes[3*i+2]);
  }

  // Fit normal and right vectors to 12 socket holes
  btVector3 right = ((holes[3] - holes[0]).normalized() +
                     (holes[4] - holes[1]).normalized() +
                     (holes[5] - holes[2]).normalized() +
                     (holes[6] - holes[9]).normalized() +
                     (holes[7] - holes[10]).normalized() +
                     (holes[8] - holes[11]).normalized()) /= 6.0;
  btVector3 normal = fitPlane(holes, 12);

  // Compute full pose
  btVector3 up = right.cross(normal).normalized();
  btMatrix3x3 rotation;
  rotation[0] = normal; // x
  rotation[1] = -right; // y
  rotation[2] = up;     // z
  rotation = rotation.transpose();
  btQuaternion orientation;
  rotation.getRotation(orientation);
  tf::Transform outlet_pose(orientation, holes[0]);
  
  // Publish to TF
  tf_broadcaster_.sendTransform(outlet_pose, ros::Time::now(),
                                "outlet_frame", "high_def_frame");

  // Fill returned pose
  tf::PoseTFToMsg(outlet_pose, pose.pose);
  pose.header.frame_id = "high_def_frame";
  pose.header.stamp = img_.header.stamp;

  /*
  ROS_INFO("Ground TL: %.5f %.5f %.5f, Ground TR: %.5f %.5f %.5f, "
           "Ground BR: %.5f %.5f %.5f, Ground BL: %.5f %.5f %.5f",
           holes[0].x(), holes[0].y(), holes[0].z(),
           holes[3].x(), holes[3].y(), holes[3].z(),
           holes[6].x(), holes[6].y(), holes[6].z(),
           holes[9].x(), holes[9].y(), holes[9].z());
  */
  
  if (display_) {
    draw_outlets(image, outlets_);
    cvShowImage(wndname, image);
  }

  return true;
}
  
void OutletTracker::spin()
{
  while (node_.ok() && !boost::this_thread::interruption_requested())
  {
    if (ros::service::call(image_service_, req_, res_)) {
      processCamInfo();
      processImage();
      // TODO: figure out what's actually causing banding
      usleep(100000); // hack to (mostly) get rid of banding
    } else {
      //ROS_WARN("Service call failed");
      // TODO: wait for service to become available
      usleep(100000);
    }
  }
}

static void changeAxes(CvPoint3D32f src, btVector3 &dst)
{
  // Scale to meters while we're at it
  dst.setValue(src.z / 1000.0, -src.x / 1000.0, -src.y / 1000.0);
}

// TODO: does fitHyperplane in <Eigen/LeastSquares> basically do this?
static btVector3 fitPlane(btVector3 *holes, int num_holes)
{
  // Compute centroid
  btVector3 centroid(0.0, 0.0, 0.0);
  for (int i = 0; i < num_holes; ++i)
    centroid += holes[i];
  double scaling = 1.0 / num_holes;
  centroid *= scaling;
  
  // Compute moments
  double sumXX = 0.0, sumXY = 0.0, sumXZ = 0.0;
  double sumYY = 0.0, sumYZ = 0.0, sumZZ = 0.0;
  for (int i = 0; i < num_holes; ++i) {
    btVector3 diff = holes[i] - centroid;
    sumXX += diff.x()*diff.x();
    sumXY += diff.x()*diff.y();
    sumXZ += diff.x()*diff.z();
    sumYY += diff.y()*diff.y();
    sumYZ += diff.y()*diff.z();
    sumZZ += diff.z()*diff.z();
  }
  sumXX *= scaling;
  sumXY *= scaling;
  sumXZ *= scaling;
  sumYY *= scaling;
  sumYZ *= scaling;
  sumZZ *= scaling;

  // Fill covariance matrix
  Eigen::Matrix3d mat;
  mat << sumXX, sumXY, sumXZ,
         sumXY, sumYY, sumYZ,
         sumXZ, sumYZ, sumZZ;

  // Normal is the eigenvector with the smallest eigenvalue
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(mat);
  Eigen::Vector3d normal = solver.eigenvectors().col(0);

  return btVector3(normal.x(), normal.y(), normal.z());
}

CvRect OutletTracker::fitToFrame(CvRect roi)
{
  CvRect fit;
  fit.x = std::max(roi.x, 0);
  fit.y = std::max(roi.y, 0);
  fit.width = std::min(roi.x + roi.width, frame_w_) - fit.x;
  fit.height = std::min(roi.y + roi.height, frame_h_) - fit.y;
  return fit;
}

void OutletTracker::setRoi(CvRect roi)
{
  req_.region_x = roi.x;
  req_.region_y = roi.y;
  req_.width = roi.width;
  req_.height = roi.height;
}

const char OutletTracker::wndname[] = "Outlet tracker";

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node node("outlet_detector");
  OutletTracker tracker(node);
  
  node.spin();

  return 0;
}

/*
bool detectOutletService(outlet_detection::OutletDetection::Request &od_req,
                         outlet_detection::OutletDetection::Response &od_res)
{
  // Transform coarse estimate to high-def camera frame
  robot_msgs::PointStamped highdef_pt;
  try {
    tf_listener_.transformPoint("high_def_frame", od_req.point, highdef_pt);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Transform exception: %s\n", ex.what());
    return false;
  }
  
  // First try for speed: use ROI around projected estimate from request
  // TODO: don't have K the first time, query for it somehow...
  if (K_) {
    // Get intrinsic matrix of full frame (translate principal point)
    K_->data.db[2] += req_.region_x;
    K_->data.db[5] += req_.region_y;
    
    // Project point onto image
    CvPoint3D64f object_pt = cvPoint3D64f(-od_req.point.point.y,
                                          -od_req.point.point.z,
                                          od_req.point.point.x);
    CvMat object_points = cvMat(1, 1, CV_64FC3, &object_pt.x);
    double zeros[] = {0, 0, 0};
    CvMat rotation_vector = cvMat(3, 1, CV_64FC1, zeros);
    CvMat translation_vector = cvMat(3, 1, CV_64FC1, zeros);
    double point[2];
    CvMat image_points = cvMat(1, 1, CV_64FC2, point);
    cvProjectPoints2(&object_points, &rotation_vector, &translation_vector,
                     K_, NULL, &image_points);
    
    // Request ROI around projected point
    static const int ROI_SIZE = 700;
    CvRect outlet_roi = cvRect(point[0] - ROI_SIZE/2, point[1] - ROI_SIZE/2,
                               ROI_SIZE, ROI_SIZE);
    setRoi( fitToFrame(outlet_roi) );
    if (!ros::service::call(image_service_, req_, res_))
      return false; // something bad happened
    processCamInfo();
    
    if (detectOutlet(od_res.pose))
      return true;
    else
      ROS_WARN("Failed to detect outlet using estimate, trying again with full frame");
  }    
  
  // Slow fallback: search full frame
  req_.region_x = req_.region_y = req_.width = req_.height = 0;
  if (!ros::service::call(image_service_, req_, res_))
    return false;
  processCamInfo();
  return detectOutlet(od_res.pose);
}
*/
