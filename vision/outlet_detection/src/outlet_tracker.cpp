#include "outlet_detection/outlet_tracker.h"

#include <Eigen/Core>
#include <Eigen/QR>

static void changeAxes(CvPoint3D32f src, btVector3 &dst);
static btVector3 fitPlane(btVector3 *holes, int num_holes);

OutletTracker::OutletTracker(ros::Node &node)
  : TrackerBase(node, "outlet")
{
  activate();
}

OutletTracker::~OutletTracker()
{
}
  
bool OutletTracker::detectObject(tf::Transform &pose)
{
  if (!img_bridge_.fromImage(img_, "bgr")) {
    ROS_ERROR("Failed to convert image");
    return false;
  }

  IplImage* image = img_bridge_.toIpl();
  outlets_.clear();
  if (!detect_outlet_tuple(image, K_, NULL, outlets_))
    return false;

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
  pose = tf::Transform(orientation, holes[0]);

  return true;
}

CvRect OutletTracker::getBoundingBox()
{
  CvRect tuple_roi[4];
  for (int i = 0; i < 4; i++)
    tuple_roi[i] = outlet_rect(outlets_[i]);
  CvRect outlet_roi;
  calc_bounding_rect(4, tuple_roi, outlet_roi);
  return outlet_roi;
}

IplImage* OutletTracker::getDisplayImage(bool success)
{
  IplImage* image = img_bridge_.toIpl();
  if (success)
    draw_outlets(image, outlets_);
  return image;
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
