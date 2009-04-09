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

#include <Eigen/Core>
#include <Eigen/QR>

#include "outlet_detector.h"


class OutletDetector : public ros::Node
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
  CvMat* K_;
  bool display_;

  // TODO: policies to listen from far outlet detection, last 3d location?
  enum { WholeFrame, LastImageLocation } roi_policy_;
  int frame_w_, frame_h_;
  static const float RESIZE_FACTOR = 1.2f;

public:
  OutletDetector()
    : ros::Node("outlet_detector"), img_(res_.image), cam_info_(res_.cam_info),
      tf_broadcaster_(*this), K_(NULL)
  {
    param("display", display_, true);

    std::string policy;
    param("roi_policy", policy, std::string("WholeFrame"));
    if (policy == std::string("WholeFrame"))
      roi_policy_ = WholeFrame;
    else if (policy == std::string("LastImageLocation"))
      roi_policy_ = LastImageLocation;
    else {
      ROS_FATAL("Unknown ROI policy setting");
      shutdown();
    }
    frame_w_ = 2448; // TODO: actually get these values from somewhere
    frame_h_ = 2050;

    req_.timeout_ms = 100;

    if (display_) {
      cvNamedWindow(wndname, 0); // no autosize
      cvStartWindowThread();
    }
    
    advertise<robot_msgs::PoseStamped>("pose", 1);
  }

  ~OutletDetector()
  {
    cvReleaseMat(&K_);
    if (display_)
      cvDestroyWindow(wndname);
  }

  void caminfo_cb()
  {
    if (K_ == NULL)
      K_ = cvCreateMat(3, 3, CV_64FC1);
    memcpy(K_->data.db, &cam_info_.K[0], 9 * sizeof(double));
  }

  void image_cb()
  {
    if (!img_bridge_.fromImage(img_, "bgr")) {
      ROS_ERROR("Failed to convert image");
      return;
    }

    IplImage* image = img_bridge_.toIpl();
    std::vector<outlet_t> outlets;
    if (!detect_outlet_tuple(image, K_, NULL, outlets)) {
      //ROS_WARN("Failed to detect outlet");

      // Expand ROI for next image
      if (roi_policy_ == LastImageLocation) {
        CvRect outlet_roi = cvRect(req_.region_x, req_.region_y,
                                   req_.width, req_.height);
        outlet_roi = fitToFrame(resize_rect(outlet_roi, RESIZE_FACTOR));
        setRoi(outlet_roi);
      }
      
      if (display_)
        cvShowImage(wndname, image);
      return;
    }

    // Change data representation and coordinate frame
    btVector3 holes[12];
    for (int i = 0; i < 4; ++i) {
      changeAxes(outlets[i].coord_hole_ground, holes[3*i]);
      changeAxes(outlets[i].coord_hole1, holes[3*i+1]);
      changeAxes(outlets[i].coord_hole2, holes[3*i+2]);
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
    
    // Publish to topic and TF
    tf::PoseTFToMsg(outlet_pose, pose_.pose);
    pose_.header.frame_id = "high_def_frame";
    pose_.header.stamp = img_.header.stamp;
    publish("pose", pose_);
    tf_broadcaster_.sendTransform(outlet_pose, ros::Time::now(),
                                  "outlet_frame", "high_def_frame");

    // Calculate ROI for next image request
    if (roi_policy_ == LastImageLocation) {
      CvRect tuple_roi[4];
      for (int i = 0; i < 4; i++)
        tuple_roi[i] = outlet_rect(outlets[i]);
      CvRect outlet_roi;
      calc_bounding_rect(4, tuple_roi, outlet_roi);
      outlet_roi.x += req_.region_x;
      outlet_roi.y += req_.region_y;
      outlet_roi = fitToFrame(resize_rect(outlet_roi, RESIZE_FACTOR));
      setRoi(outlet_roi);
    }
    /*
    ROS_INFO("Ground TL: %.5f %.5f %.5f, Ground TR: %.5f %.5f %.5f, "
             "Ground BR: %.5f %.5f %.5f, Ground BL: %.5f %.5f %.5f",
             holes[0].x(), holes[0].y(), holes[0].z(),
             holes[3].x(), holes[3].y(), holes[3].z(),
             holes[6].x(), holes[6].y(), holes[6].z(),
             holes[9].x(), holes[9].y(), holes[9].z());
    
    ROS_INFO("Ground hole distances:\n\td(TL,TR) = %.2fmm\n"
             "\td(BL,BR) = %.2fmm\n\td(TL,BR) = %.2fmm\n\td(BL,TR) = %.2fmm",
             1000*holes[0].distance(holes[3]), 1000*holes[6].distance(holes[9]),
             1000*holes[0].distance(holes[6]), 1000*holes[3].distance(holes[9]));
    */
    
    if (display_) {
      draw_outlets(image, outlets);
      cvShowImage(wndname, image);
    }
  }
  
  bool spin()
  {
    while (ok())
    {
      if (ros::service::call("/prosilica/poll", req_, res_)) {
        caminfo_cb();
        image_cb();
        // TODO: figure out what's actually causing banding
        usleep(100000); // hack to (mostly) get rid of banding
      } else {
        //ROS_WARN("Service call failed");
        // TODO: wait for service to become available
        usleep(100000);
      }
    }

    return true;
  }

private:
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

  CvRect fitToFrame(CvRect roi)
  {
    CvRect fit;
    fit.x = std::max(roi.x, 0);
    fit.y = std::max(roi.y, 0);
    fit.width = std::min(roi.x + roi.width, frame_w_) - fit.x;
    fit.height = std::min(roi.y + roi.height, frame_h_) - fit.y;
    return fit;
  }

  inline void setRoi(CvRect roi)
  {
    req_.region_x = roi.x;
    req_.region_y = roi.y;
    req_.width = roi.width;
    req_.height = roi.height;
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
