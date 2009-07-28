#include "outlet_detection/tracker_base.h"
#include <geometry_msgs/PoseStamped.h>
#include <prosilica_cam/CamInfo.h>
#include <boost/bind.hpp>
#include <unistd.h> // getpid

// TODO: don't use "high_def_frame" or handle the change of axes explicitly

static inline CvRect resizeRect(CvRect rect, double alpha)
{
  return cvRect(rect.x + int(0.5*(1 - alpha)*rect.width), rect.y + int(0.5*(1 - alpha)*rect.height),
                int(rect.width*alpha), int(rect.height*alpha));
}

TrackerBase::TrackerBase(ros::Node &node, std::string prefix)
  : node_(node), img_(res_.image), cam_info_(res_.cam_info),
    pose_topic_name_("~" + prefix + "_pose"),
    tf_listener_(node),
    object_frame_id_(prefix + "_pose"), K_(NULL),
    display_topic_name_("~" + prefix + "_image"),
    save_count_(0), save_prefix_(prefix)
{
  node_.param("~image_service", image_service_, std::string("/prosilica/poll"));
  node_.param("~cam_info_service", cam_info_service_, std::string("/prosilica/cam_info_service"));

  std::string policy;
  node_.param("~roi_policy", policy, std::string("FullResolution"));
  //node_.param("~roi_policy", policy, std::string("LastImageLocation"));
  if (policy == std::string("FullResolution"))
    roi_policy_ = FullResolution;
  else if (policy == std::string("LastImageLocation"))
    roi_policy_ = LastImageLocation;
  else if (policy == std::string("TargetFrame")) {
    roi_policy_ = TargetFrame;
    // TODO: should be error if not set
    node_.param("~target_frame_id", target_frame_id_, std::string(""));
  } else {
    ROS_FATAL("Unknown ROI policy setting");
    node_.shutdown();
    return;
  }

  ROS_ERROR("ROI policy: %d", roi_policy_);

  req_.timeout_ms = 500; // TODO: magic number

  node_.param("~resize_factor_found", resize_factor_found_, 1.2);
  node_.param("~resize_factor_failed", resize_factor_failed_, 1.2);
  node_.param("~target_roi_size", target_roi_size_, 400);

  node_.param("~save_failures", save_failures_, 0);

  node_.param("~stay_active", stay_active_, true);
  node_.subscribe("~activate_tracker", activate_msg_, &TrackerBase::activateCB, this, 2);
  ROS_ERROR("Stay active = %d", stay_active_);
}

TrackerBase::~TrackerBase()
{
  if (active_thread_.joinable()) {
    deactivate();
    active_thread_.join();
  }

  cvReleaseMat(&K_);
  node_.unsubscribe("~activate_tracker");
}

void TrackerBase::activate()
{
  node_.advertise<geometry_msgs::PoseStamped>(pose_topic_name_, 1);
  node_.advertise<sensor_msgs::Image>(display_topic_name_, 1);

  boost::thread t(boost::bind(&TrackerBase::spin, this));
  active_thread_.swap(t);
}

void TrackerBase::deactivate()
{
  active_thread_.interrupt();

  node_.unadvertise(pose_topic_name_);
  node_.unadvertise(display_topic_name_);
}

IplImage* TrackerBase::getDisplayImage(bool success)
{
  return img_bridge_.toIpl();
}

void TrackerBase::processCamInfo()
{
  if (K_ == NULL)
    K_ = cvCreateMat(3, 3, CV_64FC1);
  memcpy((char*)(K_->data.db), (char*)(&cam_info_.K[0]), 9 * sizeof(double));
  frame_w_ = cam_info_.width;
  frame_h_ = cam_info_.height;
}

void TrackerBase::processImage()
{
  tf::Transform transform;
  bool success = detectObject(transform);

  if (success) {
    // Publish to topic
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "high_def_frame";
    pose.header.stamp = img_.header.stamp;
    tf::poseTFToMsg(transform, pose.pose);
    node_.publish(pose_topic_name_, pose);

    // Publish to TF
    tf_broadcaster_.sendTransform(transform, ros::Time::now(),
                                  object_frame_id_, "high_def_frame");

    // Recenter ROI for next image request
    if (roi_policy_ == LastImageLocation) {
      CvRect object_roi = getBoundingBox();
      object_roi.x += req_.region_x;
      object_roi.y += req_.region_y;
      object_roi = fitToFrame(resizeRect(object_roi, resize_factor_found_));
      setRoi(object_roi);
    }
  }
  else {
    // Save failure for debugging
    //if (save_failures_)
    //  saveImage(false);

    // Expand ROI for next image
    if (roi_policy_ == LastImageLocation) {
      CvRect object_roi = cvRect(req_.region_x, req_.region_y,
                                 req_.width, req_.height);
      object_roi = fitToFrame(resizeRect(object_roi, resize_factor_failed_));
      setRoi(object_roi);
    }
  }

  // Visual feedback
  if (node_.numSubscribers(display_topic_name_) > 0) {
    display_img_.encoding = "rgb"; // TODO: temporary hack
    sensor_msgs::CvBridge::fromIpltoRosImage(getDisplayImage(success), display_img_);
    display_img_.encoding = "bgr";
    node_.publish(display_topic_name_, display_img_);
  }

  // DEBUG: save out everything
  //saveImage(success);
}

void TrackerBase::spin()
{
  bool informed_of_deactivation = false;
  while (node_.ok() && !boost::this_thread::interruption_requested())
  {
    if (!stay_active_)
    {
      if (ros::Time::now() - last_activate_time_ > ros::Duration(10.0))
      {
        if (!informed_of_deactivation)
        {
          informed_of_deactivation = true;
          ROS_WARN("Tracker is going inactive (%s)", node_.getName().c_str());
        }
        ros::Duration(1.0).sleep();
        continue;
      }
      else
      {
        informed_of_deactivation = false;
      }
    }

    if (roi_policy_ == TargetFrame)
      setRoiToTargetFrame();

    if (ros::service::call(image_service_, req_, res_)) {
      processCamInfo();
      processImage();
      //cvWaitKey(0);
      usleep(100000);
    } else {
      waitForService(image_service_);
    }
  }
}

CvRect TrackerBase::fitToFrame(CvRect roi)
{
  CvRect fit;

  fit.width = std::min(roi.width, frame_w_);
  fit.height = std::min(roi.height, frame_h_);
  fit.x = std::min(std::max(0, roi.x), frame_w_ - fit.width);
  fit.y = std::min(std::max(0, roi.y), frame_h_ - fit.height);

  return fit;
}

void TrackerBase::setRoi(CvRect roi)
{
  req_.region_x = roi.x;
  req_.region_y = roi.y;
  req_.width = roi.width;
  req_.height = roi.height;
}

void TrackerBase::setRoiToTargetFrame()
{
  // Try to get calibration parameters
  if (!K_) {
    if (waitForService(cam_info_service_)) {
      prosilica_cam::CamInfo::Request cam_req;
      prosilica_cam::CamInfo::Response cam_rsp;
      if (ros::service::call(cam_info_service_, cam_req, cam_rsp)) {
        cam_info_ = cam_rsp.cam_info;
        processCamInfo();
      }
    }
  }

  // Otherwise fall back to using full frame
  if (!K_) {
    ROS_WARN("setRoiToTargetFrame: falling back to full frame");
    req_.region_x = req_.region_y = req_.width = req_.height = 0;
    return;
  }

  // Get target frame pose in high def frame
  geometry_msgs::Point target;
  try {
    geometry_msgs::Pose target_pose = getTargetInHighDef();
    target = target_pose.position;
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }

  // Get intrinsic matrix of full frame by translating principal point
  K_->data.db[2] += req_.region_x;
  K_->data.db[5] += req_.region_y;

  // Project to image coordinates
  CvPoint3D64f object_pt = cvPoint3D64f(-target.y, -target.z, target.x);
  CvMat object_points = cvMat(1, 1, CV_64FC3, &object_pt.x);
  double zeros[] = {0, 0, 0};
  CvMat rotation_vector = cvMat(3, 1, CV_64FC1, zeros);
  CvMat translation_vector = cvMat(3, 1, CV_64FC1, zeros);
  double point[2] = {0.0, 0.0};
  CvMat image_points = cvMat(1, 1, CV_64FC2, point);
  cvProjectPoints2(&object_points, &rotation_vector, &translation_vector,
                   K_, NULL, &image_points);

  // Request ROI around projected point
  CvRect object_roi = cvRect(point[0] - target_roi_size_/2,
                             point[1] - target_roi_size_/2,
                             target_roi_size_, target_roi_size_);

  setRoi( fitToFrame(object_roi) );
}

geometry_msgs::Pose TrackerBase::getTargetInHighDef()
{
  geometry_msgs::PoseStamped origin, target_in_high_def;
  tf::Transform origin_tf;
  origin_tf.setIdentity();
  tf::poseTFToMsg(origin_tf, origin.pose);
  origin.header.frame_id = target_frame_id_;
  origin.header.stamp = ros::Time::now();
  tf_listener_.canTransform("high_def_frame", origin.header.frame_id, origin.header.stamp, ros::Duration(0.5));
  tf_listener_.transformPose("high_def_frame", origin, target_in_high_def);

  return target_in_high_def.pose;
}

void TrackerBase::saveImage(bool success)
{
  char filename[32];
  snprintf(filename, sizeof(filename), "%s%u_%03i.yml", save_prefix_.c_str(), getpid(), save_count_);
  CvFileStorage *fs = cvOpenFileStorage(filename, 0, CV_STORAGE_WRITE);
  time_t t;
  time( &t );
  struct tm *t2 = localtime( &t );
  char buf[1024];
  strftime( buf, sizeof(buf)-1, "%c", t2 );
  cvWriteString( fs, "save_time", buf );
  cvWriteString( fs, "success", success ? "true" : "false" );
  cvWrite(fs, "camera_matrix", K_);
  double zeros[] = {0, 0, 0, 0, 0};
  CvMat D = cvMat(5, 1, CV_64FC1, zeros);
  cvWrite(fs, "distortion_coefficients", &D);
  cvReleaseFileStorage(&fs);

  snprintf(filename, sizeof(filename), "%s%u_%03i.tf", save_prefix_.c_str(), getpid(), save_count_);
  FILE *tf_file = fopen(filename, "w");
  if (tf_file) {
    try {
      geometry_msgs::PoseStamped origin, xfm_in_base_link;
      origin.pose.orientation.w = 1.0;
      origin.header.frame_id = "high_def_frame";
      origin.header.stamp = ros::Time::now();
      tf_listener_.canTransform("base_link", origin.header.frame_id, origin.header.stamp, ros::Duration(0.5));
      tf_listener_.transformPose("base_link", origin, xfm_in_base_link);
      fprintf(tf_file, "high_def_frame: (%.5f, %.5f, %.5f), (%.5f, %.5f, %.5f, %.5f)\n",
              xfm_in_base_link.pose.position.x, xfm_in_base_link.pose.position.y,
              xfm_in_base_link.pose.position.z, xfm_in_base_link.pose.orientation.x,
              xfm_in_base_link.pose.orientation.y, xfm_in_base_link.pose.orientation.z,
              xfm_in_base_link.pose.orientation.w);

      if (!target_frame_id_.empty()) {
        origin.header.frame_id = target_frame_id_;
        origin.header.stamp = ros::Time::now();
        tf_listener_.canTransform("base_link", origin.header.frame_id, origin.header.stamp, ros::Duration(0.5));
        tf_listener_.transformPose("base_link", origin, xfm_in_base_link);
        fprintf(tf_file, "%s: (%.5f, %.5f, %.5f), (%.5f, %.5f, %.5f, %.5f)\n",
                target_frame_id_.c_str(), xfm_in_base_link.pose.position.x,
                xfm_in_base_link.pose.position.y, xfm_in_base_link.pose.position.z,
                xfm_in_base_link.pose.orientation.x, xfm_in_base_link.pose.orientation.y,
                xfm_in_base_link.pose.orientation.z, xfm_in_base_link.pose.orientation.w);
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("Transform Exception %s, couldn't save transforms", ex.what());
    }
    
    fclose(tf_file);
  } else {
    ROS_WARN("Couldn't open file to save transforms");
  }

  snprintf(filename, sizeof(filename), "%s%u_%03i.jpg", save_prefix_.c_str(), getpid(), save_count_++);
  cvSaveImage(filename, img_bridge_.toIpl());
  ROS_INFO("Saved %s", filename);
}

bool TrackerBase::waitForService(const std::string &service)
{
  std::string host;
  int port;
  while (node_.ok() && !boost::this_thread::interruption_requested()) {
    if (ros::service::exists(service, false))
      return true;
    usleep(100000);
  }

  return false;
}

void TrackerBase::activateCB()
{
  last_activate_time_ = ros::Time::now();
}
