#include <ransac_ground_plane_extraction/ransac_ground_plane_extraction.h>

#define MIN_POINTS 10

using namespace ransac_ground_plane_extraction;

RansacGroundPlaneExtraction::RansacGroundPlaneExtraction(){
  point_plane_.x = 0.0;
  point_plane_.y = 0.0;
  point_plane_.z = 0.0;

  normal_plane_.x = 0.0;
  normal_plane_.y = 0.0;
  normal_plane_.z = 1.0;
  filter_delta_ = 0.5;
  max_iterations_ = 500;
}

int RansacGroundPlaneExtraction::findGround(const std_msgs::PointCloud& baseFrameCloud, const double &min_ignore_distance, const double &max_ignore_distance, const double &distance_threshold, std_msgs::Point32 &planePoint, std_msgs::Point32 &planeNormal,std_msgs::PointStamped &sensor_origin) 
{
    SmartScan smart_scan;

    std_msgs::PointCloud *copy = new std_msgs::PointCloud();
    copy->header = baseFrameCloud.header;

    unsigned int n = baseFrameCloud.get_pts_size();
    unsigned int j = 0;
    copy->set_pts_size(n);

    for (unsigned int k = 0 ; k < n ; ++k){
      bool ok = baseFrameCloud.pts[k].z > min_ignore_distance &&   baseFrameCloud.pts[k].z < max_ignore_distance;
      if (ok){
	copy->pts[j++] = baseFrameCloud.pts[k];
      }
      else {
	ROS_DEBUG("Discarding <%f, %f, %f> \n",  baseFrameCloud.pts[k].x, baseFrameCloud.pts[k].y,   baseFrameCloud.pts[k].z);
      }
    }
    copy->set_pts_size(j);

    if(j < MIN_POINTS)
    {
      ROS_WARN("RansacGroundPlaneExtraction:: Too few points in candidate ground plane: %d",j);
      return -1;
    }

    smart_scan.setFromRosCloud(*copy);

    ROS_INFO("***************");
    ros::Time time = ros::Time::now();
    ROS_INFO("Starting RANSAC");
    ROS_INFO("***************");
    smart_scan.ransacPlane (planePoint, planeNormal, max_iterations_, distance_threshold);

    if(planeNormal.z < 0)//turn the ground normal so its always facing up
    {
      planeNormal.x = -planeNormal.x;
      planeNormal.y = -planeNormal.y;
      planeNormal.z = -planeNormal.z;
    }

    sensor_origin.header = baseFrameCloud.header;
    sensor_origin.point.x = 0.0;
    sensor_origin.point.y = 0.0;
    sensor_origin.point.z = 0.0;

    ROS_DEBUG("Plane point: %f %f %f",planePoint.x,planePoint.y,planePoint.z);
    ROS_DEBUG("Plane normal: %f %f %f",planeNormal.x,planeNormal.y,planeNormal.z);
    ROS_DEBUG("Done RANSAC %f seconds",(ros::Time::now()-time).toSec());

    return 1;
}

std_msgs::PointCloud *RansacGroundPlaneExtraction::removeGround(const std_msgs::PointCloud& baseFrameCloud, double remove_distance, const std_msgs::Point &point_plane, std_msgs::Vector3 &normal_plane, const std_msgs::PointStamped &origin, const double &threshold_distance, const double &far_remove_distance) {
  std_msgs::Point32 pt;
  std_msgs::Point32 norm;

  pt.x = point_plane.x;
  pt.y = point_plane.y;
  pt.z = point_plane.z;

  norm.x = normal_plane.x;
  norm.y = normal_plane.y;
  norm.z = normal_plane.z;

  return removeGround(baseFrameCloud, remove_distance, pt, norm, origin, threshold_distance, far_remove_distance);
}

std_msgs::PointCloud *RansacGroundPlaneExtraction::removeGround(const std_msgs::PointCloud& baseFrameCloud, double remove_distance, const std_msgs::Point32 &point_plane, std_msgs::Point32 &normal_plane, const std_msgs::PointStamped &origin, const double &threshold_distance, const double &far_remove_distance) 
{
    SmartScan full_scan;

    ROS_DEBUG("Remove: Plane point: %f %f %f",point_plane.x, point_plane.y,point_plane.z);
    ROS_DEBUG("Remove: Plane normal: %f %f %f",normal_plane.x, normal_plane.y,normal_plane.z);
    ROS_DEBUG("Threshold: %f",remove_distance);

    full_scan.setFromRosCloud(baseFrameCloud);
    full_scan.removePlane (point_plane, normal_plane, remove_distance, origin, threshold_distance, far_remove_distance);

//    ROS_DEBUG("Done removing plane %f seconds",(ros::Time::now()-time).toSec());

    std_msgs::PointCloud *return_copy = new std_msgs::PointCloud();
    return_copy->header = baseFrameCloud.header;
    *return_copy = full_scan.getPointCloud();


//    node_.publish("obstacle_cloud",*return_copy);

    return return_copy;
}

void RansacGroundPlaneExtraction::updateGround(const std_msgs::Point32 &new_plane_point, const std_msgs::Point32 &new_plane_normal, std_msgs::Point32 &return_plane_point, std_msgs::Point32 &return_plane_normal)
{

  if(fabs(new_plane_normal.z) >= 0.8)
  {
    point_plane_.x = point_plane_.x + filter_delta_ * (new_plane_point.x - point_plane_.x);
    point_plane_.y = point_plane_.y + filter_delta_ * (new_plane_point.y - point_plane_.y);
    point_plane_.z = point_plane_.z + filter_delta_ * (new_plane_point.z - point_plane_.z);

    normal_plane_.x = normal_plane_.x + filter_delta_ * (new_plane_normal.x - normal_plane_.x);
    normal_plane_.y = normal_plane_.y + filter_delta_ * (new_plane_normal.y - normal_plane_.y);
    normal_plane_.z = normal_plane_.z + filter_delta_ * (new_plane_normal.z - normal_plane_.z);

    double mag = std::max((double)sqrt(normal_plane_.x*normal_plane_.x + normal_plane_.y*normal_plane_.y + normal_plane_.z*normal_plane_.z),1e-5);

    normal_plane_.x = normal_plane_.x/mag;
    normal_plane_.y = normal_plane_.y/mag;
    normal_plane_.z = normal_plane_.z/mag;
  }
    return_plane_point.x = point_plane_.x;
    return_plane_point.y = point_plane_.y;
    return_plane_point.z = point_plane_.z;

    return_plane_normal.x = normal_plane_.x;
    return_plane_normal.y = normal_plane_.y;
    return_plane_normal.z = normal_plane_.z;
}
