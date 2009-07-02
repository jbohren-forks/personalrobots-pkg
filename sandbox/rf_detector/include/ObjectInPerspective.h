#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "topic_synchronizer2/topic_synchronizer.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv/cvaux.h"
#include "opencv_latest/CvBridge.h"
#include "image_msgs/Image.h"
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_oriented_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
//#include <point_cloud_mapping/geometry/statistics.h>
#include <point_cloud_mapping/geometry/projections.h>

#include "image_msgs/CamInfo.h"
#include "robot_msgs/PointCloud.h"
#include "robot_msgs/Point32.h"
#include "robot_msgs/PointStamped.h"

// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace std;
#define CV_PIXEL(type,img,x,y) (((type*)(img->imageData+y*img->widthStep))+x*img->nChannels)
//function lists
void filterByZBounds( const robot_msgs::PointCloud& pc, double zmin, double zmax, robot_msgs::PointCloud& filtered_pc, robot_msgs::PointCloud& filtered_outside);

bool fitSACPlane(const robot_msgs::PointCloud& points, const vector<int> &indices,  // input
			vector<int> &inliers, vector<double> &coeff,  // output
			double dist_thresh, int min_points_per_model);


void filterTablePlane(const robot_msgs::PointCloud& cloud, vector<double>& coefficients, robot_msgs::PointCloud& object_cloud, robot_msgs::PointCloud& plane_cloud);

void addTableFrame( robot_msgs::PointStamped origin, const vector<double>& plane, tf::TransformListener& tf_, tf::TransformBroadcaster& broadcaster_);

float GetCameraHeight(const robot_msgs::PointCloud& pc, tf::TransformListener& tf_);

float CalHorizontalLine( const robot_msgs::PointCloud& cloud, vector<double> plane_coeff, const image_msgs::CamInfo& lcinfo, tf::TransformListener& tf_);

void filterClusters(const robot_msgs::PointCloud& cloud, vector<robot_msgs::Point32>& centers, vector<robot_msgs::Point32>& clusters);

void findClusters2(const robot_msgs::PointCloud& cloud, vector<robot_msgs::Point32>& clusters);

//double meanShiftIteration(const PointCloud& pc, NNGridIndexer& index, vector<Point32>& centers, vector<Point32>& means, float step);

//void clearFromImage(IplImage* disp_img, const robot_msgs::PointCloud& pc);

float findObjectPositionsFromStereo(const robot_msgs::PointCloud& cloud, vector<CvPoint>& locations, vector<CvPoint>& obj_bottom,
        vector<float>& scales, vector<float>& scales_msun, tf::TransformListener& tf_, tf::TransformBroadcaster& broadcaster_,
        const image_msgs::CamInfo& lcinfo);

robot_msgs::Point project3DPointIntoImage(const image_msgs::CamInfo& cam_info, robot_msgs::PointStamped point, tf::TransformListener& tf_);
