#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/intersections.h>

#include "Hanoi.h"

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Hanoi::Hanoi()
  : nodeHandle_(), tf_(nodeHandle_)
{
  std::string cloud_topic;

  //moveArmService_ = nodeHandle_.serviceClient("/auto_arm_cmd_server");
  
  if (!nodeHandle_.getParam("parameter_frame",parameter_frame_))
    parameter_frame_ = "base_link";

  if (!nodeHandle_.getParam("cloud_topic", cloud_topic))
    cloud_topic_ = "full_cloud_filtered";

  // Subscribe to the blobs topic
  blobSubscriber_ = nodeHandle_.subscribe("blobs", 100, &Hanoi::BlobCB, this);
 
  // Create the publisher for cylinder data
  cylinderPublisher_ = nodeHandle_.advertise<hanoi::Cylinders>("cylinders", 1);

  // Get the point cloud
  tf::MessageNotifier<sensor_msgs::PointCloud> *message_notifier = 
    new tf::MessageNotifier<sensor_msgs::PointCloud>( tf_, 
       boost::bind(&Hanoi::CloudCB,this,_1), cloud_topic_, parameter_frame_, 1);

}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Hanoi::~Hanoi()
{
}

////////////////////////////////////////////////////////////////////////////////
// Got a point cloud
void Hanoi::CloudCB(const tf::MessageNotifier<sensor_msgs::PointCloud>::MessagePtr &cloud)
{
  pointcloud_ = *cloud;

  ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)pointcloud_.points.size (), pointcloud_.header.frame_id.c_str (),
            (int)pointcloud_.channels.size (), cloud_geometry::getAvailableChannels (pointcloud_).c_str ());

}


////////////////////////////////////////////////////////////////////////////////
// Blob callback
void Hanoi::BlobCB(const cmvision::BlobsConstPtr &msg)
{
  printf("Blob Count[%d]\n", msg->blobCount);

  redBlob_.area = 0;
  greenBlob_.area = 0;
  blueBlob_.area = 0;

  for (unsigned int i = 0; i < msg->blobCount; i++)
  {
    unsigned int r,g,b;
    unsigned int area, x, y;

    r = msg->blobs[i].red;
    g = msg->blobs[i].green;
    b = msg->blobs[i].blue;

    area = msg->blobs[i].area;

    x = msg->blobs[i].x;
    y = msg->blobs[i].y;

    if (r == 255 && area > redBlob_.area)
      redBlob_ = msg->blobs[i];
    if (g == 255 && area > greenBlob_.area)
      greenBlob_ = msg->blobs[i];
    if (b == 255 && area > blueBlob_.area)
      blueBlob_ = msg->blobs[i];

   //printf("Blob[%d] RGB[%d %d %d] Size[%d]\n", i, r,g,b, area);
  }

  printf("Red Size[%d] Pos[%d %d]\n", redBlob_.area, redBlob_.x, redBlob_.y);
  printf("Green Size[%d] Pos[%d %d]\n", greenBlob_.area, greenBlob_.x, greenBlob_.y);
  printf("Blue Size[%d] Pos[%d %d]\n", blueBlob_.area, blueBlob_.x, blueBlob_.y);

  this->CalculateGraspPoints();

  cylinderMessage_.redArea = redBlob_.area;
  cylinderMessage_.redX = redBlob_.x;
  cylinderMessage_.redY = redBlob_.y;
  cylinderMessage_.redLeft = redBlob_.left;
  cylinderMessage_.redRight = redBlob_.right;
  cylinderMessage_.redTop = redBlob_.top;
  cylinderMessage_.redBottom = redBlob_.bottom;

  cylinderMessage_.greenArea = greenBlob_.area;
  cylinderMessage_.greenX = greenBlob_.x;
  cylinderMessage_.greenY = greenBlob_.y;
  cylinderMessage_.greenLeft = greenBlob_.left;
  cylinderMessage_.greenRight = greenBlob_.right;
  cylinderMessage_.greenTop = greenBlob_.top;
  cylinderMessage_.greenBottom = greenBlob_.bottom;

  cylinderMessage_.blueArea = blueBlob_.area;
  cylinderMessage_.blueX = blueBlob_.x;
  cylinderMessage_.blueY = blueBlob_.y;
  cylinderMessage_.blueLeft = blueBlob_.left;
  cylinderMessage_.blueRight = blueBlob_.right;
  cylinderMessage_.blueTop = blueBlob_.top;
  cylinderMessage_.blueBottom = blueBlob_.bottom;

  cylinderPublisher_.publish(cylinderMessage_);
}

////////////////////////////////////////////////////////////////////////////////
// Calculate the 3D grasp points
bool Hanoi::CalculatedGraspPoints()
{
  // Transform the Point Cloud Data into the parameter_frame
  if (!tf_.canTransform(parameter_frame_, pointcloud_.header.frame_id, 
        pointcloud_.header.stamp, timeout))
  {
    ROS_ERROR ("Hanoi: Could not transform point cloud from frame '%s' to frame '%s' at time %f.",
        pointcloud_.header.frame_id.c_str (), 
        parameter_frame_.c_str (), pointcloud_.header.stamp.toSec());
    return false;
  }

  tf_.transformPointCloud (parameter_frame_, pointcloud_, pointcloud_);

}
