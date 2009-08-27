#ifndef HANOI_H
#define HANOI_H

#include <ros/ros.h>

#include <cmvision/Blobs.h>
#include <tf/message_notifier.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>

#include "hanoi/Cylinders.h"

class Hanoi
{
  /// \brief Constructor
  public: Hanoi();

  /// \brief Destructor
  public: virtual ~Hanoi();

 // Got a point cloud
  private: void CloudCB(const tf::MessageNotifier<sensor_msgs::PointCloud>::MessagePtr &cloud);
 
  /// \brief Blob callback
  public: void BlobCB(const cmvision::BlobsConstPtr &msg);

 /// \brief Calculate the 3D grasp points
  private: bool CalculatedGraspPoints();
   
  private: ros::NodeHandle nodeHandle_;
  private: tf::TransformListener tf_;

  private: ros::Subscriber blobSubscriber_;
  private: ros::Publisher cylinderPublisher_;
  private: ros::ServiceClient moveArmService_;

  private: cmvision::Blob redBlob_;
  private: cmvision::Blob greenBlob_;
  private: cmvision::Blob blueBlob_;

  private: sensor_msgs::PointCloud pointcloud_;

  private: std::string cloud_topic_;
  private: std::string parameter_frame_, fixed_frame_;

  private: hanoi::Cylinders cylinderMessage_;

};

#endif
