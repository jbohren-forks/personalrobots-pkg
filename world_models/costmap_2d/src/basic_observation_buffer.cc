#include <costmap_2d/basic_observation_buffer.h>

namespace costmap_2d {

  BasicObservationBuffer::BasicObservationBuffer(const std::string& frame_id, tf::TransformListener& tf, ros::Duration keepAlive, 
				       double robotRadius, double minZ, double maxZ)
    : ObservationBuffer(keepAlive), frame_id_(frame_id), tf_(tf), robotRadius_(robotRadius), minZ_(minZ), maxZ_(maxZ){}

  void BasicObservationBuffer::buffer_cloud(const std_msgs::PointCloud& local_cloud)
  {
    static const ros::Duration max_transform_delay(10, 0); // max time we will wait for a transform before chucking out the data
    point_clouds_.push_back(local_cloud);

    std_msgs::PointCloud * newData = NULL;
    std_msgs::PointCloud * map_cloud = NULL;

    while(!point_clouds_.empty()){
      const std_msgs::PointCloud& point_cloud = point_clouds_.front();
      std_msgs::Point origin;

      // Throw out point clouds that are old.
      if((local_cloud.header.stamp -  point_cloud.header.stamp) > max_transform_delay){
	ROS_DEBUG("Discarding stale point cloud\n");
	point_clouds_.pop_front();
	continue;
      }

      tf::Stamped<btVector3> map_origin;
      std_msgs::PointCloud base_cloud;

      /* Transform to the base frame */
      try
        {
	  // First we want the origin for the sensor
	  tf::Stamped<btVector3> local_origin(btVector3(0, 0, 0), point_cloud.header.stamp.toNSec(), frame_id_);
	  tf_.transformPoint("map", local_origin, map_origin);

          tf_.transformPointCloud("base", point_cloud, base_cloud);
          newData = extractFootprintAndGround(base_cloud);
          map_cloud = new std_msgs::PointCloud();
          tf_.transformPointCloud("map", *newData, *map_cloud);

	  ROS_DEBUG("Buffering cloud for %s at origin <%f, %f, %f>\n", frame_id_.c_str(), map_origin.getX(), map_origin.getY(), map_origin.getZ());
        }
      catch(tf::LookupException& ex)
        {
          ROS_ERROR("Lookup exception for %s : %s\n", frame_id_.c_str(), ex.what());
          break;
        }
      catch(tf::ExtrapolationException& ex)
        {
          ROS_DEBUG("No transform available yet for %s - have to try later: %s . Buffer size is %d\n", 
		    frame_id_.c_str(), ex.what(), point_clouds_.size());
          break;
        }
      catch(tf::ConnectivityException& ex)
        {
          ROS_ERROR("Connectivity exception for %s: %s\n", frame_id_.c_str(), ex.what());
          break;
        }
      catch(...)
        {
          ROS_ERROR("Exception in point cloud computation\n");
          break;
        }

      point_clouds_.pop_front();

      if (newData != NULL){
	delete newData;
	newData = NULL;
      }

      // Get the point from whihc we ray trace
      std_msgs::Point o;
      o.x = map_origin.getX();
      o.y = map_origin.getY();
      o.z = map_origin.getZ();

      // Allocate and buffer the observation
      Observation obs(o, map_cloud);
      buffer_observation(obs);
      map_cloud = NULL;
    }

    // In case we get thrown out on the second transform - clean up
    if (newData != NULL){
      delete newData;
      newData = NULL;
    }

    if(map_cloud != NULL){
      delete map_cloud;
      map_cloud = NULL;
    }    
  }

  void BasicObservationBuffer::get_observations(std::vector<Observation>& observations){
    ObservationBuffer::get_observations(observations);
  }

  /**
   * The point is in the footprint if its x and y values are in the range [0 robotWidth] in
   * the base frame.
   */
  bool BasicObservationBuffer::inFootprint(double x, double y) const {
    bool result = fabs(x) <= robotRadius_ && fabs(y) <= robotRadius_;

    if(result){
      ROS_INFO("Discarding point <%f, %f> in footprint for %s\n", x, y, frame_id_.c_str());
    }

    return result;
  }

  std_msgs::PointCloud * BasicObservationBuffer::extractFootprintAndGround(const std_msgs::PointCloud& baseFrameCloud) const {
    std_msgs::PointCloud *copy = new std_msgs::PointCloud();
    copy->header = baseFrameCloud.header;

    unsigned int n = baseFrameCloud.get_pts_size();
    unsigned int j = 0;
    copy->set_pts_size(n);

    for (unsigned int k = 0 ; k < n ; ++k){
      bool ok = baseFrameCloud.pts[k].z > minZ_ &&   baseFrameCloud.pts[k].z < maxZ_;
      if (ok && !inFootprint(baseFrameCloud.pts[k].x, baseFrameCloud.pts[k].y)){
	copy->pts[j++] = baseFrameCloud.pts[k];
      }
      else {
	ROS_INFO("Discarding <%f, %f, %f> for %s\n",  baseFrameCloud.pts[k].x, baseFrameCloud.pts[k].y,   baseFrameCloud.pts[k].z, frame_id_.c_str());
      }
    }

    copy->set_pts_size(j);

    ROS_INFO("Filter discarded %d points (%d left) \n", n - j, j);

    return copy;
  }
}
