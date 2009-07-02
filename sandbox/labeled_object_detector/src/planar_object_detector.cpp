/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <planar_object_detector.h>
#include <pcd_misc.h>
#include <visualization_msgs/Marker.h>



using namespace std;
using namespace robot_msgs;
using namespace pcd_misc;
using namespace labeled_object_detector;



PlanarObjectDetector::PlanarObjectDetector()
{
  max_link_distance_=0.2;
  cloud_sub_ = n_.subscribe<PointCloud>("cloud",100,boost::bind(&PlanarObjectDetector::cloudCallback, this, _1));

  marker_pub_=n_.advertise<visualization_msgs::Marker>("visualization_marker",1);

  //tf_ = new tf::TransformListener( ros::Duration(30.0));  
  /*string topic_name;
    nh_.param<string>("~full_cloud_topic", topic_name, "full_cloud");
    nh_.param("~min_height", min_height_, 0.9);
      ROS_INFO("Topic name: %s", topic_name.c_str());
  */      
  cloud_pub_ = n_.advertise<PointCloud>("whiteboard_cloud",100);

}



void PlanarObjectDetector::setup()
{
  boost::mutex::scoped_lock lock(proc_mutex_);  
  // Query map 
  
  ros::ServiceClient client = n_.serviceClient<point_cloud_assembler::BuildCloud>("build_cloud");
  
  point_cloud_assembler::BuildCloud build_cloud;

  build_cloud.request.begin.sec=0;
  build_cloud.request.end.sec=1999999999;
  
  client.call(build_cloud);
  
  ROS_INFO_STREAM("Got cloud with " <<build_cloud.response.cloud.pts.size() <<" points");
  
  unsigned int num_pts=build_cloud.response.cloud.pts.size();
  // Filter objects that are a whiteboard
  
  int lbl_idx=cloud_geometry::getChannelIndex(build_cloud.response.cloud,"ann-willow-env-4");
  float target_label=9;
  
  vector<int> keep_points;
  keep_points.resize(num_pts);
  int num_keep=0;
  for(unsigned int iPt=0;iPt<num_pts;iPt++)
  {
    float v=build_cloud.response.cloud.chan[lbl_idx].vals[iPt];
    if(fabs(v-target_label)<0.1)
    {
      num_keep++;
      keep_points[iPt]=1;
    }
    else
    {
      keep_points[iPt]=0;
    }
  }
  vector<int> keep_indices;
  keep_indices.resize(num_keep);
  unsigned int idx_keep=0;
  for(unsigned int iPt=0;iPt<num_pts;iPt++)
  {
    if(keep_points[iPt])
    {
      keep_indices[idx_keep]=iPt;
      idx_keep++;
    }
  }
  ROS_INFO_STREAM("Filtered cloud should have " << idx_keep <<"=" <<keep_indices.size() <<" points");

  cloud_geometry::getPointCloud(build_cloud.response.cloud, keep_indices, filtered_cloud_);
  
  ROS_INFO_STREAM("Filtered cloud has " <<filtered_cloud_.pts.size() <<" points");
  // Cluster objects using sinlge-link clustering
  
  
  //FYI std::vector<int> cluster_ids_;
  //FYI unsigned int num_clusters_;
  cluster_pcd_points(filtered_cloud_,max_link_distance_,cluster_ids_,num_clusters_);
  ROS_INFO_STREAM("Found " << num_clusters_ <<" clusters");
  if(num_clusters_==0)
  {
    return;
  }
  
  //FYI std::vector<std::vector<int> > clouds_by_indices_;
  cluster_ids_to_cluster_indices(cluster_ids_,num_clusters_,clouds_by_indices_);
  
  //FYI std::vector<boost::shared_ptr<sample_consensus::SACModelPlane> > plane_models_; 
  //FYI std::vector<std::vector<double> > plane_coeffs_;
  
  plane_models_.resize(num_clusters_);
  plane_coeffs_.resize(num_clusters_);
  
  unsigned int min_points_per_model_=100;

  // Fit a plane and in-plane rectangle to each of the objects.
  for(unsigned int iCluster=0;iCluster<num_clusters_;iCluster++)
  {
    
    vector<int> inliers;
    inliers.clear(); //Points that are in plane
    
    vector<double>& model=plane_coeffs_[iCluster];
    model.clear();  //Plane equation
    
    //Plane SAC model
    boost::shared_ptr<sample_consensus::SACModelPlane> & plane_model=plane_models_[iCluster]; 

    ROS_INFO("\tCluster %d has %d points", iCluster, clouds_by_indices_[iCluster].size() );

    if( clouds_by_indices_[iCluster].size()<min_points_per_model_ )
    {
      ROS_INFO_STREAM("\tskipping model " << iCluster );
      continue;
    }

    fitSACPlane(filtered_cloud_, clouds_by_indices_[iCluster], inliers, model, plane_model, 0.02, min_points_per_model_);

    ROS_INFO_STREAM("\tGot model " << model.size() );
  }
  
  ROS_INFO_STREAM("\tBuilding model KD-tree " );
  // build_models_kdtree
  //FYI boost::shared_ptr<cloud_kdtree::KdTreeANN> object_points_kd_tree_;
  object_points_kd_tree_=boost::shared_ptr<cloud_kdtree::KdTreeANN>(new cloud_kdtree::KdTreeANN(filtered_cloud_));

  ROS_INFO_STREAM("Setup done.");
}

void PlanarObjectDetector::cloudCallback(const PointCloudConstPtr& the_cloud)
{
  boost::mutex::scoped_lock lock(proc_mutex_);
  try{
    ROS_INFO("Received point cloud, starting detection");
    cloud_ = the_cloud;
    detectObject(*cloud_);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what()) ;
    return ;
  }
}

  /*


	void filterPointCloud(const PointCloud& in, PointCloud& out)
	{
		out.header.stamp = in.header.stamp;
		out.header.frame_id = in.header.frame_id;

		out.chan.resize(in.chan.size());
		for (size_t c=0;c<in.chan.size();++c) {
			out.chan[c].name = in.chan[c].name;
		}

		for (size_t i=0;i<in.get_pts_size();++i) {
			if (in.pts[i].z>min_height_) {
				out.pts.push_back(in.pts[i]);
				for (size_t c=0;c<in.chan.size();++c) {
					out.chan[c].vals.push_back(in.chan[c].vals[i]);
				}
			}
		}
	}
  */
void PlanarObjectDetector::detectObject(const PointCloud& point_cloud)
{
  PointCloud global_cloud;
  tf_.transformPointCloud("map",point_cloud,global_cloud);

  double max_search_radius_=0.1;

  std::vector<std::deque<int> > model_to_observation;
  model_to_observation.resize(num_clusters_);

  unsigned int num_pts=point_cloud.pts.size();
  for(unsigned int iPt=0;iPt<num_pts;iPt++)
  {
    std::vector<int> k_indices;
    std::vector<float> k_distances;
    object_points_kd_tree_->radiusSearch(global_cloud, iPt, max_search_radius_, k_indices, k_distances);
    if(k_distances.size()==0){
      continue;
    }
    if(k_distances[0]>max_search_radius_)
    {
      continue;
    }
    unsigned int model_id=cluster_ids_[k_indices[0]];
    model_to_observation[model_id].push_back(iPt);
  }

  for(unsigned int iModel=0;iModel<num_clusters_;iModel++)
  {
    unsigned int num_model_observations = model_to_observation[iModel].size();

    ROS_INFO("Model %d has %d observations",iModel,num_model_observations);
    if(num_model_observations<50)
    {
      ROS_INFO("\t skipping");
      continue;
    }
    
    std::vector<int> vec_observations(model_to_observation[iModel].begin(),model_to_observation[iModel].end());
    fitObjectModel2Cloud(iModel,global_cloud,point_cloud,vec_observations);
    
  }
}
void PlanarObjectDetector::fitObjectModel2Cloud(const unsigned int model_id,const PointCloud& global_cloud,const PointCloud& local_cloud,const std::vector<int> observation_ids)
{

  std::string observation_frame=local_cloud.header.frame_id;

  PointCloud model_cloud_global;
  PointCloud model_cloud_local;
  cloud_geometry::getPointCloud(filtered_cloud_, clouds_by_indices_[model_id], model_cloud_global);

  model_cloud_global.header.stamp = local_cloud.header.stamp; //Assume that global frame is fixed.
  tf_.transformPointCloud(observation_frame, model_cloud_global, model_cloud_local);


  vector<int> full_model_indices;
  full_model_indices.resize(model_cloud_local.pts.size());
  for(unsigned int i=0;i<model_cloud_local.pts.size();i++)
    full_model_indices[i]=i;



  vector<int> inliers;
  inliers.clear(); //Points that are in plane
    
  vector<double>& model=plane_coeffs_[model_id];
  model.clear();  //Plane equation
  
  //Plane SAC model
  boost::shared_ptr<sample_consensus::SACModelPlane> plane_model;

  unsigned int min_points_per_model_local_=50;
  bool is_fitting_ok=fitSACPlane(local_cloud, observation_ids, inliers, model, plane_model, 0.02, min_points_per_model_local_);

  if(! is_fitting_ok)
  {
    ROS_INFO("Model fitting failed");
    return;
  }

  PointCloud model_cloud_projected;
  cloud_geometry::projections::pointsToPlane(model_cloud_local,full_model_indices,model_cloud_projected, model);
  model_cloud_projected.header=model_cloud_local.header;


  float min_h=1e15,max_h=1e10;
  float min_w=1e15,max_w=1e10;

  Point32 dir_line1,offset;
  dir_line1.x=0;
  dir_line1.y=0;
  dir_line1.z=1;
  offset.x=model[0]*model[3];
  offset.y=model[1]*model[3];
  offset.z=model[2]*model[3];
  
  variationAlongLine(dir_line1,offset,model_cloud_projected,full_model_indices,min_h,max_h);

  Point32 dir_line2;
  btVector3 plane_n(model[0],model[1],model[2]);
  btVector3 vertical(0,0,1);
  btVector3 horizontal = vertical.cross(plane_n).normalized();
  dir_line2.x=horizontal.x();
  dir_line2.y=horizontal.y();
  dir_line2.z=horizontal.z();

  variationAlongLine(dir_line2,offset,model_cloud_projected,full_model_indices,min_w,max_w);
  ROS_INFO_STREAM("\t ["<< min_w << "," << max_w << "," <<min_h << "," << max_h <<"]");

  PointStamped object_origin;
  object_origin.point.x = offset.x-dir_line2.x*min_w;
  object_origin.point.y = offset.y-dir_line2.y*min_w;
  object_origin.point.z = offset.z-dir_line2.z*min_h;

  object_origin.header.stamp = local_cloud.header.stamp;
  object_origin.header.frame_id = local_cloud.header.frame_id;


  PointStamped object_ul;
  object_ul.point.x = offset.x+dir_line2.x*max_w;
  object_ul.point.y = offset.y+dir_line2.y*max_w;
  object_ul.point.z = offset.z+dir_line2.z*max_h;

  object_ul.header.stamp = local_cloud.header.stamp;
  object_ul.header.frame_id = local_cloud.header.frame_id;


  ROS_INFO_STREAM("\t X-range:["<<object_origin.point.x <<"," << object_ul.point.x << "]");
  ROS_INFO_STREAM("\t Y-range:["<<object_origin.point.y <<"," << object_ul.point.y << "]");
  ROS_INFO_STREAM("\t Z-range:["<<object_origin.point.z <<"," << object_ul.point.z << "]");

  addObjectFrame(object_origin, model);

  publishObjectMarker(max_w-min_w,max_h-min_h);

  publishObjectMarker2(object_origin.point.x,object_origin.point.y,object_origin.point.z);


  cloud_pub_.publish(model_cloud_projected);

  /*  PointStamped laser_origin;
  laser_origin.header.frame_id = local_cloud.header.frame_id;
  laser_origin.header.stamp = local_cloud.header.stamp;
  laser_origin.point.x = 0;
  laser_origin.point.y = 0;
  laser_origin.point.z = 0;
  
  
  PointStamped odom_laser_origin;
  tf_.transformPoint("odom_combined", laser_origin, odom_laser_origin);
  
  Point32 before_projection;
  before_projection.x = odom_laser_origin.point.x;
  before_projection.y = odom_laser_origin.point.y;
  before_projection.z = odom_laser_origin.point.z;
  Point32 origin_projection;
  
  cloud_geometry::projections::pointToPlane(before_projection, origin_projection, model);
  
  PointStamped origin;
  origin.header.stamp = odom_laser_origin.header.stamp;
  origin.header.frame_id = odom_laser_origin.header.frame_id;
  origin.point.x = origin_projection.x;
  origin.point.y = origin_projection.y;
  origin.point.z = origin_projection.z;
  */


        /*



		filterPointCloud(odom_cloud,cloud);

		cloud_pub_.publish(cloud);

		vector<int> indices(cloud.get_pts_size());
		for (size_t i=0;i<cloud.get_pts_size();++i) {
			indices[i] = i;
		}
		Point32 viewpoint;
		viewpoint.x = viewpoint.y = viewpoint.z = 0;

		// Use the entire data to estimate the plane equation.
		vector<int> inliers;
		inliers.clear(); //Points that are in plane

		vector<double> model;
		model.clear();  //Plane equation

		fitSACPlane(cloud, indices, viewpoint, inliers, model, 0.02, 50);


		PointStamped laser_origin;
		laser_origin.header.frame_id = point_cloud.header.frame_id;
		laser_origin.header.stamp = point_cloud.header.stamp;
		laser_origin.point.x = 0;
		laser_origin.point.y = 0;
		laser_origin.point.z = 0;


		PointStamped odom_laser_origin;
		tf_.transformPoint("odom_combined", laser_origin, odom_laser_origin);

		Point32 before_projection;
		before_projection.x = odom_laser_origin.point.x;
		before_projection.y = odom_laser_origin.point.y;
		before_projection.z = odom_laser_origin.point.z;
		Point32 origin_projection;

		cloud_geometry::projections::pointToPlane(before_projection, origin_projection, model);

		PointStamped origin;
		origin.header.stamp = odom_laser_origin.header.stamp;
		origin.header.frame_id = odom_laser_origin.header.frame_id;
		origin.point.x = origin_projection.x;
		origin.point.y = origin_projection.y;
		origin.point.z = origin_projection.z;

		addWhiteboardFrame(origin, model);
          */
}


void  PlanarObjectDetector::addObjectFrame(PointStamped origin, const vector<double>& plane)
{
  
  btVector3 position(origin.point.x,origin.point.y,origin.point.z);
  
  btQuaternion orientation;
  btMatrix3x3 rotation;
  btVector3 z(plane[0],plane[1],plane[2]);
  btVector3 y(0,0,1);
  btVector3 x = y.cross(z).normalized();
  rotation[0] = x; 	// x
  rotation[1] = y; 	// y
  rotation[2] = z; 	// z
  rotation = rotation.transpose();
  rotation.getRotation(orientation);
  
  tf::Transform tf_pose(orientation, position);
  
  // add wall_frame to tf
  tf::Stamped<tf::Pose> table_pose_frame(tf_pose, origin.header.stamp, "whiteboard_frame", origin.header.frame_id);
  
  tf_.setTransform(table_pose_frame);
  
  broadcaster_.sendTransform(table_pose_frame);
}


bool PlanarObjectDetector::fitSACPlane (const PointCloud& points, const vector<int> &indices, 
                                        vector<int> &inliers, vector<double> &coeff, // output
                                        boost::shared_ptr<sample_consensus::SACModelPlane> &model_output, // output
                                        double dist_thresh, int min_points_per_model)
{
    // Create and initialize the SAC model
  boost::shared_ptr<sample_consensus::SACModelPlane> model(new sample_consensus::SACModelPlane ());
  sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model.get(), dist_thresh);
  sac->setMaxIterations (100);
  model->setDataSet ((PointCloud*)&points, indices);
  
  // Search for the best plane
  if (sac->computeModel ()) {
    sac->computeCoefficients (coeff);                              // Compute the model coefficients
    sac->refineCoefficients (coeff);                             // Refine them using least-squares
    
    // Get the list of inliers
    model->selectWithinDistance (coeff, dist_thresh, inliers);
    
    if ((int)inliers.size()<min_points_per_model) {
      return false;
    }
    
    // Flip the plane normal towards the viewpoint
    //cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points.pts.at(inliers[0]), viewpoint);
    
    ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (), coeff[0], coeff[1], coeff[2], coeff[3]);
  }
  else {
    ROS_ERROR ("Could not compute a planar model for %d points.", indices.size());
    return false;
  }
  model_output=model;
  delete sac;
  return true;
}




void PlanarObjectDetector::publishObjectMarker(float w,float h)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "whiteboard_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "whiteboard";
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

  marker.lifetime = ros::Duration(10.0);


  tf::Point pt(0,0,0);
  robot_msgs::Point pt_msg;
  tf::pointTFToMsg(pt, pt_msg);
  marker.points.push_back(pt_msg);

  tf::Point pt2(1,0,0);
  tf::pointTFToMsg(pt2, pt_msg);
  marker.points.push_back(pt_msg);

  tf::Point pt3(1,1,0);
  tf::pointTFToMsg(pt3, pt_msg);
  marker.points.push_back(pt_msg);

  tf::Point pt4(0,1,0);
  tf::pointTFToMsg(pt4, pt_msg);
  marker.points.push_back(pt_msg);

  
  marker_pub_.publish(marker);
}




void PlanarObjectDetector::publishObjectMarker2(float x,float y,float z)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom_combined";
  marker.header.stamp = ros::Time();
  marker.ns = "whiteboard-1";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.lifetime = ros::Duration(10.0);


  tf::Point pt(x+0,y+0,z+0);
  robot_msgs::Point pt_msg;
  tf::pointTFToMsg(pt, pt_msg);
  marker.points.push_back(pt_msg);

  tf::Point pt2(x+1,y+0,z+0);
  tf::pointTFToMsg(pt2, pt_msg);
  marker.points.push_back(pt_msg);

  tf::Point pt3(x+1,y+1,z+0);
  tf::pointTFToMsg(pt3, pt_msg);
  marker.points.push_back(pt_msg);

  tf::Point pt4(0,1,0);
  tf::pointTFToMsg(pt4, pt_msg);
  marker.points.push_back(pt_msg);

  
  marker_pub_.publish(marker);
}





/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "whiteboard_detector");


  PlanarObjectDetector pod;

  pod.setup();


  ros::spin();

  return (0);
}
/* ]--- */

