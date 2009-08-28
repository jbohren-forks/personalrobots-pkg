/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Marius Muja

#include <boost/filesystem.hpp>
#include <boost/numeric/ublas/matrix.hpp>


#include <ros/ros.h>
#include "tabletop_objects/ModelFit.h"
#include "tabletop_objects/mesh_loader.h"

#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "mapping_msgs/Object.h"
#include "tabletop_objects/TrackObject.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include "distance_field/propagation_distance_field.h"
#include <point_cloud_mapping/geometry/transforms.h>


#include <Eigen/Core>
#include <Eigen/Geometry>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN


//#define PUBLISH_GRASH_POSE

namespace bfs = boost::filesystem;
using namespace std;



namespace model_fit {



class ModelFitSet;

class TemplateModel {

public:
	distance_field::PropagationDistanceField* distance_voxel_grid_;
	float truncate_value;
	mapping_msgs::Object mesh_;

	geometry_msgs::Point min_, max_;

	TemplateModel() : truncate_value(0.10)
	{

	}

	~TemplateModel()
	{
		delete distance_voxel_grid_;
	}

	void load(const string& file);

	geometry_msgs::Pose graspPose()
	{
		// TODO: fix this, models annotated with pose
		geometry_msgs::Pose pose;
		pose.position.x = 0;
		pose.position.y = 0;
		pose.position.z = 0.05;


		btMatrix3x3 rotation;
		rotation.setIdentity();
		rotation.setEulerZYX(M_PI/2,0,0);
		btQuaternion orientation;
		rotation.getRotation(orientation);

		pose.orientation.x = orientation.x();
		pose.orientation.y = orientation.y();
		pose.orientation.z = orientation.z();
		pose.orientation.w = orientation.w();

		return pose;
	}

	void getExtents(geometry_msgs::Point& low_extent, geometry_msgs::Point& high_extent)
	{
		low_extent = min_;
		high_extent = max_;
	}

	mapping_msgs::Object objectMesh()
	{
		return mesh_;
	}

	void show(const ros::Publisher& publisher, const geometry_msgs::PoseStamped& pose, float fit_score);

	bool fitPointCloud(sensor_msgs::PointCloud& cloud, ModelFitSet& mfs, Eigen::Matrix4d& transform, double& best_score);

	void findBestFit(sensor_msgs::PointCloud& cloud, ModelFitSet& mfs);
};


class ModelFitSet
{
public:
	struct ModelFit
	{
		TemplateModel* model_;
		Eigen::Matrix4d transform_;
		float score_;
		float max_dist_;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		ModelFit() : model_(NULL)
		{
		}

		ModelFit(TemplateModel* model, const Eigen::Matrix4d& transform, float score, float max_dist) :
			model_(model), transform_(transform), score_(score), max_dist_(max_dist)
		{
		}

		float score() {
			return score_;
		}

		geometry_msgs::Pose graspPose()
		{
			geometry_msgs::Pose object_pose = objectPose();
			geometry_msgs::Pose model_grasp_pose = model_->graspPose();
			tf::Pose tf_object_pose;
			tf::Pose tf_model_grasp_pose;

			tf::poseMsgToTF(object_pose, tf_object_pose);
			tf::poseMsgToTF(model_grasp_pose, tf_model_grasp_pose);

			tf::Pose tf_grasp_pose = tf_object_pose*tf_model_grasp_pose;
			geometry_msgs::Pose grasp_pose;

			tf::poseTFToMsg(tf_grasp_pose, grasp_pose);


			return grasp_pose;
		}

		geometry_msgs::Pose objectPose()
		{
			geometry_msgs::Pose pose;
			pose.position.x = transform_(0,3);
			pose.position.y = transform_(1,3);
			pose.position.z = transform_(2,3);


			Eigen::Quaterniond q(transform_.corner<3,3>(Eigen::TopLeft));
			pose.orientation.x = q.x();
			pose.orientation.y = q.y();
			pose.orientation.z = q.z();
			pose.orientation.w = q.w();

			return pose;
		}

	};
	int size_;
	int count_;
	ModelFit* best_fit_;

	ModelFitSet(int size) : size_(size), count_(0)
	{
		best_fit_ = new ModelFit[size_];
	}

	void add(TemplateModel* model, const Eigen::Matrix4d& transform, float score, float max_dist)
	{
		ModelFit crt(model,transform, score, max_dist);

		if (count_ < size_) {
			best_fit_[count_++] = crt;
		}
		else if (best_fit_[size_-1].score()>crt.score()) {
			int i=size_-1;
			while (i>0 && best_fit_[i].score()>crt.score()) {
				best_fit_[i] = best_fit_[i-1];
				i--;
			}
			best_fit_[i] = crt;
		}
	}

	float bestScore()
	{
		if (count_==0) {
			return numeric_limits<float>::max();
		}
		else {
			return best_fit_[0].score_;
		}
	}


};



void TemplateModel::load(const string& file)
{
	PLYModelLoader modle_loader;
	modle_loader.readFromFile(file,mesh_);

	if (mesh_.vertices.size()>0) {
		min_ = mesh_.vertices[0];
		max_ = mesh_.vertices[0];

		for (size_t i=0;i<mesh_.vertices.size();++i) {

			if (min_.x > mesh_.vertices[i].x) min_.x = mesh_.vertices[i].x;
			if (min_.y > mesh_.vertices[i].y) min_.y = mesh_.vertices[i].y;
			if (min_.z > mesh_.vertices[i].z) min_.z = mesh_.vertices[i].z;
			if (max_.x < mesh_.vertices[i].x) max_.x = mesh_.vertices[i].x;
			if (max_.y < mesh_.vertices[i].y) max_.y = mesh_.vertices[i].y;
			if (max_.z < mesh_.vertices[i].z) max_.z = mesh_.vertices[i].z;
		}
	}

	ROS_INFO("Size: (%g,%g,%g, %g, %g, %g)\n",min_.x, min_.y, min_.z, max_.x, max_.y, max_.z);

	distance_voxel_grid_ = new distance_field::PropagationDistanceField(max_.x-min_.x,max_.y-min_.y, max_.z-min_.z, 0.002, min_.x,min_.y,min_.z, 1.0 );

	std::vector<btVector3> points;
	points.reserve(mesh_.vertices.size());
	for (size_t i=0; i<mesh_.vertices.size(); ++i)
	{
		points.push_back(btVector3(mesh_.vertices[i].x,mesh_.vertices[i].y,mesh_.vertices[i].z));
	}
	distance_voxel_grid_->reset();
	distance_voxel_grid_->addPointsToField(points);
}


void TemplateModel::show(const ros::Publisher& publisher, const geometry_msgs::PoseStamped& pose, float fit_score)
{
	visualization_msgs::Marker marker;
	marker.header.stamp = pose.header.stamp;
	marker.header.frame_id = pose.header.frame_id;
	marker.ns = "voxel_grid";
	marker.id = 100;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = pose.pose;
	marker.scale.x = distance_voxel_grid_->getResolution(distance_field::PropagationDistanceField::DIM_X);
	marker.scale.y = distance_voxel_grid_->getResolution(distance_field::PropagationDistanceField::DIM_Y);
	marker.scale.z = distance_voxel_grid_->getResolution(distance_field::PropagationDistanceField::DIM_Z);
	marker.color.a = 1.0;


	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;

	for (int i=0;i<distance_voxel_grid_->getNumCells(distance_field::PropagationDistanceField::DIM_X);++i) {
		for (int j=0;j<distance_voxel_grid_->getNumCells(distance_field::PropagationDistanceField::DIM_Y);++j) {
			for (int k=0;k<distance_voxel_grid_->getNumCells(distance_field::PropagationDistanceField::DIM_Z);++k) {

				if (distance_voxel_grid_->getDistanceFromCell(i,j,k)<0.001) {
					geometry_msgs::Point p;
					distance_voxel_grid_->gridToWorld(i,j,k, p.x,p.y,p.z);
					marker.points.push_back(p);
				}
			}
		}
	}
	publisher.publish(marker);
}


bool TemplateModel::fitPointCloud(sensor_msgs::PointCloud& cloud, ModelFitSet& mfs, Eigen::Matrix4d& global_transform, double& best_score)
{
	double score = 0;
	double max_dist = 0;

	sensor_msgs::PointCloud closest;
	vector<int> indices_points;
	vector<int> indices_closest;
	indices_points.reserve(cloud.points.size());

	// compute the cost of the current fit
	for (size_t i=0;i<cloud.points.size();i++) {
		double wx = cloud.points[i].x;
		double wy = cloud.points[i].y;
		double wz = cloud.points[i].z;

		int x, y, z;
		double val;
		if (distance_voxel_grid_->worldToGrid(wx,wy,wz,x,y,z)) {

			distance_field::PropDistanceFieldVoxel& voxel = distance_voxel_grid_->getCell(x,y,z);
			double cx, cy, cz;
			distance_voxel_grid_->gridToWorld(voxel.closest_point_[0],voxel.closest_point_[1],voxel.closest_point_[2],
						cx,cy,cz);

			indices_points.push_back(i);
			geometry_msgs::Point32 closest_point;
			closest_point.x = cx;
			closest_point.y = cy;
			closest_point.z = cz;
			closest.points.push_back(closest_point);

			val = distance_voxel_grid_->getDistanceFromCell(x,y,z);
		}
		else {
			val = truncate_value;
		}
		if (val>truncate_value) {
			val = truncate_value;
		}
		max_dist = max(max_dist,val);
		score += val;
	}
	score /= (cloud.points.size());

	// if cost worst than the best so far, stop  local minima
	if (score>=best_score) {
		return false;
	}

	best_score = score;

	printf("Score: %g\n", score);

	// got better cost, add transform to set
	Eigen::Matrix4d transform;
	cloud_geometry::transforms::getInverseTransformation(global_transform, transform);
	mfs.add(this, transform, score, max_dist);

	printf("Computing transform\n");

	// continue refining transform
	indices_closest.resize(indices_points.size());
	for (size_t i = 0;i<indices_points.size();++i) {
		indices_closest[i] = i;
	}

	cloud_geometry::transforms::getPointsRigidTransformation(cloud, indices_points, closest, indices_closest, transform);
	global_transform *= transform;

	for (size_t i=0;i<cloud.points.size();i++) {
		geometry_msgs::Point32 transformed_point;
		cloud_geometry::transforms::transformPoint(cloud.points[i],transformed_point, transform);
		cloud.points[i] = transformed_point;
	}

	return true;
}



void visualize(const sensor_msgs::PointCloud& pc)
{
	ros::NodeHandle nh;

	static ros::Publisher cloud_pub;

	if (!cloud_pub) {
		cloud_pub = nh.advertise<sensor_msgs::PointCloud>("debug_cloud", 1, true);
	}

	ros::Duration(0.5).sleep();

	cloud_pub.publish(pc);
}


void visualize(ros::Publisher pub,const geometry_msgs::PoseStamped& pose)
{
	static int id = 200;
	visualization_msgs::Marker marker;
	marker.header.frame_id = pose.header.frame_id;
	marker.header.stamp = pose.header.stamp;
	marker.ns = "voxel_grid";
	marker.id = id++;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = pose.pose;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 1.0;


	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	pub.publish(marker);
}



ros::Publisher getMarkerPublisher()
{
	ros::NodeHandle nh;
	static ros::Publisher marker_pub;
	if (!marker_pub) {
		marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
	}

	return marker_pub;
}


/** \brief Convert the transform to a Homogeneous matrix for large operations */
static boost::numeric::ublas::matrix<double> transformAsMatrix(const tf::Transform& bt)
{
  boost::numeric::ublas::matrix<double> outMat(4,4);

  //  double * mat = outMat.Store();

  double mv[12];
  bt.getBasis().getOpenGLSubMatrix(mv);

  tf::Vector3 origin = bt.getOrigin();

  outMat(0,0)= mv[0];
  outMat(0,1)  = mv[4];
  outMat(0,2)  = mv[8];
  outMat(1,0)  = mv[1];
  outMat(1,1)  = mv[5];
  outMat(1,2)  = mv[9];
  outMat(2,0)  = mv[2];
  outMat(2,1)  = mv[6];
  outMat(2,2) = mv[10];

  outMat(3,0)  = outMat(3,1) = outMat(3,2) = 0;
  outMat(0,3) = origin.x();
  outMat(1,3) = origin.y();
  outMat(2,3) = origin.z();
  outMat(3,3) = 1;


  return outMat;
};


void transformPointCloud(const tf::Transform& net_transform, const sensor_msgs::PointCloud & cloudIn, sensor_msgs::PointCloud & cloudOut)
{
  boost::numeric::ublas::matrix<double> transform = transformAsMatrix(net_transform);

  unsigned int length = cloudIn.get_points_size();

  boost::numeric::ublas::matrix<double> matIn(4, length);

  double * matrixPtr = matIn.data().begin();

  for (unsigned int i = 0; i < length ; i++)
  {
    matrixPtr[i] = cloudIn.points[i].x;
    matrixPtr[i+length] = cloudIn.points[i].y;
    matrixPtr[i+ 2* length] = cloudIn.points[i].z;
    matrixPtr[i+ 3* length] = 1;
  };

  boost::numeric::ublas::matrix<double> matOut = prod(transform, matIn);

  // Copy relevant data from cloudIn, if needed
  if (&cloudIn != &cloudOut)
  {
    cloudOut.header = cloudIn.header;
    cloudOut.set_points_size(length);
    cloudOut.set_channels_size(cloudIn.get_channels_size());
    for (unsigned int i = 0 ; i < cloudIn.get_channels_size() ; ++i)
      cloudOut.channels[i] = cloudIn.channels[i];
  }

  matrixPtr = matOut.data().begin();

  //Override the positions
  for (unsigned int i = 0; i < length ; i++)
  {
    cloudOut.points[i].x = matrixPtr[i];
    cloudOut.points[i].y = matrixPtr[i + length];
    cloudOut.points[i].z = matrixPtr[i + 2* length];
  };
}


void TemplateModel::findBestFit(sensor_msgs::PointCloud& cloud, ModelFitSet& mfs)
{

//	visualize(getMarkerPublisher(), pose_stamped);
//	show(getMarkerPublisher(), pose_stamped, 1);
//
//
//	geometry_msgs::PoseStamped ps;
//	ps.header = pose_stamped.header;
//	ps.pose.orientation.w = 1.0;
//	visualize(getMarkerPublisher(), ps);
//	show(getMarkerPublisher(), ps, 1);
//
//	transformPointCloud(transf.inverse(), cloud, new_cloud);
//	//	cloud_geometry::transforms::transformPoints(cloud.points, new_cloud.points, cloud_transform);

//	visualize(new_cloud);

	Eigen::Matrix4d cloud_transform;
	cloud_transform.setIdentity();

	double best_score = numeric_limits<double>::max();
	while (fitPointCloud(cloud, mfs, cloud_transform, best_score)) ;
}





class ModelTracker
{
	ros::NodeHandle nh_;
	ros::ServiceServer service_;
	ros::Publisher marker_pub_;

	ros::Subscriber cloud_sub_;
	ros::Subscriber track_obj_sub_;

	string template_path;

	vector<TemplateModel*> templates;

	sensor_msgs::PointCloudConstPtr cloud_;

	geometry_msgs::PoseStamped object_pose_;
	std::string object_id_;
	int object_index_;
	bool tracking_on_;

	TemplateModel* track_template_;


	tf::TransformListener tf_;
	tf::TransformBroadcaster tb_;

	map<string, int> model_to_index_;

#ifdef PUBLISH_GRASH_POSE
	tf::TransformBroadcaster tfb_;
#endif

public:
	ModelTracker()
	{
		nh_.param<string>("~template_path", template_path, "");

		cloud_sub_ = nh_.subscribe(nh_.resolveName("stereo")+"/cloud", 1, &ModelTracker::cloudCallback, this);
		track_obj_sub_ = nh_.subscribe("track_object", 1, &ModelTracker::trackObjectCallback, this);

		marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker",1);

		loadTemplateModels(template_path);

		tracking_on_ = false;
	}

	~ModelTracker()
	{
		for (size_t i=0;i<templates.size();++i) {
			delete templates[i];
		}
	}

	void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud)
	{
		cloud_ = point_cloud;
		ROS_INFO("Got cloud");

		if (tracking_on_) {
			trackModel();
		}
	}

	void printPose(const geometry_msgs::PoseStamped& pose)
	{
		ROS_INFO("Pose: %s\nPosition: (%g, %g, %g)\nOrientation: (%g, %g, %g, %g)",
				pose.header.frame_id.c_str(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
				pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

	}

	void trackObjectCallback(const tabletop_objects::TrackObject::ConstPtr& object)
	{

		object_pose_ = object->pose;
		object_id_ = object->object_id;

		// publish tracking TF frame
		tf::Stamped<tf::Pose> transform;
		tf::poseStampedMsgToTF(object_pose_, transform);
		transform.parent_id_ = transform.frame_id_;
		transform.frame_id_ = "tracking_frame";
		tf_.setTransform(transform);
		tb_.sendTransform(transform);



		if (model_to_index_.find(object_id_) != model_to_index_.end()) {
			object_index_ = model_to_index_[object_id_];
			tracking_on_ =   true;
			track_template_ = templates[object_index_];

			ROS_INFO("Starting tracker");
		}

	}



	void loadTemplate(const string& filename)
	{
		TemplateModel* tpl = new TemplateModel();
		tpl->load(filename);
		templates.push_back(tpl);

		// keep mapping from mode_id (model name) to index
		std::string model_id = bfs::basename(filename);
		model_to_index_[model_id] = templates.size()-1;
	}

	void loadTemplateModels(const string& path)
	{
		bfs::path template_dir(path);

		if (!bfs::is_directory(template_dir)) {
			ROS_ERROR("Cannot load templates, %s is not a directory", template_dir.leaf().c_str());
		}

		bfs::directory_iterator dir_iter(template_dir), dir_end;
		for(;dir_iter != dir_end; ++dir_iter) {
			if (bfs::extension(*dir_iter)==".ply") {
				loadTemplate(dir_iter->string());
			}
		}

	}

#define OBJECT_CLOUD_DIST 0.25*0.25

	template <typename T, typename U>
	inline double distSquared3D(const T& a, const U& b) {
		return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
	}

	void filterPointCloud(const sensor_msgs::PointCloud& cloud, TemplateModel* model, sensor_msgs::PointCloud& object_cloud)
	{
		geometry_msgs::Point min, max;
		model->getExtents(min, max);

		const int padding = 0.01;

		printf("min: (%g,%g,%g), max: (%g,%g,%g)\n", min.x, min.y, min.z, max.x, max.y, max.z);

		object_cloud.header.frame_id = cloud.header.frame_id;
		object_cloud.header.stamp = cloud.header.stamp;
		for (size_t i=0;i<cloud.get_points_size();++i) {

			if ( (min.x - padding < cloud.points[i].x) &&
					(min.y - padding < cloud.points[i].y) &&
					(min.z - padding < cloud.points[i].z) &&
					(cloud.points[i].x < max.x + padding ) &&
					(cloud.points[i].y < max.y + padding ) &&
					(cloud.points[i].z < max.z + padding ) ) {

				object_cloud.points.push_back(cloud.points[i]);
			}
		}
	}

	void trackModel()
	{
		sensor_msgs::PointCloud tracking_cloud;
		sensor_msgs::PointCloud tracking_object_cloud;

		tf_.setExtrapolationLimit(ros::Duration(10));

		tf_.transformPointCloud("tracking_frame", *cloud_, tracking_cloud);
		filterPointCloud(tracking_cloud, track_template_, tracking_object_cloud);


		if (tracking_object_cloud.get_points_size()==0) {
			ROS_INFO("Lost tracking");
			tracking_on_ = false;
			return;
		}

		ModelFitSet mfs(1);
		clock_t start = clock();
		track_template_->findBestFit(tracking_object_cloud, mfs);
		double duration = double(clock()-start)/CLOCKS_PER_SEC;
		ROS_INFO("Tracker converged with score %g in %g ms", mfs.best_fit_[0].score(), duration*1000);
		geometry_msgs::PoseStamped ps;
		ps.pose = mfs.best_fit_[0].objectPose();
		ps.header.frame_id = tracking_object_cloud.header.frame_id;
		ps.header.stamp = tracking_object_cloud.header.stamp;

		geometry_msgs::PoseStamped ps_parent_frame;

		tf_.transformPose(cloud_->header.frame_id, ps, ps_parent_frame);

		// publish tracking TF frame
		tf::Stamped<tf::Pose> transform;
		tf::poseStampedMsgToTF(ps_parent_frame, transform);
		transform.parent_id_ = transform.frame_id_;
		transform.frame_id_ = "tracking_frame";
		tf_.setTransform(transform);
		tb_.sendTransform(transform);

		mfs.best_fit_[0].model_->show(marker_pub_, ps,  mfs.best_fit_[0].score());
	}

//	bool fitModel(tabletop_objects::ModelFit::Request& req,
//			tabletop_objects::ModelFit::Response& resp)
//	{
//		ROS_INFO("Service called");
//
//		ModelFitSet mfs(1);
//
//		clock_t start = clock();
//		fitBestModel(req.cloud, mfs);
//		double duration = double(clock()-start)/CLOCKS_PER_SEC;
//
//		ROS_INFO("Best score: %g, time:%g", mfs.best_fit_[0].score(), duration);
//
//		mfs.best_fit_[0].model_->show(marker_pub_, req.cloud.header.stamp,  mfs.best_fit_[0].objectPose(),  mfs.best_fit_[0].score());
////		mfs.best_fit_[0].model_->distance_voxel_grid_->visualize(0.0,0.0001, "table_frame", ros::Time::now());
//
//		resp.object.grasp_pose.pose = mfs.best_fit_[0].graspPose();
//		resp.object.grasp_pose.header.stamp = req.cloud.header.stamp;
//		resp.object.grasp_pose.header.frame_id = req.cloud.header.frame_id;
//		resp.object.object_pose.pose = mfs.best_fit_[0].objectPose();
//		resp.object.object_pose.header.stamp = req.cloud.header.stamp;
//		resp.object.object_pose.header.frame_id = req.cloud.header.frame_id;
//		resp.object.object = mfs.best_fit_[0].model_->objectMesh();
//		resp.score = mfs.best_fit_[0].score();
//
//
//
//
//		return true;
//	}
};


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "model_tracker");

	model_fit::ModelTracker mf;
	ros::spin();
}
