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

#include <ros/ros.h>
#include "tabletop_objects/ModelFit.h"
#include "tabletop_objects/mesh_loader.h"

#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "mapping_msgs/Object.h"

#include "tf/transform_datatypes.h"

#include "distance_field/propagation_distance_field.h"


//#define PUBLISH_GRASH_POSE

#ifdef PUBLISH_GRASH_POSE
#include <tf/transform_broadcaster.h>
#endif

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

	void getExtents(geometry_msgs::Point32& low_extent, geometry_msgs::Point32& high_extent)
	{
		geometry_msgs::Pose pose = graspPose();

		low_extent.x = pose.position.x-min_.x;
		low_extent.y = pose.position.y-min_.y;
		low_extent.z = pose.position.z-min_.z;

		high_extent.x = min_.x-pose.position.x;
		high_extent.y = min_.y-pose.position.y;
		high_extent.z = min_.z-pose.position.z;

	}

	mapping_msgs::Object objectMesh()
	{
		return mesh_;
	}

	void show(const ros::Publisher& publisher,const ros::Time& time, const geometry_msgs::Point32& location, float fit_score);

	double fitPointCloud(const sensor_msgs::PointCloud& cloud, const geometry_msgs::Point32& location, geometry_msgs::Point32& vector, ModelFitSet& mfs);

	void findBestFit(const sensor_msgs::PointCloud& cloud, ModelFitSet& mfs);
};


class ModelFitSet
{
public:
	struct ModelFit
	{
		TemplateModel* model_;
		geometry_msgs::Point32 location_;
		float score_;
		float max_dist_;

		ModelFit() : model_(NULL)
		{
		}

		ModelFit(TemplateModel* model, const geometry_msgs::Point32& location, float score, float max_dist) :
			model_(model), location_(location), score_(score), max_dist_(max_dist)
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
			pose.position.x = location_.x;
			pose.position.y = location_.y;
			pose.position.z = location_.z;

			pose.orientation.x = 0;
			pose.orientation.y = 0;
			pose.orientation.z = 0;
			pose.orientation.w = 1.0;

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

	void add(TemplateModel* model, const geometry_msgs::Point32& location, float score, float max_dist)
	{
		ModelFit crt(model,location, score, max_dist);

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


};



void TemplateModel::load(const string& file)
{
	PLYModelLoader modle_loader;
	modle_loader.readFromFile(file,mesh_);

	geometry_msgs::Point min, max;

	if (mesh_.vertices.size()>0) {
		min = mesh_.vertices[0];
		max = mesh_.vertices[0];

		for (size_t i=0;i<mesh_.vertices.size();++i) {

			if (min.x > mesh_.vertices[i].x) min.x = mesh_.vertices[i].x;
			if (min.y > mesh_.vertices[i].y) min.y = mesh_.vertices[i].y;
			if (min.z > mesh_.vertices[i].z) min.z = mesh_.vertices[i].z;
			if (max.x < mesh_.vertices[i].x) max.x = mesh_.vertices[i].x;
			if (max.y < mesh_.vertices[i].y) max.y = mesh_.vertices[i].y;
			if (max.z < mesh_.vertices[i].z) max.z = mesh_.vertices[i].z;
		}
	}

	ROS_INFO("Size: (%g,%g,%g, %g, %g, %g)\n",min.x, min.y, min.z, max.x, max.y, max.z);

	distance_voxel_grid_ = new distance_field::PropagationDistanceField(max.x-min.x,max.y-min.y, max.z-min.z, 0.002, min.x,min.y,min.z, 1.0 );

	std::vector<btVector3> points;
	points.reserve(mesh_.vertices.size());
	for (size_t i=0; i<mesh_.vertices.size(); ++i)
	{
		points.push_back(btVector3(mesh_.vertices[i].x,mesh_.vertices[i].y,mesh_.vertices[i].z));
	}
	distance_voxel_grid_->reset();
	distance_voxel_grid_->addPointsToField(points);
}


void TemplateModel::show(const ros::Publisher& publisher, const ros::Time& time, const geometry_msgs::Point32& location, float fit_score)
{
	static int model_id = 0;

	visualization_msgs::Marker marker;
	marker.header.stamp = time;
	marker.header.frame_id = "table_frame";
	marker.ns = "voxel_grid";
	marker.id = model_id++;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = location.x;
	marker.pose.position.y = location.y;
	marker.pose.position.z = location.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = distance_voxel_grid_->getResolution(distance_field::PropagationDistanceField::DIM_X)/2;
	marker.scale.y = distance_voxel_grid_->getResolution(distance_field::PropagationDistanceField::DIM_Y)/2;
	marker.scale.z = distance_voxel_grid_->getResolution(distance_field::PropagationDistanceField::DIM_Z)/2;
	marker.color.a = 1.0;


	if (fit_score<7.0) {
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
	} else {
		return;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
	}

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


double TemplateModel::fitPointCloud(const sensor_msgs::PointCloud& cloud, const geometry_msgs::Point32& location, geometry_msgs::Point32& vector, ModelFitSet& mfs)
{
	double score = 0;
	double max_dist = 0;

	vector.x = 0;
	vector.y = 0;
	vector.z = 0;
	int cnt = 0;

	for (size_t i=0;i<cloud.points.size();i++) {
		double wx = cloud.points[i].x-location.x;
		double wy = cloud.points[i].y-location.y;
		double wz = cloud.points[i].z-location.z;

		int x, y, z;
		double val;
		if (distance_voxel_grid_->worldToGrid(wx,wy,wz,x,y,z)) {
			distance_field::PropDistanceFieldVoxel& voxel = distance_voxel_grid_->getCell(x,y,z);
			double cx, cy, cz;
			distance_voxel_grid_->gridToWorld(voxel.closest_point_[0],voxel.closest_point_[1],voxel.closest_point_[2],
						cx,cy,cz);
			val = distance_voxel_grid_->getDistanceFromCell(x,y,z);
			vector.x += (cx-wx);
			vector.y += (cy-wy);
			vector.z += (cz-wz);
			cnt++;
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
	if (cnt!=0) {
		vector.x /=  cnt;
		vector.y /=  cnt;
		vector.z /=  cnt;
	}

	mfs.add(this, location, score, max_dist);

	return score;
}

void TemplateModel::findBestFit(const sensor_msgs::PointCloud& cloud, ModelFitSet& mfs)
{
	// compute center of point cloud
	geometry_msgs::Point32 center;
	center.x =0; center.y = 0; center.z = 0;
	int count = cloud.points.size();

	for (int i=0;i<count;++i) {
		center.x += cloud.points[i].x;
		center.y += cloud.points[i].y;
		center.z = min(center.z, cloud.points[i].z );
	}
	center.x /=count;
	center.y /=count;

	geometry_msgs::Point32 location = center;
	geometry_msgs::Point32 vector;


	double newcost = fitPointCloud(cloud, location, vector, mfs);;
	double cost = newcost + 1;

	while (newcost<cost) {
		cost = newcost;
		location.x -= vector.x;
		location.y -= vector.y;
//		location.z -= vector.z;
		newcost = fitPointCloud(cloud, location, vector, mfs);
	}
}





class ModelFitter
{
	ros::NodeHandle nh_;
	ros::ServiceServer service_;
	ros::Publisher marker_pub_;

	string template_path;

	vector<TemplateModel*> templates;

#ifdef PUBLISH_GRASH_POSE
	tf::TransformBroadcaster tfb_;
#endif

public:
	ModelFitter()
	{
		nh_.param<string>("~template_path", template_path, "");
		service_ = nh_.advertiseService("tabletop_objects/model_fit", &ModelFitter::fitModel, this);

		marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker",1);

		loadTemplateModels(template_path);
	}

	~ModelFitter()
	{
		for (size_t i=0;i<templates.size();++i) {
			delete templates[i];
		}
	}


	void loadTemplate(const string& filename)
	{
		TemplateModel* tpl = new TemplateModel();
		tpl->load(filename);
		templates.push_back(tpl);
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


	void fitBestModel(const sensor_msgs::PointCloud& cloud, ModelFitSet& mfs)
	{
		for (size_t i=0;i<templates.size();++i) {
			templates[i]->findBestFit(cloud, mfs);
		}
	}

	bool fitModel(tabletop_objects::ModelFit::Request& req,
			tabletop_objects::ModelFit::Response& resp)
	{
		ROS_INFO("Service called");

		ModelFitSet mfs(1);

		clock_t start = clock();
		fitBestModel(req.cloud, mfs);
		double duration = double(clock()-start)/CLOCKS_PER_SEC;

		ROS_INFO("Best score: %g, time:%g", mfs.best_fit_[0].score(), duration);

		mfs.best_fit_[0].model_->show(marker_pub_, req.cloud.header.stamp,  mfs.best_fit_[0].location_,  mfs.best_fit_[0].score());
//		mfs.best_fit_[0].model_->distance_voxel_grid_->visualize(0.0,0.0001, "table_frame", ros::Time::now());

		resp.object.grasp_pose.pose = mfs.best_fit_[0].graspPose();
		resp.object.grasp_pose.header.stamp = req.cloud.header.stamp;
		resp.object.grasp_pose.header.frame_id = req.cloud.header.frame_id;
		resp.object.object_pose.pose = mfs.best_fit_[0].objectPose();
		resp.object.object_pose.header.stamp = req.cloud.header.stamp;
		resp.object.object_pose.header.frame_id = req.cloud.header.frame_id;
		resp.object.object = mfs.best_fit_[0].model_->objectMesh();
		resp.score = mfs.best_fit_[0].score();





#ifdef PUBLISH_GRASH_POSE
		tf::Pose tf_pose;
		tf::poseMsgToTF(mfs.best_fit_[0].graspPose(), tf_pose);
		tf::Stamped<tf::Pose> grasp_frame(tf_pose, req.cloud.header.stamp, "grasp_frame", req.cloud.header.frame_id);
		tfb_.sendTransform(grasp_frame);
#endif



		return true;
	}
};


}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "model_fitter");

	model_fit::ModelFitter mf;
	ros::spin();
}
