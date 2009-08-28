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

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include "tabletop_objects/mesh_loader.h"
#include "mapping_msgs/Object.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include "opencv/cvaux.h"

#include <string>

using namespace std;
namespace bfs = boost::filesystem;


class ComputeFeatures
{
	cv::Mesh3D* model_mesh_;
	cv::SpinImageModel* model_;

	ros::NodeHandle nh_;
	ros::Subscriber cloud_sub_;
	ros::Publisher marker_pub_;

	sensor_msgs::PointCloud::ConstPtr cloud_;

public:

	ComputeFeatures()
	{
		cloud_sub_ = nh_.subscribe(nh_.resolveName("stereo")+"/cloud", 1, &ComputeFeatures::cloudCallback, this);
		marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker",1);
	}

	void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud)
	{
		cloud_ = point_cloud;

		matchModel();
	}

	void loadModel(const string& filename)
	{
		mapping_msgs::Object mesh;
		PLYModelLoader modle_loader;
		modle_loader.readFromFile(filename,mesh);

		vector<cv::Point3f> points;
		points.resize(mesh.vertices.size());
		for (size_t i=0;i<mesh.vertices.size();++i) {
			points[i] = cv::Point3f(mesh.vertices[i].x,mesh.vertices[i].y,mesh.vertices[i].z);
		}

		model_mesh_ = new cv::Mesh3D(points);
		model_ = new cv::SpinImageModel(*model_mesh_);
		ROS_INFO("Constructing spin image model");
		model_->selectRandomSubset(0.02);
		model_->compute();
		ROS_INFO("Model has %d spin images", model_->getSpinCount());
	}


	void visualize(const vector<cv::Vec2i> points)
	{
		visualization_msgs::Marker marker;
		marker.header.stamp = cloud_->header.stamp;
		marker.header.frame_id = cloud_->header.frame_id;
		marker.ns = "spin_points";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.003;
		marker.scale.y = 0.003;
		marker.scale.z = 0.003;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;

		for (size_t i=0;i<points.size();++i) {
			geometry_msgs::Point p;
			p.x = cloud_->points[points[i][1]].x;
			p.y = cloud_->points[points[i][1]].y;
			p.z = cloud_->points[points[i][1]].z;
			marker.points.push_back(p);
		}

		marker_pub_.publish(marker);
	}

	void visualizeSpinImageLocations(const cv::SpinImageModel& model)
	{

		visualization_msgs::Marker marker;
		marker.header.frame_id = cloud_->header.frame_id;
		marker.header.stamp = ros::Time((uint64_t)0ULL);
		marker.ns = "normals";
		marker.id = 1;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.001;
		marker.color.a = 1.0;
		marker.color.g = 1.0;

		marker.set_points_size(2*model.getSpinCount());

		for (size_t i=0;i<model.getSpinCount();++i) {

			cv::Point3f point = model.getSpinVertex(i);
			cv::Point3f normal = model.getSpinNormal(i);
			float alpha = 0.02;

			marker.points[2*i].x = point.x;
			marker.points[2*i].y = point.y;
			marker.points[2*i].z = point.z;

			marker.points[2*i+1].x = point.x + alpha * normal.x;
			marker.points[2*i+1].y = point.y + alpha * normal.y;
			marker.points[2*i+1].z = point.z + alpha * normal.z;

		}

		marker_pub_.publish(marker);
	}


	void matchModel()
	{
		vector<cv::Point3f> points;
		points.resize(cloud_->get_points_size());
		for (size_t i=0;i<cloud_->get_points_size();++i) {
			points[i] = cv::Point3f(cloud_->points[i].x,cloud_->points[i].y,cloud_->points[i].z);
		}

		cv::Mesh3D scene_mesh(points);
		cv::SpinImageModel scene(scene_mesh);
		ROS_INFO("Computing scene spin images");
		scene.selectRandomSubset(0.05);
		scene.compute();

		ROS_INFO("Scene has %d spin images", scene.getSpinCount());


		visualizeSpinImageLocations(scene);

//		vector<vector<cv::Vec2i> > matches;
//		model_->match(scene, matches);
//
//		ROS_INFO("I have found %d match groups\n", matches.size());
//
//		for (size_t i=0;i<matches.size();++i) {
//			ROS_INFO("\tGroup %d, size: %d", i, matches[i].size());
//			for (size_t j=0;j<matches[i].size();++j) {
//				printf ("%d ", matches[i][j][0]);
//			}
//			printf("\n");
//			for (size_t j=0;j<matches[i].size();++j) {
//				printf ("%d ", matches[i][j][1]);
//			}
//			printf("\n");
//		}
//
//		visualize(matches[0]);

	}

	void init(const string& path)
	{
		bfs::path template_dir(path);

		if (!bfs::is_directory(template_dir)) {
			ROS_ERROR("Cannot load templates, %s is not a directory", template_dir.leaf().c_str());
		}

		bfs::directory_iterator dir_iter(template_dir), dir_end;
		for(;dir_iter != dir_end; ++dir_iter) {
			if (bfs::extension(*dir_iter)==".ply") {
				loadModel(dir_iter->string());
			}
		}
	}


};




int main(int argc, char** argv)
{
	ros::init(argc,argv,"match_features");
	if (argc<2) {
		ROS_INFO("Usage: %s path", argv[0]);
	}
	ComputeFeatures cf;

	cf.init(string(argv[1]));

	ROS_INFO("Starting spinning");
	ros::spin();

	return 0;
}
