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

#include<boost/filesystem.hpp>


#include <ros/ros.h>
#include "recognition_lambertian/ModelFit.h"

#include "robot_msgs/Point.h"
#include "visualization_msgs/Marker.h"


namespace bfs = boost::filesystem;
using namespace std;
using namespace robot_msgs;

namespace model_fit {


class TemplateModel;

class ModelFitSet
{
public:
	struct ModelFit
	{
		TemplateModel* model_;
		Point32 location_;
		float score_;
		float max_dist_;

		ModelFit() : model_(NULL)
		{
		}

		ModelFit(TemplateModel* model, const Point32& location, float score, float max_dist) :
			model_(model), location_(location), score_(score), max_dist_(max_dist)
		{
		}

		float score() {
			return score_;
		}
	};
	int size_;
	int count_;
	ModelFit* best_fit_;

	ModelFitSet(int size) : size_(size), count_(0)
	{
		best_fit_ = new ModelFit[size_];
	}

	void add(TemplateModel* model, const Point32& location, float score, float max_dist)
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

class TemplateModel {
	int x_res, y_res, z_res;
	float x_min, y_min, z_min;
	float x_max, y_max, z_max;
	float x_d, y_d, z_d;
	float truncate_value;

	float *grid;

public:
	TemplateModel()
	{

	}

	~TemplateModel()
	{
		delete[] grid;
	}

	void load(const string& file)
	{
		FILE* f;
		f = fopen(file.c_str(),"r");
		if (f==NULL) {
			ROS_ERROR("Cannot open template file: %s", file.c_str());
		}
		fscanf(f,"%d %d %d ", &x_res, &y_res, &z_res );
		fscanf(f,"%g %g %g ", &x_min, &y_min, &z_min );
		fscanf(f,"%g %g %g ", &x_max, &y_max, &z_max );
		x_d = (x_max-x_min)/(x_res-1);
		y_d = (y_max-y_min)/(y_res-1);
		z_d = (z_max-z_min)/(z_res-1);

		grid = new float[x_res*y_res*z_res];
		fread(grid, sizeof(float), x_res*y_res*z_res, f);
		fclose(f);

		truncate_value = *max_element(grid, grid+x_res*y_res*z_res);
	}


	void show(const ros::Publisher& publisher, const Point32& location, float fit_score)
	{
		static int model_id = 0;

		visualization_msgs::Marker marker;
		marker.header.stamp = ros::Time::now();
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
		marker.scale.x = x_d;
		marker.scale.y = y_d;
		marker.scale.z = z_d;
		marker.color.a = 0.7;

		if (fit_score<7) {
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 1.0;
		} else {
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
		}

		for (int i=0;i<x_res;++i) {
			for (int j=0;j<y_res;++j) {
				for (int k=0;k<z_res;++k) {
					if (grid[((i*y_res)+j)*z_res+k]==0) {
						Point p;
						p.x = i*x_d+x_min;
						p.y = j*y_d+y_min;
						p.z = k*z_d+z_min;
                        marker.points.push_back(p);
					}
				}
			}
		}
		publisher.publish(marker);
	}

	bool in_bounds(int x, int y, int z)
	{
		return (x>=0 && x<x_res && y>=0 && y<y_res && z>=0 && z<z_res);
	}

	void fitPointCloud(const PointCloud& cloud, const Point32& location, ModelFitSet& mfs)
	{
		float score = 0;
		float max_dist = 0;
		for (size_t i=0;i<cloud.pts.size();i++) {
			int x = int(((cloud.pts[i].x-location.x)-x_min)/x_d);
			int y = int(((cloud.pts[i].y-location.y)-y_min)/y_d);
			int z = int(((cloud.pts[i].z-location.z)-z_min)/z_d);

			float val;
			if (in_bounds(x,y,z)) {
				val = grid[((x*y_res)+y)*z_res+z];
			}
			else {
				val = truncate_value;
			}
			max_dist = max(max_dist,val);
			score += val;
		}
		score /= (cloud.pts.size());

		mfs.add(this, location, score, max_dist);
	}

	void findBestFit(const PointCloud& cloud, ModelFitSet& mfs)
	{
		// compute center of point cloud
		Point32 center;
		center.x =0; center.y = 0; center.z = 0;
		int count = cloud.pts.size();

		for (int i=0;i<count;++i) {
			center.x += cloud.pts[i].x;
			center.y += cloud.pts[i].y;
//			center.z += cloud.pts[i].z;
		}

		center.x /=count;
		center.y /=count;
//		center.z /=count;

		Point32 location = center;

		for (float dx=-0.02; dx<=0.02; dx+=0.01) {
			for (float dy=-0.02; dy<=0.02; dy+=0.01) {
				location.x = center.x + dx;
				location.y = center.y + dy;

				fitPointCloud(cloud, location, mfs);
			}
		}
	}
};



class ModelFitter
{
	ros::NodeHandle nh_;
	ros::ServiceServer service_;
	ros::Publisher marker_pub_;

	string template_path;

	vector<TemplateModel*> templates;

public:
	ModelFitter()
	{
		nh_.param<string>("~template_path", template_path, "");
		service_ = nh_.advertiseService("recognition_lambertian/model_fit", &ModelFitter::fitModel, this);

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
		ROS_INFO("Loading template: %s", filename.c_str());
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
			if (bfs::extension(*dir_iter)==".dt") {
				loadTemplate(dir_iter->string());
			}
		}

	}


	void fitBestModel(const PointCloud& cloud, ModelFitSet& mfs)
	{
		for (size_t i=0;i<templates.size();++i) {
			templates[i]->findBestFit(cloud, mfs);
		}
	}

	bool fitModel(recognition_lambertian::ModelFit::Request& req,
			recognition_lambertian::ModelFit::Response& resp)
	{
		ROS_INFO("Service called");

		ModelFitSet mfs(1);

		clock_t start = clock();
		fitBestModel(req.cloud, mfs);
		double duration = double(clock()-start)/CLOCKS_PER_SEC;

		ROS_INFO("Best score: %g, time:%g", mfs.best_fit_[0].score(), duration);

		mfs.best_fit_[0].model_->show(marker_pub_,  mfs.best_fit_[0].location_,  mfs.best_fit_[0].score());

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
